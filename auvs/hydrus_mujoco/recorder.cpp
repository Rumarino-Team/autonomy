#include "recorder.h"

#include "blender_link.h"

#include <algorithm>
#include <cinttypes>

namespace recorder {
namespace {

void writeString(std::FILE* file, const char* text) {
    std::fputc('"', file);
    for (const char* p = text != nullptr ? text : ""; *p != '\0'; ++p) {
        const unsigned char c = static_cast<unsigned char>(*p);
        if (c == '"' || c == '\\')
            std::fprintf(file, "\\%c", c);
        else if (c < 0x20)
            std::fprintf(file, "\\u%04x", c);
        else
            std::fputc(c, file);
    }
    std::fputc('"', file);
}

void writeNumbers(std::FILE* file, const double* values, int count) {
    std::fputc('[', file);
    for (int i = 0; i < count; ++i)
        std::fprintf(file, i == 0 ? "%.10g" : ",%.10g", std::isfinite(values[i]) ? values[i] : 0.0);
    std::fputc(']', file);
}

}  // namespace

Recorder* Recorder::open(
    const char* path,
    double fps,
    uint64_t seed,
    const auv::SimulationManager& sim,
    const AuvFrame& frame,
    const float* thrusters,
    int thruster_count) {
    if (path == nullptr || path[0] == '\0' || !(fps > 0))
        return nullptr;
    std::FILE* file = std::fopen(path, "w");
    if (file == nullptr) {
        std::perror("[auv_mujoco] record");
        return nullptr;
    }
    // Line buffered so a reader can follow progress while the run continues.
    std::setvbuf(file, nullptr, _IOLBF, 1 << 16);

    const mjModel* model = sim.model();
    const double timestep = model->opt.timestep;
    const int every = std::max(1, static_cast<int>(std::lround(1.0 / (fps * timestep))));
    auto* recorder = new Recorder(file, every);

    blender_link::Pose poses[blender_link::kMaxPoses];
    const int count = blender_link::collectPoses(model, sim.data(), sim.auvModel().body, sim.auvModel().camera_site, poses);
    std::fprintf(
        file,
        "{\"t\":\"header\",\"version\":1,\"seed\":%" PRIu64 ",\"fps\":%.10g,\"timestep\":%.17g,\"every\":%d,\"vehicle\":",
        seed,
        1.0 / (every * timestep),
        timestep,
        every);
    writeString(file, mj_id2name(model, mjOBJ_BODY, sim.auvModel().body));
    std::fputs(",\"objects\":[", file);
    for (int i = 0; i < count; ++i) {
        if (i > 0)
            std::fputc(',', file);
        writeString(file, poses[i].name);
    }
    std::fprintf(file, "],\"image\":[%u,%u],\"thrusters\":%d}\n", frame.image_width, frame.image_height, thruster_count);
    recorder->sample(sim, frame, thrusters, thruster_count);
    return recorder;
}

Recorder::Recorder(std::FILE* file, int every) : file_(file), every_(every) {}

Recorder::~Recorder() { stop("closed"); }

void Recorder::step(const auv::SimulationManager& sim, const AuvFrame& frame, const float* thrusters, int thruster_count) {
    if (file_ == nullptr)
        return;
    ++step_;
    time_ = sim.data()->time;
    if (step_ % static_cast<uint64_t>(every_) == 0)
        sample(sim, frame, thrusters, thruster_count);
}

void Recorder::sample(const auv::SimulationManager& sim, const AuvFrame& frame, const float* thrusters, int thruster_count) {
    std::FILE* file = file_;
    const mjData* data = sim.data();
    time_ = data->time;
    std::fprintf(file, "{\"t\":\"sample\",\"step\":%" PRIu64 ",\"time\":%.10g,\"poses\":{", step_, time_);
    blender_link::Pose poses[blender_link::kMaxPoses];
    const int count = blender_link::collectPoses(sim.model(), data, sim.auvModel().body, sim.auvModel().camera_site, poses);
    for (int i = 0; i < count; ++i) {
        if (i > 0)
            std::fputc(',', file);
        writeString(file, poses[i].name);
        const double pose[7] = {
            poses[i].pos[0], poses[i].pos[1], poses[i].pos[2],
            poses[i].quat[0], poses[i].quat[1], poses[i].quat[2], poses[i].quat[3],
        };
        std::fputc(':', file);
        writeNumbers(file, pose, 7);
    }
    const mjtNum* pos = sim.position();
    const mjtNum* quat = sim.quaternion();
    const double ned[7] = {pos[0], pos[1], pos[2], quat[1], quat[2], quat[3], quat[0]};
    mjtNum lin[3], ang[3];
    sim.velocity(lin, ang);
    const double vel[6] = {lin[0], lin[1], lin[2], ang[0], ang[1], ang[2]};
    std::fputs("},\"ned\":", file);
    writeNumbers(file, ned, 7);
    std::fputs(",\"vel\":", file);
    writeNumbers(file, vel, 6);
    std::fputs(",\"thr\":[", file);
    for (int i = 0; i < thruster_count; ++i)
        std::fprintf(file, i == 0 ? "%.6g" : ",%.6g", thrusters != nullptr ? static_cast<double>(thrusters[i]) : 0.0);
    std::fputs("],\"boxes\":[", file);
    for (int i = 0; i < frame.objects2d_len; ++i) {
        const AuvObject2d& box = frame.objects2d[i];
        std::fprintf(
            file,
            "%s[%u,%u,%u,%u,%u,%u]",
            i == 0 ? "" : ",",
            box.id,
            static_cast<unsigned>(box.cls),
            box.top_left.x,
            box.top_left.y,
            box.bottom_right.x,
            box.bottom_right.y);
    }
    std::fprintf(file, "],\"hash\":\"%016" PRIx64 "\"}\n", sim.stateHash());
}

void Recorder::event(const char* text) {
    if (file_ == nullptr)
        return;
    std::fprintf(file_, "{\"t\":\"event\",\"step\":%" PRIu64 ",\"time\":%.10g,\"text\":", step_, time_);
    writeString(file_, text);
    std::fputs("}\n", file_);
}

void Recorder::stop(const char* reason) {
    if (file_ == nullptr)
        return;
    std::fprintf(file_, "{\"t\":\"end\",\"step\":%" PRIu64 ",\"time\":%.10g,\"reason\":", step_, time_);
    writeString(file_, reason);
    std::fputs("}\n", file_);
    std::fclose(file_);
    file_ = nullptr;
}

}  // namespace recorder
