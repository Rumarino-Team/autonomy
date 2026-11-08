#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#endif
#include "auv_frame.hpp"
#include "auv_simulator.h"
#include "scene_presets.h"
#include "internal.hpp"
#include "dynlib.h"
#include <algorithm>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>
#ifndef _WIN32
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#ifndef SIMULATOR_MUJOCO_BACKEND_FILENAME
#define SIMULATOR_MUJOCO_BACKEND_FILENAME "libsimulator_backend_mujoco.so"
#endif

namespace {
using namespace core;

// AMJB version 2. NED maps to Blender with M = diag(-1, 1, -1).
struct BlenderLink {
#ifdef _WIN32
    SOCKET socket = INVALID_SOCKET;
#else
    int socket = -1;
#endif
    uint16_t port = 9760;
    double next_time = 0;
    bool open_ok = false;
    std::vector<unsigned char> packet;

    void open() {
        close();
        const char *flag = std::getenv("AUTONOMY_SIM_BLENDER");
        if (!flag || !*flag || flag[0] == '0')
            return;
        port = 9760;
        if (const char *text = std::getenv("AUTONOMY_SIM_BLENDER_PORT")) {
            unsigned value = 0;
            auto [end, error] = std::from_chars(text, text + std::strlen(text), value);
            if (error == std::errc() && !*end && value > 0 && value <= 65535)
                port = static_cast<uint16_t>(value);
        }
#ifdef _WIN32
        WSADATA data;
        if (WSAStartup(MAKEWORD(2, 2), &data) != 0)
            return;
        socket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (socket == INVALID_SOCKET) {
            WSACleanup();
            return;
        }
        u_long nonblocking = 1;
        ioctlsocket(socket, FIONBIO, &nonblocking);
#else
        socket = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (socket < 0)
            return;
#endif
        open_ok = true;
    }

    void close() {
        if (!open_ok &&
#ifdef _WIN32
            socket == INVALID_SOCKET
#else
            socket < 0
#endif
        )
            return;
#ifdef _WIN32
        if (socket != INVALID_SOCKET)
            closesocket(socket);
        socket = INVALID_SOCKET;
        if (open_ok)
            WSACleanup();
#else
        if (socket >= 0)
            ::close(socket);
        socket = -1;
#endif
        open_ok = false;
        next_time = 0;
    }

    void publish(
        const SimulationRuntime *runtime, const Simulation &model, const Observation &observation,
        uint64_t time_ns
    ) {
        if (!open_ok || !runtime)
            return;
        const double time = double(time_ns) * 1e-9;
        if (time + 1e-9 < next_time)
            return;
        const SimBodyState *states = nullptr;
        uint32_t count = 0;
        if (simulation_runtime_states(runtime, &states, &count) != SIM_OK || !states)
            return;
        packet.clear();
        uint16_t poses = 0;
        auto pose = [&](uint32_t id, const char *name, MathPose value) {
            if (!name || !name[0] || poses == 64)
                return;
            ++poses;
            for (int i = 0; i < 4; ++i)
                packet.push_back(static_cast<unsigned char>(id >> (8 * i)));
            const auto offset = packet.size();
            packet.resize(offset + 32, 0);
            std::memcpy(packet.data() + offset, name, std::min<size_t>(std::strlen(name), 31));
            auto number = [&](double x) {
                uint64_t bits = 0;
                std::memcpy(&bits, &x, sizeof bits);
                for (int i = 0; i < 8; ++i)
                    packet.push_back(static_cast<unsigned char>(bits >> (8 * i)));
            };
            number(-value.pos.buf[0]);
            number(value.pos.buf[1]);
            number(-value.pos.buf[2]);
            number(value.quat.buf[3]);
            number(-value.quat.buf[0]);
            number(value.quat.buf[1]);
            number(-value.quat.buf[2]);
        };
        for (uint32_t i = 0; i < model.object_count && i < count; ++i) {
            const auto &body = model.object_table[i]->body;
            if (body.motion_type != SIM_DYNAMIC && !body.tracked && !body.fuzz)
                continue;
            pose(body.id, body.name, states[i].pose);
            if (std::strcmp(body.name, "gate opening") == 0)
                pose(body.id, "gate", states[i].pose);
        }
        pose(UINT32_MAX, "camera", observation.camera);
        std::vector<unsigned char> header;
        header.insert(header.end(), {'A', 'M', 'J', 'B', 2, 0});
        header.push_back(static_cast<unsigned char>(poses & 255));
        header.push_back(static_cast<unsigned char>(poses >> 8));
        uint64_t bits = 0;
        std::memcpy(&bits, &time, sizeof bits);
        for (int i = 0; i < 8; ++i)
            header.push_back(static_cast<unsigned char>(bits >> (8 * i)));
        header.insert(header.end(), packet.begin(), packet.end());
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(port);
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
#ifdef _WIN32
        ::sendto(
            socket, reinterpret_cast<const char *>(header.data()), static_cast<int>(header.size()), 0,
            reinterpret_cast<sockaddr *>(&address), sizeof address
        );
#else
        ::sendto(
            socket, header.data(), header.size(), MSG_DONTWAIT, reinterpret_cast<sockaddr *>(&address),
            sizeof address
        );
#endif
        next_time = time + 1.0 / 30.0;
    }
};

struct Context {
    Simulation owned_model{};
    const Simulation *model = nullptr;
    PhysicsBackend backend{};
    SimulationRuntime *runtime = nullptr;
    AuvSimulationStatus status{};
    double commands[8]{};
    uint64_t limit = 21600, frame_ticks = 6;
    double min_depth = 0, max_depth = 0;
    bool realtime = false, print_summary = true;
    std::chrono::steady_clock::time_point started;
    BlenderLink link;

    void release() {
        link.close();
        simulation_runtime_destroy(runtime);
        runtime = nullptr;
        physics_backend_close(&backend);
        simulation_destroy(&owned_model);
        model = nullptr;
    }

    ~Context() {
        release();
    }
};

thread_local Context ctx;

const char *environment(const char *key, const char *fallback) {
    const char *value = std::getenv(key);
    return value && *value ? value : fallback;
}

uint64_t setting(const char *key, uint64_t fallback) {
    const char *s = std::getenv(key);
    if (!s)
        return fallback;
    uint64_t value;
    auto [end, error] = std::from_chars(s, s + std::strlen(s), value);
    require(error == std::errc() && !*end, "invalid simulator integer setting");
    return value;
}

void error(const char *message) {
    ctx.status.code = 2;
    ctx.status.tick = simulation_runtime_tick(ctx.runtime);
    std::snprintf(ctx.status.message, sizeof ctx.status.message, "%s", message);
    std::fill_n(ctx.commands, 8, 0.);
}

bool ends_with_ci(const std::string &value, const char *suffix) {
    const size_t width = std::strlen(suffix);
    if (value.size() < width)
        return false;
    for (size_t i = 0; i < width; ++i) {
        char left = value[value.size() - width + i];
        char right = suffix[i];
        if (left >= 'A' && left <= 'Z')
            left = char(left - 'A' + 'a');
        if (right >= 'A' && right <= 'Z')
            right = char(right - 'A' + 'a');
        if (left != right)
            return false;
    }
    return true;
}

std::filesystem::path snapshot_script(const std::filesystem::path &start) {
    auto dir = start;
    for (int i = 0; i < 8; ++i) {
        auto candidate = dir / "tools" / "blender" / "export_snapshot.py";
        if (std::filesystem::is_regular_file(candidate))
            return candidate;
        auto parent = dir.parent_path();
        if (parent == dir)
            break;
        dir = parent;
    }
    return {};
}

std::filesystem::path blender_executable() {
    if (const char *configured = std::getenv("AUTONOMY_BLENDER"))
        if (*configured && std::filesystem::is_regular_file(configured))
            return configured;
#ifdef _WIN32
    const char *installed = "C:/Program Files/Blender Foundation/Blender 4.5/blender.exe";
    if (std::filesystem::is_regular_file(installed))
        return installed;
#endif
    return "blender";
}

std::string quote_arg(const std::filesystem::path &path) {
    std::string quoted = "\"";
    for (char c : path.string()) {
        if (c == '"')
            quoted += "\\\"";
        else
            quoted.push_back(c);
    }
    quoted.push_back('"');
    return quoted;
}

struct TemporarySnapshot {
    std::filesystem::path path;

    ~TemporarySnapshot() {
        if (!path.empty())
            std::filesystem::remove(path);
    }
};

std::filesystem::path export_blend(const std::filesystem::path &blend, const std::filesystem::path &module_dir) {
    require(std::filesystem::is_regular_file(blend), "cannot read blend file");
    auto script = snapshot_script(module_dir);
    require(!script.empty(), "cannot find blend snapshot exporter");
    auto output = std::filesystem::temp_directory_path() /
                  ("autonomy-scene-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
                   ".json");
    std::string command = quote_arg(blender_executable()) + " --background --factory-startup --disable-autoexec " +
                          quote_arg(std::filesystem::absolute(blend)) + " --python " + quote_arg(script) + " -- " +
                          quote_arg(output);
#ifdef _WIN32
    // cmd.exe /c drops the first and last quote when a line contains more than one pair.
    command = "\"" + command + "\"";
#endif
    require(std::system(command.c_str()) == 0 && std::filesystem::is_regular_file(output), "blend export failed");
    return output;
}

uint64_t timestamp() {
    return simulation_runtime_tick(ctx.runtime) * uint64_t(ctx.model->settings.dt_numerator) *
           1000000000ULL / ctx.model->settings.dt_denominator;
}

void validate_timing() {
    require(
        ctx.frame_ticks && ctx.limit && ctx.limit % ctx.frame_ticks == 0,
        "tick budget must be positive and divisible by frame ticks"
    );
    require(
        ctx.model->settings.dt_numerator && ctx.model->settings.dt_denominator &&
            ctx.limit <= INT64_MAX / (uint64_t(ctx.model->settings.dt_numerator) * 1000000000ULL),
        "simulation timestamp overflow"
    );
}

void depth() {
    const SimBodyState *states;
    uint32_t count;
    check(simulation_runtime_states(ctx.runtime, &states, &count), "cannot read depth");
    double z = states[ctx.model->robot.object.id.index].pose.pos.buf[2];
    ctx.min_depth = std::min(ctx.min_depth, z);
    ctx.max_depth = std::max(ctx.max_depth, z);
}

} // namespace

extern "C" {
SIM_API void auv_deinit(void) {
    if (ctx.runtime && ctx.print_summary) {
        std::printf(
            "{\"steps\":%llu,\"hash\":%llu,\"min_depth\":%.9g,\"max_depth\":%.9g,\"status\":%u}\n",
            (unsigned long long)simulation_runtime_tick(ctx.runtime),
            (unsigned long long)simulation_runtime_hash(ctx.runtime),
            ctx.min_depth,
            ctx.max_depth,
            ctx.status.code
        );
        std::fflush(stdout);
    }
    ctx.release();
    std::fill_n(ctx.commands, 8, 0.);
}

SIM_API void auv_init(void) {
    auv_deinit();
    ctx.status = {};
    char message[512];
    auto result = guard(message, sizeof message, [&] {
        const auto *robot = sim_robot_find(environment("AUTONOMY_SIM_ROBOT", "hydrus"));
        require(robot, "unknown robot preset");
        const std::string scene = environment("AUTONOMY_SIM_SCENE", "robosub_course");
        TemporarySnapshot snapshot;
        if (ends_with_ci(scene, ".blend") || ends_with_ci(scene, ".json")) {
            std::filesystem::path source = scene;
            if (ends_with_ci(scene, ".blend")) {
                char module_path[32768];
                require(
                    dynlib_module_path(reinterpret_cast<void *>(&auv_init), module_path, sizeof module_path),
                    "cannot locate simulator plugin"
                );
                snapshot.path = export_blend(source, std::filesystem::path(module_path).parent_path());
                source = snapshot.path;
            }
            check(
                simulation_scene_load(
                    &ctx.owned_model, source.string().c_str(), robot, message, sizeof message
                ),
                message
            );
        } else {
            check(
                simulation_scene_create(&ctx.owned_model, scene.c_str(), robot, message, sizeof message),
                message
            );
        }
        ctx.model = &ctx.owned_model;
        char module_path[32768];
        require(
            dynlib_module_path(reinterpret_cast<void *>(&auv_init), module_path, sizeof module_path),
            "cannot locate simulator plugin"
        );
        auto path = std::filesystem::absolute(module_path).parent_path() /
                    SIMULATOR_MUJOCO_BACKEND_FILENAME;
        const auto backend_path = path.string();
        check(
            physics_backend_open(
                &ctx.backend,
                environment("AUTONOMY_SIM_BACKEND", backend_path.c_str()),
                message,
                sizeof message
            ),
            message
        );
        ctx.limit = setting("AUTONOMY_SIM_MAX_TICKS", 21600);
        ctx.frame_ticks = setting("AUTONOMY_SIM_FRAME_TICKS", 6);
        validate_timing();
        const auto realtime = setting("AUTONOMY_SIM_REALTIME", 0);
        require(realtime <= 1, "realtime setting must be 0 or 1");
        ctx.realtime = realtime;
        ctx.print_summary = true;
        check(
            simulation_runtime_create(
                ctx.model, &ctx.backend, nullptr, &ctx.runtime, message, sizeof message
            ),
            message
        );
        ctx.min_depth = ctx.max_depth = ctx.model->robot.object.body.initial_pose.pos.buf[2];
        ctx.started = std::chrono::steady_clock::now();
        ctx.link.open();
    });
    if (result != SIM_OK) {
        error(message);
        ctx.release();
    }
}

SIM_API void auv_set_thrustor_values(const float *values, uint8_t count) {
    if (ctx.status.code)
        return;
    if (!ctx.runtime || !values || count != ctx.model->robot.thruster_count) {
        error("thruster command count does not match robot");
        return;
    }
    for (uint32_t i = 0; i < count; ++i)
        if (!std::isfinite(values[i]) || values[i] < -1 || values[i] > 1) {
            error("thruster command must be finite and in [-1,1]");
            return;
        }
    for (uint32_t i = 0; i < count; ++i)
        ctx.commands[i] = values[i];
}

SIM_API void auv_yield_until_next_frame(AuvFrame *frame) {
    if (!frame) {
        error("null AUV frame");
        return;
    }
    *frame = {};
    frame->camera_pose.quat = frame->imu_quat = q();
    if (ctx.status.code)
        return;
    char message[512];
    auto result = guard(message, sizeof message, [&] {
        require(ctx.runtime, "simulator is not initialized");
        if (simulation_runtime_tick(ctx.runtime) >= ctx.limit) {
            ctx.status.code = 1;
            std::snprintf(
                ctx.status.message, sizeof ctx.status.message, "simulation tick budget exhausted"
            );
            std::fill_n(ctx.commands, 8, 0.);
            return;
        }
        for (uint64_t i = 0; i < ctx.frame_ticks; ++i) {
            check(
                simulation_runtime_step(
                    ctx.runtime,
                    ctx.commands,
                    ctx.model->robot.thruster_count,
                    message,
                    sizeof message
                ),
                message
            );
            depth();
        }
        ctx.status.tick = simulation_runtime_tick(ctx.runtime);
        if (ctx.realtime)
            std::this_thread::sleep_until(ctx.started + std::chrono::nanoseconds(timestamp()));
        Observation o{};
        check(simulation_runtime_observe(ctx.runtime, &o, message, sizeof message), message);
        fill_auv_frame(frame, o, timestamp());
        ctx.link.publish(ctx.runtime, *ctx.model, o, timestamp());
    });
    if (result != SIM_OK) {
        error(message);
        *frame = {};
        frame->camera_pose.quat = frame->imu_quat = q();
    }
}

SIM_API SimResult auv_simulator_init_scene(
    const Simulation *model, const AuvSimulatorOptions *options, char *message, uint32_t capacity
) {
    ctx.release();
    ctx.status = {};
    std::fill_n(ctx.commands, 8, 0.);
    auto result = guard(message, capacity, [&] {
        require(
            options && options->struct_size == sizeof(*options) &&
                options->version == AUV_SIMULATOR_EXTENSION_VERSION,
            "simulator extension v1 required",
            SIM_UNSUPPORTED
        );
        validate_model(model);
        require(model->robot.registered, "scene requires a registered robot");
        require(
            options->backend_path && options->realtime <= 1 && options->print_summary <= 1,
            "invalid simulator options"
        );
        ctx.model = model;
        ctx.limit = options->max_ticks;
        ctx.frame_ticks = options->frame_ticks;
        ctx.realtime = options->realtime;
        ctx.print_summary = options->print_summary;
        validate_timing();
        char detail[512];
        check(
            physics_backend_open(&ctx.backend, options->backend_path, detail, sizeof detail), detail
        );
        require(
            !std::strcmp(ctx.backend.api.name, "mujoco"),
            "scene extension requires MuJoCo",
            SIM_UNSUPPORTED
        );
        check(
            simulation_runtime_create(
                model, &ctx.backend, nullptr, &ctx.runtime, detail, sizeof detail
            ),
            detail
        );
        ctx.min_depth = ctx.max_depth = model->robot.object.body.initial_pose.pos.buf[2];
        ctx.started = std::chrono::steady_clock::now();
        ctx.link.open();
    });
    if (result != SIM_OK) {
        error(message ? message : "scene initialization failed");
        ctx.release();
    }
    return result;
}

SIM_API SimResult auv_simulator_read_result(
    AuvSimulatorResult *out, uint32_t size, char *message, uint32_t capacity
) {
    return guard(message, capacity, [&] {
        require(out && size == sizeof(*out), "invalid simulator result size", SIM_UNSUPPORTED);
        *out = {};
        out->struct_size = sizeof(*out);
        out->version = AUV_SIMULATOR_EXTENSION_VERSION;
        out->status = ctx.status.code;
        out->tick = ctx.status.tick;
        std::memcpy(out->message, ctx.status.message, sizeof out->message);
        if (!ctx.runtime)
            return;
        out->tick = simulation_runtime_tick(ctx.runtime);
        out->hash = simulation_runtime_hash(ctx.runtime);
        out->min_depth = ctx.min_depth;
        out->max_depth = ctx.max_depth;
        const SimBodyState *states;
        uint32_t count;
        check(simulation_runtime_states(ctx.runtime, &states, &count), "cannot read result states");
        uint32_t index = ctx.model->robot.object.id.index;
        if (index < count) {
            out->robot = states[index];
            out->has_final_state = 1;
        }
    });
}

SIM_API bool auv_simulation_status(AuvSimulationStatus *out) {
    if (!out)
        return false;
    *out = ctx.status;
    return true;
}
} // extern "C"
