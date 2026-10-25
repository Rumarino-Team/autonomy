#include "auv_batch.h"
#include "auv_camera.h"
#include "auv_core.h"

#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace {

void writeState(const auv::SimulationManager& sim, double* out) {
    const mjtNum* pos = sim.position();
    const mjtNum* quat = sim.quaternion();
    out[0] = pos[0];
    out[1] = pos[1];
    out[2] = pos[2];
    out[3] = quat[1];
    out[4] = quat[2];
    out[5] = quat[3];
    out[6] = quat[0];
    sim.velocity(out + 7, out + 10);
    const mjtNum* gyro = sim.gyro();
    const mjtNum* accel = sim.accel();
    for (int i = 0; i < 3; ++i) {
        out[13 + i] = gyro != nullptr ? gyro[i] : 0.0;
        out[16 + i] = accel != nullptr ? accel[i] : 0.0;
    }
    out[19] = pos[2];
    out[20] = sim.data()->time;
}

}  // namespace

struct AuvBatch {
    std::unique_ptr<auv::AuvModel> model;
    std::vector<std::unique_ptr<auv::SimulationManager>> sims;
    auv::CameraModel camera;
    std::vector<auv::CameraView> views;
};

namespace {

void applyReset(
    auv::SimulationManager& sim,
    auv::CameraView& view,
    const double* init_row,
    const double* rand_row) {
    auv::Randomization r;
    if (rand_row != nullptr) {
        r.dry_mass_scale = rand_row[0];
        r.volume_scale = rand_row[1];
        r.drag_scale = rand_row[2];
        r.thrust_scale = rand_row[3];
        r.rotor_inertia_scale = rand_row[4];
        r.current[0] = rand_row[5];
        r.current[1] = rand_row[6];
        r.current[2] = rand_row[7];
    }
    sim.setRandomization(r);
    view.clear();
    if (init_row == nullptr) {
        sim.reset();
        return;
    }
    const mjtNum quat[4] = {init_row[6], init_row[3], init_row[4], init_row[5]};
    sim.reset(init_row, quat, init_row + 7, init_row + 10);
}

void resetEnv(AuvBatch* batch, int i, const uint8_t* mask, const double* init_state, const double* randomization) {
    if (mask != nullptr && mask[i] == 0)
        return;
    const double* init_row = init_state == nullptr ? nullptr : init_state + AUV_BATCH_INIT_SIZE * i;
    const double* rand_row = randomization == nullptr ? nullptr : randomization + AUV_BATCH_RANDOMIZATION_SIZE * i;
    applyReset(*batch->sims[i], batch->views[i], init_row, rand_row);
}

}  // namespace

extern "C" {

AuvBatch* auv_batch_create(const char* xml_path, int num_envs, int num_threads) {
    (void)num_threads;
    if (xml_path == nullptr || num_envs <= 0) {
        std::fprintf(stderr, "[auv_batch] need an xml path and num_envs > 0\n");
        return nullptr;
    }
    std::string error;
    auv::AuvModel* model = auv::AuvModel::load(xml_path, error);
    if (model == nullptr) {
        std::fprintf(stderr, "[auv_batch] failed to load %s: %s\n", xml_path, error.c_str());
        return nullptr;
    }
    auto* batch = new AuvBatch();
    batch->model.reset(model);
    batch->sims.reserve(num_envs);
    for (int i = 0; i < num_envs; ++i)
        batch->sims.emplace_back(new auv::SimulationManager(*model));
    batch->camera = auv::CameraModel::build(model->model);
    batch->views.resize(num_envs);
    return batch;
}

void auv_batch_destroy(AuvBatch* batch) {
    if (batch == nullptr)
        return;
    batch->sims.clear();
    delete batch;
}

int auv_batch_num_envs(const AuvBatch* batch) {
    return batch != nullptr ? static_cast<int>(batch->sims.size()) : 0;
}

int auv_batch_num_thrusters(const AuvBatch* batch) {
    return batch != nullptr ? batch->model->thrusterCount() : 0;
}

double auv_batch_timestep(const AuvBatch* batch) {
    return batch != nullptr ? batch->model->model->opt.timestep : 0.0;
}

void auv_batch_reset(
    AuvBatch* batch,
    const uint8_t* mask,
    const double* init_state,
    const double* randomization) {
    if (batch == nullptr)
        return;
    const int n = static_cast<int>(batch->sims.size());
    for (int i = 0; i < n; ++i)
        resetEnv(batch, i, mask, init_state, randomization);
}

void auv_batch_reset_env(AuvBatch* batch, int env, const double* init_row, const double* rand_row) {
    if (batch == nullptr || env < 0 || env >= static_cast<int>(batch->sims.size()))
        return;
    applyReset(*batch->sims[env], batch->views[env], init_row, rand_row);
}

void auv_batch_step_env(AuvBatch* batch, int env, const float* thrusters, int substeps, double* state_out) {
    if (batch == nullptr || env < 0 || env >= static_cast<int>(batch->sims.size()))
        return;
    auv::SimulationManager& sim = *batch->sims[env];
    sim.setThrusters(thrusters, batch->model->thrusterCount());
    for (int k = 0; k < substeps; ++k)
        sim.step();
    if (state_out != nullptr)
        writeState(sim, state_out);
}

void auv_batch_step(AuvBatch* batch, const float* thrusters, int substeps, double* state_out) {
    if (batch == nullptr)
        return;
    const int n = static_cast<int>(batch->sims.size());
    const int nthr = batch->model->thrusterCount();
    for (int i = 0; i < n; ++i) {
        const float* cmd = thrusters != nullptr ? thrusters + static_cast<std::ptrdiff_t>(nthr * i) : nullptr;
        double* out = state_out != nullptr ? state_out + static_cast<std::ptrdiff_t>(AUV_BATCH_STATE_SIZE * i) : nullptr;
        auv_batch_step_env(batch, i, cmd, substeps, out);
    }
}

int auv_batch_mocap_count(const AuvBatch* batch) {
    return batch != nullptr ? batch->model->model->nmocap : 0;
}

static void writeMocap(mjModel* model, mjData* data, const double* poses) {
    const int nmocap = model->nmocap;
    for (int m = 0; m < nmocap; ++m) {
        const double* pose = poses + AUV_BATCH_MOCAP_POSE * m;
        mju_copy3(data->mocap_pos + 3 * m, pose);
        data->mocap_quat[4 * m] = pose[6];
        data->mocap_quat[4 * m + 1] = pose[3];
        data->mocap_quat[4 * m + 2] = pose[4];
        data->mocap_quat[4 * m + 3] = pose[5];
    }
    mj_forward(model, data);
}

void auv_batch_set_mocap(AuvBatch* batch, const double* poses) {
    if (batch == nullptr || poses == nullptr)
        return;
    const int nmocap = batch->model->model->nmocap;
    const int n = static_cast<int>(batch->sims.size());
    for (int i = 0; i < n; ++i)
        writeMocap(batch->model->model, batch->sims[i]->data(), poses + AUV_BATCH_MOCAP_POSE * nmocap * i);
}

void auv_batch_set_mocap_env(AuvBatch* batch, int env, const double* poses) {
    if (batch == nullptr || poses == nullptr || env < 0 || env >= static_cast<int>(batch->sims.size()))
        return;
    writeMocap(batch->model->model, batch->sims[env]->data(), poses);
}

void auv_batch_camera(AuvBatch* batch, float* features) {
    if (batch == nullptr || features == nullptr)
        return;
    const int n = static_cast<int>(batch->sims.size());
    for (int i = 0; i < n; ++i) {
        batch->views[i].render(
            batch->model->model,
            batch->sims[i]->data(),
            batch->camera,
            features + AUV_CAM_FEATURE_SIZE * i);
    }
}

void auv_batch_camera_image(const AuvBatch* batch, int env, uint8_t* pixels) {
    if (batch == nullptr || pixels == nullptr || env < 0 || env >= static_cast<int>(batch->views.size()))
        return;
    const auv::CameraView& view = batch->views[env];
    const std::vector<uint8_t>& src = view.has_previous ? view.previous : view.image;
    std::memcpy(pixels, src.data(), src.size());
}

void auv_batch_get_state(const AuvBatch* batch, double* state_out) {
    if (batch == nullptr || state_out == nullptr)
        return;
    for (size_t i = 0; i < batch->sims.size(); ++i)
        writeState(*batch->sims[i], state_out + AUV_BATCH_STATE_SIZE * i);
}

}  // extern "C"

auv::SimulationManager* auv_batch_env(AuvBatch* batch, int env) {
    if (batch == nullptr || env < 0 || env >= static_cast<int>(batch->sims.size()))
        return nullptr;
    return batch->sims[env].get();
}

const auv::AuvModel* auv_batch_model(const AuvBatch* batch) {
    return batch != nullptr ? batch->model.get() : nullptr;
}
