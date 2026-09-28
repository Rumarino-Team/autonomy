#include "hydrus_batch.h"
#include "hydrus_camera.h"
#include "hydrus_core.h"

#include <algorithm>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

// Fixed workers that each run a contiguous slice of [0, count) and then park.
class ThreadPool {
public:
    explicit ThreadPool(int threads) {
        for (int i = 1; i < threads; ++i)
            workers_.emplace_back([this, i] { workerMain(i); });
    }

    ~ThreadPool() {
        {
            std::lock_guard<std::mutex> lock(mu_);
            stop_ = true;
            ++generation_;
        }
        start_.notify_all();
        for (std::thread& t : workers_)
            t.join();
    }

    int size() const { return static_cast<int>(workers_.size()) + 1; }

    void parallelFor(int count, const std::function<void(int)>& fn) {
        if (workers_.empty() || count <= 1) {
            for (int i = 0; i < count; ++i)
                fn(i);
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mu_);
            fn_ = &fn;
            count_ = count;
            pending_ = static_cast<int>(workers_.size());
            ++generation_;
        }
        start_.notify_all();
        runSlice(0);
        std::unique_lock<std::mutex> lock(mu_);
        done_.wait(lock, [this] { return pending_ == 0; });
        fn_ = nullptr;
    }

private:
    void runSlice(int worker) {
        const int n = size();
        const int begin = static_cast<int>(static_cast<long>(count_) * worker / n);
        const int end = static_cast<int>(static_cast<long>(count_) * (worker + 1) / n);
        for (int i = begin; i < end; ++i)
            (*fn_)(i);
    }

    void workerMain(int worker) {
        uint64_t seen = 0;
        for (;;) {
            {
                std::unique_lock<std::mutex> lock(mu_);
                start_.wait(lock, [&] { return generation_ != seen; });
                seen = generation_;
                if (stop_)
                    return;
            }
            runSlice(worker);
            {
                std::lock_guard<std::mutex> lock(mu_);
                if (--pending_ == 0)
                    done_.notify_one();
            }
        }
    }

    std::vector<std::thread> workers_;
    std::mutex mu_;
    std::condition_variable start_;
    std::condition_variable done_;
    const std::function<void(int)>* fn_ = nullptr;
    int count_ = 0;
    int pending_ = 0;
    uint64_t generation_ = 0;
    bool stop_ = false;
};

void writeState(const hydrus::HydrusSim& sim, double* out) {
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

struct HydrusBatch {
    std::unique_ptr<hydrus::HydrusModel> model;
    std::vector<std::unique_ptr<hydrus::HydrusSim>> sims;
    std::unique_ptr<ThreadPool> pool;
    hydrus::CameraModel camera;
    std::vector<hydrus::CameraView> views;
};

extern "C" {

HydrusBatch* hydrus_batch_create(const char* xml_path, int num_envs, int num_threads) {
    if (xml_path == nullptr || num_envs <= 0) {
        std::fprintf(stderr, "[hydrus_batch] need an xml path and num_envs > 0\n");
        return nullptr;
    }
    std::string error;
    hydrus::HydrusModel* model = hydrus::HydrusModel::load(xml_path, error);
    if (model == nullptr) {
        std::fprintf(stderr, "[hydrus_batch] failed to load %s: %s\n", xml_path, error.c_str());
        return nullptr;
    }
    auto* batch = new HydrusBatch();
    batch->model.reset(model);
    batch->sims.reserve(num_envs);
    for (int i = 0; i < num_envs; ++i)
        batch->sims.emplace_back(new hydrus::HydrusSim(*model));
    if (num_threads <= 0)
        num_threads = static_cast<int>(std::max(1u, std::thread::hardware_concurrency()));
    batch->pool.reset(new ThreadPool(std::min(num_threads, num_envs)));
    batch->camera = hydrus::CameraModel::build(model->model);
    batch->views.resize(num_envs);
    return batch;
}

void hydrus_batch_destroy(HydrusBatch* batch) {
    if (batch == nullptr)
        return;
    batch->pool.reset();
    batch->sims.clear();
    delete batch;
}

int hydrus_batch_num_envs(const HydrusBatch* batch) {
    return batch != nullptr ? static_cast<int>(batch->sims.size()) : 0;
}

double hydrus_batch_timestep(const HydrusBatch* batch) {
    return batch != nullptr ? batch->model->model->opt.timestep : 0.0;
}

void hydrus_batch_reset(
    HydrusBatch* batch,
    const uint8_t* mask,
    const double* init_state,
    const double* randomization) {
    if (batch == nullptr)
        return;
    batch->pool->parallelFor(static_cast<int>(batch->sims.size()), [&](int i) {
        if (mask != nullptr && mask[i] == 0)
            return;
        hydrus::HydrusSim& sim = *batch->sims[i];
        hydrus::Randomization r;
        if (randomization != nullptr) {
            const double* src = randomization + HYDRUS_BATCH_RANDOMIZATION_SIZE * i;
            r.dry_mass_scale = src[0];
            r.volume_scale = src[1];
            r.drag_scale = src[2];
            r.thrust_scale = src[3];
            r.rotor_inertia_scale = src[4];
            r.current[0] = src[5];
            r.current[1] = src[6];
            r.current[2] = src[7];
        }
        sim.setRandomization(r);
        batch->views[i].clear();
        if (init_state == nullptr) {
            sim.reset();
            return;
        }
        const double* s = init_state + HYDRUS_BATCH_INIT_SIZE * i;
        const mjtNum quat[4] = {s[6], s[3], s[4], s[5]};
        sim.reset(s, quat, s + 7, s + 10);
    });
}

void hydrus_batch_step(HydrusBatch* batch, const float* thrusters, int substeps, double* state_out) {
    if (batch == nullptr)
        return;
    batch->pool->parallelFor(static_cast<int>(batch->sims.size()), [&](int i) {
        hydrus::HydrusSim& sim = *batch->sims[i];
        sim.setThrusters(
            thrusters != nullptr ? thrusters + HYDRUS_BATCH_THRUSTERS * i : nullptr,
            HYDRUS_BATCH_THRUSTERS);
        for (int k = 0; k < substeps; ++k)
            sim.step();
        if (state_out != nullptr)
            writeState(sim, state_out + HYDRUS_BATCH_STATE_SIZE * i);
    });
}

int hydrus_batch_mocap_count(const HydrusBatch* batch) {
    return batch != nullptr ? batch->model->model->nmocap : 0;
}

void hydrus_batch_set_mocap(HydrusBatch* batch, const double* poses) {
    if (batch == nullptr || poses == nullptr)
        return;
    const int nmocap = batch->model->model->nmocap;
    batch->pool->parallelFor(static_cast<int>(batch->sims.size()), [&](int i) {
        mjData* data = batch->sims[i]->data();
        const double* src = poses + HYDRUS_BATCH_MOCAP_POSE * nmocap * i;
        for (int m = 0; m < nmocap; ++m) {
            const double* pose = src + HYDRUS_BATCH_MOCAP_POSE * m;
            mju_copy3(data->mocap_pos + 3 * m, pose);
            data->mocap_quat[4 * m] = pose[6];
            data->mocap_quat[4 * m + 1] = pose[3];
            data->mocap_quat[4 * m + 2] = pose[4];
            data->mocap_quat[4 * m + 3] = pose[5];
        }
        mj_forward(batch->model->model, data);
    });
}

void hydrus_batch_camera(HydrusBatch* batch, float* features) {
    if (batch == nullptr || features == nullptr)
        return;
    batch->pool->parallelFor(static_cast<int>(batch->sims.size()), [&](int i) {
        batch->views[i].render(
            batch->model->model,
            batch->sims[i]->data(),
            batch->camera,
            features + HYDRUS_CAM_FEATURE_SIZE * i);
    });
}

void hydrus_batch_camera_image(const HydrusBatch* batch, int env, uint8_t* pixels) {
    if (batch == nullptr || pixels == nullptr || env < 0 || env >= static_cast<int>(batch->views.size()))
        return;
    const hydrus::CameraView& view = batch->views[env];
    const std::vector<uint8_t>& src = view.has_previous ? view.previous : view.image;
    std::memcpy(pixels, src.data(), src.size());
}

void hydrus_batch_get_state(const HydrusBatch* batch, double* state_out) {
    if (batch == nullptr || state_out == nullptr)
        return;
    for (size_t i = 0; i < batch->sims.size(); ++i)
        writeState(*batch->sims[i], state_out + HYDRUS_BATCH_STATE_SIZE * i);
}

}  // extern "C"
