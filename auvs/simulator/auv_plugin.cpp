#include "auv_frame.hpp"
#include "auv_simulator.h"
#include "scene_presets.h"
#include "internal.hpp"
#include <charconv>
#include <chrono>
#include <dlfcn.h>
#include <filesystem>
#include <thread>

namespace {
using namespace core;

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

    void release() {
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
        check(
            simulation_scene_create(
                &ctx.owned_model,
                environment("AUTONOMY_SIM_SCENE", "robosub_course"),
                robot,
                message,
                sizeof message
            ),
            message
        );
        ctx.model = &ctx.owned_model;
        Dl_info info{};
        require(
            dladdr(reinterpret_cast<void *>(&auv_init), &info) && info.dli_fname,
            "cannot locate simulator plugin"
        );
        auto path = std::filesystem::absolute(info.dli_fname).parent_path() /
                    "libsimulator_backend_mujoco.so";
        check(
            physics_backend_open(
                &ctx.backend,
                environment("AUTONOMY_SIM_BACKEND", path.c_str()),
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
