#include "../examples/scene.h"
#include "../internal.hpp"
#include "../dynlib.h"
#include <future>
#include <iostream>
using namespace core;

namespace {
void test(bool v, const char *s) {
    require(v, s);
}

struct Scene {
    Simulation m{};

    ~Scene() {
        simulation_destroy(&m);
    }
};

struct Runtime {
    SimulationRuntime *p = nullptr;

    ~Runtime() {
        simulation_runtime_destroy(p);
    }
};

SimResult
constant_force(const SimForceContext *, void *, SimWrench *w, uint32_t count, char *, uint32_t) {
    if (!count)
        return SIM_INVALID;
    sim_add_force(w, v(2, 0, 0), v(0, 1, 0), v());
    return SIM_OK;
}

uint64_t run(const PhysicsBackend &backend, uint32_t thrusters) {
    Scene scene;
    char e[512];
    check(example_scene(&scene.m, thrusters, e, sizeof e), e);
    check(simulation_finalize(&scene.m, e, sizeof e), e);
    Runtime runtime;
    check(simulation_runtime_create(&scene.m, &backend, nullptr, &runtime.p, e, sizeof e), e);
    double commands[8];
    std::fill_n(commands, 8, .1);
    for (unsigned i = 0; i < 72; ++i)
        check(simulation_runtime_step(runtime.p, commands, thrusters, e, sizeof e), e);
    auto hash = simulation_runtime_hash(runtime.p);
    check(simulation_runtime_reset(runtime.p, nullptr, 0, e, sizeof e), e);
    for (unsigned i = 0; i < 72; ++i)
        check(simulation_runtime_step(runtime.p, commands, thrusters, e, sizeof e), e);
    test(hash == simulation_runtime_hash(runtime.p), "cold reset determinism");
    Observation observation;
    check(simulation_runtime_observe(runtime.p, &observation, e, sizeof e), e);
    test(observation.object_count == 0 && observation.imu_ok, "observations");
    return hash;
}
} // namespace

int main(int argc, char **argv) {
    try {
        require(argc == 2, "backend path required");
        PhysicsBackend backend{};
        char e[512];
        check(physics_backend_open(&backend, argv[1], e, sizeof e), e);

        struct Close {
            PhysicsBackend *b;

            ~Close() {
                physics_backend_close(b);
            }
        } close{&backend};

        test(backend.api.abi_version == 5, "ABI version");
        PhysicsBackendApi rejected{};
        auto *symbol = dynlib_symbol(backend.library, "sim_backend_get_api");
        PhysicsBackendGetApi get;
        std::memcpy(&get, &symbol, sizeof get);
        test(get(4, sizeof rejected, &rejected) == SIM_UNSUPPORTED, "ABI v4 rejection");
        test(get(5, sizeof rejected - 1, &rejected) == SIM_UNSUPPORTED, "API size rejection");
        test(
            backend.api.world_create(nullptr, nullptr, e, sizeof e) == SIM_INVALID, "null creation"
        );
        auto a = run(backend, 6), b = run(backend, 6);
        test(a == b, "repeat determinism");
        auto future = std::async(std::launch::async, [&] { return run(backend, 8); });
        auto c = run(backend, 8);
        test(c == future.get(), "concurrent isolation");
        Scene force;
        check(example_scene(&force.m, 0, e, sizeof e), e);
        check(simulation_add_module(&force.m, "gravity_compensation", e, sizeof e), e);
        check(simulation_add_module(&force.m, "constant", e, sizeof e), e);
        check(simulation_finalize(&force.m, e, sizeof e), e);
        ForceModule module{};
        std::strcpy(module.name, "constant");
        module.evaluate = constant_force;
        ForceRegistry registry{&module, 1};
        Runtime runtime;
        check(simulation_runtime_create(&force.m, &backend, &registry, &runtime.p, e, sizeof e), e);
        check(simulation_runtime_step(runtime.p, nullptr, 0, e, sizeof e), e);
        const SimBodyState *states;
        uint32_t count;
        check(simulation_runtime_states(runtime.p, &states, &count), "state view");
        test(std::abs(states[0].linear_velocity.buf[0] - 1. / 360) < 1e-6, "F=ma");
        const SimWrench *w;
        check(simulation_runtime_wrenches(runtime.p, &w, &count), "wrench view");
        test(std::abs(w[0].torque.buf[2] + 2) < 1e-6, "off-center torque");
        test(states[0].angular_velocity.buf[2] < 0, "torque integrated");
        if (backend.api.capabilities & SIM_CAP_NATIVE_HYDRO) {
            Scene native;
            check(example_scene(&native.m, 0, e, sizeof e), e);
            native.m.settings.native_hydrodynamics = 1;
            native.m.settings.gravity = v(0, 0, 9.81);
            check(simulation_finalize(&native.m, e, sizeof e), e);
            Runtime instance;
            check(
                simulation_runtime_create(&native.m, &backend, nullptr, &instance.p, e, sizeof e), e
            );
            check(simulation_runtime_step(instance.p, nullptr, 0, e, sizeof e), e);
        }
        std::cout << backend.api.name
                  << " C ABI v5, compound, six/eight motors, forces, reset, "
                     "observations, and concurrent worlds passed hash="
                  << c << '\n';
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
