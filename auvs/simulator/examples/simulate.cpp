#include "sim.h"
#include <cstdio>
#include <memory>

struct Scene {
    Simulation model{};

    ~Scene() {
        simulation_destroy(&model);
    }

    Scene() = default;
    Scene(const Scene &) = delete;
};

int main(int argc, char **argv) {
    if (argc != 2)
        return 2;
    Scene scene;
    PhysicsBackend backend{};
    char error[512]{};
    if (simulation_init(&scene.model, nullptr, error, sizeof error) != SIM_OK)
        return 1;
    scene.model.settings.gravity = {};
    UnderwaterBody body;
    underwater_body_default(&body);
    body.id = 7;
    body.motion_type = SIM_DYNAMIC;
    body.mass = body.dry_mass = 2;
    Collider collider;
    collider_default(&collider);
    ObjectBlock *object = nullptr;
    if (simulation_add_object(&scene.model, &body, &collider, 1, 0, &object, error, sizeof error) !=
        SIM_OK)
        return 1;
    if (simulation_finalize(&scene.model, error, sizeof error) != SIM_OK)
        return 1;
    if (physics_backend_open(&backend, argv[1], error, sizeof error) != SIM_OK)
        return 1;
    SimulationRuntime *raw = nullptr;
    auto result =
        simulation_runtime_create(&scene.model, &backend, nullptr, &raw, error, sizeof error);
    if (result == SIM_OK) {
        std::unique_ptr<SimulationRuntime, decltype(&simulation_runtime_destroy)> runtime(
            raw, simulation_runtime_destroy
        );
        result = simulation_runtime_step(runtime.get(), nullptr, 0, error, sizeof error);
    }
    physics_backend_close(&backend);
    if (result != SIM_OK)
        std::fprintf(stderr, "%s\n", error);
    return result == SIM_OK ? 0 : 1;
}
