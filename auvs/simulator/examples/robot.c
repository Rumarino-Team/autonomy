#include "robots.h"
#include <stdio.h>

int main(int argc, char **argv) {
    if (argc < 2 || argc > 3) {
        fprintf(
            stderr, "usage: %s /path/to/backend.so [hydrus|proteus|bluerov|girona500]\n", argv[0]
        );
        return 2;
    }
    const Robot *definition = sim_robot_find(argc == 3 ? argv[2] : "hydrus");
    if (!definition) {
        fprintf(stderr, "unknown robot: %s\n", argv[2]);
        return 2;
    }
    Simulation model = {0};
    SimulationConfig config;
    PhysicsBackend backend = {0};
    SimulationRuntime *runtime = NULL;
    char error[512] = {0};
    int result = 1;
    if (simulation_robot_config(definition, &config, error, sizeof error) != SIM_OK)
        goto done;
    if (simulation_init(&model, &config, error, sizeof error) != SIM_OK)
        goto done;
    /* Use the saved standalone asset pose (NED depth 2 m) and external ID 1.
     * Add scenery, fault schedules, or module selections before finalizing. */
    if (simulation_add_robot(&model, definition, 1, NULL, error, sizeof error) != SIM_OK)
        goto done;
    if (simulation_finalize(&model, error, sizeof error) != SIM_OK)
        goto done;
    if (physics_backend_open(&backend, argv[1], error, sizeof error) != SIM_OK)
        goto done;
    if (simulation_runtime_create(&model, &backend, NULL, &runtime, error, sizeof error) != SIM_OK)
        goto done;
    double commands[SIM_MAX_THRUSTERS] = {0};
    for (uint32_t i = 0; i < model.robot.thruster_count; ++i)
        commands[i] = .05;
    for (uint32_t tick = 0; tick < 360; ++tick)
        if (simulation_runtime_step(
                runtime, commands, model.robot.thruster_count, error, sizeof error
            ) != SIM_OK)
            goto done;
    const SimBodyState *states;
    uint32_t count;
    if (simulation_runtime_states(runtime, &states, &count) != SIM_OK || count != 1)
        goto done;
    printf(
        "%s / %s: %u colliders, %u thrusters, %u hulls; depth=%.6g m; hash=%llu\n",
        model.robot.name,
        backend.api.name,
        model.robot.object.collider_count,
        model.robot.thruster_count,
        model.robot.hull_count,
        states[0].pose.pos.buf[2],
        (unsigned long long)simulation_runtime_hash(runtime)
    );
    if (simulation_runtime_reset(runtime, NULL, 0, error, sizeof error) != SIM_OK)
        goto done;
    result = 0;
done:
    if (result)
        fprintf(stderr, "%s\n", error);
    simulation_runtime_destroy(runtime);
    physics_backend_close(&backend);
    simulation_destroy(&model);
    return result;
}
