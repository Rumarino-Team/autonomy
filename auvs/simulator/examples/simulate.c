#include "scene.h"
#include <stdio.h>

int main(int argc, char **argv) {
    if (argc != 2) {
        fprintf(stderr, "usage: %s /path/to/backend.so\n", argv[0]);
        return 2;
    }
    char error[512] = {0};
    Simulation model = {0};
    PhysicsBackend backend = {0};
    SimulationRuntime *runtime = NULL;
    int result = 1;
    if (example_scene(&model, 6, error, sizeof error) != SIM_OK)
        goto done;
    if (simulation_finalize(&model, error, sizeof error) != SIM_OK)
        goto done;
    if (physics_backend_open(&backend, argv[1], error, sizeof error) != SIM_OK)
        goto done;
    if (simulation_runtime_create(&model, &backend, NULL, &runtime, error, sizeof error) != SIM_OK)
        goto done;
    double commands[6] = {.1, .1, .1, .1, .1, .1};
    for (unsigned i = 0; i < 360; ++i)
        if (simulation_runtime_step(runtime, commands, 6, error, sizeof error) != SIM_OK)
            goto done;
    const SimBodyState *states;
    uint32_t count;
    if (simulation_runtime_states(runtime, &states, &count) != SIM_OK || count != 1)
        goto done;
    printf(
        "%s tick=%llu position=(%.9g, %.9g, %.9g) hash=%llu\n",
        backend.api.name,
        (unsigned long long)simulation_runtime_tick(runtime),
        states[0].pose.pos.buf[0],
        states[0].pose.pos.buf[1],
        states[0].pose.pos.buf[2],
        (unsigned long long)simulation_runtime_hash(runtime)
    );
    result = 0;
done:
    if (result)
        fprintf(stderr, "%s\n", error);
    simulation_runtime_destroy(runtime);
    physics_backend_close(&backend);
    simulation_destroy(&model);
    return result;
}
