#include "../examples/scene.h"
#include <stdio.h>

int main(int argc, char **argv) {
    if (argc != 2)
        return 2;
    Simulation model = {0};
    PhysicsBackend backend = {0};
    SimulationRuntime *runtime = NULL;
    char e[512] = {0};
    int result = 1;
    if (example_scene(&model, 6, e, sizeof e) != SIM_OK ||
        simulation_finalize(&model, e, sizeof e) != SIM_OK ||
        physics_backend_open(&backend, argv[1], e, sizeof e) != SIM_OK ||
        simulation_runtime_create(&model, &backend, NULL, &runtime, e, sizeof e) != SIM_OK)
        goto done;
    for (unsigned i = 0; i < 360; ++i) {
        double commands[6];
        for (unsigned j = 0; j < 6; ++j)
            commands[j] = i < 180 ? .1 : -.1;
        if (simulation_runtime_step(runtime, commands, 6, e, sizeof e) != SIM_OK)
            goto done;
        if (i % 36 == 35) {
            const SimBodyState *s;
            const RotorState *r;
            uint32_t n;
            Observation o;
            simulation_runtime_states(runtime, &s, &n);
            simulation_runtime_rotors(runtime, &r, &n);
            if (simulation_runtime_observe(runtime, &o, e, sizeof e) != SIM_OK)
                goto done;
            printf("%u", i + 1);
            for (unsigned k = 0; k < 3; ++k)
                printf(" %.17g", s[0].pose.pos.buf[k]);
            for (unsigned k = 0; k < 4; ++k)
                printf(" %.17g", s[0].pose.quat.buf[k]);
            for (unsigned k = 0; k < 3; ++k)
                printf(" %.17g", s[0].linear_velocity.buf[k]);
            for (unsigned k = 0; k < 3; ++k)
                printf(" %.17g", s[0].angular_velocity.buf[k]);
            printf(
                " %.17g %.17g %.17g %.17g %.17g\n",
                r[0].omega,
                r[0].integral,
                r[0].torque,
                r[0].thrust,
                o.depth
            );
        }
    }
    result = 0;
done:
    if (result)
        fprintf(stderr, "%s\n", e);
    simulation_runtime_destroy(runtime);
    physics_backend_close(&backend);
    simulation_destroy(&model);
    return result;
}
