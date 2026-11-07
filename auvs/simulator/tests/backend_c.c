#include "sim.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define CHECK(x)                                                                                   \
    do {                                                                                           \
        if (!(x)) {                                                                                \
            fprintf(stderr, "%s: %s\n", #x, error);                                                \
            goto done;                                                                             \
        }                                                                                          \
    } while (0)

int main(int argc, char **argv) {
    if (argc != 2)
        return 2;
    char error[512] = {0};
    int result = 1;
    Simulation model = {0};
    PhysicsBackend backend = {0};
    PhysicsWorld *world = NULL;
    CHECK(simulation_init(&model, NULL, error, sizeof error) == SIM_OK);
    memset(&model.settings.gravity, 0, sizeof model.settings.gravity);
    MathVector3f positions[] = {{{0, 0, 0}}, {{1, 0, 0}}, {{0, 1, 0}}, {{0, 0, 1}}};
    SimTriangle triangles[] = {{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}};
    Collider c;
    collider_default(&c);
    c.data.kind = SIM_MESH;
    c.data.collisions = 0;
    c.geometry = (MeshData){positions, 4, triangles, 4};
    UnderwaterBody body;
    underwater_body_default(&body);
    body.id = 10;
    body.motion_type = SIM_DYNAMIC;
    body.mass = body.dry_mass = 2;
    ObjectBlock *a = NULL;
    ObjectBlock *b = NULL;
    CHECK(simulation_add_object(&model, &body, &c, 1, 0, &a, error, sizeof error) == SIM_OK);
    body.id = 20;
    body.initial_pose.pos.buf[0] = 4;
    CHECK(simulation_add_object(&model, &body, &c, 1, 0, &b, error, sizeof error) == SIM_OK);
    CHECK(a->colliders[0].geometry.positions == b->colliders[0].geometry.positions);
    CHECK(simulation_finalize(&model, error, sizeof error) == SIM_OK);
    CHECK(physics_backend_open(&backend, argv[1], error, sizeof error) == SIM_OK);
    CHECK(backend.api.world_create(&model, &world, error, sizeof error) == SIM_OK);
    uint32_t count = 0;
    CHECK(backend.api.read_state(world, NULL, 0, &count) == SIM_BUFFER_TOO_SMALL && count == 2);
    SimWrench w = {0};
    w.id = 10;
    w.force.buf[0] = 2;
    CHECK(backend.api.step(world, &w, 1, error, sizeof error) == SIM_OK);
    SimBodyState states[2];
    CHECK(
        backend.api.read_state(world, states, 2, &count) == SIM_OK && count == 2 &&
        states[0].id == 10 && states[1].id == 20
    );
    CHECK(fabs(states[0].linear_velocity.buf[0] - 1. / 360) < 1e-6);
    CHECK(states[1].pose.pos.buf[0] == 4);
    w.force.buf[0] = NAN;
    CHECK(backend.api.step(world, &w, 1, error, sizeof error) == SIM_INVALID);
    w.force.buf[0] = 0;
    w.id = 999;
    CHECK(backend.api.step(world, &w, 1, error, sizeof error) == SIM_INVALID);
    CHECK(backend.api.reset(world, NULL, 0, error, sizeof error) == SIM_OK);
    CHECK(backend.api.read_state(world, states, 2, &count) == SIM_OK);
    CHECK(states[0].pose.pos.buf[0] == 0);
    result = 0;
done:
    if (world)
        backend.api.world_destroy(world);
    physics_backend_close(&backend);
    simulation_destroy(&model);
    return result;
}
