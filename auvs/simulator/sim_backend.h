#ifndef AUTONOMY_PHYSICS_BACKEND_H
#define AUTONOMY_PHYSICS_BACKEND_H

#include "simulation_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PHYSICS_BACKEND_ABI_VERSION 5u

typedef struct PhysicsWorld PhysicsWorld;

typedef struct PhysicsBackendApi {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t capabilities;
    uint32_t integrators;

    char name[32];
    char version[64];

    /* Borrow the frozen model until world_destroy. Failure sets *out to NULL. */
    SimResult (*world_create)(const Simulation *, PhysicsWorld **out, char *, uint32_t);
    void (*world_destroy)(PhysicsWorld *);

    /* NULL/0 restores initial state, including solver history. */
    SimResult (*reset)(PhysicsWorld *, const SimBodyState *, uint32_t, char *, uint32_t);

    /* World-frame forces/torques at COM. Input borrowed for this call only. */
    SimResult (*step)(PhysicsWorld *, const SimWrench *, uint32_t, char *, uint32_t);

    /* Always report required count. NULL/0 queries capacity. */
    SimResult (*read_state)(const PhysicsWorld *, SimBodyState *, uint32_t, uint32_t *);
    SimResult (*read_contacts)(const PhysicsWorld *, SimContact *, uint32_t, uint32_t *);

    uint64_t (*state_hash)(const PhysicsWorld *);
} PhysicsBackendApi;

typedef SimResult (*PhysicsBackendGetApi)(uint32_t, uint32_t, PhysicsBackendApi *);

SIM_API SimResult sim_backend_get_api(uint32_t version, uint32_t size, PhysicsBackendApi *);

#ifdef __cplusplus
}
#endif

#endif
