#ifndef AUTONOMY_SIM_H
#define AUTONOMY_SIM_H

#include "sim_backend.h"

#ifdef __cplusplus
extern "C" {
#endif

SIM_API void simulation_config_default(SimulationConfig *);
SIM_API void simulation_settings_default(SimulationSettings *);
SIM_API void underwater_body_default(UnderwaterBody *);
SIM_API void collider_default(Collider *);
SIM_API void rotor_spec_default(RotorSpec *);
SIM_API SimResult sim_list_append(ListBase *, IntrusiveDataNode *);
SIM_API SimResult sim_list_unlink(ListBase *, IntrusiveDataNode *);
SIM_API SimResult
sim_node_cast(const IntrusiveDataNode *, DataKind, const void *owner, const void **out);
/* Roots must be zero initialized, kept at a stable address, and destroyed once.
 */
SIM_API SimResult simulation_init(Simulation *, const SimulationConfig *, char *, uint32_t);
SIM_API void simulation_destroy(Simulation *);
/* Native wrappers use these same per-model allocation hooks for their buffers.
 */
SIM_API SimResult simulation_get_allocator(const Simulation *, SimAllocator *);
SIM_API SimResult simulation_add_object(
    Simulation *,
    const UnderwaterBody *,
    const Collider *,
    uint32_t,
    uint32_t fixed_robot,
    ObjectBlock **out,
    char *,
    uint32_t
);
SIM_API SimResult simulation_add_thruster(Simulation *, const Thruster *, char *, uint32_t);
SIM_API SimResult simulation_add_hull(Simulation *, const HullRec *, char *, uint32_t);
SIM_API void sensor_default(SensorRec *, SensorKind);
SIM_API SimResult
simulation_add_sensor(Simulation *, const SensorRec *, const SensorRec **out, char *, uint32_t);
SIM_API SimResult
simulation_set_sensor_routing(Simulation *, const SensorRouting *, char *, uint32_t);
SIM_API SimResult simulation_add_fault(Simulation *, const FaultBlock *, char *, uint32_t);
SIM_API SimResult simulation_add_module(Simulation *, const char *, char *, uint32_t);
SIM_API SimResult simulation_validate(const Simulation *, char *, uint32_t);
/* Borrowed mutable record, available only before finalization. */
SIM_API SimResult
simulation_draft_object(Simulation *, uint32_t body_id, ObjectBlock **out, char *, uint32_t);
SIM_API SimResult simulation_finalize(Simulation *, char *, uint32_t);
/* Produces a mutable independent draft. Finalize before creating a world. */
SIM_API SimResult simulation_clone(const Simulation *, Simulation *destination, char *, uint32_t);

typedef struct PhysicsBackend {
    void *library;
    PhysicsBackendApi api;
} PhysicsBackend;

SIM_API SimResult physics_backend_open(PhysicsBackend *, const char *, char *, uint32_t);
SIM_API void physics_backend_close(PhysicsBackend *);

typedef struct SimForceContext {
    const Simulation *model;
    const SimBodyState *bodies;
    uint32_t body_count;
    const double *commands;
    uint32_t command_count;
    uint64_t tick;
    double dt;
    RotorState *rotors;
    SimWrench *fluid_cache;
    ArenaAllocator *scratch;
} SimForceContext;

typedef struct ForceModule {
    char name[64];
    char contribution[64];
    void *user;
    SimResult (*create)(const Simulation *, void *user, void **state, char *, uint32_t);
    SimResult (*reset)(void *state, char *, uint32_t);
    SimResult (*evaluate)(
        const SimForceContext *, void *state, SimWrench *, uint32_t, char *, uint32_t
    );
    void (*destroy)(void *state);
} ForceModule;

/* Entries are borrowed only during runtime creation; user data/code must
 * outlive their callbacks. */
typedef struct ForceRegistry {
    const ForceModule *entries;
    uint32_t count;
} ForceRegistry;

SIM_API void
sim_add_force(SimWrench *, MathVector3f force, MathVector3f point, MathVector3f center_of_mass);
typedef struct SimulationRuntime SimulationRuntime;
/* Borrow model and loaded backend until destroy; optional registry extends
 * builtins. */
SIM_API SimResult simulation_runtime_create(
    const Simulation *,
    const PhysicsBackend *,
    const ForceRegistry *,
    SimulationRuntime **,
    char *,
    uint32_t
);
SIM_API void simulation_runtime_destroy(SimulationRuntime *);
SIM_API SimResult
simulation_runtime_reset(SimulationRuntime *, const SimBodyState *, uint32_t, char *, uint32_t);
SIM_API SimResult
simulation_runtime_step(SimulationRuntime *, const double *, uint32_t, char *, uint32_t);
/* Views expire on the next step/reset or destruction; reads never mutate them. */
SIM_API SimResult
simulation_runtime_states(const SimulationRuntime *, const SimBodyState **, uint32_t *);
SIM_API SimResult
simulation_runtime_contacts(const SimulationRuntime *, const SimContact **, uint32_t *);
SIM_API SimResult
simulation_runtime_rotors(const SimulationRuntime *, const RotorState **, uint32_t *);
SIM_API SimResult
simulation_runtime_wrenches(const SimulationRuntime *, const SimWrench **, uint32_t *);
SIM_API SimResult simulation_runtime_observe(SimulationRuntime *, Observation *, char *, uint32_t);
/* Samples and detection views expire on step/reset/destroy. Reading is side-effect free. */
SIM_API SimResult
simulation_runtime_sensor_samples(const SimulationRuntime *, const SensorSample **, uint32_t *);
SIM_API uint64_t simulation_runtime_tick(const SimulationRuntime *);
SIM_API uint64_t simulation_runtime_hash(const SimulationRuntime *);

#ifdef __cplusplus
}
#endif

#endif
