#ifndef AUTONOMY_SIM_ROBOTS_H
#define AUTONOMY_SIM_ROBOTS_H

#include "sim.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Immutable templates, in Hydrus, Proteus, BlueROV, Girona500 order. These use
 * the shared Robot records, but have no database membership. Attachment owners
 * are NULL until simulation_add_robot copies them into a model. Never assign a
 * template directly into Simulation.robot. Geometry has static lifetime. */
SIM_API uint32_t sim_robot_count(void);
SIM_API const Robot *sim_robot_at(uint32_t index);
SIM_API const Robot *sim_robot_find(const char *name);

/* Start with the small default capacities, increasing geometry capacities to
 * fit this preset. Call before simulation_init; enlarge further for scenery.
 * The template must come from sim_robot_at/find. */
SIM_API SimResult
simulation_robot_config(const Robot *, SimulationConfig *, char *error, uint32_t capacity);

/* Add one complete robot to an initialized, mutable model. Geometry and sensor
 * storage are copied into model-owned buffers; attachment references are repaired.
 * Other world objects may already exist. The model must have no robot hardware.
 * NULL pose uses the standalone Blender asset's saved placement; body_id is
 * caller-selected. Leaves the model unchanged on failure. Finalize separately. */
SIM_API SimResult simulation_add_robot(
    Simulation *, const Robot *, uint32_t body_id, const MathPose *, char *error, uint32_t capacity
);

#ifdef __cplusplus
}
#endif

#endif
