#ifndef SIM_SCENE_PRESETS_H
#define SIM_SCENE_PRESETS_H
#include "robots.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Zero-initialized stable root. Creates/finalizes robot_only or robosub_course.
 * Failure destroys only storage allocated by this call. */
SIM_API SimResult
simulation_scene_create(Simulation *, const char *scene, const Robot *, char *, uint32_t);
#ifdef __cplusplus
}
#endif
#endif
