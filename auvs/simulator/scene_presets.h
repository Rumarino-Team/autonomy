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
/* AUVDATA1 binary scene snapshot, schema 1 (JSON is rejected). The robot preset supplies the vehicle; its start
 * pose comes from the snapshot body with the same name. Other bodies are
 * scenery. Failure destroys only storage allocated by this call. */
SIM_API SimResult
simulation_scene_load(Simulation *, const char *snapshot_path, const Robot *, char *, uint32_t);
#ifdef __cplusplus
}
#endif
#endif
