#ifndef AUV_SIMULATOR_H
#define AUV_SIMULATOR_H
#include "sim.h"
#include "auv.h"
#ifdef __cplusplus
extern "C" {
#endif
#define AUV_SIMULATOR_EXTENSION_VERSION 1u

/* All calls operate on the calling OS thread's world. The model is borrowed
 * until auv_deinit; option strings are borrowed only during initialization.
 * An initialized model must never move. Failures retain no model reference. */
typedef struct AuvSimulatorOptions {
    uint32_t struct_size, version;
    const char *backend_path;
    uint64_t max_ticks, frame_ticks;
    uint32_t realtime, print_summary;
} AuvSimulatorOptions;

typedef struct AuvSimulatorResult {
    uint32_t struct_size, version, status, has_final_state;
    uint64_t tick, hash;
    double min_depth, max_depth;
    SimBodyState robot;
    char message[256];
} AuvSimulatorResult;

SIM_API SimResult
auv_simulator_init_scene(const Simulation *, const AuvSimulatorOptions *, char *, uint32_t);
/* Copy before auv_deinit, including after a step/command error. */
SIM_API SimResult auv_simulator_read_result(AuvSimulatorResult *, uint32_t size, char *, uint32_t);
#ifdef __cplusplus
}
#endif
#endif
