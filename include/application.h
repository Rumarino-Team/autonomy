#ifndef AUTONOMY_APPLICATION_H
#define AUTONOMY_APPLICATION_H
#include "auv.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define AUTONOMY_APPLICATION_ABI 1u
/* Every string/view is borrowed. Host-owned services and callbacks must outlive
 * the application. No host callback may point into an application generation.
 */
typedef struct AutonomyAuvCallbacks {
  void (*init)(void);
  void (*yield_frame)(AuvFrame *);
  void (*commands)(const float *, uint8_t);
  void (*deinit)(void);
  bool (*status)(AuvSimulationStatus *);
} AutonomyAuvCallbacks;
typedef struct AutonomyLaunch {
  const char *plugin, *backend, *robot, *scene, *telemetry_address;
  uint64_t frame_ticks, max_ticks;
  uint32_t dt_numerator, dt_denominator, hydro_period;
  const char *result_path;
  uint16_t blender_port;
  uint8_t blender_enabled;
  uint16_t teleop_port;
  uint8_t realtime, telemetry, fuzz, teleop_enabled;
} AutonomyLaunch;
typedef struct AutonomyHostServices {
  uint32_t struct_size, abi_version, argc;
  const char *const *argv;
  bool (*cancelled)(void);
  void (*telemetry)(const uint8_t *, size_t);
  bool (*teleop)(float *);
  const AutonomyAuvCallbacks *auv;
  const char *plugin_path, *backend_path, *executable_path, *application_path;
} AutonomyHostServices;
typedef struct AutonomyApplicationApi {
  uint32_t struct_size, abi_version;
  int32_t (*create)(const AutonomyHostServices *, void **, char *, size_t);
  void (*destroy)(void *);
  void (*describe)(void *, AutonomyLaunch *);
  int32_t (*build_scene)(void *, void *stable_simulation_root, char *, size_t);
  uint32_t (*flags)(void *); /* watch=1, preserve=2, campaign=4 */
  int32_t (*update)(void *, char *,
                    size_t); /* 0=running,1=done,2=stopped,-1=error */
  int32_t (*capture)(void *, uint8_t *, size_t, size_t *, char *, size_t);
  int32_t (*restore)(void *, const uint8_t *, size_t, char *, size_t);
  void (*activate)(void *);
} AutonomyApplicationApi;
/* Creation/restoration prepare an inactive instance without native side
 * effects. Capture with NULL obtains required capacity. Destroy uses the
 * creating DLL. Activate only after host resource setup/commit, then call
 * update per frame. No application code or worker may be active when the DLL is
 * unloaded. */
const AutonomyApplicationApi *autonomy_application_api(uint32_t requested_abi);
#ifdef __cplusplus
}
#endif
#endif
