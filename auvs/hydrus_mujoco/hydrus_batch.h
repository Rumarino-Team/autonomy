#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Many independent Hydrus vehicles sharing one compiled model, stepped in parallel.
typedef struct HydrusBatch HydrusBatch;

enum {
    HYDRUS_BATCH_THRUSTERS = 8,
    // pos[3], quat xyzw[4], lin_vel[3], ang_vel[3] (world frame).
    HYDRUS_BATCH_INIT_SIZE = 13,
    // dry_mass_scale, volume_scale, drag_scale, thrust_scale, rotor_inertia_scale, current[3].
    HYDRUS_BATCH_RANDOMIZATION_SIZE = 8,
    // pos[3], quat xyzw[4], lin_vel[3], ang_vel[3] (world frame), gyro[3], accel[3], depth,
    // time.
    HYDRUS_BATCH_STATE_SIZE = 21,
    // Low-res camera the planner filters. Same horizontal FOV as camera_spec.
    HYDRUS_CAM_W = 96,
    HYDRUS_CAM_H = 72,
    HYDRUS_CAM_GRID_X = 8,
    HYDRUS_CAM_GRID_Y = 6,
    HYDRUS_CAM_BOXES = 4,
    // class one-hot (cube, rect, gate), cx, cy, width, height. Image center is the origin.
    HYDRUS_CAM_BOX_STRIDE = 7,
    HYDRUS_CAM_EDGES = HYDRUS_CAM_GRID_X * HYDRUS_CAM_GRID_Y,
    HYDRUS_CAM_FLOW = HYDRUS_CAM_GRID_X * HYDRUS_CAM_GRID_Y * 2,
    // boxes, then mean Sobel magnitude per cell, then optical flow (du, dv) per cell.
    HYDRUS_CAM_FEATURE_SIZE =
        HYDRUS_CAM_BOXES * HYDRUS_CAM_BOX_STRIDE + HYDRUS_CAM_EDGES + HYDRUS_CAM_FLOW,
    // xyz plus quaternion xyzw, in mocap-id order (gate, then marker).
    HYDRUS_BATCH_MOCAP_POSE = 7,
};

// num_threads <= 0 picks the hardware concurrency. Returns NULL on failure and prints why.
HydrusBatch* hydrus_batch_create(const char* xml_path, int num_envs, int num_threads);
void hydrus_batch_destroy(HydrusBatch* batch);
int hydrus_batch_num_envs(const HydrusBatch* batch);
double hydrus_batch_timestep(const HydrusBatch* batch);

// Resets every env whose mask entry is nonzero (all envs when mask is NULL). init_state
// is num_envs * HYDRUS_BATCH_INIT_SIZE; NULL keeps the pose in the XML. randomization is
// num_envs * HYDRUS_BATCH_RANDOMIZATION_SIZE; NULL restores the nominal physics.
void hydrus_batch_reset(
    HydrusBatch* batch,
    const uint8_t* mask,
    const double* init_state,
    const double* randomization);

// Holds thrusters[num_envs * 8] for substeps physics steps, then writes state_out
// (num_envs * HYDRUS_BATCH_STATE_SIZE, may be NULL).
void hydrus_batch_step(HydrusBatch* batch, const float* thrusters, int substeps, double* state_out);

void hydrus_batch_get_state(const HydrusBatch* batch, double* state_out);

int hydrus_batch_mocap_count(const HydrusBatch* batch);
// poses is num_envs * mocap_count * HYDRUS_BATCH_MOCAP_POSE. Forward kinematics run after.
void hydrus_batch_set_mocap(HydrusBatch* batch, const double* poses);

// Rasterizes the camera, projects gate and marker boxes, and pools Sobel and optical flow.
// features is num_envs * HYDRUS_CAM_FEATURE_SIZE. The first call after a reset has zero flow.
void hydrus_batch_camera(HydrusBatch* batch, float* features);
// Last raster of one env, row-major, HYDRUS_CAM_W * HYDRUS_CAM_H bytes. NULL if it has not rendered.
void hydrus_batch_camera_image(const HydrusBatch* batch, int env, uint8_t* pixels);

#ifdef __cplusplus
}
#endif
