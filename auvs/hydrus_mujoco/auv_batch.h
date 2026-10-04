#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Many independent vehicles sharing one compiled model, stepped in parallel.
typedef struct AuvBatch AuvBatch;

enum {
    // pos[3], quat xyzw[4], lin_vel[3], ang_vel[3] (world frame).
    AUV_BATCH_INIT_SIZE = 13,
    // dry_mass_scale, volume_scale, drag_scale, thrust_scale, rotor_inertia_scale, current[3].
    AUV_BATCH_RANDOMIZATION_SIZE = 8,
    // pos[3], quat xyzw[4], lin_vel[3], ang_vel[3] (world frame), gyro[3], accel[3], depth,
    // time.
    AUV_BATCH_STATE_SIZE = 21,
    // Low-res camera the planner filters. Same horizontal FOV as camera_spec.
    AUV_CAM_W = 96,
    AUV_CAM_H = 72,
    AUV_CAM_GRID_X = 16,
    AUV_CAM_GRID_Y = 12,
    AUV_CAM_BOXES = 8,
    // class one-hot (cube, rect, gate), cx, cy, width, height, visibility fraction.
    AUV_CAM_BOX_STRIDE = 8,
    // Four Sobel orientation bins, optical flow (du, dv), and match confidence.
    AUV_CAM_CELL_STRIDE = 7,
    AUV_CAM_CELLS = AUV_CAM_GRID_X * AUV_CAM_GRID_Y,
    // boxes, then one cell record per grid cell.
    AUV_CAM_FEATURE_SIZE =
        AUV_CAM_BOXES * AUV_CAM_BOX_STRIDE + AUV_CAM_CELLS * AUV_CAM_CELL_STRIDE,
    // xyz plus quaternion xyzw, in mocap-id order (gate, then marker).
    AUV_BATCH_MOCAP_POSE = 7,
};

// num_threads <= 0 picks the hardware concurrency. Returns NULL on failure and prints why.
AuvBatch* auv_batch_create(const char* xml_path, int num_envs, int num_threads);
void auv_batch_destroy(AuvBatch* batch);
int auv_batch_num_envs(const AuvBatch* batch);
int auv_batch_num_thrusters(const AuvBatch* batch);
double auv_batch_timestep(const AuvBatch* batch);

// Resets every env whose mask entry is nonzero (all envs when mask is NULL). init_state
// is num_envs * AUV_BATCH_INIT_SIZE; NULL keeps the pose in the XML. randomization is
// num_envs * AUV_BATCH_RANDOMIZATION_SIZE; NULL restores the nominal physics.
void auv_batch_reset(
    AuvBatch* batch,
    const uint8_t* mask,
    const double* init_state,
    const double* randomization);

// Holds thrusters[num_envs * auv_batch_num_thrusters] for substeps physics steps, then
// writes state_out (num_envs * AUV_BATCH_STATE_SIZE, may be NULL).
void auv_batch_step(AuvBatch* batch, const float* thrusters, int substeps, double* state_out);

void auv_batch_get_state(const AuvBatch* batch, double* state_out);

int auv_batch_mocap_count(const AuvBatch* batch);
// poses is num_envs * mocap_count * AUV_BATCH_MOCAP_POSE. Forward kinematics run after.
void auv_batch_set_mocap(AuvBatch* batch, const double* poses);

// Rasterizes the camera, projects gate and marker boxes, and pools Sobel and optical flow.
// features is num_envs * AUV_CAM_FEATURE_SIZE. The first call after a reset has zero flow.
void auv_batch_camera(AuvBatch* batch, float* features);
// Last raster of one env, row-major, AUV_CAM_W * AUV_CAM_H bytes. NULL if it has not rendered.
void auv_batch_camera_image(const AuvBatch* batch, int env, uint8_t* pixels);

#ifdef __cplusplus
}
#endif
