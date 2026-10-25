#ifndef AUV_H
#define AUV_H

#include <stdbool.h>
#include <stdint.h>
#include "math.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t AuvObjectCls;

typedef uint8_t AuvError;
enum {
  AUV_ERROR_NONE = 0,
  AUV_ERROR_SCENARIO_RESTART = 1,
  AUV_ERROR_SEED_GOAL = 2,
  AUV_ERROR_SURFACED = 3,
  AUV_ERROR_NON_FINITE = 4,
};

typedef struct {
  uint32_t x;
  uint32_t y;
} AuvPoint2u;

typedef struct {
  AuvPoint2u top_left;
  AuvPoint2u bottom_right;
  uint32_t id;
  AuvObjectCls cls;
} AuvObject2d;

typedef struct {
  MathBoundingBox bbox;
  uint32_t id;
  AuvObjectCls cls;
} AuvObject;

#define AUV_FRAME_MAX_OBJECTS 128
#define AUV_ABI_VERSION 5
// Optional geometry extension; it does not change AuvFrame.
#define AUV_MAX_THRUSTERS 8
typedef struct {
  MathVector3f position;
  MathVector3f direction;
} AuvThrusterMount;
typedef struct {
  MathVector3f center_of_mass;
  AuvThrusterMount mounts[AUV_MAX_THRUSTERS];
  uint8_t len;
} AuvThrusterGeometry;
typedef struct {
  AuvObject objects[AUV_FRAME_MAX_OBJECTS];
  uint8_t objects_len;
  MathPose camera_pose;
  uint64_t timestamp;
  bool tracking_ok;
  MathVector3f accel;
  MathVector3f gyro;
  MathQuaternionf imu_quat;
  AuvObject2d objects2d[AUV_FRAME_MAX_OBJECTS];
  uint8_t objects2d_len;
  uint32_t image_width;
  uint32_t image_height;
  float pressure_depth;
  bool pressure_depth_ok;
  // Sensor timestamps share the frame timestamp's clock. Zero means absent.
  uint64_t position_timestamp;
  uint64_t imu_timestamp;
  uint64_t pressure_timestamp;
  uint64_t observations_timestamp;
  bool imu_ok;
  bool gyro_ok;
  // Increment whenever the localization coordinate frame is reset.
  uint32_t localization_epoch;
  AuvError error;
} AuvFrame;

uint32_t auv_abi_version(void);
void auv_init(void);
void auv_yield_until_next_frame(AuvFrame *frame);
void auv_set_thrustor_values(const float *thrustor_values, uint8_t thrustor_values_len);
void auv_deinit(void);
// Optional export. Geometry is in the same body frame as camera_pose, ordered
// exactly like auv_set_thrustor_values. Direction denotes positive physical
// thrust, not positive motor command. Query after auv_init.
bool auv_get_thruster_geometry(AuvThrusterGeometry *geometry);
// Optional export. Call once, before auv_init, to run num_envs headless worlds.
// Each later auv_init binds the calling thread to one world. It does not change
// AuvFrame or ABI version 3. Plugins that cannot do this omit the symbol.
bool auv_prepare_batch(int num_envs);
// Optional export. Call after auv_init, on the thread bound to a world. Draws the
// fuzz case for this seed from the loaded model, resets that world, and applies
// the case on later frames. A world without this call stays at the loaded pose.
bool auv_begin_fuzz(uint64_t seed);
// JSON lines: a header, one sample per 1/fps of simulator time, events, then an
// end line. Sample steps count physics steps since this call.
bool auv_record_start(const char* path, double fps);
void auv_record_event(const char* text);
void auv_record_stop(const char* reason);

#ifdef __cplusplus

using AuvInitFunc = decltype(auv_init);
using AuvYieldUntilNextFrameFunc = decltype(auv_yield_until_next_frame);
using AuvSetThrustorsInputFunc = decltype(auv_set_thrustor_values);
using AuvDeinitFunc = decltype(auv_deinit);

#else

typedef typeof(auv_init) AuvInitFunc;
typedef typeof(auv_yield_until_next_frame) AuvYieldUntilNextFrameFunc;
typedef typeof(auv_set_thrustor_values) AuvSetThrustorsInputFunc;
typedef typeof(auv_deinit) AuvDeinitFunc;

#endif // __cplusplus

#ifdef __cplusplus
}
#endif

#endif // AUV_H
