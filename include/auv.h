#ifndef AUV_H
#define AUV_H

#include <stdbool.h>
#include <stdint.h>
#include "math.h"
#include "object_classes.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t AuvObjectCls;

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

/// One flag per axis. False means that component of the pose must not be used.
typedef struct {
  bool x;
  bool y;
  bool z;
} AuvAxisOk;

#define AUV_FRAME_MAX_OBJECTS 128
typedef struct {
  AuvObject objects[AUV_FRAME_MAX_OBJECTS];
  uint8_t objects_len;
  MathPose camera_pose;
  uint64_t timestamp;
  /// World X, Y, Z. A false axis means `camera_pose.pos` on that axis is not usable.
  AuvAxisOk position_ok;
  /// Roll, pitch, yaw. A false axis means that attitude component is not usable.
  AuvAxisOk attitude_ok;
  MathVector3f accel;
  MathVector3f gyro;
  MathQuaternionf imu_quat;
  AuvObject2d objects2d[AUV_FRAME_MAX_OBJECTS];
  uint8_t objects2d_len;
  uint32_t image_width;
  uint32_t image_height;
  float pressure_depth;
  bool pressure_depth_ok;
} AuvFrame;

void auv_init(void);
void auv_yield_until_next_frame(AuvFrame *frame);
void auv_set_thrustor_values(const float *thrustor_values, uint8_t thrustor_values_len);
void auv_deinit(void);

/* Optional simulator export; required functions and AuvFrame remain unchanged.
 * code: 0 running, 1 tick budget exhausted, 2 initialization/command/step error. */
typedef struct AuvSimulationStatus {
  uint32_t code;
  uint64_t tick;
  char message[256];
} AuvSimulationStatus;
bool auv_simulation_status(AuvSimulationStatus *status);

#ifdef __cplusplus

using AuvInitFunc = decltype(auv_init);
using AuvYieldUntilNextFrameFunc = decltype(auv_yield_until_next_frame);
using AuvSetThrustorsInputFunc = decltype(auv_set_thrustor_values);
using AuvDeinitFunc = decltype(auv_deinit);

#else

typedef void AuvInitFunc(void);
typedef void AuvYieldUntilNextFrameFunc(AuvFrame *);
typedef void AuvSetThrustorsInputFunc(const float *, uint8_t);
typedef void AuvDeinitFunc(void);

#endif // __cplusplus

#ifdef __cplusplus
}
#endif

#endif // AUV_H
