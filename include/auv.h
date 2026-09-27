#ifndef AUV_H
#define AUV_H

#include <stdbool.h>
#include <stdint.h>
#include "math.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t AuvObjectCls;
typedef struct {
  MathBoundingBox bbox;
  uint32_t id;
  AuvObjectCls cls;
} AuvObject;

#define AUV_FRAME_MAX_OBJECTS 128
typedef struct {
  AuvObject objects[AUV_FRAME_MAX_OBJECTS];
  uint8_t objects_len;
  MathPose camera_pose;
  uint64_t timestamp;
  bool tracking_ok;
} AuvFrame;



//Reactives Additions
// 
//  We use only two points for the bounding box
//  becasue we assumed the output is always an aligned box

typedef struct {
  uint32_t x, y;
} AuvPoint2u;


typedef struct {
  AuvPoint2u top_left;      /* ZED bounding_box_2d[0] */
  AuvPoint2u bottom_right;  
  uint32_t id;
  AuvObjectCls cls;
} Object2DYolo;

typedef struct {
  MathVector3f accel;
  MathVector3f gyro;
  MathQuaternionf quat;
  Object2DYolo objects2d[AUV_FRAME_MAX_OBJECTS];
  uint8_t object_len;
  uint64_t timestamp;
} AuvReactiveFrame;
void auv_yield_until_reactive_frame(AuvReactiveFrame *frame);
// Reactive Additions
// 






void auv_init(void);
void auv_yield_until_next_frame(AuvFrame *frame);
void auv_set_thrustor_values(const float *thrustor_values, uint8_t thrustor_values_len);
void auv_deinit(void);

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
