#ifndef AUV_H
#define AUV_H

#include <stdalign.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// for simd vector sizes
#define AUV_VECTOR3F_ALIGN 16
#define AUV_QUATERNIONF_ALIGN 16
#define AUV_VECTOR6F_ALIGN 32

typedef struct {
  alignas(AUV_VECTOR3F_ALIGN) float buf[3];
} AuvVector3f;

typedef struct {
  alignas(AUV_QUATERNIONF_ALIGN) float buf[4];
} AuvQuaternionf;

typedef struct {
  alignas(AUV_VECTOR6F_ALIGN) float buf[6];
} AuvVector6f;

typedef struct {
  AuvVector3f pos;
  AuvQuaternionf quat;
} AuvPose;

typedef uint8_t AuvObjectCls;
typedef struct {
  AuvPose pose;
  AuvVector3f bounding_box[8];
  uint32_t id;
  AuvObjectCls cls;
} AuvObject;

#define AUV_FRAME_MAX_OBJECTS 128
typedef struct {
  AuvObject objects[AUV_FRAME_MAX_OBJECTS];
  uint8_t objects_len;
  AuvPose camera_pose;
  uint64_t timestamp;
} AuvFrame;

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

#endif

#ifdef __cplusplus
}
#endif

#endif
