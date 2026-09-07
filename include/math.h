#ifndef MATH_H
#define MATH_H

#include <stdalign.h>

// for simd vector sizes
#define MATH_VECTOR3F_ALIGN 16
#define MATH_QUATERNIONF_ALIGN 16
#define MATH_VECTOR6F_ALIGN 32

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  alignas(MATH_VECTOR3F_ALIGN) float buf[3];
} MathVector3f;

typedef struct {
  alignas(MATH_QUATERNIONF_ALIGN) float buf[4];
} MathQuaternionf;

typedef struct {
  alignas(MATH_VECTOR6F_ALIGN) float buf[6];
} MathVector6f;

typedef struct {
  MathVector3f pos;
  MathQuaternionf quat;
} MathPose;

typedef struct {
  MathPose pose;
  MathVector3f size;
} MathBoundingBox;


#ifdef __cplusplus
}
#endif

#endif // MATH_H
