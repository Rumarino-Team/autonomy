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

// Double-precision storage uses natural double alignment, without SIMD padding.
// Named components preserve the simulator's C layout and field access. The
// existing float types above retain their SIMD alignment and controller ABI.
typedef struct {
  double x, y, z;
} MathVector3d;

// Quaternion components are xyzw; rotations use unit quaternions.
typedef struct {
  double x, y, z, w;
} MathQuaterniond;

// A spatial vector: linear xyz followed by angular xyz, in the same frame.
// For a wrench these halves hold force and torque respectively.
typedef struct {
  MathVector3d linear;
  MathVector3d angular;
} MathVector6d;

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
