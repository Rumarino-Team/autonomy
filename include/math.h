#ifndef MATH_H
#define MATH_H

#include <stdalign.h>

// for simd vector sizes
#define MATH_VECTOR3F_ALIGN 16
#define MATH_QUATERNIONF_ALIGN 16
#define MATH_VECTOR6F_ALIGN 32

// Explicit type alignment also survives Zig 0.16's MinGW C translation,
// which otherwise clamps an aligned member under the CRT's default packing.
#if defined(__GNUC__) || defined(__clang__)
#define MATH_TYPE_ALIGN(n) __attribute__((aligned(n)))
#elif defined(_MSC_VER)
#define MATH_TYPE_ALIGN(n) __declspec(align(n))
#else
#define MATH_TYPE_ALIGN(n)
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct MATH_TYPE_ALIGN(MATH_VECTOR3F_ALIGN) {
  alignas(MATH_VECTOR3F_ALIGN) float buf[3];
} MathVector3f;

typedef struct MATH_TYPE_ALIGN(MATH_QUATERNIONF_ALIGN) {
  alignas(MATH_QUATERNIONF_ALIGN) float buf[4];
} MathQuaternionf;

typedef struct MATH_TYPE_ALIGN(MATH_VECTOR6F_ALIGN) {
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
