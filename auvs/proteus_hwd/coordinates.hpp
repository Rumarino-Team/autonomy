#pragma once

#include <cmath>

// ZED RIGHT_HANDED_Z_UP_X_FWD (X fwd, Y left, Z up) -> autonomy (+X right, +Y fwd, +Z down).
static void zed_world_to_autonomy(float zx, float zy, float zz, float out[3]) {
  out[0] = -zy;
  out[1] = zx;
  out[2] = -zz;
}

static void mat3_mul(const float a[3][3], const float b[3][3], float out[3][3]) {
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      out[row][col] = a[row][0] * b[0][col] + a[row][1] * b[1][col] + a[row][2] * b[2][col];
    }
  }
}

static void quat_to_mat3(float ox, float oy, float oz, float ow, float m[3][3]) {
  const float xx = ox * ox;
  const float yy = oy * oy;
  const float zz = oz * oz;
  const float xy = ox * oy;
  const float xz = ox * oz;
  const float yz = oy * oz;
  const float wx = ow * ox;
  const float wy = ow * oy;
  const float wz = ow * oz;
  m[0][0] = 1.f - 2.f * (yy + zz);
  m[0][1] = 2.f * (xy - wz);
  m[0][2] = 2.f * (xz + wy);
  m[1][0] = 2.f * (xy + wz);
  m[1][1] = 1.f - 2.f * (xx + zz);
  m[1][2] = 2.f * (yz - wx);
  m[2][0] = 2.f * (xz - wy);
  m[2][1] = 2.f * (yz + wx);
  m[2][2] = 1.f - 2.f * (xx + yy);
}

static void mat3_to_quat(const float m[3][3], float q[4]) {
  float t = m[0][0] + m[1][1] + m[2][2];
  if (t > 0.f) {
    t = std::sqrt(t + 1.f);
    q[3] = 0.5f * t;
    t = 0.5f / t;
    q[0] = (m[2][1] - m[1][2]) * t;
    q[1] = (m[0][2] - m[2][0]) * t;
    q[2] = (m[1][0] - m[0][1]) * t;
    return;
  }
  if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
    t = std::sqrt(1.f + m[0][0] - m[1][1] - m[2][2]);
    q[0] = 0.5f * t;
    t = 0.5f / t;
    q[1] = (m[0][1] + m[1][0]) * t;
    q[2] = (m[0][2] + m[2][0]) * t;
    q[3] = (m[2][1] - m[1][2]) * t;
    return;
  }
  if (m[1][1] > m[2][2]) {
    t = std::sqrt(1.f + m[1][1] - m[0][0] - m[2][2]);
    q[1] = 0.5f * t;
    t = 0.5f / t;
    q[0] = (m[0][1] + m[1][0]) * t;
    q[2] = (m[1][2] + m[2][1]) * t;
    q[3] = (m[0][2] - m[2][0]) * t;
    return;
  }
  t = std::sqrt(1.f + m[2][2] - m[0][0] - m[1][1]);
  q[2] = 0.5f * t;
  t = 0.5f / t;
  q[0] = (m[0][2] + m[2][0]) * t;
  q[1] = (m[1][2] + m[2][1]) * t;
  q[3] = (m[1][0] - m[0][1]) * t;
}

static void zed_quat_to_autonomy(float ox, float oy, float oz, float ow, float out[4]) {
  static const float kConv[3][3] = {
      {0.f, -1.f, 0.f},
      {1.f, 0.f, 0.f},
      {0.f, 0.f, -1.f},
  };
  static const float kConvT[3][3] = {
      {0.f, 1.f, 0.f},
      {-1.f, 0.f, 0.f},
      {0.f, 0.f, -1.f},
  };
  float rz[3][3];
  quat_to_mat3(ox, oy, oz, ow, rz);
  float temp[3][3];
  float ra[3][3];
  mat3_mul(kConv, rz, temp);
  mat3_mul(temp, kConvT, ra);
  mat3_to_quat(ra, out);
  const float n = std::sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2] + out[3] * out[3]);
  if (n > 1e-6f) {
    out[0] /= n;
    out[1] /= n;
    out[2] /= n;
    out[3] /= n;
  }
}

