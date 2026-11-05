#pragma once
#include "sim_backend.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
namespace sim {
inline SimVec3 operator+(SimVec3 a, SimVec3 b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
inline SimVec3 operator-(SimVec3 a, SimVec3 b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
inline SimVec3 operator-(SimVec3 a) { return {-a.x, -a.y, -a.z}; }
inline SimVec3 operator*(SimVec3 a, double s) {
  return {a.x * s, a.y * s, a.z * s};
}
inline SimVec3 operator/(SimVec3 a, double s) { return a * (1 / s); }
inline double dot(SimVec3 a, SimVec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline SimVec3 cross(SimVec3 a, SimVec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double norm(SimVec3 a) { return std::sqrt(dot(a, a)); }
inline bool finite(SimVec3 a) {
  return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z);
}
inline bool finite(SimQuat q) {
  return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
         std::isfinite(q.w);
}
inline double norm2(SimQuat q) {
  return q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
}
inline SimQuat conjugate(SimQuat q) { return {-q.x, -q.y, -q.z, q.w}; }
inline SimQuat multiply(SimQuat a, SimQuat b) {
  return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
          a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
inline SimVec3 rotate(SimQuat q, SimVec3 v) {
  SimVec3 u{q.x, q.y, q.z};
  return v + cross(u, cross(u, v) + v * q.w) * 2;
}
inline SimVec3 point(SimPose p, SimVec3 v) {
  return p.position + rotate(p.orientation, v);
}
inline SimPose compose(SimPose a, SimPose b) {
  return {point(a, b.position), multiply(a.orientation, b.orientation)};
}
inline SimQuat euler(double x, double y, double z) {
  return multiply(multiply({0, 0, std::sin(z / 2), std::cos(z / 2)},
                           {0, std::sin(y / 2), 0, std::cos(y / 2)}),
                  {std::sin(x / 2), 0, 0, std::cos(x / 2)});
}
inline uint64_t mix64(uint64_t x) {
  x += 0x9e3779b97f4a7c15ULL;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
  return x ^ (x >> 31);
}
inline double random(uint64_t seed, uint64_t channel, uint64_t tick = 0) {
  return double(mix64(seed + channel * 0x94d049bb133111ebULL +
                      tick * 0xd1b54a32d192ed03ULL) >>
                11) *
         0x1p-53;
}
struct Hash {
  uint64_t value = 14695981039346656037ULL;
  void bytes(const void *p, size_t n) {
    auto b = static_cast<const unsigned char *>(p);
    for (size_t i = 0; i < n; ++i) {
      value ^= b[i];
      value *= 1099511628211ULL;
    }
  }
  void scalar(uint64_t v) {
    for (int i = 0; i < 8; ++i) {
      unsigned char b = static_cast<unsigned char>(v >> (8 * i));
      bytes(&b, 1);
    }
  }
  void scalar(double v) {
    uint64_t u;
    static_assert(sizeof(u) == sizeof(v));
    __builtin_memcpy(&u, &v, 8);
    scalar(u);
  }
  void vector(SimVec3 v) {
    scalar(v.x);
    scalar(v.y);
    scalar(v.z);
  }
  void quat(SimQuat q) {
    scalar(q.x);
    scalar(q.y);
    scalar(q.z);
    scalar(q.w);
  }
  void state(const SimBodyState &s) {
    scalar(uint64_t(s.id));
    vector(s.pose.position);
    quat(s.pose.orientation);
    vector(s.center_of_mass);
    vector(s.linear_velocity);
    vector(s.angular_velocity);
  }
};
} // namespace sim
