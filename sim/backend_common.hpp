#pragma once
#include "math.hpp"
#include <cstdio>
#include <exception>
#include <set>
#include <vector>
namespace sim {
inline uint64_t file_digest(const char *path) noexcept {
  FILE *file = std::fopen(path, "rb");
  if (!file)
    return 0;
  Hash h;
  unsigned char buffer[16384];
  size_t size;
  while ((size = std::fread(buffer, 1, sizeof(buffer), file)))
    h.bytes(buffer, size);
  bool ok = !std::ferror(file);
  std::fclose(file);
  return ok ? h.value : 0;
}

inline SimResult fail(char *dst, uint32_t n, const char *text,
                      SimResult code = SIM_BACKEND_ERROR) {
  if (dst && n)
    std::snprintf(dst, n, "%s", text);
  return code;
}
template <class F> SimResult guarded(char *error, uint32_t n, F &&f) noexcept {
  try {
    f();
    return SIM_OK;
  } catch (const std::invalid_argument &e) {
    return fail(error, n, e.what(), SIM_INVALID);
  } catch (const std::exception &e) {
    return fail(error, n, e.what());
  } catch (...) {
    return fail(error, n, "unknown backend failure");
  }
}
inline void require(bool b, const char *error) {
  if (!b)
    throw std::invalid_argument(error);
}
inline void valid_pose(SimPose p) {
  require(finite(p.position) && finite(p.orientation) &&
              std::abs(norm2(p.orientation) - 1) < 1e-6,
          "pose must be finite with a unit xyzw quaternion");
}
inline void validate(const SimModelDesc *d) {
  require(d && d->struct_size == sizeof(SimModelDesc),
          "invalid model structure size");
  require(d->integrator <= SIM_INTEGRATOR_RK4, "unknown integrator");
  require(d->dt_numerator && d->dt_denominator &&
              double(d->dt_numerator) / d->dt_denominator <= 0.1,
          "invalid fixed timestep");
  require(finite(d->gravity) && finite(d->water_current) &&
              std::isfinite(d->water_density) && d->water_density >= 0,
          "invalid environment");
  require(d->native_hydrodynamics <= 1, "invalid native provider flag");
  require(d->bodies && d->body_count && d->body_count <= 4096,
          "invalid bodies");
  std::set<uint32_t> ids;
  for (uint32_t i = 0; i < d->body_count; ++i) {
    const auto &b = d->bodies[i];
    require(ids.insert(b.id).second && b.motion <= SIM_KINEMATIC,
            "duplicate body id or invalid motion");
    require(std::isfinite(b.mass) && b.mass >= 0 && finite(b.inertia),
            "invalid mass or inertia");
    if (b.motion == SIM_DYNAMIC)
      require(b.mass > 0 && b.inertia.x > 0 && b.inertia.y > 0 &&
                  b.inertia.z > 0,
              "dynamic body needs positive mass and inertia");
    valid_pose(b.center_of_mass);
    valid_pose(b.initial_pose);
    require(finite(b.initial_linear_velocity) &&
                finite(b.initial_angular_velocity),
            "non-finite initial velocity");
    require(b.shapes && b.shape_count && b.shape_count <= 1024,
            "invalid shapes");
    for (uint32_t j = 0; j < b.shape_count; ++j) {
      const auto &s = b.shapes[j];
      valid_pose(s.pose);
      require(s.kind <= SIM_MESH && s.collisions <= 1 && finite(s.size) &&
                  std::isfinite(s.friction) && s.friction >= 0 &&
                  std::isfinite(s.restitution) && s.restitution >= 0 &&
                  s.restitution <= 1,
              "invalid shape properties");
      require(finite(s.drag) && finite(s.skin) && s.drag.x >= 0 &&
                  s.drag.y >= 0 && s.drag.z >= 0 && s.skin.x >= 0 &&
                  s.skin.y >= 0 && s.skin.z >= 0,
              "invalid drag coefficients");
      if (s.kind == SIM_BOX)
        require(s.size.x > 0 && s.size.y > 0 && s.size.z > 0,
                "invalid box size");
      if (s.kind == SIM_SPHERE)
        require(s.size.x > 0, "invalid sphere radius");
      if (s.kind == SIM_CYLINDER)
        require(s.size.x > 0 && s.size.z > 0, "invalid cylinder size");
      if (s.kind == SIM_MESH) {
        require(s.vertices && s.triangles && s.vertex_count >= 4 &&
                    s.vertex_count <= 1000000 && s.triangle_count &&
                    s.triangle_count <= 2000000,
                "invalid mesh");
        for (uint32_t k = 0; k < s.vertex_count; ++k)
          require(finite(s.vertices[k]), "non-finite mesh vertex");
        for (uint32_t k = 0; k < s.triangle_count; ++k) {
          const auto t = s.triangles[k];
          require(t.a < s.vertex_count && t.b < s.vertex_count &&
                      t.c < s.vertex_count,
                  "mesh index out of range");
        }
      }
    }
  }
}
template <class T>
SimResult copy_out(const std::vector<T> &v, T *out, uint32_t capacity,
                   uint32_t *count) {
  if (!count)
    return SIM_INVALID;
  *count = static_cast<uint32_t>(v.size());
  if (capacity < v.size())
    return SIM_BUFFER_TOO_SMALL;
  if (v.empty())
    return SIM_OK;
  if (!out)
    return SIM_INVALID;
  std::copy(v.begin(), v.end(), out);
  return SIM_OK;
}
} // namespace sim
