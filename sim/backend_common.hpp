#pragma once
#include "math.hpp"
#include <cstdio>
#include <exception>
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
template <class Range, class T>
SimResult copy_out(const Range &v, T *out, uint32_t capacity, uint32_t *count) {
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
