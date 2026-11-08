#pragma once
#include "sim.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <new>
#include <span>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace core {
struct Error : std::runtime_error {
    SimResult code;

    Error(SimResult c, const char *s)
        : std::runtime_error(s),
          code(c) {
    }
};

inline void require(bool b, const char *s, SimResult c = SIM_INVALID) {
    if (!b)
        throw Error(c, s);
}

inline void check(SimResult r, const char *s) {
    if (r != SIM_OK)
        throw Error(r, s && *s ? s : "simulator call failed");
}

inline SimResult fail(char *s, uint32_t n, const char *m, SimResult r) {
    if (s && n)
        std::snprintf(s, n, "%s", m);
    return r;
}

template <class F> SimResult guard(char *s, uint32_t n, F &&f) noexcept {
    if (s && n)
        *s = 0;
    try {
        f();
        return SIM_OK;
    } catch (const Error &e) {
        return fail(s, n, e.what(), e.code);
    } catch (const std::bad_alloc &) {
        return fail(s, n, "allocation failed", SIM_NO_MEMORY);
    } catch (const std::exception &e) {
        return fail(s, n, e.what(), SIM_BACKEND_ERROR);
    } catch (...) {
        return fail(s, n, "unknown backend failure", SIM_BACKEND_ERROR);
    }
}

inline size_t bytes(size_t count, size_t width) {
    require(!width || count <= SIZE_MAX / width, "size overflow", SIM_CAPACITY);
    return count * width;
}

inline void *default_alloc(void *, size_t n, size_t alignment) {
    alignment = std::max(alignment, alignof(void *));
    if (n > SIZE_MAX - alignment - sizeof(void *))
        return nullptr;
#ifdef _WIN32
    // MinGW's libc++ does not provide aligned_alloc. Keep the returned pointer
    // freeable by remembering the original malloc block just before it.
    void *raw = std::malloc(n + alignment + sizeof(void *));
    if (!raw)
        return nullptr;
    auto aligned = reinterpret_cast<uintptr_t>(raw) + sizeof(void *);
    aligned = (aligned + alignment - 1) & ~(static_cast<uintptr_t>(alignment) - 1);
    reinterpret_cast<void **>(aligned)[-1] = raw;
    return reinterpret_cast<void *>(aligned);
#else
    return std::aligned_alloc(alignment, (n + alignment - 1) & ~(alignment - 1));
#endif
}

inline void default_free(void *, void *p, size_t, size_t) {
#ifdef _WIN32
    if (p)
        std::free(reinterpret_cast<void **>(p)[-1]);
#else
    std::free(p);
#endif
}

inline SimAllocator allocator(SimAllocator a = {}) {
    require(bool(a.allocate) == bool(a.deallocate), "allocator callbacks must be paired");
    if (!a.allocate)
        a = {nullptr, default_alloc, default_free};
    return a;
}

/* Reusable, aligned, trivially destructible arrays. Only growth allocates. */
template <class T> struct Buffer {
    static_assert(std::is_trivially_destructible_v<T>);
    SimAllocator alloc{};
    T *p = nullptr;
    size_t n = 0, cap = 0;

    explicit Buffer(SimAllocator a = {})
        : alloc(allocator(a)) {
    }

    ~Buffer() {
        if (p)
            alloc.deallocate(alloc.user, p, bytes(cap, sizeof(T)), alignof(T));
    }

    Buffer(const Buffer &) = delete;
    Buffer &operator=(const Buffer &) = delete;

    void reserve(size_t c) {
        if (c <= cap)
            return;
        auto size = bytes(c, sizeof(T));
        require(size <= SIM_MAX_SCENE_BYTES, "buffer exceeds size limit", SIM_CAPACITY);
        auto *q = static_cast<T *>(alloc.allocate(alloc.user, size, alignof(T)));
        if (!q)
            throw std::bad_alloc();
        if (reinterpret_cast<uintptr_t>(q) % alignof(T)) {
            alloc.deallocate(alloc.user, q, size, alignof(T));
            throw Error(SIM_INVALID, "allocator returned misaligned memory");
        }
        std::uninitialized_value_construct_n(q, c);
        if (n)
            std::copy_n(p, n, q);
        if (p)
            alloc.deallocate(alloc.user, p, bytes(cap, sizeof(T)), alignof(T));
        p = q;
        cap = c;
    }

    void resize(size_t c) {
        if (c > cap)
            reserve(std::max(c, cap ? cap * 2 : size_t(8)));
        if (c > n)
            std::fill(p + n, p + c, T{});
        n = c;
    }

    void push_back(const T &v) {
        resize(n + 1);
        p[n - 1] = v;
    }

    template <class It> void insert(T *position, It first, It last) {
        require(position == end(), "buffer supports append only");
        if (first == last)
            return;
        size_t count = size_t(std::distance(first, last));
        auto old = n;
        resize(n + count);
        std::copy(first, last, p + old);
    }

    void clear() {
        n = 0;
    }

    T *data() {
        return p;
    }

    const T *data() const {
        return p;
    }

    size_t size() const {
        return n;
    }

    bool empty() const {
        return n == 0;
    }

    T &operator[](size_t i) {
        return p[i];
    }

    const T &operator[](size_t i) const {
        return p[i];
    }

    T *begin() {
        return p;
    }

    T *end() {
        return n ? p + n : p;
    }

    const T *begin() const {
        return p;
    }

    const T *end() const {
        return n ? p + n : p;
    }
};

inline MathVector3f v(double x = 0, double y = 0, double z = 0) {
    return {{float(x), float(y), float(z)}};
}

inline MathQuaternionf q(double x = 0, double y = 0, double z = 0, double w = 1) {
    return {{float(x), float(y), float(z), float(w)}};
}

inline MathVector3f add(MathVector3f a, MathVector3f b) {
    return v(a.buf[0] + b.buf[0], a.buf[1] + b.buf[1], a.buf[2] + b.buf[2]);
}

inline MathVector3f sub(MathVector3f a, MathVector3f b) {
    return v(a.buf[0] - b.buf[0], a.buf[1] - b.buf[1], a.buf[2] - b.buf[2]);
}

inline MathVector3f scale(MathVector3f a, double s) {
    return v(a.buf[0] * s, a.buf[1] * s, a.buf[2] * s);
}

inline double dot(MathVector3f a, MathVector3f b) {
    return double(a.buf[0]) * b.buf[0] + double(a.buf[1]) * b.buf[1] + double(a.buf[2]) * b.buf[2];
}

inline double norm(MathVector3f a) {
    return std::sqrt(dot(a, a));
}

inline MathVector3f cross(MathVector3f a, MathVector3f b) {
    return v(
        double(a.buf[1]) * b.buf[2] - double(a.buf[2]) * b.buf[1],
        double(a.buf[2]) * b.buf[0] - double(a.buf[0]) * b.buf[2],
        double(a.buf[0]) * b.buf[1] - double(a.buf[1]) * b.buf[0]
    );
}

inline MathQuaternionf conjugate(MathQuaternionf a) {
    return q(-a.buf[0], -a.buf[1], -a.buf[2], a.buf[3]);
}

inline MathQuaternionf multiply(MathQuaternionf a, MathQuaternionf b) {
    return q(
        double(a.buf[3]) * b.buf[0] + double(a.buf[0]) * b.buf[3] + double(a.buf[1]) * b.buf[2] -
            double(a.buf[2]) * b.buf[1],
        double(a.buf[3]) * b.buf[1] - double(a.buf[0]) * b.buf[2] + double(a.buf[1]) * b.buf[3] +
            double(a.buf[2]) * b.buf[0],
        double(a.buf[3]) * b.buf[2] + double(a.buf[0]) * b.buf[1] - double(a.buf[1]) * b.buf[0] +
            double(a.buf[2]) * b.buf[3],
        double(a.buf[3]) * b.buf[3] - double(a.buf[0]) * b.buf[0] - double(a.buf[1]) * b.buf[1] -
            double(a.buf[2]) * b.buf[2]
    );
}

inline MathVector3f rotate(MathQuaternionf a, MathVector3f b) {
    auto u = v(a.buf[0], a.buf[1], a.buf[2]);
    auto t = scale(cross(u, b), 2);
    return add(b, add(scale(t, a.buf[3]), cross(u, t)));
}

inline MathVector3f point(MathPose a, MathVector3f b) {
    return add(a.pos, rotate(a.quat, b));
}

inline MathPose compose(MathPose a, MathPose b) {
    return {point(a, b.pos), multiply(a.quat, b.quat)};
}

inline bool finite(MathVector3f a) {
    return std::isfinite(a.buf[0]) && std::isfinite(a.buf[1]) && std::isfinite(a.buf[2]);
}

inline bool finite(MathQuaternionf a) {
    return std::isfinite(a.buf[0]) && std::isfinite(a.buf[1]) && std::isfinite(a.buf[2]) &&
           std::isfinite(a.buf[3]);
}

inline void valid_pose(MathPose a) {
    double n = 0;
    for (float x : a.quat.buf)
        n += double(x) * x;
    require(
        finite(a.pos) && finite(a.quat) && std::abs(n - 1) < 1e-4,
        "pose requires finite position and unit xyzw quaternion"
    );
}

inline uint32_t index(const Simulation &m, uint32_t id) {
    for (uint32_t i = 0; i < m.object_count; ++i)
        if (m.object_table[i]->body.id == id)
            return i;
    throw Error(SIM_INVALID, "unknown object ID");
}

inline bool member(const Simulation &m, const ObjectBlock *p) {
    for (uint32_t i = 0; i < m.object_count; ++i)
        if (m.object_table[i] == p)
            return true;
    return false;
}

struct Hash {
    uint64_t value = 1469598103934665603ULL;

    void raw(const void *p, size_t n) {
        auto *b = static_cast<const uint8_t *>(p);
        for (size_t i = 0; i < n; ++i) {
            value ^= b[i];
            value *= 1099511628211ULL;
        }
    }

    template <class T> void scalar(T n) {
        if constexpr (std::is_floating_point_v<T>) {
            if (n == 0)
                n = 0;
            raw(&n, sizeof(n));
        } else
            raw(&n, sizeof(n));
    }

    void vec(MathVector3f a) {
        for (float f : a.buf)
            scalar(f);
    }

    void quat(MathQuaternionf a) {
        for (float f : a.buf)
            scalar(f);
    }

    void state(const SimBodyState &s) {
        scalar(s.id);
        vec(s.pose.pos);
        quat(s.pose.quat);
        vec(s.center_of_mass);
        vec(s.linear_velocity);
        vec(s.angular_velocity);
    }
};

template <class T> SimResult copy_out(const Buffer<T> &b, T *out, uint32_t c, uint32_t *n) {
    if (!n)
        return SIM_INVALID;
    *n = uint32_t(b.size());
    if (c < b.size())
        return SIM_BUFFER_TOO_SMALL;
    if (!b.empty() && !out)
        return SIM_INVALID;
    if (!b.empty())
        std::copy(b.begin(), b.end(), out);
    return SIM_OK;
}

inline void validate_model(const Simulation *m) {
    char e[256]{};
    check(simulation_validate(m, e, sizeof e), e);
    require(m->frozen, "backend requires finalized model");
}
} // namespace core

namespace core {
inline MathVector3f operator+(MathVector3f a, MathVector3f b) {
    return add(a, b);
}

inline MathVector3f operator-(MathVector3f a, MathVector3f b) {
    return sub(a, b);
}

inline MathVector3f operator*(MathVector3f a, double b) {
    return scale(a, b);
}

inline MathVector3f operator/(MathVector3f a, double b) {
    return scale(a, 1 / b);
}

const ForceModule *builtin_modules(uint32_t &count);
} // namespace core

namespace core {
const SimulationConfig &model_config(const Simulation &);
}

namespace core {
inline void validate_reset(const Simulation &m, const SimBodyState *s, uint32_t count) {
    require((!s && !count) || (s && count == m.object_count), "reset requires all object states");
    for (uint32_t i = 0; i < count; ++i) {
        auto &b = m.object_table[i]->body;
        require(
            s[i].id == b.id && finite(s[i].linear_velocity) && finite(s[i].angular_velocity),
            "invalid reset state"
        );
        valid_pose(s[i].pose);
        if (b.motion_type == SIM_STATIC) {
            for (unsigned k = 0; k < 3; ++k)
                require(
                    s[i].pose.pos.buf[k] == b.initial_pose.pos.buf[k],
                    "static body cannot move on reset"
                );
            for (unsigned k = 0; k < 4; ++k)
                require(
                    s[i].pose.quat.buf[k] == b.initial_pose.quat.buf[k],
                    "static body cannot rotate on reset"
                );
        }
    }
}

inline void validate_wrenches(const Simulation &m, const SimWrench *w, uint32_t count) {
    require(count <= m.object_count && (!count || w), "invalid wrench count");
    bool ordered = true;
    for (uint32_t i = 0; i < count; ++i) {
        require(finite(w[i].force) && finite(w[i].torque), "invalid wrench values");
        ordered &= w[i].id == m.object_table[i]->body.id;
        if (!ordered) {
            (void)index(m, w[i].id);
            for (uint32_t j = 0; j < i; ++j)
                require(w[j].id != w[i].id, "duplicate wrench ID");
        }
    }
}
} // namespace core

namespace core {
inline SimAllocator model_allocator(const Simulation &m) {
    SimAllocator a{};
    check(simulation_get_allocator(&m, &a), "missing model allocator");
    return a;
}
} // namespace core
