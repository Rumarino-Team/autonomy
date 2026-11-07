#include "image.hpp"
#include <charconv>
#include <cstring>
#include <filesystem>
#include <limits>
#include <unistd.h>
#include <numbers>
#include <set>
#include <type_traits>
namespace sim {
namespace {
bool zero_tail(const char *text, size_t n) {
  auto end = static_cast<const char *>(std::memchr(text, '\0', n));
  if (!end)
    return false;
  for (auto p = end; p != text + n; ++p)
    if (*p)
      return false;
  return true;
}
} // namespace
void copy_text(char *dst, size_t n, const char *src) {
  require(dst && n && src && std::strlen(src) < n, "text exceeds fixed field");
  std::memset(dst, 0, n);
  std::memcpy(dst, src, std::strlen(src));
}
bool Main::from_blender() const {
  return std::strcmp(header.source_kind, "blender_native") == 0 ||
         std::strcmp(header.source_kind, "blender_evaluated") == 0;
}
std::string Main::content_hash() const {
  size_t n = 0;
  while (n < sizeof(header.content_hash) && header.content_hash[n])
    ++n;
  return {header.content_hash, n};
}
void Main::validate() const {
  const auto &faults = run.faults;
  const auto &modules = run.modules;
  auto thrusters = robot.thrusters();
  auto hulls = robot.hulls();
  if (frozen()) validate_graph();
  const auto &d = header;
  require(d.version == 1, "unsupported scenario version");
  require(d.integrator <= SIM_INTEGRATOR_RK4, "unknown integrator");
  require(d.dt_numerator && d.dt_denominator &&
              double(d.dt_numerator) / d.dt_denominator <= 0.1,
          "invalid fixed timestep");
  require(finite(d.gravity) && finite(d.current) &&
              std::isfinite(d.water_density) && d.water_density >= 0,
          "invalid environment");
  require(d.native_hydrodynamics <= 1 && d.hydro_period > 0 && d.reserved == 0,
          "invalid scene header");
  if (commands.empty())
    require(d.command_width == 0, "command width set without samples");
  else
    require(d.command_width && commands.size() % d.command_width == 0,
            "command sequence width does not divide its samples");
  require(!objects.empty() && objects.size() <= 4096, "invalid objects");
  require(zero_tail(d.source_kind, sizeof(d.source_kind)) &&
              zero_tail(d.scene_name, sizeof(d.scene_name)),
          "scene name is not a fixed string");
  std::set<uint32_t> ids;
  size_t object_index = 0, shape_cursor = 0;
  for (const auto &b : objects) {
    const auto &object = objects.block(object_index++);
    require(ids.insert(b.id).second && b.id != UINT32_MAX && b.motion <= SIM_KINEMATIC && b.pad == 0,
            "duplicate body id or invalid motion");
    require(b.cls <= 255 && b.tracked <= 1 && b.fuzz <= 1 &&
                zero_tail(b.name, sizeof(b.name)),
            "invalid body name or class");
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
    require(std::isfinite(b.dry_mass) && b.dry_mass >= 0 &&
                std::isfinite(b.volume) && b.volume >= 0 &&
                finite(b.buoyancy_center),
            "invalid body fluid properties");
    require(b.shape_count && b.shape_count <= 1024 &&
                b.shape_begin == shape_cursor && b.shape_count == object.colliders.size(),
            "invalid shapes");
    shape_cursor += object.colliders.size();
    for (const auto &c : object.colliders) {
      require(c.object == &object, "collider has a foreign owning object");
      const auto &s = c.data;
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
        require(s.size.x > 0 && s.size.y > 0 && s.size.z > 0, "invalid box size");
      if (s.kind == SIM_SPHERE)
        require(s.size.x > 0, "invalid sphere radius");
      if (s.kind == SIM_CYLINDER)
        require(s.size.x > 0 && s.size.z > 0, "invalid cylinder size");
      require(positions(c).size() == s.vertex_count && faces(c).size() == s.triangle_count,
              "shape mesh range is outside the scene");
      if (s.kind == SIM_MESH) {
        require(s.vertex_count >= 4 && s.vertex_count <= 1000000 &&
                    s.triangle_count && s.triangle_count <= 2000000,
                "invalid mesh");
        for (uint32_t v = 0; v < s.vertex_count; ++v)
          require(finite(positions(c)[v]), "non-finite mesh vertex");
        for (uint32_t t = 0; t < s.triangle_count; ++t) {
          auto tri = faces(c)[t];
          require(tri.a < s.vertex_count && tri.b < s.vertex_count &&
                      tri.c < s.vertex_count,
                  "mesh index out of range");
        }
      }
    }
  }
  require(thrusters.size() <= Robot::max_thrusters, "too many fixed robot thrusters");
  for (const auto &t : thrusters) {
    require(t.pad == 0 && t.body < objects.size() &&
                objects[t.body].motion == SIM_DYNAMIC,
            "thruster requires a dynamic body");
    require(finite(t.position) && finite(t.direction) &&
                std::abs(norm(t.direction) - 1) < 1e-6,
            "invalid thruster mount");
    const auto &p = t.spec;
    require(std::isfinite(p.max_rpm) && p.max_rpm > 0 &&
                std::isfinite(p.diameter) && p.diameter > 0 &&
                std::isfinite(p.inertia) && p.inertia > 0 &&
                std::isfinite(p.thrust) && p.thrust >= 0 &&
                std::isfinite(p.torque) && p.torque >= 0 &&
                std::isfinite(p.kp) && p.kp >= 0 && std::isfinite(p.ki) &&
                p.ki >= 0 && std::abs(p.command_sign) == 1,
            "invalid rotor specification");
  }
  for (const auto &h : hulls) {
    require(h.pad == 0 && h.body < objects.size(), "invalid hull body");
    valid_pose(h.pose);
    require(finite(h.coefficient_frame) &&
                std::abs(norm2(h.coefficient_frame) - 1) < 1e-6,
            "invalid hull coefficient frame");
    require(finite(h.drag) && finite(h.skin) && h.drag.x >= 0 && h.drag.y >= 0 &&
                h.drag.z >= 0 && h.skin.x >= 0 && h.skin.y >= 0 && h.skin.z >= 0 &&
                std::isfinite(h.volume) && h.volume >= 0 && finite(h.centroid),
            "invalid hull");
    require(h.vertex_count && h.vertex_count % 3 == 0 &&
                positions(h).size() == h.vertex_count,
            "invalid hull triangles");
    for (uint32_t v = 0; v < h.vertex_count; ++v)
      require(finite(positions(h)[v]), "invalid hull vertex");
  }
  require(d.camera.pad == 0 && d.camera.body < objects.size() && d.camera.width &&
              d.camera.height && d.camera.horizontal_fov > 0 &&
              d.camera.horizontal_fov < 180,
          "invalid camera");
  valid_pose(d.camera.pose);
  require(std::isfinite(d.bounds.position) && d.bounds.position > 0 &&
              std::isfinite(d.bounds.speed) && d.bounds.speed > 0 &&
              std::isfinite(d.bounds.angular_speed) && d.bounds.angular_speed > 0,
          "assertion bounds must be positive and finite");
  std::set<std::string> names;
  for (const auto &module : modules) {
    require(zero_tail(module.text, sizeof(module.text)) && module.text[0],
            "invalid force module name");
    require(names.insert(module.text).second, "duplicate force module");
  }
  for (const auto &f : faults) {
    require(uint32_t(f.kind) <= uint32_t(FaultKind::Weak), "unknown fault kind");
    require(f.start < f.end && std::isfinite(f.magnitude), "invalid fault");
    if (f.kind == FaultKind::Dead || f.kind == FaultKind::Weak) {
      require(f.thruster < thrusters.size(), "unknown fault thruster");
      if (f.kind == FaultKind::Weak)
        require(f.magnitude >= 0 && f.magnitude <= 1,
                "weak fault magnitude must be in [0,1]");
    }
  }
}
uint64_t Main::fingerprint() const {
  auto bytes = binary();
  Hash h;
  h.bytes(bytes.data(), bytes.size());
  return h.value;
}
std::vector<std::array<SimVec3, 3>>
surface(uint32_t kind, SimVec3 size, const SimVec3 *vertices,
        const SimTriangle *triangles, uint32_t vertex_count,
        uint32_t triangle_count) {
  std::vector<std::array<SimVec3, 3>> out;
  auto triangle = [&](SimVec3 a, SimVec3 b, SimVec3 c) {
    out.push_back({a, b, c});
  };
  if (kind == SIM_MESH) {
    require(vertices && triangles, "mesh surface is missing its vertices");
    for (uint32_t i = 0; i < triangle_count; ++i) {
      auto t = triangles[i];
      require(t.a < vertex_count && t.b < vertex_count && t.c < vertex_count,
              "mesh index out of range");
      triangle(vertices[t.a], vertices[t.b], vertices[t.c]);
    }
  } else if (kind == SIM_BOX) {
    double x = size.x, y = size.y, z = size.z;
    SimVec3 v[] = {{-x, -y, -z}, {-x, y, -z}, {x, y, -z}, {x, -y, -z},
                   {-x, -y, z},  {-x, y, z},  {x, y, z},  {x, -y, z}};
    int f[][4] = {{0, 1, 2, 3}, {3, 2, 6, 7}, {4, 5, 1, 0},
                  {7, 6, 5, 4}, {6, 2, 1, 5}, {3, 7, 4, 0}};
    for (auto &q : f) {
      triangle(v[q[0]], v[q[1]], v[q[2]]);
      triangle(v[q[0]], v[q[2]], v[q[3]]);
    }
    for (int level = 0; level < 3; ++level) {
      auto prev = std::move(out);
      out.clear();
      for (auto &t : prev) {
        auto a = (t[0] + t[1]) * .5, b = (t[1] + t[2]) * .5,
             c = (t[2] + t[0]) * .5;
        triangle(t[0], a, c);
        triangle(t[1], b, a);
        triangle(t[2], c, b);
        triangle(a, b, c);
      }
    }
  } else if (kind == SIM_CYLINDER) {
    constexpr int n = 32;
    double dp = 2 * std::numbers::pi / n,
           r = std::sqrt(dp * size.x * size.x / std::sin(dp)), z = size.z;
    for (int i = 0; i < n; ++i) {
      double a = i * dp, b = (i + 1) * dp;
      SimVec3 t0{r * std::cos(a), r * std::sin(a), z},
          t1{r * std::cos(b), r * std::sin(b), z}, b0{t0.x, t0.y, -z},
          b1{t1.x, t1.y, -z};
      triangle(t0, b0, t1);
      triangle(b0, b1, t1);
      triangle({0, 0, z}, t0, t1);
      triangle({0, 0, -z}, b1, b0);
    }
  } else {
    constexpr int n = 16;
    constexpr int m = 8;
    auto v = [&](int i, int k) {
      double a = 2 * std::numbers::pi * i / n, b = std::numbers::pi * k / m;
      return SimVec3{size.x * std::sin(b) * std::cos(a),
                     size.x * std::sin(b) * std::sin(a),
                     size.x * std::cos(b)};
    };
    for (int k = 0; k < m; ++k)
      for (int i = 0; i < n; ++i) {
        triangle(v(i, k), v(i, k + 1), v(i + 1, k + 1));
        triangle(v(i, k), v(i + 1, k + 1), v(i + 1, k));
      }
  }
  return out;
}
MainPtr Main::load(const std::string &file) {
  std::filesystem::path path(file);
  if (file == "-")
    return load_fd(STDIN_FILENO);
  if (file.starts_with("fd:")) {
    int fd = -1;
    auto [end, error] =
        std::from_chars(file.data() + 3, file.data() + file.size(), fd);
    require(error == std::errc() && end == file.data() + file.size() && fd >= 0,
            "invalid scene descriptor");
    return load_fd(fd);
  }
  require(path.extension() == ".blend", "scene file must be a .blend");
  return load_blend(path);
}
} // namespace sim
