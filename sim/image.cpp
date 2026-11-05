#include "image.hpp"
#include <charconv>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numbers>
#include <set>
#include <type_traits>
#include <unistd.h>
namespace sim {
namespace {
template <class T>
void append_items(std::vector<uint8_t> &out, const std::vector<T> &items) {
  static_assert(std::is_trivially_copyable_v<T>);
  if (items.empty())
    return;
  auto bytes = items.size() * sizeof(T);
  auto at = out.size();
  out.resize(at + bytes);
  std::memcpy(out.data() + at, items.data(), bytes);
}
template <class T>
void read_items(const uint8_t *&p, const uint8_t *end, std::vector<T> &out,
                uint32_t count) {
  static_assert(std::is_trivially_copyable_v<T>);
  size_t bytes = size_t(count) * sizeof(T);
  require(size_t(end - p) >= bytes, "truncated scene snapshot");
  out.resize(count);
  if (bytes)
    std::memcpy(out.data(), p, bytes);
  p += bytes;
}
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
Scene::Scene() {
  for (const char *name :
       {"gravity_compensation", "hydrodynamics", "thrusters"}) {
    NameRec module{};
    copy_text(module.text, sizeof(module.text), name);
    modules.push_back(module);
  }
}
bool Scene::from_blender() const {
  return std::strcmp(header.source_kind, "blender_evaluated") == 0;
}
std::string Scene::content_hash() const {
  size_t n = 0;
  while (n < sizeof(header.content_hash) && header.content_hash[n])
    ++n;
  return {header.content_hash, n};
}
void Scene::validate() const {
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
  require(!bodies.empty() && bodies.size() <= 4096, "invalid bodies");
  require(zero_tail(d.source_kind, sizeof(d.source_kind)) &&
              zero_tail(d.scene_name, sizeof(d.scene_name)),
          "scene name is not a fixed string");
  std::set<uint32_t> ids;
  for (const auto &b : bodies) {
    require(ids.insert(b.id).second && b.motion <= SIM_KINEMATIC && b.pad == 0,
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
                size_t(b.shape_begin) + b.shape_count <= shapes.size(),
            "invalid shapes");
    for (uint32_t k = 0; k < b.shape_count; ++k) {
      const auto &s = shapes[b.shape_begin + k];
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
      require(size_t(s.vertex_begin) + s.vertex_count <= vertices.size() &&
                  size_t(s.triangle_begin) + s.triangle_count <= triangles.size(),
              "shape mesh range is outside the scene");
      if (s.kind == SIM_MESH) {
        require(s.vertex_count >= 4 && s.vertex_count <= 1000000 &&
                    s.triangle_count && s.triangle_count <= 2000000,
                "invalid mesh");
        for (uint32_t v = 0; v < s.vertex_count; ++v)
          require(finite(vertices[s.vertex_begin + v]), "non-finite mesh vertex");
        for (uint32_t t = 0; t < s.triangle_count; ++t) {
          auto tri = triangles[s.triangle_begin + t];
          require(tri.a < s.vertex_count && tri.b < s.vertex_count &&
                      tri.c < s.vertex_count,
                  "mesh index out of range");
        }
      }
    }
  }
  require(thrusters.size() <= 4096, "too many thrusters");
  for (const auto &t : thrusters) {
    require(t.pad == 0 && t.body < bodies.size() &&
                bodies[t.body].motion == SIM_DYNAMIC,
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
    require(h.pad == 0 && h.body < bodies.size(), "invalid hull body");
    valid_pose(h.pose);
    require(finite(h.coefficient_frame) &&
                std::abs(norm2(h.coefficient_frame) - 1) < 1e-6,
            "invalid hull coefficient frame");
    require(finite(h.drag) && finite(h.skin) && h.drag.x >= 0 && h.drag.y >= 0 &&
                h.drag.z >= 0 && h.skin.x >= 0 && h.skin.y >= 0 && h.skin.z >= 0 &&
                std::isfinite(h.volume) && h.volume >= 0 && finite(h.centroid),
            "invalid hull");
    require(h.vertex_count && h.vertex_count % 3 == 0 &&
                size_t(h.vertex_begin) + h.vertex_count <= hull_vertices.size(),
            "invalid hull triangles");
    for (uint32_t v = 0; v < h.vertex_count; ++v)
      require(finite(hull_vertices[h.vertex_begin + v]), "invalid hull vertex");
  }
  require(d.camera.pad == 0 && d.camera.body < bodies.size() && d.camera.width &&
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
std::vector<uint8_t> Scene::binary() const {
  SceneHeader hdr = header;
  require(bodies.size() <= std::numeric_limits<uint32_t>::max() &&
              shapes.size() <= std::numeric_limits<uint32_t>::max() &&
              vertices.size() <= std::numeric_limits<uint32_t>::max() &&
              triangles.size() <= std::numeric_limits<uint32_t>::max() &&
              thrusters.size() <= std::numeric_limits<uint32_t>::max() &&
              hulls.size() <= std::numeric_limits<uint32_t>::max() &&
              hull_vertices.size() <= std::numeric_limits<uint32_t>::max() &&
              faults.size() <= std::numeric_limits<uint32_t>::max() &&
              modules.size() <= std::numeric_limits<uint32_t>::max(),
          "scene array exceeds its count field");
  hdr.body_count = bodies.size();
  hdr.shape_count = shapes.size();
  hdr.vertex_count = vertices.size();
  hdr.triangle_count = triangles.size();
  hdr.thruster_count = thrusters.size();
  hdr.hull_count = hulls.size();
  hdr.hull_vertex_count = hull_vertices.size();
  hdr.fault_count = faults.size();
  hdr.module_count = modules.size();
  if (commands.empty()) {
    hdr.command_ticks = 0;
    hdr.command_width = 0;
  } else {
    require(hdr.command_width && commands.size() % hdr.command_width == 0,
            "command sequence width does not divide its samples");
    hdr.command_ticks = commands.size() / hdr.command_width;
  }
  std::vector<uint8_t> payload(sizeof(hdr));
  std::memcpy(payload.data(), &hdr, sizeof(hdr));
  append_items(payload, bodies);
  append_items(payload, shapes);
  append_items(payload, vertices);
  append_items(payload, triangles);
  append_items(payload, thrusters);
  append_items(payload, hulls);
  append_items(payload, hull_vertices);
  append_items(payload, faults);
  append_items(payload, modules);
  append_items(payload, commands);
  require(!payload.empty() && payload.size() <= max_scene_bytes,
          "invalid binary scene size");
  std::vector<uint8_t> out(scene_magic, scene_magic + 8);
  uint64_t size = payload.size();
  for (int i = 0; i < 8; ++i)
    out.push_back(uint8_t(size >> (8 * i)));
  out.insert(out.end(), payload.begin(), payload.end());
  return out;
}
Scene Scene::read_prefix(const void *data, size_t n, size_t *used) {
  require(data && n >= 16, "truncated scene snapshot");
  auto bytes = static_cast<const uint8_t *>(data);
  require(std::memcmp(bytes, scene_magic, 8) == 0, "unsupported binary scene version");
  uint64_t count = 0;
  for (int i = 0; i < 8; ++i)
    count |= uint64_t(bytes[8 + i]) << (8 * i);
  require(count && count <= max_scene_bytes && n >= 16 + count,
          "invalid binary scene size");
  const uint8_t *p = bytes + 16;
  const uint8_t *end = p + count;
  require(size_t(end - p) >= sizeof(SceneHeader), "truncated scene header");
  Scene scene;
  std::memcpy(&scene.header, p, sizeof(SceneHeader));
  p += sizeof(SceneHeader);
  // The default constructor installs the standard modules. A snapshot replaces them.
  scene.modules.clear();
  read_items(p, end, scene.bodies, scene.header.body_count);
  read_items(p, end, scene.shapes, scene.header.shape_count);
  read_items(p, end, scene.vertices, scene.header.vertex_count);
  read_items(p, end, scene.triangles, scene.header.triangle_count);
  read_items(p, end, scene.thrusters, scene.header.thruster_count);
  read_items(p, end, scene.hulls, scene.header.hull_count);
  read_items(p, end, scene.hull_vertices, scene.header.hull_vertex_count);
  read_items(p, end, scene.faults, scene.header.fault_count);
  read_items(p, end, scene.modules, scene.header.module_count);
  uint64_t command_count = uint64_t(scene.header.command_ticks) *
                           scene.header.command_width;
  require(command_count <= std::numeric_limits<uint32_t>::max(),
          "command sequence too large");
  read_items(p, end, scene.commands, uint32_t(command_count));
  require(p == end, "scene snapshot has trailing bytes inside its payload");
  scene.validate();
  if (used)
    *used = 16 + count;
  return scene;
}
Scene Scene::read(const void *data, size_t n) {
  size_t used = 0;
  auto scene = read_prefix(data, n, &used);
  require(used == n, "scene snapshot file size does not match its header");
  return scene;
}
uint64_t Scene::fingerprint() const {
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
Scene Scene::load(const std::string &file) {
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
  if (path.extension() == ".blend")
    return load_blend(path);
  require(std::filesystem::is_regular_file(path), "cannot open scenario");
  require(std::filesystem::file_size(path) <= max_scene_bytes + 16,
          "scenario exceeds 128 MiB");
  std::ifstream f(path, std::ios::binary);
  require(bool(f), "cannot open scenario");
  char prefix[8]{};
  f.read(prefix, 8);
  require(std::string(prefix, 8) == std::string(scene_magic, 8),
          "scene files are binary snapshots");
  int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
  require(fd >= 0, "cannot open binary scene");
  try {
    auto scene = load_fd(fd);
    ::close(fd);
    scene.path = std::filesystem::absolute(path).string();
    return scene;
  } catch (...) {
    ::close(fd);
    throw;
  }
}
} // namespace sim
