#include "blend/robot_registry.hpp"
#include "image.hpp"
#include <cstring>
#include <tuple>
namespace sim {
namespace {
template <class F> void fields(SimVec3 &v, F &&f) { f(v.x, v.y, v.z); }
template <class F> void fields(SimQuat &v, F &&f) { f(v.x, v.y, v.z, v.w); }
template <class F> void fields(SimPose &v, F &&f) {
  f(v.position, v.orientation);
}
template <class F> void fields(SimTriangle &v, F &&f) { f(v.a, v.b, v.c); }
template <class F> void fields(RotorSpec &v, F &&f) {
  f(v.max_rpm, v.diameter, v.thrust, v.torque, v.inertia, v.kp, v.ki,
    v.command_sign);
}
template <class F> void fields(BoundsRec &v, F &&f) {
  f(v.position, v.speed, v.angular_speed);
}
template <class F> void fields(CameraRec &v, F &&f) {
  f(v.body, v.width, v.height, v.pad, v.horizontal_fov, v.pose);
}
template <class F> void fields(NameRec &v, F &&f) { f(v.text); }
template <class F> void fields(BodyRec &v, F &&f) {
  f(v.id, v.name, v.cls, v.motion, v.tracked, v.fuzz, v.pad, v.mass, v.inertia,
    v.center_of_mass, v.initial_pose, v.initial_linear_velocity,
    v.initial_angular_velocity, v.dry_mass, v.volume, v.buoyancy_center,
    v.shape_begin, v.shape_count);
}
template <class F> void fields(ShapeRec &v, F &&f) {
  f(v.kind, v.collisions, v.pose, v.size, v.drag, v.skin, v.friction,
    v.restitution, v.vertex_begin, v.vertex_count, v.triangle_begin,
    v.triangle_count);
}
template <class F> void fields(ThrusterRec &v, F &&f) {
  f(v.body, v.pad, v.position, v.direction, v.spec);
}
template <class F> void fields(HullRec &v, F &&f) {
  f(v.body, v.vertex_begin, v.vertex_count, v.pad, v.pose, v.drag, v.skin,
    v.centroid, v.coefficient_frame, v.volume);
}
template <class F> void fields(FaultRec &v, F &&f) {
  f(v.kind, v.thruster, v.start, v.end, v.magnitude);
}
template <class F> void fields(SceneHeader &v, F &&f) {
  f(v.version, v.dt_numerator, v.dt_denominator, v.integrator,
    v.native_hydrodynamics, v.hydro_period, v.body_count, v.shape_count,
    v.vertex_count, v.triangle_count, v.thruster_count, v.hull_count,
    v.hull_vertex_count, v.fault_count, v.module_count, v.command_ticks,
    v.command_width, v.source_frame, v.case_seed, v.generator_version,
    v.reserved, v.water_density, v.gravity, v.current, v.camera, v.bounds,
    v.source_kind, v.content_hash, v.scene_name);
}
struct Writer {
  std::vector<uint8_t> bytes;
  template <class T> void put(T value) {
    if constexpr (std::is_enum_v<T>)
      put(uint32_t(value));
    else if constexpr (std::is_integral_v<T>) {
      using U = std::make_unsigned_t<T>;
      auto u = static_cast<U>(value);
      for (size_t i = 0; i < sizeof(T); ++i)
        bytes.push_back(uint8_t(u >> (i * 8)));
    } else if constexpr (std::is_same_v<T, double>)
      put(std::bit_cast<uint64_t>(value));
    else
      fields(value, [&](auto &...v) { (put(v), ...); });
  }
  template <size_t N> void put(const char (&v)[N]) {
    for (char c : v)
      put(uint8_t(c));
  }
};
struct Reader {
  std::span<const uint8_t> bytes;
  size_t at = 0;
  template <class T> void get(T &value) {
    if constexpr (std::is_enum_v<T>) {
      uint32_t v;
      get(v);
      value = T(v);
    } else if constexpr (std::is_integral_v<T>) {
      require(bytes.size() - at >= sizeof(T), "truncated scene section");
      using U = std::make_unsigned_t<T>;
      U u = 0;
      for (size_t i = 0; i < sizeof(T); ++i)
        u |= U(bytes[at++]) << (i * 8);
      value = T(u);
    } else if constexpr (std::is_same_v<T, double>) {
      uint64_t u;
      get(u);
      value = std::bit_cast<double>(u);
    } else
      fields(value, [&](auto &...v) { (get(v), ...); });
  }
  template <size_t N> void get(char (&v)[N]) {
    for (char &c : v) {
      uint8_t b;
      get(b);
      c = char(b);
    }
  }
  template <class T> T take() {
    T v{};
    get(v);
    return v;
  }
};
struct Section {
  uint32_t count = 0;
  Writer data;
};
constexpr size_t widths[] = {408, 328, 168, 24, 12, 120, 184,
                             24,  32,  32,  8,  16, 4};
} // namespace
std::vector<uint8_t> Main::binary() const {
  const auto &faults = run.faults;
  const auto &modules = run.modules;
  auto thrusters = robot.thrusters();
  auto hulls = robot.hulls();
  require(frozen(), "serialization requires a frozen database");
  Section sections[13];
  auto add = [&](size_t k, auto value) {
    sections[k].data.put(value);
    ++sections[k].count;
  };
  auto hdr = header;
  hdr.body_count = objects.size();
  hdr.shape_count = collider_count();
  hdr.thruster_count = thrusters.size();
  hdr.hull_count = hulls.size();
  hdr.fault_count = faults.size();
  hdr.module_count = modules.size();
  hdr.command_ticks =
      commands.empty() ? 0 : commands.size() / hdr.command_width;
  hdr.command_width = commands.empty() ? 0 : hdr.command_width;
  std::vector<std::array<uint32_t, 4>> ranges;
  for (const auto &m : geometry()) {
    ranges.push_back({sections[3].count, uint32_t(m.positions.size()),
                      sections[4].count, uint32_t(m.triangles.size())});
    for (auto p : m.positions)
      add(3, p);
    for (auto t : m.triangles)
      add(4, t);
    for (auto n : ranges.back())
      sections[11].data.put(n);
    ++sections[11].count;
  }
  for (const auto &b : objects)
    add(1, b);
  for (size_t i = 0; i < objects.size(); ++i) {
    for (const auto &c : objects.block(i).colliders) {
      auto s = c.data;
      uint32_t index = geometry_index(c);
      if (index != UINT32_MAX) {
        const auto &r = ranges[index];
        s.vertex_begin = r[0];
        s.vertex_count = r[1];
        s.triangle_begin = r[2];
        s.triangle_count = r[3];
      } else {
        s.vertex_begin = sections[3].count;
        s.triangle_begin = sections[4].count;
        s.vertex_count = s.triangle_count = 0;
      }
      add(2, s);
      add(12, index);
    }
  }
  for (auto &t : thrusters)
    add(5, t);
  for (auto &h : hulls) {
    auto v = h;
    v.vertex_begin = sections[7].count;
    for (auto p : positions(h))
      add(7, p);
    add(6, v);
  }
  for (auto &f : faults)
    add(8, f);
  for (auto &m : modules)
    add(9, m);
  for (auto v : commands)
    add(10, v);
  hdr.vertex_count = sections[3].count;
  hdr.triangle_count = sections[4].count;
  hdr.hull_vertex_count = sections[7].count;
  add(0, hdr);
  Writer payload;
  payload.put(uint32_t(13));
  payload.put(uint32_t(1));
  uint64_t offset = 8 + 13 * 24;
  for (uint32_t i = 0; i < 13; ++i) {
    auto &s = sections[i];
    require(s.data.bytes.size() == size_t(s.count) * widths[i],
            "scene field schema mismatch");
    payload.put(i + 1);
    payload.put(s.count);
    payload.put(offset);
    payload.put(uint64_t(s.data.bytes.size()));
    offset += s.data.bytes.size();
  }
  require(offset <= max_scene_bytes, "scene exceeds binary size limit");
  for (auto &s : sections)
    payload.bytes.insert(payload.bytes.end(), s.data.bytes.begin(),
                         s.data.bytes.end());
  Writer out;
  for (char c : scene_magic)
    out.put(uint8_t(c));
  out.put(uint64_t(payload.bytes.size()));
  out.bytes.insert(out.bytes.end(), payload.bytes.begin(), payload.bytes.end());
  return out.bytes;
}
MainPtr Main::read(const void *data, size_t n) {
  require(data && n >= 16, "truncated binary scene header");
  auto raw = std::span(static_cast<const uint8_t *>(data), n);
  require(!std::memcmp(data, scene_magic, 8),
          "unsupported scene format; ASIMSC03 is required (regenerate old "
          "recordings)");
  Reader prefix{raw.subspan(8)};
  auto length = prefix.take<uint64_t>();
  require(length && length <= max_scene_bytes && length == n - 16,
          "invalid binary scene size");
  Reader directory{raw.subspan(16)};
  require(directory.take<uint32_t>() == 13 && directory.take<uint32_t>() == 1,
          "unsupported scene directory");
  std::array<Reader, 13> readers{};
  std::array<uint32_t, 13> counts{};
  uint64_t end = 8 + 13 * 24;
  for (size_t i = 0; i < 13; ++i) {
    auto kind = directory.take<uint32_t>(), count = directory.take<uint32_t>();
    auto offset = directory.take<uint64_t>(),
         bytes = directory.take<uint64_t>();
    require(kind == i + 1 && offset == end &&
                bytes == uint64_t(count) * widths[i] && offset <= length &&
                bytes <= length - offset,
            "invalid scene section directory");
    readers[i] = {raw.subspan(16 + offset, bytes)};
    counts[i] = count;
    end += bytes;
  }
  require(end == length && counts[0] == 1 && counts[1] <= 4096,
          "invalid scene sections");
  auto out = std::make_unique<Main>();
  auto &s = *out;
  s.header = readers[0].take<SceneHeader>();
  s.run.modules.clear();
  uint32_t expected[] = {1,
                         s.header.body_count,
                         s.header.shape_count,
                         s.header.vertex_count,
                         s.header.triangle_count,
                         s.header.thruster_count,
                         s.header.hull_count,
                         s.header.hull_vertex_count,
                         s.header.fault_count,
                         s.header.module_count,
                         0,
                         counts[11],
                         s.header.shape_count};
  auto command_count =
      uint64_t(s.header.command_ticks) * s.header.command_width;
  require(command_count <= UINT32_MAX, "command sequence too large");
  expected[10] = command_count;
  for (size_t i = 0; i < 13; ++i)
    require(counts[i] == expected[i],
            "scene header disagrees with section counts");
  auto records = [&](size_t k, auto &table) {
    using T = std::remove_cvref_t<decltype(table[0])>;
    for (uint32_t i = 0; i < counts[k]; ++i)
      table.push_back(readers[k].template take<T>());
  };
  // Flat descriptions exist only while decoding the transport tables. Objects
  // take ownership of their collider arrays after references have been checked.
  std::vector<BodyRec> bodies;
  std::vector<bool> fixed;
  for (uint32_t i = 0; i < counts[1]; ++i) {
    auto b = readers[1].take<BodyRec>();
    if (auto *definition = robot_definition(
            std::string_view(b.name, strnlen(b.name, sizeof(b.name))))) {
      require(!s.robot.definition,
              "only one fixed robot is supported per scene");
      s.robot.definition = definition;
      fixed.push_back(true);
    } else
      fixed.push_back(false);
    bodies.push_back(b);
  }
  std::vector<ShapeRec> shapes;
  records(2, shapes);
  records(3, s.vertices);
  records(4, s.triangles);
  require(counts[5] <= Robot::max_thrusters && counts[6] <= Robot::max_hulls,
          "scene exceeds fixed robot capacity");
  for (uint32_t i = 0; i < counts[5]; ++i)
    s.robot.add_thruster(readers[5].take<ThrusterRec>());
  for (uint32_t i = 0; i < counts[6]; ++i)
    s.robot.add_hull(readers[6].take<HullRec>());
  records(7, s.hull_vertices);
  records(8, s.run.faults);
  records(9, s.run.modules);
  records(10, s.commands);
  std::vector<std::array<uint32_t, 4>> ranges;
  for (uint32_t i = 0; i < counts[11]; ++i) {
    std::array<uint32_t, 4> r;
    for (auto &v : r)
      v = readers[11].take<uint32_t>();
    require(uint64_t(r[0]) + r[1] <= s.vertices.size() &&
                uint64_t(r[2]) + r[3] <= s.triangles.size(),
            "invalid mesh section range");
    ranges.push_back(r);
  }
  for (const auto &g : shapes) {
    auto index = readers[12].take<uint32_t>();
    if (g.kind == SIM_MESH) {
      require(index < ranges.size(), "unknown mesh reference");
      auto &r = ranges[index];
      require(g.vertex_begin == r[0] && g.vertex_count == r[1] &&
                  g.triangle_begin == r[2] && g.triangle_count == r[3],
              "shape disagrees with mesh reference");
    } else
      require(index == UINT32_MAX, "primitive references a mesh");
  }
  std::vector<bool> claimed(shapes.size(), false);
  for (size_t i = 0; i < bodies.size(); ++i) {
    const auto &body = bodies[i];
    require(body.shape_count && body.shape_count <= 1024 &&
                uint64_t(body.shape_begin) + body.shape_count <= shapes.size(),
            "invalid object collider range");
    std::vector<Collider> colliders;
    for (uint32_t k = 0; k < body.shape_count; ++k) {
      size_t index = body.shape_begin + k;
      require(!claimed[index], "collider has multiple owning objects");
      claimed[index] = true;
      colliders.push_back(Collider{shapes[index]});
    }
    s.add_object(body, colliders, fixed[i]);
  }
  for (bool used : claimed)
    require(used, "collider has no owning object");
  s.finalize();
  return out;
}
} // namespace sim
