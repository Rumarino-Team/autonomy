#pragma once
#include "backend_common.hpp"
#include <array>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>
namespace sim {
using Json = nlohmann::json;
inline Json json(SimVec3 v) { return {v.x, v.y, v.z}; }
inline Json json(SimQuat q) { return {q.x, q.y, q.z, q.w}; }
inline Json json(SimPose p) {
  return {{"position", json(p.position)}, {"orientation", json(p.orientation)}};
}
inline SimVec3 vec(const Json &j) {
  require(j.is_array() && j.size() == 3, "expected vector of length 3");
  return {j[0].get<double>(), j[1].get<double>(), j[2].get<double>()};
}
inline SimQuat quat(const Json &j) {
  require(j.is_array() && j.size() == 4, "expected xyzw quaternion");
  return {j[0].get<double>(), j[1].get<double>(), j[2].get<double>(),
          j[3].get<double>()};
}
inline SimPose pose(const Json &j) {
  return {vec(j.value("position", Json{0, 0, 0})),
          quat(j.value("orientation", Json{0, 0, 0, 1}))};
}
inline Json json(const SimBodyState &s) {
  return {{"id", s.id},
          {"pose", json(s.pose)},
          {"com", json(s.center_of_mass)},
          {"linear", json(s.linear_velocity)},
          {"angular", json(s.angular_velocity)}};
}
inline SimBodyState state(const Json &j) {
  return {j.at("id"), pose(j.at("pose")), vec(j.at("com")), vec(j.at("linear")),
          vec(j.at("angular"))};
}
struct ShapeStorage {
  SimShape value{};
  std::vector<SimVec3> vertices;
  std::vector<SimTriangle> triangles;
};
struct BodyStorage {
  SimBody value{};
  std::string name;
  uint32_t cls = 0;
  bool tracked = false, fuzz = false;
  std::vector<ShapeStorage> shapes;
  std::vector<SimShape> views;
};
struct ModelStorage {
  SimModelDesc value{};
  std::vector<BodyStorage> bodies;
  std::vector<SimBody> views;
  void bind() {
    views.clear();
    for (auto &b : bodies) {
      b.views.clear();
      for (auto &s : b.shapes) {
        s.value.vertices = s.vertices.data();
        s.value.vertex_count = s.vertices.size();
        s.value.triangles = s.triangles.data();
        s.value.triangle_count = s.triangles.size();
        b.views.push_back(s.value);
      }
      b.value.shapes = b.views.data();
      b.value.shape_count = b.views.size();
      views.push_back(b.value);
    }
    value.struct_size = sizeof(value);
    value.bodies = views.data();
    value.body_count = views.size();
  }
};
inline Json encode(const SimModelDesc &d) {
  Json j = {{"dt", {d.dt_numerator, d.dt_denominator}},
            {"integrator", d.integrator},
            {"gravity", json(d.gravity)},
            {"water_density", d.water_density},
            {"current", json(d.water_current)},
            {"native_hydrodynamics", d.native_hydrodynamics != 0},
            {"bodies", Json::array()}};
  for (uint32_t i = 0; i < d.body_count; ++i) {
    const auto &b = d.bodies[i];
    Json o = {{"id", b.id},
              {"motion", b.motion},
              {"mass", b.mass},
              {"inertia", json(b.inertia)},
              {"center_of_mass", json(b.center_of_mass)},
              {"pose", json(b.initial_pose)},
              {"linear_velocity", json(b.initial_linear_velocity)},
              {"angular_velocity", json(b.initial_angular_velocity)},
              {"shapes", Json::array()}};
    for (uint32_t k = 0; k < b.shape_count; ++k) {
      const auto &s = b.shapes[k];
      Json g = {{"kind", s.kind},
                {"collisions", s.collisions != 0},
                {"pose", json(s.pose)},
                {"size", json(s.size)},
                {"friction", s.friction},
                {"restitution", s.restitution},
                {"drag", json(s.drag)},
                {"skin", json(s.skin)},
                {"vertices", Json::array()},
                {"triangles", Json::array()}};
      for (uint32_t v = 0; v < s.vertex_count; ++v)
        g["vertices"].push_back(json(s.vertices[v]));
      for (uint32_t v = 0; v < s.triangle_count; ++v) {
        auto t = s.triangles[v];
        g["triangles"].push_back({t.a, t.b, t.c});
      }
      o["shapes"].push_back(g);
    }
    j["bodies"].push_back(o);
  }
  return j;
}
inline ModelStorage decode(const Json &j) {
  ModelStorage m;
  auto dt = j.value("dt", Json{1, 360});
  require(dt.is_array() && dt.size() == 2,
          "dt must contain numerator and denominator");
  auto integrator = j.value("integrator", Json(0));
  if (integrator.is_string()) {
    static const std::vector<std::string> names = {
        "default", "euler", "implicit", "implicitfast", "rk4"};
    auto name = integrator.get<std::string>();
    auto found = std::find(names.begin(), names.end(), name);
    require(found != names.end(), "unknown integrator");
    m.value.integrator = found - names.begin();
  } else
    m.value.integrator = integrator.get<uint32_t>();
  m.value.dt_numerator = dt[0];
  m.value.dt_denominator = dt[1];
  m.value.gravity = vec(j.value("gravity", Json{0, 0, 9.81}));
  m.value.water_density = j.value("water_density", 1025.0);
  m.value.water_current = vec(j.value("current", Json{0, 0, 0}));
  m.value.native_hydrodynamics = j.value("native_hydrodynamics", false);
  require(j.at("bodies").is_array() && j.at("bodies").size() <= 4096,
          "too many bodies");
  for (const auto &o : j.at("bodies")) {
    BodyStorage b;
    b.value.id = o.at("id");
    b.name = o.value("name", std::to_string(b.value.id));
    b.cls = o.value("class", 0u);
    require(b.cls <= 255, "observation class must fit the AUV ABI");
    b.tracked = o.value("tracked", false);
    b.fuzz = o.value("fuzz", false);
    b.value.motion = o.value("motion", uint32_t(SIM_STATIC));
    b.value.mass = o.value("mass", 0.0);
    b.value.inertia = vec(o.value("inertia", Json{1, 1, 1}));
    b.value.center_of_mass = pose(o.value("center_of_mass", Json::object()));
    b.value.initial_pose = pose(o.value("pose", Json::object()));
    b.value.initial_linear_velocity =
        vec(o.value("linear_velocity", Json{0, 0, 0}));
    b.value.initial_angular_velocity =
        vec(o.value("angular_velocity", Json{0, 0, 0}));
    require(o.at("shapes").is_array() && o.at("shapes").size() <= 1024,
            "too many shapes");
    for (const auto &g : o.at("shapes")) {
      ShapeStorage s;
      s.value.kind = g.at("kind");
      s.value.collisions = g.value("collisions", true);
      s.value.pose = pose(g.value("pose", Json::object()));
      s.value.size = vec(g.value("size", Json{1, 1, 1}));
      s.value.friction = g.value("friction", 0.5);
      s.value.restitution = g.value("restitution", 0.0);
      s.value.drag = vec(g.value("drag", Json{0, 0, 0}));
      s.value.skin = vec(g.value("skin", Json{0, 0, 0}));
      require(g.value("vertices", Json::array()).size() <= 1000000 &&
                  g.value("triangles", Json::array()).size() <= 2000000,
              "mesh too large");
      for (const auto &v : g.value("vertices", Json::array()))
        s.vertices.push_back(vec(v));
      for (const auto &t : g.value("triangles", Json::array())) {
        require(t.is_array() && t.size() == 3, "expected triangle indices");
        s.triangles.push_back({t[0], t[1], t[2]});
      }
      b.shapes.push_back(std::move(s));
    }
    m.bodies.push_back(std::move(b));
  }
  m.bind();
  validate(&m.value);
  return m;
}
struct RotorSpec {
  double max_rpm = 1000, diameter = .18, thrust = .48, torque = .05,
         inertia = .00146468, kp = 1, ki = 10, command_sign = -1;
};
struct Thruster {
  uint32_t body = 0;
  SimVec3 position{}, direction{1, 0, 0};
  RotorSpec spec;
};
struct Hull {
  uint32_t body = 0;
  SimPose pose{{0, 0, 0}, {0, 0, 0, 1}};
  SimVec3 drag{}, skin{}, centroid{};
  SimQuat coefficient_frame{0, 0, 0, 1};
  double volume = 0;
  std::vector<std::array<SimVec3, 3>> faces;
};
struct Camera {
  uint32_t body = 0, width = 800, height = 600;
  double horizontal_fov = 60;
  SimPose pose{{0, .3, 0}, {0, 0, 0, 1}};
};
struct Scene {
  ModelStorage model;
  Json document;
  std::filesystem::path path;
  std::vector<Thruster> thrusters;
  std::vector<Hull> hull;
  std::vector<double> dry_mass, volume;
  std::vector<SimVec3> buoyancy_center;
  SimVec3 current{};
  uint32_t hydro_period = 7;
  Camera camera;
  struct Bounds { double position=10000, speed=1000, angular_speed=1000; } bounds;
  enum class FaultKind { Tracking, Imu, Gyro, Depth, Dead, Weak };
  struct Fault {
    FaultKind kind;
    uint64_t start, end;
    size_t thruster=0;
    double magnitude=0;
  };
  std::vector<Fault> faults;
  std::vector<std::string> force_modules;
  bool blender_source=false;
  static Scene load(const std::filesystem::path &path);
  static Scene load_fd(int fd);
  static Scene load_blend(const std::filesystem::path &path);
  std::vector<uint8_t> binary() const;
  static Scene parse(Json document, const std::filesystem::path &path = {});
  uint64_t fingerprint() const {
    Hash h;
    auto s = document.dump();
    h.bytes(s.data(), s.size());
    return h.value;
  }
};
std::vector<std::array<SimVec3, 3>> surface(const SimShape &shape);
} // namespace sim
