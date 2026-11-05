#pragma once
// The scene file is a little-endian copy of these records. sizeof and the
// offsetof asserts are the schema; tools/blender/autonomy_sim/wire.py matches them.
#include "backend_common.hpp"
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <string>
namespace sim {
inline constexpr char scene_magic[8] = {'A', 'S', 'I', 'M', 'S', 'C', '0', '2'};
inline constexpr size_t max_scene_bytes = 128 * 1024 * 1024;

enum class FaultKind : uint32_t { Tracking, Imu, Gyro, Depth, Dead, Weak };

struct RotorSpec {
  double max_rpm = 1000, diameter = .18, thrust = .48, torque = .05,
         inertia = .00146468, kp = 1, ki = 10, command_sign = -1;
};
struct BoundsRec {
  double position = 10000, speed = 1000, angular_speed = 1000;
};
struct CameraRec {
  uint32_t body = 0, width = 800, height = 600, pad = 0;
  double horizontal_fov = 60;
  SimPose pose{{0, .3, 0}, {0, 0, 0, 1}};
};
struct NameRec {
  char text[32];
};
struct BodyRec {
  uint32_t id = 0;
  char name[64]{};
  uint32_t cls = 0, motion = SIM_STATIC, tracked = 0, fuzz = 0, pad = 0;
  double mass = 0;
  SimVec3 inertia{1, 1, 1};
  SimPose center_of_mass{{}, {0, 0, 0, 1}};
  SimPose initial_pose{{}, {0, 0, 0, 1}};
  SimVec3 initial_linear_velocity{}, initial_angular_velocity{};
  double dry_mass = 0, volume = 0;
  SimVec3 buoyancy_center{};
  uint32_t shape_begin = 0, shape_count = 0;
};
struct ShapeRec {
  uint32_t kind = SIM_BOX, collisions = 1;
  SimPose pose{{}, {0, 0, 0, 1}};
  SimVec3 size{1, 1, 1}, drag{}, skin{};
  double friction = 0.5, restitution = 0;
  uint32_t vertex_begin = 0, vertex_count = 0, triangle_begin = 0,
           triangle_count = 0;
};
struct ThrusterRec {
  uint32_t body = 0, pad = 0;
  SimVec3 position{}, direction{1, 0, 0};
  RotorSpec spec{};
};
struct HullRec {
  uint32_t body = 0, vertex_begin = 0, vertex_count = 0, pad = 0;
  SimPose pose{{}, {0, 0, 0, 1}};
  SimVec3 drag{}, skin{}, centroid{};
  SimQuat coefficient_frame{0, 0, 0, 1};
  double volume = 0;
};
struct FaultRec {
  FaultKind kind = FaultKind::Tracking;
  uint32_t thruster = 0;
  uint64_t start = 0, end = 0;
  double magnitude = 0;
};
struct SceneHeader {
  uint32_t version = 1;
  uint32_t dt_numerator = 1, dt_denominator = 360, integrator = 0;
  uint32_t native_hydrodynamics = 0, hydro_period = 7;
  uint32_t body_count = 0, shape_count = 0, vertex_count = 0, triangle_count = 0;
  uint32_t thruster_count = 0, hull_count = 0, hull_vertex_count = 0,
           fault_count = 0;
  uint32_t module_count = 0, command_ticks = 0, command_width = 0,
           source_frame = 0;
  uint64_t case_seed = 0;
  uint32_t generator_version = 0, reserved = 0;
  double water_density = 1025;
  SimVec3 gravity{0, 0, 9.81}, current{};
  CameraRec camera{};
  BoundsRec bounds{};
  char source_kind[32]{};
  char content_hash[64]{};
  char scene_name[64]{};
};
struct Scene {
  using FaultKind = sim::FaultKind;
  SceneHeader header{};
  std::vector<BodyRec> bodies;
  std::vector<ShapeRec> shapes;
  std::vector<SimVec3> vertices;
  std::vector<SimTriangle> triangles;
  std::vector<ThrusterRec> thrusters;
  std::vector<HullRec> hulls;
  std::vector<SimVec3> hull_vertices;
  std::vector<FaultRec> faults;
  std::vector<NameRec> modules;
  std::vector<double> commands;
  std::string path;

  Scene();
  void validate() const;
  bool from_blender() const;
  std::string content_hash() const;
  std::vector<uint8_t> binary() const;
  uint64_t fingerprint() const;
  static Scene read(const void *data, size_t n);
  static Scene read_prefix(const void *data, size_t n, size_t *used);
  static Scene load(const std::string &path);
  static Scene load_fd(int fd);
  static Scene load_blend(const std::string &path);
};
std::vector<std::array<SimVec3, 3>>
surface(uint32_t kind, SimVec3 size, const SimVec3 *vertices,
        const SimTriangle *triangles, uint32_t vertex_count,
        uint32_t triangle_count);

static_assert(std::endian::native == std::endian::little);
static_assert(sizeof(RotorSpec) == 64);
static_assert(sizeof(BoundsRec) == 24);
static_assert(sizeof(CameraRec) == 80 && offsetof(CameraRec, horizontal_fov) == 16);
static_assert(sizeof(NameRec) == 32);
static_assert(sizeof(BodyRec) == 328 && offsetof(BodyRec, mass) == 88);
static_assert(sizeof(ShapeRec) == 168 && offsetof(ShapeRec, friction) == 136);
static_assert(sizeof(ThrusterRec) == 120 && offsetof(ThrusterRec, spec) == 56);
static_assert(sizeof(HullRec) == 184 && offsetof(HullRec, volume) == 176);
static_assert(sizeof(FaultRec) == 32 && offsetof(FaultRec, magnitude) == 24);
static_assert(sizeof(SceneHeader) == 408 && offsetof(SceneHeader, water_density) == 88);

void copy_text(char *dst, size_t n, const char *src);
} // namespace sim
