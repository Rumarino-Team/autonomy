#pragma once
// Physics values are embedded in pooled datablocks. Wire fields are encoded
// explicitly; neither object layout nor intrusive pointers are serialized.
#include "arena.hpp"
#include "database.hpp"
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
namespace sim {
inline constexpr char scene_magic[8] = {'A', 'S', 'I', 'M', 'S', 'C', '0', '3'};
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
// Physical values and cached flat transport ranges; ownership lives in
// ObjectBlock.
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
// Collider values; geometry ranges are draft/serialized fields, not ownership.
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
  uint32_t body_count = 0, shape_count = 0, vertex_count = 0,
           triangle_count = 0;
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
// Geometry has no datablock identity. Each collider holds its own small view;
// identical views point into the same immutable model-arena buffers.
struct MeshData {
  std::span<const SimVec3> positions;
  std::span<const SimTriangle> triangles;
};
struct ObjectBlock;
struct Collider {
  ShapeRec data;
  MeshData geometry{};
  // Parent, never a separate registry entry.
  const ObjectBlock *object = nullptr;
};
struct ObjectBlock {
  using Value = BodyRec;
  DataBlock id;
  BodyRec data;
  // Owned arena array or embedded Robot slots.
  std::span<const Collider> colliders;
};
struct FaultBlock {
  using Value = FaultRec;
  DataBlock id;
  FaultRec data;
  const ThrusterRec *thruster = nullptr;
};
struct ModuleBlock {
  using Value = NameRec;
  DataBlock id;
  NameRec data;
};
struct RunSettings {
  Records<FaultBlock, DataKind::Fault> faults;
  Records<ModuleBlock, DataKind::Module> modules;
  RunSettings(Memory &memory, const void *owner, const bool &frozen)
      : faults(memory, owner, frozen), modules(memory, owner, frozen) {}
};
struct RobotDefinition;
// One non-modular vehicle instance. Parts live at fixed addresses inside this
// struct; there are no per-part allocations or owning component vectors. The
// object registry borrows its embedded object; colliders have no registry.
struct Robot {
  Robot() = default;
  Robot(const Robot &) = delete;
  Robot &operator=(const Robot &) = delete;
  Robot(Robot &&) = delete;
  Robot &operator=(Robot &&) = delete;
  static constexpr size_t max_shapes = 5, max_thrusters = 8, max_hulls = 5;
  const RobotDefinition *definition = nullptr;
  ObjectBlock object;
  std::array<Collider, max_shapes> colliders{};
  std::array<ThrusterRec, max_thrusters> motors{};
  std::array<HullRec, max_hulls> fluid_surfaces{};
  std::array<std::span<const SimVec3>, max_hulls> surface_positions{};
  CameraRec camera{};
  size_t shape_count = 0, motor_count = 0, hull_count = 0;
  bool object_registered = false;
  bool frozen = false;
  auto thrusters() { return std::span(motors).first(motor_count); }
  auto thrusters() const { return std::span(motors).first(motor_count); }
  auto hulls() { return std::span(fluid_surfaces).first(hull_count); }
  auto hulls() const { return std::span(fluid_surfaces).first(hull_count); }
  void add_thruster(const ThrusterRec &value) {
    require(!frozen, "database is frozen");
    require(motor_count < motors.size(),
            "fixed robot supports at most eight thrusters");
    motors[motor_count++] = value;
  }
  void add_hull(const HullRec &value) {
    require(!frozen, "database is frozen");
    require(hull_count < fluid_surfaces.size(),
            "fixed robot supports at most five hulls");
    fluid_surfaces[hull_count++] = value;
  }
};
class Main;
using MainPtr = std::unique_ptr<Main>;
class Main {
  // Declaration order is deliberate: tables and their destructors run before
  // the allocator resources. A Main is never relocated underneath its nodes.
  Memory memory_;
  bool frozen_ = false;
  mutable SimSceneView view_{};
  // Final immutable geometry has its own typed buffers, independent of PMR.
  ArenaBuffer<SimVec3> mesh_vertices_{max_scene_bytes / sizeof(SimVec3)};
  ArenaBuffer<SimTriangle> mesh_triangles_{max_scene_bytes /
                                           sizeof(SimTriangle)};
  ArenaBuffer<SimVec3> hull_positions_{max_scene_bytes / sizeof(SimVec3)};
  // Ordered views serve validation and transport; they own no mesh records.
  std::pmr::vector<MeshData> geometry_;
  std::pmr::unordered_map<const SimVec3 *, uint32_t> geometry_lookup_;
  std::pmr::unordered_map<uint64_t, const DataBlock *> lookup_;

public:
  using FaultKind = sim::FaultKind;
  SceneHeader header{};
  Robot robot;
  Records<ObjectBlock, DataKind::Object> objects;
  // Fault schedules and force selection belong to a run, not robot hardware.
  RunSettings run;
  // Draft-only geometry staging. Finalization interns immutable arena buffers
  // and releases staging capacity; colliders already live in their object
  // storage.
  // Draft ranges use u32 indices. Reserving their address range commits no
  // pages; only appended values acquire writable storage. Repeated input meshes
  // may exceed the final binary limit before geometry interning removes copies.
  ArenaBuffer<SimVec3> vertices{UINT32_MAX};
  ArenaBuffer<SimTriangle> triangles{UINT32_MAX};
  ArenaBuffer<SimVec3> hull_vertices{UINT32_MAX};
  std::pmr::vector<double> commands;
  std::string path;
  explicit Main(
      std::pmr::memory_resource *upstream = std::pmr::get_default_resource(),
      std::pmr::memory_resource *model_upstream = nullptr);
  ~Main() = default;
  Main(const Main &) = delete;
  Main &operator=(const Main &) = delete;
  Main(Main &&) = delete;
  Main &operator=(Main &&) = delete;
  Memory &memory() { return memory_; }
  bool frozen() const { return frozen_; }
  void finalize();
  // Adds one complete rigid object. Only the fixed robot uses embedded storage.
  ObjectBlock &add_object(const BodyRec &, std::span<const Collider>,
                          bool fixed_robot = false);
  Collider &edit_collider(size_t object, size_t collider);
  size_t collider_count() const;
  std::span<const MeshData> geometry() const { return geometry_; }
  uint32_t geometry_index(const Collider &) const;
  void validate_graph() const;
  const DataBlock *find(DataKind kind, uint32_t uid) const;
  size_t body_index(uint32_t id) const;
  MainPtr clone() const; // returns an independent mutable draft, fixing
                         // references on finalize
  std::span<const SimVec3> positions(const Collider &) const;
  std::span<const SimTriangle> faces(const Collider &) const;
  std::span<const SimVec3> positions(const HullRec &) const;
  void validate() const;
  bool from_blender() const;
  std::string content_hash() const;
  std::vector<uint8_t> binary() const;
  uint64_t fingerprint() const;
  SimSceneView view() const;
  static MainPtr from_view(const SimSceneView &);
  static MainPtr read(const void *data, size_t n);
  static MainPtr load(const std::string &path);
  static MainPtr load_fd(int fd);
  static MainPtr load_blend(const std::string &path);
};
// A builder owns a draft. finish() publishes a complete, frozen database;
// failure leaves ownership in the builder, so partially linked graphs unwind.
class MainBuilder {
  MainPtr draft_;

public:
  explicit MainBuilder(
      std::pmr::memory_resource *r = std::pmr::get_default_resource())
      : draft_(std::make_unique<Main>(r)) {}
  explicit MainBuilder(const Main &source) : draft_(source.clone()) {}
  Main &draft() {
    require(bool(draft_), "builder has already finished");
    return *draft_;
  }
  MainPtr finish() {
    draft().finalize();
    return std::move(draft_);
  }
};
std::vector<std::array<SimVec3, 3>> surface(uint32_t kind, SimVec3 size,
                                            const SimVec3 *vertices,
                                            const SimTriangle *triangles,
                                            uint32_t vertex_count,
                                            uint32_t triangle_count);

static_assert(std::endian::native == std::endian::little);
static_assert(sizeof(RotorSpec) == 64);
static_assert(sizeof(BoundsRec) == 24);
static_assert(sizeof(CameraRec) == 80 &&
              offsetof(CameraRec, horizontal_fov) == 16);
static_assert(sizeof(NameRec) == 32);
static_assert(sizeof(BodyRec) == 328 && offsetof(BodyRec, mass) == 88);
static_assert(sizeof(ShapeRec) == 168 && offsetof(ShapeRec, friction) == 136);
static_assert(sizeof(ThrusterRec) == 120 && offsetof(ThrusterRec, spec) == 56);
static_assert(sizeof(HullRec) == 184 && offsetof(HullRec, volume) == 176);
static_assert(sizeof(FaultRec) == 32 && offsetof(FaultRec, magnitude) == 24);
static_assert(sizeof(SceneHeader) == 408 &&
              offsetof(SceneHeader, water_density) == 88);

void copy_text(char *dst, size_t n, const char *src);
} // namespace sim
