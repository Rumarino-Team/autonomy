#pragma once
#include "forces.hpp"
#include <memory>
namespace sim {
class Backend {
  void *library_ = nullptr;

public:
  SimBackendApi api{};
  uint64_t build_fingerprint = 0;
  std::string path;
  explicit Backend(const std::string &path);
  ~Backend();
  Backend(const Backend &) = delete;
  Backend &operator=(const Backend &) = delete;
  void check(SimResult result, const char *error) const;
};
struct ObjectObservation {
  uint32_t id, cls;
  SimPose pose;
  SimVec3 size;
  uint32_t x0 = 0, y0 = 0, x1 = 0, y1 = 0;
  bool visible = false;
};
struct Observation {
  SimPose camera;
  SimQuat imu;
  SimVec3 gyro{}, accel{};
  double depth = 0;
  bool position_ok = true, attitude_ok = true, gyro_ok = true, imu_ok = true,
       depth_ok = true;
  // Borrowed until the next observe() call or destruction of the World.
  std::span<const ObjectObservation> objects;
};
class World {
  MainPtr model_;
  Memory memory_;
  std::shared_ptr<Backend> backend_;
  void *world_ = nullptr;
  ForceRegistry registry_;
  std::pmr::vector<std::string> modules_;
  ModuleState module_state_;
  WrenchBuffer fluid_cache_;
  std::pmr::vector<SimVec3> previous_velocity_, previous_angular_velocity_;
  mutable std::pmr::vector<ObjectObservation> observation_objects_;
  std::array<RotorState, Robot::max_thrusters> rotor_storage_{};
  std::array<double, Robot::max_thrusters> command_storage_{}, input_storage_{};
  void read();

public:
  const Main &scene;
  std::pmr::vector<SimBodyState> bodies;
  std::pmr::vector<SimContact> contacts;
  std::span<RotorState> rotors;
  std::span<double> commands, input_commands;
  WrenchBuffer wrenches;
  uint64_t tick = 0;
  explicit World(
      MainPtr scene, std::shared_ptr<Backend> backend,
      ForceRegistry registry = ForceRegistry{},
      std::pmr::memory_resource *upstream = std::pmr::get_default_resource());
  World(const Main &scene, std::shared_ptr<Backend> backend,
        ForceRegistry registry = ForceRegistry{},
        std::pmr::memory_resource *upstream = std::pmr::get_default_resource());
  ~World();
  World(const World &) = delete;
  World &operator=(const World &) = delete;
  void reset(std::span<const SimBodyState> state = {});
  void step(std::span<const double> commands);
  void step(std::initializer_list<double> commands) {
    step(std::span(commands.begin(), commands.size()));
  }
  double dt() const {
    return double(scene.header.dt_numerator) / scene.header.dt_denominator;
  }
  uint64_t timestamp_ns() const;
  uint64_t state_hash() const;
  Observation observe() const;
  const Backend &backend() const { return *backend_; }
  std::shared_ptr<Backend> backend_handle() const { return backend_; }
};
std::vector<double> generated_commands(const Main &, uint64_t seed,
                                       uint64_t tick);
MainPtr draw_case(const Main &scene, uint64_t seed);
} // namespace sim
