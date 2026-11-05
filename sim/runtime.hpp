#pragma once
#include "forces.hpp"
#include <memory>
namespace sim {
class Backend {
  void *library_ = nullptr;

public:
  SimBackendApi api{};
  uint64_t build_fingerprint = 0;
  std::filesystem::path path;
  explicit Backend(const std::filesystem::path &path);
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
  std::vector<ObjectObservation> objects;
};
class World {
  std::shared_ptr<Backend> backend_;
  void *model_ = nullptr;
  void *world_ = nullptr;
  ForceRegistry registry_;
  std::vector<std::string> modules_;
  std::map<std::string, std::vector<double>> module_state_;
  std::vector<SimWrench> fluid_cache_;
  std::vector<SimVec3> previous_velocity_, previous_angular_velocity_;
  void read();

public:
  Scene scene;
  std::vector<SimBodyState> bodies;
  std::vector<SimContact> contacts;
  std::vector<RotorState> rotors;
  std::vector<double> commands, input_commands;
  std::vector<SimWrench> wrenches;
  uint64_t tick = 0;
  explicit World(Scene scene, std::shared_ptr<Backend> backend,
                 ForceRegistry registry = ForceRegistry{});
  ~World();
  World(const World &) = delete;
  World &operator=(const World &) = delete;
  void reset(const std::vector<SimBodyState> &state = {});
  void step(const std::vector<double> &commands);
  double dt() const {
    return double(scene.model.value.dt_numerator) /
           scene.model.value.dt_denominator;
  }
  uint64_t timestamp_ns() const;
  uint64_t state_hash() const;
  Observation observe() const;
  Json sample() const;
  const Backend &backend() const { return *backend_; }
};
Json draw_case(Json source, uint64_t seed);
std::vector<double> generated_commands(const Scene &, uint64_t seed,
                                       uint64_t tick);
} // namespace sim
