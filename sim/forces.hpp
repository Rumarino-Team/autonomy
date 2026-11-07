#pragma once
#include "image.hpp"
#include <functional>
#include <map>
namespace sim {
struct RotorState {
  double omega = 0, integral = 0, torque = 0, thrust = 0;
};
using WrenchBuffer = std::pmr::vector<SimWrench>;
using ModuleState = std::pmr::map<std::pmr::string, std::pmr::vector<double>>;
struct ForceContext {
  const Main &scene;
  std::span<const SimBodyState> bodies;
  std::span<const double> commands;
  uint64_t tick;
  double dt;

  std::span<RotorState> rotors;
  WrenchBuffer &fluid_cache;
  ModuleState &state;
  std::pmr::memory_resource *scratch = std::pmr::get_default_resource();
};
using ForceFunction = std::function<void(ForceContext &, WrenchBuffer &)>;
class ForceRegistry {
  struct Entry {
    ForceFunction evaluate;
    std::string contribution;
  };
  std::map<std::string, Entry> functions_;

public:
  ForceRegistry();
  // A nonempty contribution names an exclusive physical provider (for example
  // hydrodynamics).
  void add(std::string name, ForceFunction function,
           std::string contribution = {});
  const std::string &contribution(const std::string &name) const;
  const ForceFunction &get(const std::string &name) const;
};
void add_force(SimWrench &wrench, SimVec3 force, SimVec3 application_point,
               SimVec3 center_of_mass);
} // namespace sim
