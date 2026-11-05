#pragma once
#include "image.hpp"
#include <functional>
#include <map>
namespace sim {
struct RotorState {
  double omega = 0, integral = 0, torque = 0, thrust = 0;
};
struct ForceContext {
  const Scene &scene;
  const std::vector<SimBodyState> &bodies;
  const std::vector<double> &commands;
  uint64_t tick;
  double dt;
  std::vector<RotorState> &rotors;
  std::vector<SimWrench> &fluid_cache;
  std::map<std::string, std::vector<double>> &state;
};
using ForceFunction =
    std::function<void(ForceContext &, std::vector<SimWrench> &)>;
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
