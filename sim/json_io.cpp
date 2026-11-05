#include "runtime.hpp"
#include <algorithm>
#include <numbers>
namespace sim {
Scene draw_case(Scene scene, uint64_t seed) {
  auto u = [&](uint64_t channel, double lo, double hi) {
    return lo + (hi - lo) * random(seed, channel);
  };
  for (auto &b : scene.bodies) {
    uint64_t channel = uint64_t(b.id) * 32;
    if (b.motion != SIM_DYNAMIC && !b.fuzz)
      continue;
    b.initial_pose.position =
        b.initial_pose.position +
        SimVec3{u(channel, -.2, .2), u(channel + 1, -.2, .2),
                u(channel + 2, -.1, .1)};
    b.initial_pose.orientation =
        multiply(b.initial_pose.orientation,
                 euler(u(channel + 3, -.05, .05), u(channel + 4, -.05, .05),
                       u(channel + 5, -.15, .15)));
    if (b.motion == SIM_DYNAMIC) {
      b.dry_mass *= u(channel + 6, .97, 1.03);
      b.volume *= u(channel + 7, .97, 1.03);
      b.initial_linear_velocity =
          b.initial_linear_velocity +
          SimVec3{u(channel + 8, -.2, .2), u(channel + 9, -.2, .2),
                  u(channel + 10, -.2, .2)};
      b.initial_angular_velocity =
          b.initial_angular_velocity +
          SimVec3{u(channel + 11, -.2, .2), u(channel + 12, -.2, .2),
                  u(channel + 13, -.2, .2)};
    }
  }
  double speed = u(900, 0, .15),
         angle = u(901, -std::numbers::pi, std::numbers::pi);
  scene.header.current = {speed * std::cos(angle), speed * std::sin(angle), 0};
  for (auto &h : scene.hulls) {
    uint32_t id = scene.bodies[h.body].id;
    double scale = u(910 + id, .7, 1.4);
    h.drag = h.drag * scale;
    h.skin = h.skin * scale;
    h.volume *= u(uint64_t(id) * 32 + 7, .97, 1.03);
  }
  for (size_t i = 0; i < scene.thrusters.size(); ++i) {
    auto &spec = scene.thrusters[i].spec;
    uint64_t channel = 20000 +
                       uint64_t(scene.bodies[scene.thrusters[i].body].id) * 4096 +
                       i * 8;
    double scale = u(channel, .8, 1.2);
    spec.thrust *= scale;
    spec.torque *= scale;
    spec.inertia *= u(channel + 1, .7, 1.5);
  }
  if (!scene.thrusters.empty()) {
    uint64_t start = uint64_t(u(930, 36, 360));
    FaultRec fault;
    fault.kind = random(seed, 933) < .25 ? FaultKind::Dead : FaultKind::Weak;
    fault.start = start;
    fault.end = start + 180;
    fault.thruster = uint32_t(u(931, 0, scene.thrusters.size()));
    fault.magnitude = u(932, .3, .7);
    scene.faults.push_back(fault);
  }
  const FaultKind sensor[] = {FaultKind::Tracking, FaultKind::Imu,
                              FaultKind::Gyro, FaultKind::Depth};
  FaultRec sensed;
  sensed.kind = sensor[std::min(3u, uint32_t(random(seed, 940) * 4))];
  sensed.start = 360;
  sensed.end = 720;
  sensed.magnitude = u(941, -.05, .05);
  scene.faults.push_back(sensed);
  scene.header.case_seed = seed;
  scene.header.generator_version = 1;
  return scene;
}
} // namespace sim
