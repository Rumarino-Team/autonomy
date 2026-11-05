#include "forces.hpp"
#include <iostream>
#include <numbers>
using namespace sim;
int main(int argc, char **argv) {
  try {
    require(argc == 2, "scenario required");
    auto scene = Scene::load(argv[1]);
    std::vector<SimBodyState> bodies(scene.bodies.size());
    for (size_t i = 0; i < bodies.size(); ++i) {
      bodies[i].pose = scene.bodies[i].initial_pose;
      bodies[i].pose.orientation = {0, 0, 0, 1};
      bodies[i].pose.position.z = 2;
      bodies[i].center_of_mass = point(bodies[i].pose, scene.bodies[i].center_of_mass.position);
    }
    ForceRegistry registry;
    for (double dt : {1.0 / 90, 1.0 / 360, 1.0 / 720}) {
      std::vector<RotorState> rotors(scene.thrusters.size());
      std::vector<double> commands(rotors.size(), .5);
      std::vector<SimWrench> fluid(bodies.size()), w(bodies.size());
      std::map<std::string, std::vector<double>> state;
      ForceContext context{scene, bodies, commands, 0, dt, rotors, fluid, state};
      const double target = scene.thrusters[0].spec.command_sign * .5 * scene.thrusters[0].spec.max_rpm / 60 * 2 * std::numbers::pi;
      for (int phase = 0; phase < 3; ++phase) {
        double goal = phase == 0 ? target : phase == 1 ? -target : 0;
        std::fill(commands.begin(), commands.end(), phase == 0 ? .5 : phase == 1 ? -.5 : 0);
        for (int i = 0; i < int(2 / dt); ++i) {
          std::fill(w.begin(), w.end(), SimWrench{});
          registry.get("thrusters")(context, w);
          for (auto &r : rotors)
            require(std::isfinite(r.omega) && std::abs(r.omega) < 1.25 * std::abs(target), "unstable rotor speed response");
        }
        for (auto &r : rotors)
          require(std::abs(r.omega - goal) < .01, "rotor PI failed to settle after a speed step");
      }
    }
    std::cout << "rotor speed steps, reversals and stop settle at 90, 360 and 720 Hz\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
