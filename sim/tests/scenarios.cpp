#include "runtime.hpp"
#include "blend/robot_registry.hpp"
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <nlohmann/json.hpp>
using namespace sim;
using Json = nlohmann::json;
SimVec3 vec(const Json &j) {
  require(j.is_array() && j.size() == 3, "expected vector of length 3");
  return {j[0].get<double>(), j[1].get<double>(), j[2].get<double>()};
}
SimQuat quat(const Json &j) {
  require(j.is_array() && j.size() == 4, "expected xyzw quaternion");
  return {j[0].get<double>(), j[1].get<double>(), j[2].get<double>(),
          j[3].get<double>()};
}
int main(int argc, char **argv) {
  try {
    require(argc == 3, "backend and scenario required");
    auto backend = std::make_shared<Backend>(argv[1]);
    auto source_owner = Main::load(argv[2]);
    auto &source = *source_owner;
    if (source.robot.definition) {
      auto check_storage = [](const Main &model) {
        const auto &r = model.robot;
        require(r.object_registered &&
                    &model.objects.block(r.object.id.index) == &r.object &&
                    !r.object.id.pooled,
                "robot body escaped its embedded storage");
        require(r.object.colliders.data() == r.colliders.data() &&
                    r.object.colliders.size() == r.shape_count,
                "robot colliders escaped their embedded array");
        for (const auto &c : r.object.colliders)
          require(c.object == &r.object, "robot collider has a foreign object");
        require(r.thrusters().data() == r.motors.data() &&
                    r.hulls().data() == r.fluid_surfaces.data(),
                "robot hardware escaped its fixed arrays");
      };
      check_storage(source);
      auto copy = source.clone();
      copy->finalize();
      check_storage(*copy);
      require(&copy->robot.object != &source.robot.object &&
                  copy->robot.thrusters().data() != source.robot.thrusters().data(),
              "robot clone shares mutable hardware");
      auto bytes = source.binary();
      auto decoded = Main::read(bytes.data(), bytes.size());
      check_storage(*decoded);
      require(decoded->binary() == bytes, "fixed robot binary round trip changed physics");
      auto definition = *source.robot.definition;
      const auto hash = robot_configuration_hash(definition);
      definition.body.mass += 1;
      require(robot_configuration_hash(definition) != hash,
              "typed robot hash ignored its physical values");
    }
    auto trajectory = [&](const Main &scene) {
      World world(scene, backend);
      std::vector<uint64_t> result;
      for (int i = 0; i < 360; ++i) {
        world.step(std::vector<double>(world.commands.size(), 0));
        result.push_back(world.state_hash());
      }
      return result;
    };
    auto hashes = trajectory(source);
    require(hashes == trajectory(source), "vehicle replay diverged");
    World world(source, backend);
    auto scenario = std::filesystem::path(argv[2]);
    std::ifstream baseline(scenario.parent_path() /
                           (scenario.stem().string() + ".json.baseline.json"));
    Json reference;
    if (std::string(backend->api.name) == "mujoco" && baseline)
      baseline >> reference;
    for (int i = 0; i < 360; ++i) {
      world.step(std::vector<double>(world.commands.size(), 0));
      if (!reference.empty() && (i + 1) % 36 == 0) {
        auto &sample = reference["samples"][i / 36];
        auto &body = world.bodies[world.scene.header.camera.body];
        require(norm(body.pose.position - vec(sample.at("position"))) < 1e-8,
                "vehicle position changed from legacy MuJoCo baseline");
        auto q = quat(sample.at("orientation"));
        double difference = std::abs(q.x - body.pose.orientation.x) +
                            std::abs(q.y - body.pose.orientation.y) +
                            std::abs(q.z - body.pose.orientation.z) +
                            std::abs(q.w - body.pose.orientation.w);
        require(difference < 1e-8,
                "vehicle attitude changed from legacy MuJoCo baseline");
      }
    }
    world.reset();
    for (int i = 0; i < 360; ++i) {
      world.step(std::vector<double>(world.commands.size(), 0));
      require(hashes[i] == world.state_hash(), "vehicle cold reset diverged");
    }
    std::vector<std::future<std::vector<uint64_t>>> jobs;
    for (int i = 0; i < 4; ++i)
      jobs.push_back(
          std::async(std::launch::async, [&] { return trajectory(source); }));
    for (auto &job : jobs)
      require(job.get() == hashes, "vehicle parallel replay diverged");
    if (backend->api.capabilities & SIM_CAP_NATIVE_HYDRO) {
      auto native_owner = source.clone();
      auto &native = *native_owner;
      native.header.native_hydrodynamics = 1;
      auto h = trajectory(native);
      require(h == trajectory(native), "native hydrodynamics replay diverged");
    }
    // Exercise every authored motor, not only a passive zero-command replay.
    // Repeat from a cold reset so rotor PI state and engine state must agree.
    for (size_t motor = 0; motor < source.robot.thrusters().size(); ++motor) {
      uint64_t expected = 0;
      for (int repeat = 0; repeat < 2; ++repeat) {
        world.reset();
        std::vector<double> commands(source.robot.thrusters().size(), 0);
        commands[motor] = .35;
        for (int tick = 0; tick < 36; ++tick)
          world.step(commands);
        require(std::isfinite(world.rotors[motor].thrust) &&
                    std::abs(world.rotors[motor].thrust) > 1e-8,
                "authored thruster did not produce thrust");
        if (repeat == 0)
          expected = world.state_hash();
        else
          require(world.state_hash() == expected,
                  "commanded vehicle cold reset diverged");
      }
    }
    std::cout << backend->api.name << " "
              << source.objects[source.header.camera.body].name
              << " qualification passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
