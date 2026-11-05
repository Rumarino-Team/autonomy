#include "runtime.hpp"
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
    auto source = Scene::load(argv[2]);
    auto trajectory = [&](Scene scene) {
      World world(std::move(scene), backend);
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
      auto native = source;
      native.header.native_hydrodynamics = 1;
      auto h = trajectory(native);
      require(h == trajectory(native), "native hydrodynamics replay diverged");
    }
    std::cout << backend->api.name << " "
              << source.bodies[source.header.camera.body].name
              << " qualification passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
