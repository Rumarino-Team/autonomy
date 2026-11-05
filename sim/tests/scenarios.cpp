#include "runtime.hpp"
#include <future>
#include <iostream>
using namespace sim;
int main(int argc, char **argv) {
  try {
    require(argc == 3, "backend and scenario required");
    auto backend = std::make_shared<Backend>(argv[1]);
    auto source = Scene::load(argv[2]).document;
    auto trajectory = [&](const Json &document) {
      World world(Scene::parse(document), backend);
      std::vector<uint64_t> result;
      for (int i = 0; i < 360; ++i) {
        world.step(std::vector<double>(world.commands.size(), 0));
        result.push_back(world.state_hash());
      }
      return result;
    };
    auto hashes = trajectory(source);
    require(hashes == trajectory(source), "vehicle replay diverged");
    World world(Scene::parse(source), backend);
    std::ifstream baseline(std::string(argv[2]) + ".baseline.json");
    Json reference;
    if (std::string(backend->api.name) == "mujoco" && baseline)
      baseline >> reference;
    for (int i = 0; i < 360; ++i) {
      world.step(std::vector<double>(world.commands.size(), 0));
      if (!reference.empty() && (i + 1) % 36 == 0) {
        auto &sample = reference["samples"][i / 36];
        auto &body = world.bodies[world.scene.camera.body];
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
      native["native_hydrodynamics"] = true;
      auto h = trajectory(native);
      require(h == trajectory(native), "native hydrodynamics replay diverged");
    }
    std::cout << backend->api.name << " " << source.value("vehicle", "vehicle")
              << " qualification passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
