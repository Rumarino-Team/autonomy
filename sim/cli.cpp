#include "runtime.hpp"
#include <algorithm>
#include <charconv>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
using namespace sim;
using Json = nlohmann::json;
uint64_t number(const char *s) {
  uint64_t result;
  auto end = s + std::strlen(s);
  auto [p, e] = std::from_chars(s, end, result);
  require(e == std::errc() && p == end, "expected unsigned integer");
  return result;
}
void write_snapshot(const Scene &scene, const char *path) {
  auto data = scene.binary();
  std::ofstream output(path, std::ios::binary);
  require(bool(output), "cannot write binary scene snapshot");
  output.write(reinterpret_cast<const char *>(data.data()), data.size());
  require(bool(output), "binary scene snapshot write failed");
}
int main(int argc, char **argv) {
  std::string backend_name;
  uint64_t steps = 0, hash = 0;
  std::unique_ptr<World> world;
  try {
    if (argc == 4 && std::string(argv[1]) == "snapshot") {
      write_snapshot(Scene::load(argv[2]), argv[3]);
      return 0;
    }
    if (argc == 5 && std::string(argv[1]) == "resolve") {
      write_snapshot(draw_case(Scene::load(argv[2]), number(argv[4])), argv[3]);
      return 0;
    }
    if (argc == 3 && std::string(argv[1]) == "validate") {
      auto scene = Scene::load(argv[2]);
      std::cout << Json{{"status", "valid"},
                        {"bodies", scene.bodies.size()},
                        {"fingerprint", scene.fingerprint()},
                        {"source",
                         {{"kind", scene.header.source_kind},
                          {"content_hash", scene.content_hash()},
                          {"scene", scene.header.scene_name},
                          {"frame", scene.header.source_frame}}}}
                << '\n';
      return 0;
    }
    require(argc >= 4 && std::string(argv[1]) == "run",
            "usage: simctl run <backend.so> <scene.blend|snapshot.asim> [ticks] "
            "[seed] [--resolved] | simctl resolve <scene.asim> <out.asim> <seed>");
    auto backend = std::make_shared<Backend>(argv[2]);
    backend_name = backend->api.name;
    uint64_t ticks = argc > 4 ? number(argv[4]) : 21600,
             seed = argc > 5 ? number(argv[5]) : 0;
    bool generated = argc > 5;
    auto scene = Scene::load(argv[3]);
    if (generated && !(argc > 6 && std::string(argv[6]) == "--resolved"))
      scene = draw_case(std::move(scene), seed);
    world = std::make_unique<World>(std::move(scene), backend);
    for (uint64_t i = 0; i < ticks; ++i) {
      steps = i + 1;
      std::vector<double> commands(world->commands.size(), 0);
      if (world->scene.header.command_ticks) {
        require(i < world->scene.header.command_ticks, "command sequence ended");
        require(world->scene.header.command_width == commands.size(),
                "command sequence width does not match the thrusters");
        std::copy_n(world->scene.commands.begin() +
                        i * world->scene.header.command_width,
                    commands.size(), commands.begin());
      } else if (generated)
        commands = generated_commands(world->scene, seed, i);
      world->step(commands);
    }
    steps = world->tick;
    hash = world->state_hash();
    std::cout << Json{{"backend", backend_name},
                      {"steps", steps},
                      {"hash", hash},
                      {"status", "passed"}}
              << '\n';
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "simctl: " << e.what() << '\n';
    if (!backend_name.empty()) {
      if (world)
        hash = world->state_hash();
      std::cout << Json{{"backend", backend_name},
                        {"steps", steps},
                        {"hash", hash},
                        {"status", "failed"},
                        {"error", e.what()}}
                << '\n';
    }
    return 1;
  }
}
