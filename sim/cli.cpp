#include "runtime.hpp"
#include <charconv>
#include <iostream>
using namespace sim;
uint64_t number(const char *s) {
  uint64_t result;
  auto end = s + std::strlen(s);
  auto [p, e] = std::from_chars(s, end, result);
  require(e == std::errc() && p == end, "expected unsigned integer");
  return result;
}
int main(int argc, char **argv) {
  try {
    if (argc == 4 && std::string(argv[1]) == "snapshot") {
      auto data=Scene::load(argv[2]).binary();
      std::ofstream output(argv[3],std::ios::binary);
      require(bool(output),"cannot write binary scene snapshot");
      output.write(reinterpret_cast<const char *>(data.data()),data.size());
      require(bool(output),"binary scene snapshot write failed");
      return 0;
    }
    if (argc == 3 && std::string(argv[1]) == "validate") {
      auto scene = Scene::load(argv[2]);
      std::cout << Json{{"status", "valid"}, {"bodies", scene.model.bodies.size()},
                        {"fingerprint", scene.fingerprint()}, {"source", scene.document.value("source", Json::object())}} << '\n';
      return 0;
    }
    require(
        argc >= 5,
        "usage: simctl run <backend.so> <scene.blend|snapshot.asim|scenario.json> <trace.jsonl> [ticks] "
        "[seed] | simctl replay <backend.so> <trace.jsonl> <output.jsonl>");
    auto backend = std::make_shared<Backend>(argv[2]);
    std::string operation = argv[1];
    Json header;
    std::vector<Json> replay;
    uint64_t ticks = argc > 5 ? number(argv[5]) : 21600,
             seed = argc > 6 ? number(argv[6]) : 0;
    bool generated = argc > 6;
    if (operation == "replay") {
      std::ifstream file(argv[3]);
      require(bool(file), "cannot open replay trace");
      std::string line;
      require(bool(std::getline(file, line)), "empty trace");
      header = Json::parse(line);
      require(header.at("version") == 1, "unsupported trace version");
      require(header.at("backend") == backend->api.name &&
                  header.at("backend_version") == backend->api.version,
              "replay requires matching backend and version");
      require(header.at("build_fingerprint") == backend->build_fingerprint,
              "replay requires matching adapter and runtime builds");
      while (std::getline(file, line)) {
        auto j = Json::parse(line);
        if (j.value("t", "") == "sample")
          replay.push_back(j);
      }
      require(!replay.empty(), "trace has no samples");
      ticks = replay.size() - 1;
    } else {
      require(operation == "run", "unknown operation");
      auto scene = Scene::load(argv[3]);
      if (generated && !(argc > 7 && std::string(argv[7]) == "--resolved"))
        scene = Scene::parse(draw_case(scene.document, seed), argv[3]);
      header = {{"t", "header"},
                {"version", 1},
                {"backend", backend->api.name},
                {"backend_version", backend->api.version},
                {"build_fingerprint", backend->build_fingerprint},
                {"scenario", scene.document},
                {"fingerprint", scene.fingerprint()},
                {"seed", seed},
                {"generated_commands", generated},
                {"ticks", ticks}};
    }
    World world(Scene::parse(header.at("scenario")), backend);
    std::ofstream output(argv[4]);
    require(bool(output), "cannot open output trace");
    output << header << '\n';
    auto record = [&] {
      auto sample = world.sample();
      if (operation == "replay") {
        auto &expected = replay.at(world.tick);
        require(expected.at("step") == world.tick,
                "nonconsecutive replay trace");
        require(expected.at("hash") == sample.at("hash"),
                "deterministic replay diverged");
      }
      output << sample << '\n';
    };
    record();
    for (uint64_t i = 0; i < ticks; ++i) {
      std::vector<double> commands;
      if (operation == "replay")
        commands =
            replay.at(i + 1).at("input_commands").get<std::vector<double>>();
      else if (world.scene.document.contains("command_sequence"))
        commands = world.scene.document.at("command_sequence")
                       .at(i)
                       .get<std::vector<double>>();
      else
        commands = generated ? generated_commands(world.scene, seed, i)
                             : std::vector<double>(world.commands.size(), 0);
      world.step(commands);
      record();
    }
    output << Json{{"t", "end"}, {"step", world.tick}, {"reason", "complete"}}
           << '\n';
    std::cout << Json{{"backend", backend->api.name},
                      {"ticks", world.tick},
                      {"hash", world.state_hash()},
                      {"status", "passed"}}
              << '\n';
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "simctl: " << e.what() << '\n';
    return 1;
  }
}
