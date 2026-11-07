#include "runtime.hpp"
#include "blend/robot_registry.hpp"
#include <algorithm>
#include <charconv>
#include <cstring>
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
void write_stdout(const Main &scene) {
  auto data = scene.binary();
  std::cout.write(reinterpret_cast<const char *>(data.data()), data.size());
  std::cout.flush();
  require(bool(std::cout), "cannot write resolved scene");
}
void print_valid(const Main &scene) {
  std::cout << Json{{"status", "valid"},
                    {"bodies", scene.objects.size()},
                    {"fingerprint", scene.fingerprint()},
                    {"source",
                     {{"kind", scene.header.source_kind},
                      {"content_hash", scene.content_hash()},
                      {"scene", scene.header.scene_name},
                      {"frame", scene.header.source_frame}}}}
            << '\n';
}
Json vector(SimVec3 v) { return Json::array({v.x, v.y, v.z}); }
Json pose(SimPose p) {
  auto q = p.orientation;
  return {{"position", vector(p.position)},
          {"orientation", Json::array({q.x, q.y, q.z, q.w})}};
}
void inspect(const Main &scene) {
  // Read-only authoring tools use native model values to inspect calibration
  // and regenerate controller allocation. This output is never a launch input.
  Json bodies = Json::array(), thrusters = Json::array();
  for (auto &b : scene.objects)
    bodies.push_back({{"id", b.id},
                      {"name", b.name},
                      {"mass", b.mass},
                      {"dry_mass", b.dry_mass},
                      {"volume", b.volume},
                      {"inertia", vector(b.inertia)},
                      {"pose", pose(b.initial_pose)},
                      {"center_of_mass", pose(b.center_of_mass)}});
  for (auto &t : scene.robot.thrusters()) {
    auto &s = t.spec;
    thrusters.push_back({{"body", scene.objects[t.body].id},
                         {"position", vector(t.position)},
                         {"direction", vector(t.direction)},
                         {"spec",
                          {{"diameter", s.diameter},
                           {"max_rpm", s.max_rpm},
                           {"thrust_coeff", s.thrust},
                           {"torque_coeff", s.torque},
                           {"command_sign", s.command_sign}}}});
  }
  std::cout << Json{{"bodies", bodies},
                    {"thrusters", thrusters},
                    {"gravity", vector(scene.header.gravity)},
                    {"water_density", scene.header.water_density},
                    {"camera",
                     {{"body", scene.objects[scene.robot.camera.body].id},
                      {"pose", pose(scene.robot.camera.pose)}}}}
            << '\n';
}
// JSON is an inspection/authoring boundary only. Definitions and native loading
// use typed fixed structs directly, without parsing robot configuration JSON.
Json robot_body(const RobotDefinition &r) {
  const auto &b = r.body;
  return {{"mass", b.mass}, {"dry_mass", b.dry_mass}, {"inertia", vector(b.inertia)},
          {"volume", b.volume}, {"center_of_mass", pose(b.center_of_mass)},
          {"buoyancy_center", vector(b.buoyancy_center)}};
}
Json robot_description(const RobotDefinition &r, bool geometry) {
  Json mounts = Json::array(), specs = Json::array();
  for (const auto &t : std::span(r.thrusters).first(r.thruster_count)) {
    mounts.push_back({{"position", vector(t.position)}, {"direction", vector(t.direction)}});
    const auto &v = t.spec;
    specs.push_back({{"max_rpm", v.max_rpm}, {"diameter", v.diameter},
                    {"thrust_coeff", v.thrust}, {"torque_coeff", v.torque},
                    {"rotor_inertia", v.inertia}, {"kp", v.kp}, {"ki", v.ki},
                    {"command_sign", v.command_sign}});
  }
  Json result{{"name", r.name}, {"asset", r.asset}, {"description", r.description},
              {"asset_sha256", robot_asset_hash(r.name)},
              {"configuration_sha256", robot_configuration_hash(r)},
              {"body", robot_body(r)}, {"thrusters", r.thruster_count}, {"mounts", mounts},
              {"shapes", r.shape_count}, {"hulls", r.hull_count}};
  if (!geometry) return result;
  result["version"] = 2;
  result["thruster_specs"] = specs;
  result["geometry_reference"] = Json::array();
  for (const auto &shape : std::span(r.shapes).first(r.shape_count)) {
    Json points = Json::array();
    for (auto p : shape.points) points.push_back(vector(p));
    result["geometry_reference"].push_back({{"id", shape.id}, {"kind", shape.kind},
                                           {"collision", shape.collisions}, {"points", points}});
  }
  result["hull"] = Json::array();
  for (const auto &h : std::span(r.hulls).first(r.hull_count)) {
    const auto &v = h.physics;
    Json value{{"pose", pose(v.pose)}, {"drag", vector(v.drag)}, {"skin", vector(v.skin)},
               {"centroid", vector(v.centroid)}, {"volume", v.volume}};
    if (h.positions.empty()) value["shape"] = h.collider;
    else {
      Json faces = Json::array();
      for (size_t i = 0; i < h.positions.size(); i += 3)
        faces.push_back(Json::array({vector(h.positions[i]), vector(h.positions[i+1]), vector(h.positions[i+2])}));
      value["faces"] = faces;
    }
    result["hull"].push_back(value);
  }
  const auto &c = r.camera;
  result["camera"] = {{"pose", pose(c.pose)}, {"width", c.width}, {"height", c.height},
                       {"horizontal_fov", c.horizontal_fov}};
  return result;
}
int main(int argc, char **argv) {
  std::string backend_name;
  uint64_t steps = 0, hash = 0;
  std::unique_ptr<World> world;
  try {
    if (argc == 2 && std::string(argv[1]) == "robots") {
      Json robots = Json::array();
      for (const auto *r : robot_definitions())
        robots.push_back(robot_description(*r, false));
      std::cout << robots << '\n';
      return 0;
    }
    if (argc == 3 && std::string(argv[1]) == "robot-config") {
      auto *definition = robot_definition(argv[2]);
      require(definition, "unknown fixed robot");
      std::cout << robot_description(*definition, true) << '\n';
      return 0;
    }
    if (argc == 3 && std::string(argv[1]) == "inspect") {
      inspect(*Main::load(argv[2]));
      return 0;
    }
    if ((argc == 3 || argc == 4) && std::string(argv[1]) == "resolve") {
      auto scene = Main::load(argv[2]);
      write_stdout(
          *(argc == 4 ? draw_case(*scene, number(argv[3])) : std::move(scene)));
      return 0;
    }
    if (argc >= 2 && std::string(argv[1]) == "validate" &&
        (argc == 3 || (argc == 4 && std::string(argv[3]) == "--twice"))) {
      auto scene = Main::load(argv[2]);
      if (argc == 4) {
        auto again = Main::load(argv[2]);
        require(scene->fingerprint() == again->fingerprint(),
                "repeated scene load diverged");
      }
      print_valid(*scene);
      return 0;
    }
    require(
        argc >= 4 && std::string(argv[1]) == "run",
        "usage: simctl run <backend.so> <scene.blend|-|fd:N> [ticks] "
        "[seed] [--resolved] | simctl resolve <scene.blend|-|fd:N> <seed> | "
        "simctl validate <scene.blend|-|fd:N> [--twice] | simctl inspect "
        "<scene.blend> | simctl robots | simctl robot-config <name>");
    auto backend = std::make_shared<Backend>(argv[2]);
    backend_name = backend->api.name;
    uint64_t ticks = argc > 4 ? number(argv[4]) : 21600,
             seed = argc > 5 ? number(argv[5]) : 0;
    bool generated = argc > 5;
    auto scene = Main::load(argv[3]);
    if (generated && !(argc > 6 && std::string(argv[6]) == "--resolved"))
      scene = draw_case(*scene, seed);
    world = std::make_unique<World>(std::move(scene), backend);
    std::vector<double> commands(world->commands.size(), 0);
    for (uint64_t i = 0; i < ticks; ++i) {
      steps = i + 1;
      std::fill(commands.begin(), commands.end(), 0);
      if (world->scene.header.command_ticks) {
        require(i < world->scene.header.command_ticks,
                "command sequence ended");
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
