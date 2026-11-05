#include "runtime.hpp"
#include <cstring>
#include <future>
#include <iostream>
using namespace sim;
void expect(bool value, const char *message) {
  if (!value)
    throw std::runtime_error(message);
}
Json fixture() {
  return {
      {"version", 1},
      {"dt", {1, 360}},
      {"gravity", {0, 0, 0}},
      {"bodies",
       Json::array({{{"id", 7},
                     {"motion", SIM_DYNAMIC},
                     {"mass", 2},
                     {"inertia", {1, 1, 1}},
                     {"pose", {{"position", {0, 0, 2}}}},
                     {"shapes", Json::array({{{"kind", SIM_BOX},
                                              {"size", {.1, .2, .3}}}})}}})}};
}
int main(int argc, char **argv) {
  try {
    expect(argc == 2, "backend required");
    auto backend = std::make_shared<Backend>(argv[1]);
    auto &api = backend->api;
    char error[1024]{};
    void *model = nullptr;
    expect(api.model_create(nullptr, &model, error, sizeof(error)) ==
               SIM_INVALID,
           "null model accepted");
    auto scene = Scene::parse(fixture());
    scene.model.bind();
    auto invalid = scene.model.value;
    invalid.dt_denominator = 0;
    expect(api.model_create(&invalid, &model, error, sizeof(error)) ==
               SIM_INVALID,
           "invalid timestep accepted");
    ForceRegistry registry;
    registry.add("test_force", [](ForceContext &, std::vector<SimWrench> &w) {
      w[0].force.x += 2;
    });
    auto j = fixture();
    j["force_modules"] = {"gravity_compensation", "test_force"};
    World forced(Scene::parse(j), backend, registry);
    uint32_t count = 0;
    expect(api.read_state(nullptr, nullptr, 0, &count) == SIM_INVALID,
           "null state accepted");
    forced.step({});
    expect(std::abs(forced.bodies[0].linear_velocity.x - 1.0 / 360) < 1e-10,
           "F=ma failed");
    expect(std::abs(forced.bodies[0].pose.position.x - 1.0 / (360 * 360)) <
               1e-9,
           "fixed integration failed");
    // The ABI velocity is at COM, while attachments are relative to the body
    // origin.
    auto offset = fixture();
    offset["bodies"][0]["center_of_mass"] = {{"position", {.2, 0, 0}}};
    offset["bodies"][0]["pose"]["orientation"] =
        json(euler(0, 0, 1.5707963267948966));
    offset["force_modules"] = {"gravity_compensation", "lever"};
    ForceRegistry lever;
    lever.add("lever", [](ForceContext &c, std::vector<SimWrench> &out) {
      auto p = point(c.bodies[0].pose, {1.2, 0, 0});
      add_force(out[0], {2, 0, 0}, p, c.bodies[0].center_of_mass);
    });
    World torque(Scene::parse(offset), backend, lever);
    torque.step({});
    expect(std::abs(torque.wrenches[0].torque.z + 2) < 1e-10,
           "off-center torque/frame conversion failed");
    expect(std::abs(torque.bodies[0].angular_velocity.z + 2.0 / 360) < 1e-9,
           "angular acceleration failed");
    if (api.integrators & (1u << SIM_INTEGRATOR_RK4)) {
      auto rk4 = j;
      rk4["integrator"] = "rk4";
      World integrated(Scene::parse(rk4), backend, registry);
      integrated.step({});
      expect(std::abs(integrated.bodies[0].pose.position.x - .5 / (360 * 360)) <
                 1e-10,
             "RK4 request was not honored");
    } else {
      bool rejected = false;
      try {
        auto rk4 = j;
        rk4["integrator"] = "rk4";
        World unsupported(Scene::parse(rk4), backend, registry);
      } catch (const std::invalid_argument &) {
        rejected = true;
      }
      expect(rejected, "unsupported integrator accepted");
    }
    ForceRegistry duplicate;
    duplicate.add(
        "second_fluid", [](ForceContext &, std::vector<SimWrench> &) {},
        "hydrodynamics");
    auto conflict = fixture();
    conflict["force_modules"] = {"gravity_compensation", "hydrodynamics",
                                 "second_fluid"};
    bool rejected = false;
    try {
      World invalid_world(Scene::parse(conflict), backend, duplicate);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    expect(rejected, "duplicate fluid providers accepted");
    auto digest = [&]() {
      World w(Scene::parse(j), backend, registry);
      std::vector<uint64_t> h;
      for (int i = 0; i < 40; ++i) {
        w.step({});
        h.push_back(w.state_hash());
      }
      return h;
    };
    auto expected = digest();
    expect(expected == digest(), "fresh replay diverged");
    std::vector<std::future<std::vector<uint64_t>>> jobs;
    for (int i = 0; i < 4; ++i)
      jobs.push_back(std::async(std::launch::async, digest));
    for (auto &job : jobs)
      expect(expected == job.get(), "parallel replay diverged");
    forced.reset();
    std::vector<uint64_t> reset;
    for (int i = 0; i < 40; ++i) {
      forced.step({});
      reset.push_back(forced.state_hash());
    }
    expect(expected == reset, "cold reset replay diverged");
    auto collision = fixture();
    collision["gravity"] = {0, 0, 9.81};
    collision["bodies"][0]["pose"]["position"] = {0, 0, .5};
    collision["bodies"].push_back(
        {{"id", 8},
         {"motion", SIM_STATIC},
         {"pose", {{"position", {0, 0, 1.1}}}},
         {"shapes", Json::array({{{"kind", SIM_BOX}, {"size", {2, 2, .1}}}})}});
    World contact(Scene::parse(collision), backend);
    bool touched = false;
    for (int i = 0; i < 180; ++i) {
      contact.step({});
      touched |= !contact.contacts.empty();
    }
    expect(touched, "no collision reported");
    expect(contact.bodies[0].pose.position.z < .85, "body fell through floor");
    auto pass = fixture();
    pass["bodies"][0]["linear_velocity"] = {0, 1, 0};
    pass["bodies"].push_back({
        {"id", 8}, {"motion", SIM_KINEMATIC},
        {"pose", {{"position", {0, .5, 2}}}},
        {"shapes", Json::array({{{"kind", SIM_BOX}, {"size", {1, .1, 1}},
                                  {"collisions", false}}})}});
    World noncolliding(Scene::parse(pass), backend);
    for (int i = 0; i < 360; ++i) {
      noncolliding.step({});
      expect(noncolliding.contacts.empty(), "disabled obstacle produced a contact");
    }
    expect(std::abs(noncolliding.bodies[0].pose.position.y - 1) < 1e-8,
           "disabled obstacle blocked the vehicle");
    pass["bodies"][0]["linear_velocity"]={0,0,0};
    pass["bodies"][1]["pose"]["position"]={0,0,2};
    World overlapping_disabled(Scene::parse(pass),backend);
    overlapping_disabled.step({});
    expect(overlapping_disabled.contacts.empty(),"initially overlapping disabled geometry generated contacts");
    auto floating = fixture();
    floating["gravity"] = {0, 0, 9.81};
    floating["bodies"][0]["volume"] = 2.0 / 1025;
    floating["hull"] = Json::array({{{"body", 7},
                                     {"shape", 0},
                                     {"volume", 2.0 / 1025},
                                     {"drag", {1, 1, 1}},
                                     {"skin", {.1, .1, .1}}}});
    World buoyant(Scene::parse(floating), backend);
    buoyant.step({});
    expect(std::abs(buoyant.bodies[0].linear_velocity.z) < 1e-9,
           "neutral buoyancy failed");
    std::cout << api.name
              << " conformance passed: ABI, F=ma, collision, buoyancy, cold "
                 "reset, parallel replay\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "conformance: " << e.what() << '\n';
    return 1;
  }
}
