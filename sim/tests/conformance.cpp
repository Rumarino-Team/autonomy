#include "runtime.hpp"
#include <cstring>
#include <future>
#include <iostream>
using namespace sim;
void expect(bool value, const char *message) {
  if (!value)
    throw std::runtime_error(message);
}
void modules(Scene &scene, std::initializer_list<const char *> names) {
  scene.modules.clear();
  for (auto name : names) {
    NameRec module{};
    copy_text(module.text, sizeof(module.text), name);
    scene.modules.push_back(module);
  }
}
BodyRec make_body(uint32_t id, uint32_t motion, SimVec3 position, double mass) {
  BodyRec body{};
  body.id = id;
  copy_text(body.name, sizeof(body.name), "body");
  body.motion = motion;
  body.mass = mass;
  body.dry_mass = mass;
  body.inertia = {1, 1, 1};
  body.center_of_mass.orientation = {0, 0, 0, 1};
  body.initial_pose = {position, {0, 0, 0, 1}};
  return body;
}
ShapeRec make_box(SimVec3 size, bool collisions = true) {
  ShapeRec shape{};
  shape.kind = SIM_BOX;
  shape.collisions = collisions ? 1 : 0;
  shape.size = size;
  shape.pose.orientation = {0, 0, 0, 1};
  shape.friction = 0.5;
  return shape;
}
void add_body(Scene &scene, BodyRec body, ShapeRec shape) {
  body.shape_begin = scene.shapes.size();
  body.shape_count = 1;
  scene.bodies.push_back(body);
  scene.shapes.push_back(shape);
}
Scene fixture() {
  Scene scene;
  scene.header.gravity = {0, 0, 0};
  add_body(scene, make_body(7, SIM_DYNAMIC, {0, 0, 2}, 2),
           make_box({.1, .2, .3}));
  return scene;
}
int main(int argc, char **argv) {
  try {
    expect(argc == 2, "backend required");
    auto backend = std::make_shared<Backend>(argv[1]);
    auto &api = backend->api;
    char error[1024]{};
    void *world = nullptr;
    expect(api.world_create(nullptr, 0, &world, error, sizeof(error)) ==
               SIM_INVALID,
           "null scenario accepted");
    auto invalid = fixture();
    invalid.header.dt_denominator = 0;
    auto bytes = invalid.binary();
    expect(api.world_create(reinterpret_cast<const char *>(bytes.data()),
                            bytes.size(), &world, error, sizeof(error)) ==
               SIM_INVALID,
           "invalid timestep accepted");
    ForceRegistry registry;
    registry.add("test_force", [](ForceContext &, std::vector<SimWrench> &w) {
      w[0].force.x += 2;
    });
    auto j = fixture();
    modules(j, {"gravity_compensation", "test_force"});
    World forced(j, backend, registry);
    uint32_t count = 0;
    expect(api.read_state(nullptr, nullptr, 0, &count) == SIM_INVALID,
           "null state accepted");
    forced.step({});
    expect(std::abs(forced.bodies[0].linear_velocity.x - 1.0 / 360) < 1e-10,
           "F=ma failed");
    expect(std::abs(forced.bodies[0].pose.position.x - 1.0 / (360 * 360)) <
               1e-9,
           "fixed integration failed");
    auto offset = fixture();
    offset.bodies[0].center_of_mass.position = {.2, 0, 0};
    offset.bodies[0].initial_pose.orientation =
        euler(0, 0, 1.5707963267948966);
    modules(offset, {"gravity_compensation", "lever"});
    ForceRegistry lever;
    lever.add("lever", [](ForceContext &c, std::vector<SimWrench> &out) {
      auto p = point(c.bodies[0].pose, {1.2, 0, 0});
      add_force(out[0], {2, 0, 0}, p, c.bodies[0].center_of_mass);
    });
    World torque(offset, backend, lever);
    torque.step({});
    expect(std::abs(torque.wrenches[0].torque.z + 2) < 1e-10,
           "off-center torque/frame conversion failed");
    expect(std::abs(torque.bodies[0].angular_velocity.z + 2.0 / 360) < 1e-9,
           "angular acceleration failed");
    if (api.integrators & (1u << SIM_INTEGRATOR_RK4)) {
      auto rk4 = j;
      rk4.header.integrator = SIM_INTEGRATOR_RK4;
      World integrated(rk4, backend, registry);
      integrated.step({});
      expect(std::abs(integrated.bodies[0].pose.position.x - .5 / (360 * 360)) <
                 1e-10,
             "RK4 request was not honored");
    } else {
      bool rejected = false;
      try {
        auto rk4 = j;
        rk4.header.integrator = SIM_INTEGRATOR_RK4;
        World unsupported(rk4, backend, registry);
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
    modules(conflict, {"gravity_compensation", "hydrodynamics", "second_fluid"});
    bool rejected = false;
    try {
      World invalid_world(conflict, backend, duplicate);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    expect(rejected, "duplicate fluid providers accepted");
    auto digest = [&]() {
      World w(j, backend, registry);
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
    collision.header.gravity = {0, 0, 9.81};
    collision.bodies[0].initial_pose.position = {0, 0, .5};
    add_body(collision, make_body(8, SIM_STATIC, {0, 0, 1.1}, 0),
             make_box({2, 2, .1}));
    World contact(collision, backend);
    bool touched = false;
    for (int i = 0; i < 180; ++i) {
      contact.step({});
      touched |= !contact.contacts.empty();
    }
    expect(touched, "no collision reported");
    expect(contact.bodies[0].pose.position.z < .85, "body fell through floor");
    auto pass = fixture();
    pass.bodies[0].initial_linear_velocity = {0, 1, 0};
    add_body(pass, make_body(8, SIM_KINEMATIC, {0, .5, 2}, 0),
             make_box({1, .1, 1}, false));
    World noncolliding(pass, backend);
    for (int i = 0; i < 360; ++i) {
      noncolliding.step({});
      expect(noncolliding.contacts.empty(), "disabled obstacle produced a contact");
    }
    expect(std::abs(noncolliding.bodies[0].pose.position.y - 1) < 1e-8,
           "disabled obstacle blocked the vehicle");
    pass.bodies[0].initial_linear_velocity = {};
    pass.bodies[1].initial_pose.position = {0, 0, 2};
    World overlapping_disabled(pass, backend);
    overlapping_disabled.step({});
    expect(overlapping_disabled.contacts.empty(),
           "initially overlapping disabled geometry generated contacts");
    auto floating = fixture();
    floating.header.gravity = {0, 0, 9.81};
    floating.bodies[0].volume = 2.0 / 1025;
    auto faces = surface(floating.shapes[0].kind, floating.shapes[0].size,
                         nullptr, nullptr, 0, 0);
    HullRec hull{};
    hull.pose = floating.shapes[0].pose;
    hull.drag = {1, 1, 1};
    hull.skin = {.1, .1, .1};
    hull.volume = 2.0 / 1025;
    hull.coefficient_frame = {0, 0, 0, 1};
    for (auto &face : faces)
      for (auto vertex : face)
        floating.hull_vertices.push_back(vertex);
    hull.vertex_count = floating.hull_vertices.size();
    floating.hulls.push_back(hull);
    World buoyant(floating, backend);
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
