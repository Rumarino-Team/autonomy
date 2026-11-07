#include "runtime.hpp"
#include <algorithm>
#include <iostream>
#include <vector>
using namespace sim;
int main(int argc, char **argv) {
  try {
    require(argc == 10, "backend and eight saved .blend cases required");
    auto backend = std::make_shared<Backend>(argv[1]);
    int next_file = 2;
    auto next = [&] {
      return Main::load(argv[next_file++]);
    };
    // Order matches tools/blender/test_source.py.
    World baseline(next(), backend), moved(next(), backend), deleted(next(), backend);
    auto state = [](World &w, uint32_t id) -> const SimBodyState & {
      for (auto &b : w.bodies)
        if (b.id == id)
          return b;
      throw std::runtime_error("missing authored body");
    };
    require(std::abs(state(moved, 2).pose.position.x - 1.75) < 1e-5, "Blender gate transform did not reach backend");
    auto find = [](const Observation &o, uint32_t id) {
      return std::find_if(o.objects.begin(), o.objects.end(), [&](auto &v) { return v.id == id; }) != o.objects.end();
    };
    require(find(baseline.observe(), 3) && !find(deleted.observe(), 3), "deleted marker survived in sensor observations");
    auto a = baseline.observe(), b = moved.observe();
    auto gate = [](const Observation &o) {
      for (auto &v : o.objects)
        if (v.id == 2)
          return v.pose.position;
      throw std::runtime_error("missing gate observation");
    };
    require(norm(gate(a) - gate(b)) > .6, "observations ignored edited gate geometry/pose");
    World robot(next(), backend), mounts(next(), backend), instances(next(), backend);
    require(std::abs(state(robot, 1).pose.position.x - state(baseline, 1).pose.position.x + .3) < 1e-5, "robot initial pose did not come from Blender");
    require(std::abs(mounts.scene.robot.thrusters()[0].position.x - baseline.scene.robot.thrusters()[0].position.x) > .09, "thruster mount edit lost");
    require(std::abs(mounts.scene.header.camera.pose.position.z - baseline.scene.header.camera.pose.position.z) > .14, "camera mount edit lost");
    require(instances.bodies.size() > deleted.bodies.size(), "instanced geometry missing");
    for (const char *name : {"added", "disabled"}) {
      World w(next(), backend);
      auto reset = w.bodies;
      for (auto &s : reset)
        if (s.id == 1)
          s.linear_velocity = {0, 0, .5};
      w.reset(reset);
      bool contact = false;
      for (int i = 0; i < 720; ++i) {
        w.step(std::vector<double>(w.commands.size(), 0));
        for (auto &c : w.contacts)
          if ((c.a == 200 && c.b == 1) || (c.a == 1 && c.b == 200))
            contact = true;
      }
      require(contact == (std::string(name) == "added"), (std::string("authored collision enablement did not reach backend: ") + name).c_str());
      require(find(w.observe(), 200), "new authored obstacle missing from observations");
    }
    require(next_file == argc, "unexpected saved scene case count");
    std::cout << backend->api.name << " Blender source ownership passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
