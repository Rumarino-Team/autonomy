#include "blend/blend_scene.hpp"
#include "scene.hpp"
#include <iostream>
using namespace sim;
Json geometry(const blender_scene_cpp::Object &object) {
  Json g = {{"kind", SIM_MESH},
            {"collisions", object.convex},
            {"vertices", object.mesh->vertices},
            {"triangles", Json::array()},
            {"pose", {{"position", {0, 0, 0}}, {"orientation", {0, 0, 0, 1}}}}};
  for (auto &face : object.mesh->faces)
    for (size_t k = 1; k + 1 < face.size(); ++k)
      g["triangles"].push_back({face[0], face[k], face[k + 1]});
  return g;
}
int main(int argc, char **argv) {
  try {
    require(argc == 5, "usage: sim_import_blend <base-scenario.json> "
                       "<scene.blend> <config.yaml> <output.json>");
    auto document = Scene::load(argv[1]).document;
    auto scene = blender_scene_cpp::ReadScene(argv[2], argv[3]);
    for (auto &object : scene.objects) {
      if (!object.mesh)
        continue;
      auto g = geometry(object);
      if (object.placement == blender_scene_cpp::Placement::mesh_asset) {
        for (auto &b : document["bodies"])
          for (auto &shape : b["shapes"])
            if (shape.value("geometry_slot", "") == object.name) {
              bool collisions = shape.value("collisions", true);
              auto transform = shape.value("pose", Json::object());
              shape.update(g);
              shape["collisions"] = collisions;
              shape["pose"] = transform;
            }
        continue;
      }
      if (object.name == "PoolShell" || object.blender_name == "pool_tile") {
        SimVec3 lo{INFINITY, INFINITY, INFINITY},
            hi{-INFINITY, -INFINITY, -INFINITY};
        for (auto &a : object.mesh->vertices) {
          SimVec3 p{a[0] + object.position[0], a[1] + object.position[1],
                    a[2] + object.position[2]};
          lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
          hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
        }
        document["bodies"].erase(
            std::remove_if(document["bodies"].begin(), document["bodies"].end(),
                           [](const Json &b) {
                             return b.value("name", "") == "portable_pool" ||
                                    b.value("name", "") == "portable_ground";
                           }),
            document["bodies"].end());
        for (auto &body : document["bodies"]) {
          auto &shapes = body["shapes"];
          shapes.erase(std::remove_if(shapes.begin(), shapes.end(),
                                      [](const Json &shape) {
                                        return shape.value("pool_wall", false);
                                      }),
                       shapes.end());
        }
        auto center = (lo + hi) * .5, half = (hi - lo) * .5;
        Json shapes = Json::array();
        auto box = [&](SimVec3 position, SimVec3 size) {
          shapes.push_back({{"kind", SIM_BOX},
                            {"pose", {{"position", json(position)}}},
                            {"size", json(size)}});
        };
        box({lo.x - .1, center.y, center.z}, {.1, half.y, half.z});
        box({hi.x + .1, center.y, center.z}, {.1, half.y, half.z});
        box({center.x, lo.y - .1, center.z}, {half.x, .1, half.z});
        box({center.x, hi.y + .1, center.z}, {half.x, .1, half.z});
        box({center.x, center.y, hi.z + .1}, {half.x, half.y, .1});
        document["bodies"].push_back({{"id", 4294967294u},
                                      {"name", "portable_pool"},
                                      {"motion", SIM_STATIC},
                                      {"shapes", shapes}});
        continue;
      }
      auto found = std::find_if(
          document["bodies"].begin(), document["bodies"].end(),
          [&](auto &b) { return b.value("name", "") == object.name; });
      SimQuat q{object.quat[1], object.quat[2], object.quat[3], object.quat[0]};
      if (object.placement != blender_scene_cpp::Placement::mocap_fuzz)
        q = euler(object.rotation[0], object.rotation[1], object.rotation[2]);
      Json body = {
          {"name", object.name},
          {"motion",
           object.placement == blender_scene_cpp::Placement::mocap_fuzz
               ? SIM_KINEMATIC
               : SIM_STATIC},
          {"pose", json(SimPose{{object.position[0], object.position[1],
                                 object.position[2]},
                                q})},
          {"shapes", Json::array({g})},
          {"tracked", !object.cls.empty() && object.cls != "scenery"},
          {"class", object.cls == "gate" ? 2 : 1},
          {"fuzz", !object.fuzz.empty()}};
      if (found != document["bodies"].end()) {
        body["id"] = found->at("id");
        found->update(body);
      } else {
        Hash h;
        h.bytes(object.id.data(), object.id.size());
        body["id"] = uint32_t(h.value);
        document["bodies"].push_back(body);
      }
    }
    auto resolved = Scene::parse(document);
    std::ofstream output(argv[4]);
    require(bool(output), "cannot open output scenario");
    output << resolved.document.dump() << '\n';
    std::cout << "Imported " << scene.objects.size()
              << " Blender objects into portable scenario\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
