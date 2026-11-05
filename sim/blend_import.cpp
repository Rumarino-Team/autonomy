#include "image.hpp"
#include "blend/blend_scene.hpp"
#include <fstream>
#include <iostream>
using namespace sim;
void erase_body(Scene &scene, size_t index) {
  scene.bodies.erase(scene.bodies.begin() + index);
  for (auto &thruster : scene.thrusters) {
    require(thruster.body != index, "removed a thruster body");
    if (thruster.body > index)
      --thruster.body;
  }
  for (auto &hull : scene.hulls) {
    require(hull.body != index, "removed a hull body");
    if (hull.body > index)
      --hull.body;
  }
  if (scene.header.camera.body == index)
    scene.header.camera.body = 0;
  else if (scene.header.camera.body > index)
    --scene.header.camera.body;
}
void add_shape(Scene &scene, BodyRec &body, ShapeRec shape) {
  uint32_t at = body.shape_begin + body.shape_count;
  if (!body.shape_count)
    at = body.shape_begin = scene.shapes.size();
  scene.shapes.insert(scene.shapes.begin() + at, shape);
  ++body.shape_count;
  for (auto &other : scene.bodies)
    if (&other != &body && other.shape_begin >= at)
      ++other.shape_begin;
}
ShapeRec mesh_shape(Scene &scene, const blender_scene_cpp::Mesh &mesh,
                    bool collisions, SimPose pose) {
  ShapeRec shape{};
  shape.kind = SIM_MESH;
  shape.collisions = collisions ? 1 : 0;
  shape.pose = pose;
  shape.friction = 0.5;
  shape.vertex_begin = scene.vertices.size();
  shape.vertex_count = mesh.vertices.size();
  shape.triangle_begin = scene.triangles.size();
  for (auto &vertex : mesh.vertices)
    scene.vertices.push_back({vertex[0], vertex[1], vertex[2]});
  for (auto &face : mesh.faces)
    for (size_t k = 1; k + 1 < face.size(); ++k)
      scene.triangles.push_back({face[0], face[k], face[k + 1]});
  shape.triangle_count = scene.triangles.size() - shape.triangle_begin;
  return shape;
}
int main(int argc, char **argv) {
  try {
    require(argc == 5, "usage: sim_import_blend <base-scene.asim> "
                       "<scene.blend> <config.yaml> <output.asim>");
    auto scene = Scene::load(argv[1]);
    auto imported = blender_scene_cpp::ReadScene(argv[2], argv[3]);
    for (auto &object : imported.objects) {
      if (!object.mesh)
        continue;
      if (object.placement == blender_scene_cpp::Placement::mesh_asset) {
        auto found = std::find_if(scene.bodies.begin(), scene.bodies.end(),
                                  [&](auto &body) {
                                    return body.name == object.name ||
                                           body.name == object.geometry_slot;
                                  });
        if (found == scene.bodies.end() || !found->shape_count)
          continue;
        auto &shape = scene.shapes[found->shape_begin];
        auto replacement = mesh_shape(scene, *object.mesh, shape.collisions != 0,
                                      shape.pose);
        shape.vertex_begin = replacement.vertex_begin;
        shape.vertex_count = replacement.vertex_count;
        shape.triangle_begin = replacement.triangle_begin;
        shape.triangle_count = replacement.triangle_count;
        shape.kind = SIM_MESH;
        continue;
      }
      if (object.name == "PoolShell" || object.blender_name == "pool_tile") {
        SimVec3 lo{INFINITY, INFINITY, INFINITY},
            hi{-INFINITY, -INFINITY, -INFINITY};
        for (auto &vertex : object.mesh->vertices) {
          SimVec3 p{vertex[0] + object.position[0],
                    vertex[1] + object.position[1],
                    vertex[2] + object.position[2]};
          lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
          hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
        }
        for (size_t i = scene.bodies.size(); i-- > 0;)
          if (std::string(scene.bodies[i].name) == "portable_pool" ||
              std::string(scene.bodies[i].name) == "portable_ground")
            erase_body(scene, i);
        auto center = (lo + hi) * .5, half = (hi - lo) * .5;
        BodyRec body{};
        body.id = 4294967294u;
        copy_text(body.name, sizeof(body.name), "portable_pool");
        body.motion = SIM_STATIC;
        body.center_of_mass.orientation = {0, 0, 0, 1};
        body.initial_pose.orientation = {0, 0, 0, 1};
        body.shape_begin = scene.shapes.size();
        auto box = [&](SimVec3 position, SimVec3 size) {
          ShapeRec shape{};
          shape.kind = SIM_BOX;
          shape.collisions = 1;
          shape.pose = {position, {0, 0, 0, 1}};
          shape.size = size;
          shape.friction = 0.5;
          scene.shapes.push_back(shape);
          ++body.shape_count;
        };
        box({lo.x - .1, center.y, center.z}, {.1, half.y, half.z});
        box({hi.x + .1, center.y, center.z}, {.1, half.y, half.z});
        box({center.x, lo.y - .1, center.z}, {half.x, .1, half.z});
        box({center.x, hi.y + .1, center.z}, {half.x, .1, half.z});
        box({center.x, center.y, hi.z + .1}, {half.x, half.y, .1});
        scene.bodies.push_back(body);
        continue;
      }
      auto found = std::find_if(
          scene.bodies.begin(), scene.bodies.end(),
          [&](auto &body) { return body.name == object.name; });
      SimQuat orientation{object.quat[1], object.quat[2], object.quat[3],
                          object.quat[0]};
      if (object.placement != blender_scene_cpp::Placement::mocap_fuzz)
        orientation = euler(object.rotation[0], object.rotation[1],
                            object.rotation[2]);
      SimPose pose{{object.position[0], object.position[1], object.position[2]},
                   orientation};
      auto shape = mesh_shape(scene, *object.mesh, object.convex, pose);
      if (found != scene.bodies.end()) {
        found->motion = object.placement == blender_scene_cpp::Placement::mocap_fuzz
                            ? SIM_KINEMATIC
                            : SIM_STATIC;
        found->initial_pose = pose;
        found->tracked = !object.cls.empty() && object.cls != "scenery";
        found->cls = object.cls == "gate" ? 2 : 1;
        found->fuzz = !object.fuzz.empty();
        found->shape_count = 0;
        add_shape(scene, *found, shape);
      } else {
        BodyRec body{};
        Hash hash;
        hash.bytes(object.id.data(), object.id.size());
        body.id = uint32_t(hash.value);
        copy_text(body.name, sizeof(body.name), object.name.c_str());
        body.motion = object.placement == blender_scene_cpp::Placement::mocap_fuzz
                          ? SIM_KINEMATIC
                          : SIM_STATIC;
        body.initial_pose = pose;
        body.center_of_mass.orientation = {0, 0, 0, 1};
        body.tracked = !object.cls.empty() && object.cls != "scenery";
        body.cls = object.cls == "gate" ? 2 : 1;
        body.fuzz = !object.fuzz.empty();
        body.shape_begin = scene.shapes.size();
        body.shape_count = 1;
        scene.shapes.push_back(shape);
        scene.bodies.push_back(body);
      }
    }
    scene.validate();
    auto bytes = scene.binary();
    std::ofstream output(argv[4], std::ios::binary);
    require(bool(output), "cannot open output scenario");
    output.write(reinterpret_cast<const char *>(bytes.data()), bytes.size());
    std::cout << "Imported " << imported.objects.size()
              << " Blender objects into portable scenario\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
