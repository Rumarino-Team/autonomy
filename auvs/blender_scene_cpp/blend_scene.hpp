#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace blender_scene_cpp {
using Vec3 = std::array<double, 3>;
using Vec2 = std::array<double, 2>;
struct Mesh {
    std::vector<Vec3> vertices;
    std::vector<std::vector<uint32_t>> faces;
    std::vector<std::vector<Vec2>> face_uvs;
};
enum class Placement : uint8_t {
    world_static,
    mocap_fuzz,
    mesh_asset,
};
struct Object {
    std::string blender_name, name, material, look, cls;
    std::string id;
    std::string fuzz;
    // When set, vertices are object-local; the MJCF skeleton owns the geom transform.
    std::string mujoco_geom;
    Placement placement = Placement::world_static;
    Vec3 position{};
    Vec3 rotation{};
    Vec3 scale{};
    // NED world-frame unit quaternion wxyz for mocap bodies.
    double quat[4]{1, 0, 0, 0};
    bool convex = false;
    std::shared_ptr<const Mesh> mesh;
};
struct ViewCamera {
    std::string name;
    Vec3 location{};
    Vec3 rotation{};
};
struct Scene {
    std::vector<Object> objects;
    std::vector<ViewCamera> cameras;
};
Scene ReadScene(const std::filesystem::path& blend, const std::filesystem::path& config);
std::string MeshFingerprint(const Mesh& mesh);
void WriteObj(const Mesh& mesh, const std::filesystem::path& filename);
}  // namespace blender_scene_cpp
