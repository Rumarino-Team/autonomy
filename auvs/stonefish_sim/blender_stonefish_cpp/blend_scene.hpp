#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace blender_stonefish_cpp {
using Vec3 = std::array<double, 3>;
using Vec2 = std::array<double, 2>;
struct Mesh {
    std::vector<Vec3> vertices;
    // Zero-based vertex indices and matching UVs for each polygon corner.
    std::vector<std::vector<uint32_t>> faces;
    std::vector<std::vector<Vec2>> face_uvs;
};
struct Object {
    std::string blender_name, name, material, look, cls;
    Vec3 position{}, rotation{}, scale{};
    bool convex = false;
    std::shared_ptr<const Mesh> mesh;
};
// Blender object location and XYZ Euler, in radians. The simulator converts these
// with the same axis mapping it uses for static meshes.
struct ViewCamera {
    std::string name;
    Vec3 location{};
    Vec3 rotation{};
};
struct Scene {
    std::vector<Object> objects;
    std::vector<ViewCamera> cameras;
};
// Reads stored mesh geometry, preserving the existing pool's axis convention.
// Throws std::runtime_error on unsupported data, before adding any entities.
Scene ReadScene(const std::filesystem::path& blend, const std::filesystem::path& config);
// Stable identity of vertex, face, and UV bytes. Unchanged meshes compare equal.
std::string MeshFingerprint(const Mesh& mesh);
void WriteObj(const Mesh& mesh, const std::filesystem::path& filename);
} // namespace blender_stonefish_cpp
