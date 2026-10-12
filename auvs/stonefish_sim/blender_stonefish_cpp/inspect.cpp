#include "blend_scene.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
#include <cmath>
#include <limits>
int main(int argc, char** argv) {
    if (argc != 3 && argc != 4) {
        std::cerr << "Usage: stonefish_blend_inspect scene.blend config.yaml [export-directory]\n";
        return 2;
    }
    try {
        auto scene = blender_stonefish_cpp::ReadScene(argv[1], argv[2]);
        nlohmann::json result = nlohmann::json::array();
        for (const auto& object : scene.objects) {
            blender_stonefish_cpp::Vec3 min{INFINITY, INFINITY, INFINITY}, max{-INFINITY, -INFINITY, -INFINITY};
            for (auto v : object.mesh->vertices) for (size_t i = 0; i < 3; ++i) { min[i] = std::min(min[i], v[i]); max[i] = std::max(max[i], v[i]); }
            size_t uv_corners = 0;
            for (const auto& face : object.mesh->face_uvs) uv_corners += face.size();
            result.push_back({{"blender_name", object.blender_name}, {"name", object.name}, {"id", object.id}, {"material", object.material},
                {"look", object.look}, {"cls", object.cls}, {"position", object.position}, {"rotation", object.rotation},
                {"scale", object.scale}, {"convex", object.convex}, {"vertices", object.mesh->vertices.size()},
                {"faces", object.mesh->faces.size()}, {"uv_corners", uv_corners}, {"min", min}, {"max", max}});
            if (argc == 4) blender_stonefish_cpp::WriteObj(*object.mesh, std::filesystem::path(argv[3]) / (object.name + ".obj"));
        }
        std::cout << result.dump(2) << '\n';
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
