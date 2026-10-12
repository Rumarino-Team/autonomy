#pragma once
#include "blend_scene.hpp"
#include <unordered_map>
#include <map>
namespace sf { class SimulationManager; }
namespace blender_stonefish_cpp {
struct MaterialSettings {
    double density, restitution, magnetic;
    bool operator==(const MaterialSettings&) const = default;
};
struct LookSettings {
    Vec3 color;
    double roughness, metalness, reflectivity;
    std::string texture, normal_map;
    bool operator==(const LookSettings&) const = default;
};
struct FrictionSettings {
    std::string a, b;
    double stat, dynamic;
    bool operator==(const FrictionSettings&) const = default;
};
struct EnvironmentSettings {
    bool enabled, particles;
    double density, jerlov, waves, temperature, azimuth, elevation, latitude, longitude;
    Vec3 current;
    bool operator==(const EnvironmentSettings&) const = default;
};
struct SceneSettings {
    std::map<std::string, MaterialSettings> materials;
    std::map<std::string, LookSettings> looks;
    std::vector<FrictionSettings> friction;
    EnvironmentSettings environment;
};
SceneSettings ReadSceneSettings(const std::filesystem::path& config, const std::filesystem::path& data);
// Decode/validate and stage replacement meshes/textures before changing live bodies.
// Call between physics steps on the GL context's owning thread.
void UpdateScene(sf::SimulationManager& manager, const Scene& previous, const Scene& next,
                 const SceneSettings& previous_settings, const SceneSettings& settings,
                 const std::filesystem::path& data, bool reload_textures);
// All Blender environment construction goes directly through the C++ API.
// `scene` is a blend that has already been decoded. Converted meshes are reused
// when MeshFingerprint matches a mesh built earlier in this process.
std::unordered_map<std::string, std::string> BuildScene(
    sf::SimulationManager& manager, const Scene& scene,
    const std::filesystem::path& config, const std::filesystem::path& data,
    const std::filesystem::path& blend);
std::unordered_map<std::string, std::string> BuildScene(
    sf::SimulationManager& manager, const std::filesystem::path& blend,
    const std::filesystem::path& config, const std::filesystem::path& data);
}
