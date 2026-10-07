#pragma once
#include "blend_scene.hpp"
#include <unordered_map>
namespace sf { class SimulationManager; }
namespace blender_stonefish_cpp {
// All Blender environment construction goes directly through the C++ API.
std::unordered_map<std::string, std::string> BuildScene(
    sf::SimulationManager& manager, const std::filesystem::path& blend,
    const std::filesystem::path& config, const std::filesystem::path& data);
}
