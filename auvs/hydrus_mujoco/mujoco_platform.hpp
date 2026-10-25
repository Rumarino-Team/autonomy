#pragma once

#include <filesystem>
#include <string>

namespace mujoco_platform {

struct Paths {
    std::filesystem::path skeleton_xml;
    std::filesystem::path blend_file;
    std::filesystem::path stonefish_config;
};

// Resolves platform.json, AUV_BLEND, and AUV_MJCF environment overrides.
Paths ResolvePaths(const char* mjcf_override, std::string& error);

}  // namespace mujoco_platform
