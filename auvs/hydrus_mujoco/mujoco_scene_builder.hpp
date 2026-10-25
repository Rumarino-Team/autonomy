#pragma once

#include <filesystem>
#include <mujoco/mujoco.h>
#include <string>

namespace blender_scene_cpp {
struct Scene;
}

namespace mujoco_scene {

// Parse the MJCF skeleton, merge geometry from the Blender scene, and compile mjModel.
mjModel* ComposeModel(
    const std::filesystem::path& skeleton_xml,
    const std::filesystem::path& blend_file,
    const std::filesystem::path& stonefish_config,
    std::string& error);

void ApplyBlendToSpec(mjSpec* spec, const blender_scene_cpp::Scene& scene, std::string& error);

}  // namespace mujoco_scene
