#include "mujoco_platform.hpp"

#include <cstdlib>
#include <fstream>
#include <nlohmann/json.hpp>

namespace mujoco_platform {
namespace {

std::filesystem::path RepoRelative(const std::filesystem::path& path) {
    if (path.is_absolute()) return path;
    return std::filesystem::current_path() / path;
}

}  // namespace

Paths ResolvePaths(const char* mjcf_override, std::string& error) {
    const char* platform_path = std::getenv("AUV_PLATFORM");
    const std::filesystem::path platform_file =
        platform_path != nullptr && platform_path[0] != '\0'
            ? RepoRelative(platform_path)
            : RepoRelative("auvs/hydrus_mujoco/platform.json");

    std::ifstream input(platform_file);
    if (!input) {
        error = "cannot open platform config: " + platform_file.string();
        return {};
    }
    nlohmann::json document;
    try {
        input >> document;
    } catch (const std::exception& exception) {
        error = std::string("platform json: ") + exception.what();
        return {};
    }

    Paths paths;
    const std::filesystem::path data_path = RepoRelative(document.value("data_path", "auvs/stonefish_sim/data/"));
    const std::string blend_name = document.value("blend_file", "pool_scene.blend");
    paths.stonefish_config = RepoRelative(document.value("stonefish_config", "auvs/stonefish_sim/stonefish_config.yaml"));

    if (const char* blend_env = std::getenv("AUV_BLEND"); blend_env != nullptr && blend_env[0] != '\0') {
        paths.blend_file = RepoRelative(blend_env);
    } else {
        paths.blend_file = data_path / blend_name;
    }

    const char* mjcf = mjcf_override;
    if (mjcf == nullptr || mjcf[0] == '\0') {
        if (const char* mjcf_env = std::getenv("AUV_MJCF"); mjcf_env != nullptr && mjcf_env[0] != '\0') {
            mjcf = mjcf_env;
        } else {
            paths.skeleton_xml = RepoRelative(document.value("mjcf_default", "auvs/hydrus_mujoco/hydrus.xml"));
            return paths;
        }
    }
    paths.skeleton_xml = RepoRelative(mjcf);
    return paths;
}

}  // namespace mujoco_platform
