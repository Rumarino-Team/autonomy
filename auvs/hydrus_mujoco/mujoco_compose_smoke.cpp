#include "mujoco_platform.hpp"
#include "mujoco_scene_builder.hpp"

#include <cstdio>
#include <mujoco/mujoco.h>

int main() {
    std::string error;
    const mujoco_platform::Paths paths = mujoco_platform::ResolvePaths(nullptr, error);
    if (!error.empty()) {
        std::fprintf(stderr, "resolve: %s\n", error.c_str());
        return 1;
    }
    mjModel* model = mujoco_scene::ComposeModel(paths.skeleton_xml, paths.blend_file, paths.stonefish_config, error);
    if (model == nullptr) {
        std::fprintf(stderr, "compose: %s\n", error.c_str());
        return 1;
    }
    mjData* data = mj_makeData(model);
    for (int i = 0; i < 36; ++i) mj_step(model, data);
    std::fprintf(stdout, "mujoco_compose_smoke ok t=%.3f\n", data->time);
    mj_deleteData(data);
    mj_deleteModel(model);
    return 0;
}
