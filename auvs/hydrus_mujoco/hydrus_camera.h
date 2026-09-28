#pragma once

#include "hydrus_batch.h"

#include <mujoco/mujoco.h>

#include <cstdint>
#include <vector>

namespace hydrus {

// Triangles and boxes used to fake a YOLO camera without opening a window.
struct CameraModel {
    int site = -1;
    double fov_h_deg = 60;
    struct Tri {
        int geom = -1;
        uint8_t shade = 0;
        float v[3][3]{};
    };
    struct Box {
        int geom = -1;
        int cls = 0;
        float local_min[3]{};
        float local_max[3]{};
    };
    std::vector<Tri> tris;
    std::vector<Box> boxes;

    static CameraModel build(const mjModel* model);
};

// Previous raster for one vehicle, so optical flow has a frame to compare.
struct CameraView {
    std::vector<uint8_t> image;
    std::vector<uint8_t> previous;
    std::vector<float> depth;
    bool has_previous = false;

    CameraView();
    void clear();
    void render(const mjModel* model, const mjData* data, const CameraModel& camera, float* features);
};

}  // namespace hydrus
