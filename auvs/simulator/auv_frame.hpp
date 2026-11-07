#pragma once
#include "auv.h"
#include "internal.hpp"

namespace core {
inline AuvAxisOk auv_axes(uint32_t mask) {
    return {bool(mask & 1), bool(mask & 2), bool(mask & 4)};
}

inline void fill_auv_frame(AuvFrame *frame, const Observation &o, uint64_t time) {
    *frame = {};
    frame->timestamp = time;
    frame->camera_pose = o.body;
    frame->position_ok = auv_axes(o.position_axes);
    frame->attitude_ok = auv_axes(o.attitude_axes);
    frame->imu_quat = o.imu;
    frame->accel = o.accel;
    frame->gyro = o.gyro;
    frame->pressure_depth = float(o.depth);
    frame->pressure_depth_ok = o.depth_ok;
    frame->image_width = o.image_width;
    frame->image_height = o.image_height;
    for (uint32_t i = 0; i < std::min(o.object_count, uint32_t(AUV_FRAME_MAX_OBJECTS)); ++i) {
        const auto &object = o.objects[i];
        require(object.cls <= UINT8_MAX, "object class cannot be represented by AuvFrame");
        auto &out = frame->objects[frame->objects_len++];
        out = {{object.pose, object.size}, object.id, uint8_t(object.cls)};
        auto &box = frame->objects2d[frame->objects2d_len++];
        box = {{object.x0, object.y0}, {object.x1, object.y1}, object.id, uint8_t(object.cls)};
    }
}
} // namespace core
