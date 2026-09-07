// TODO: zed stuff
// TODO: serial arduino io
// #include "../../include/auv.h"
// #include <assert.h>
// #include <sl/Camera.hpp>
//
// static sl::Camera zed;
//
// static sl::Mat image;
// static sl::Pose pose;
// static sl::Objects objects;
//
// static sl::ObjectDetectionRuntimeParameters object_params;
//
// void auv_init(void) {
//   sl::InitParameters params;
//   params.camera_fps = 60;
//
//   auto err = zed.open(params);
//   assert(err == sl::ERROR_CODE::SUCCESS);
//
//   assert(zed.enablePositionalTracking() == sl::ERROR_CODE::SUCCESS);
//
//   assert(zed.enableObjectDetection() == sl::ERROR_CODE::SUCCESS);
// }
//
// void auv_yield_until_next_frame(AuvFrame *frame) {
//   if (zed.grab() != sl::ERROR_CODE::SUCCESS)
//     return;
//
//   zed.getPosition(pose, sl::REFERENCE_FRAME::WORLD);
//
//   zed.retrieveObjects(objects, object_params);
//
//   frame->timestamp = zed.getTimestamp(sl::TIME_REFERENCE::IMAGE);
//
//   frame->camera_pose.pos.buf[0] = pose.getTranslation().tx;
//   frame->camera_pose.pos.buf[1] = pose.getTranslation().ty;
//   frame->camera_pose.pos.buf[2] = pose.getTranslation().tz;
//
//   frame->camera_pose.quat.buf[0] = pose.getOrientation().ox;
//   frame->camera_pose.quat.buf[1] = pose.getOrientation().oy;
//   frame->camera_pose.quat.buf[2] = pose.getOrientation().oz;
//   frame->camera_pose.quat.buf[3] = pose.getOrientation().ow;
//
//   frame->objects_len = 0;
//
//   for (const auto &object : objects.object_list) {
//     if (frame->objects_len >= AUV_FRAME_MAX_OBJECTS)
//       break;
//
//     AuvObject &dst = frame->objects[frame->objects_len++];
//
//     dst.id = object.id;
//
//     dst.pose.pos.buf[0] = object.position.x;
//     dst.pose.pos.buf[1] = object.position.y;
//     dst.pose.pos.buf[2] = object.position.z;
//
//     for (int i = 0; i < 8; ++i) {
//       dst.bounding_box[i].buf[0] = object.bounding_box[i].x;
//       dst.bounding_box[i].buf[1] = object.bounding_box[i].y;
//       dst.bounding_box[i].buf[2] = object.bounding_box[i].z;
//     }
//   }
// }
//
// void auv_set_thrustor_values(const float *thrustor_values,
//                              uint8_t thrustor_values_len) {
//   assert(thrustor_values_len == 6);
//   (void)thrustor_values;
// }
//
// void auv_deinit(void) { zed.close(); }
