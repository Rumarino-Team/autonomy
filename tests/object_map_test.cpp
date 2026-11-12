#include "../auvs/proteus_hwd/object_map.hpp"
#include "../auvs/proteus_hwd/coordinates.hpp"

#include <cassert>
#include <cmath>

static MappedDetection detection(AuvObjectCls cls, float y, bool box = true) {
  MappedDetection result{};
  result.cls = cls;
  result.conf = 0.9f;
  result.pos[1] = y;
  result.size[0] = result.size[1] = result.size[2] = 1.f;
  result.distance = std::fabs(y);
  result.has_box2d = box;
  result.box2d_top_left[0] = 10;
  result.box2d_top_left[1] = 20;
  result.box2d_bottom_right[0] = 30;
  result.box2d_bottom_right[1] = 40;
  return result;
}

int main() {
  float converted[3];
  zed_world_to_autonomy(1.f, 2.f, 3.f, converted);
  assert(converted[0] == -2.f && converted[1] == 1.f && converted[2] == -3.f);
  float orientation[4];
  zed_quat_to_autonomy(0.f, 0.f, std::sqrt(0.5f), std::sqrt(0.5f), orientation);
  assert(std::fabs(orientation[0]) < 1e-5f && std::fabs(orientation[1]) < 1e-5f);
  assert(std::fabs(orientation[2] - std::sqrt(0.5f)) < 1e-5f);
  assert(std::fabs(orientation[3] - std::sqrt(0.5f)) < 1e-5f);

  object_map_reset();
  object_map_init(nullptr);
  assert(object_map_class_enabled(AUV_COCO_PERSON));
  assert(object_map_class_enabled(AUV_COCO_BOTTLE));
  assert(!object_map_class_enabled(AUV_COCO_CAT));

  MappedDetection first[] = {detection(AUV_COCO_PERSON, 1.f), detection(AUV_COCO_CAT, 1.f)};
  object_map_update(first, 2);
  AuvFrame frame{};
  object_map_fill_frame(&frame);
  assert(frame.objects_len == 1 && frame.objects2d_len == 1);
  assert(frame.objects[0].cls == AUV_COCO_PERSON);
  assert(frame.objects[0].id == frame.objects2d[0].id);
  const uint32_t id = frame.objects[0].id;

  auto second = detection(AUV_COCO_PERSON, 1.5f);
  object_map_update(&second, 1);
  auto third = detection(AUV_COCO_PERSON, 2.f);
  object_map_update(&third, 1);
  object_map_fill_frame(&frame);
  assert(frame.objects_len == 1 && frame.objects[0].id == id);
  assert(frame.objects[0].bbox.pose.pos.buf[1] == 1.5f); // median of history
  assert(frame.objects2d[0].top_left.x == 10 && frame.objects2d[0].bottom_right.y == 40);

  auto bottle = detection(AUV_COCO_BOTTLE, 4.f, false);
  object_map_update(&bottle, 1);
  object_map_fill_frame(&frame);
  assert(frame.objects_len == 2 && frame.objects2d_len == 1);
  object_map_reset();
  object_map_fill_frame(&frame);
  assert(frame.objects_len == 0 && frame.objects2d_len == 0);
}
