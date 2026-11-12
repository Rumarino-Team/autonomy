// Port of detection_core.map_objects: class allowlist, proximity matching,
// median smoothing, and eviction by expected per-class count.
#include "object_map.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <unordered_map>
#include <vector>

namespace {

struct ClassSpec {
  AuvObjectCls cls;
  uint8_t expected_count;
};

// Mission-relevant COCO classes and how many stable map entries to keep per class.
static constexpr ClassSpec kClassSpecs[] = {
    {static_cast<AuvObjectCls>(AUV_COCO_PERSON), 1},
    {static_cast<AuvObjectCls>(AUV_COCO_BOTTLE), 1},
};

static constexpr float kDefaultMatchDistanceM = 2.0f;
static constexpr size_t kDefaultHistorySize = 5;
static constexpr size_t kMaxMapObjects = AUV_FRAME_MAX_OBJECTS;

struct MapObject {
  uint32_t track_id;
  AuvObjectCls cls;
  float conf;
  float pos[3];
  float size[3];
  uint32_t box2d_top_left[2];
  uint32_t box2d_bottom_right[2];
  bool has_box2d;
  uint32_t update_count;
};

ObjectMapConfig g_config{kDefaultMatchDistanceM, kDefaultHistorySize};
std::vector<MapObject> g_objects;
std::unordered_map<uint32_t, std::deque<MappedDetection>> g_history;
uint32_t g_next_track_id = 1;

const ClassSpec *find_class_spec(AuvObjectCls cls) {
  for (const auto &spec : kClassSpecs) {
    if (spec.cls == cls)
      return &spec;
  }
  return nullptr;
}

float point_distance(const float a[3], const float b[3]) {
  const float dx = a[0] - b[0];
  const float dy = a[1] - b[1];
  const float dz = a[2] - b[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

MappedDetection median_by_distance(const std::deque<MappedDetection> &history) {
  std::vector<MappedDetection> sorted(history.begin(), history.end());
  std::sort(sorted.begin(), sorted.end(),
            [](const MappedDetection &a, const MappedDetection &b) { return a.distance < b.distance; });
  return sorted[sorted.size() / 2];
}

static void copy_box2d(MapObject *object, const MappedDetection &detection) {
  if (!detection.has_box2d) {
    object->has_box2d = false;
    return;
  }
  object->has_box2d = true;
  object->box2d_top_left[0] = detection.box2d_top_left[0];
  object->box2d_top_left[1] = detection.box2d_top_left[1];
  object->box2d_bottom_right[0] = detection.box2d_bottom_right[0];
  object->box2d_bottom_right[1] = detection.box2d_bottom_right[1];
}

static void apply_smoothed(MapObject *object, const MappedDetection &smoothed) {
  object->conf = smoothed.conf;
  object->pos[0] = smoothed.pos[0];
  object->pos[1] = smoothed.pos[1];
  object->pos[2] = smoothed.pos[2];
  object->size[0] = smoothed.size[0];
  object->size[1] = smoothed.size[1];
  object->size[2] = smoothed.size[2];
  if (smoothed.has_box2d)
    copy_box2d(object, smoothed);
}

void push_history(uint32_t track_id, const MappedDetection &detection) {
  auto &buffer = g_history[track_id];
  if (buffer.size() >= g_config.history_size)
    buffer.pop_front();
  buffer.push_back(detection);
}

void seed_first_of_class(const MappedDetection &detection) {
  const ClassSpec *spec = find_class_spec(detection.cls);
  if (!spec)
    return;

  size_t class_count = 0;
  for (const auto &obj : g_objects) {
    if (obj.cls == detection.cls)
      class_count++;
  }
  if (class_count > 0)
    return;

  MapObject object{};
  object.track_id = g_next_track_id++;
  object.cls = detection.cls;
  object.conf = detection.conf;
  object.pos[0] = detection.pos[0];
  object.pos[1] = detection.pos[1];
  object.pos[2] = detection.pos[2];
  object.size[0] = detection.size[0];
  object.size[1] = detection.size[1];
  object.size[2] = detection.size[2];
  copy_box2d(&object, detection);
  object.update_count = 1;
  g_objects.push_back(object);
  push_history(object.track_id, detection);
}

void add_map_object(const MappedDetection &detection) {
  if (g_objects.size() >= kMaxMapObjects)
    return;

  MapObject object{};
  object.track_id = g_next_track_id++;
  object.cls = detection.cls;
  object.conf = detection.conf;
  object.pos[0] = detection.pos[0];
  object.pos[1] = detection.pos[1];
  object.pos[2] = detection.pos[2];
  object.size[0] = detection.size[0];
  object.size[1] = detection.size[1];
  object.size[2] = detection.size[2];
  copy_box2d(&object, detection);
  object.update_count = 1;
  g_objects.push_back(object);
  push_history(object.track_id, detection);
}

void evict_by_expected_counts() {
  std::vector<MapObject> kept;
  kept.reserve(g_objects.size());

  for (const auto &spec : kClassSpecs) {
    std::vector<MapObject *> class_objects;
    for (auto &obj : g_objects) {
      if (obj.cls == spec.cls)
        class_objects.push_back(&obj);
    }

    if (class_objects.size() <= spec.expected_count) {
      for (auto *obj : class_objects)
        kept.push_back(*obj);
      continue;
    }

    std::sort(class_objects.begin(), class_objects.end(),
              [](const MapObject *a, const MapObject *b) { return a->update_count > b->update_count; });

    for (size_t i = 0; i < class_objects.size(); i++) {
      if (i < spec.expected_count) {
        kept.push_back(*class_objects[i]);
      } else {
        g_history.erase(class_objects[i]->track_id);
      }
    }
  }

  g_objects = std::move(kept);
}

} // namespace

void object_map_init(const ObjectMapConfig *config) {
  if (config)
    g_config = *config;
  else
    g_config = {kDefaultMatchDistanceM, kDefaultHistorySize};
}

void object_map_reset(void) {
  g_objects.clear();
  g_history.clear();
  g_next_track_id = 1;
}

bool object_map_class_enabled(AuvObjectCls cls) { return find_class_spec(cls) != nullptr; }

void object_map_update(const MappedDetection *detections, size_t detection_count) {
  std::vector<MappedDetection> frame;
  frame.reserve(detection_count);
  for (size_t i = 0; i < detection_count; i++) {
    MappedDetection detection = detections[i];
    if (!find_class_spec(detection.cls))
      continue;
    frame.push_back(detection);
    seed_first_of_class(detection);
  }

  std::vector<bool> matched(frame.size(), false);

  for (auto &obj : g_objects) {
    float best_distance = std::numeric_limits<float>::infinity();
    int best_index = -1;

    for (size_t i = 0; i < frame.size(); i++) {
      if (matched[i])
        continue;
      if (obj.cls != frame[i].cls)
        continue;

      const float distance = point_distance(obj.pos, frame[i].pos);
      if (distance < best_distance) {
        best_distance = distance;
        best_index = static_cast<int>(i);
      }
    }

    if (best_index < 0 || best_distance >= g_config.match_distance_m)
      continue;

    const MappedDetection &detection = frame[static_cast<size_t>(best_index)];
    push_history(obj.track_id, detection);
    const MappedDetection smoothed = median_by_distance(g_history[obj.track_id]);
    apply_smoothed(&obj, smoothed);
    obj.update_count++;
    matched[static_cast<size_t>(best_index)] = true;
  }

  for (size_t i = 0; i < frame.size(); i++) {
    if (matched[i])
      continue;
    add_map_object(frame[i]);
  }

  evict_by_expected_counts();
}

void object_map_fill_frame(AuvFrame *frame) {
  if (!frame)
    return;

  frame->objects_len = 0;
  frame->objects2d_len = 0;
  const uint8_t limit = static_cast<uint8_t>(std::min(g_objects.size(), static_cast<size_t>(AUV_FRAME_MAX_OBJECTS)));
  for (uint8_t i = 0; i < limit; i++) {
    const MapObject &mapped = g_objects[i];
    AuvObject &out = frame->objects[i];
    out.id = mapped.track_id;
    out.cls = mapped.cls;
    out.bbox.pose.pos.buf[0] = mapped.pos[0];
    out.bbox.pose.pos.buf[1] = mapped.pos[1];
    out.bbox.pose.pos.buf[2] = mapped.pos[2];
    out.bbox.pose.quat.buf[0] = 0.f;
    out.bbox.pose.quat.buf[1] = 0.f;
    out.bbox.pose.quat.buf[2] = 0.f;
    out.bbox.pose.quat.buf[3] = 1.f;
    out.bbox.size.buf[0] = mapped.size[0];
    out.bbox.size.buf[1] = mapped.size[1];
    out.bbox.size.buf[2] = mapped.size[2];
    frame->objects_len++;

    if (mapped.has_box2d && frame->objects2d_len < AUV_FRAME_MAX_OBJECTS) {
      AuvObject2d &out2d = frame->objects2d[frame->objects2d_len];
      out2d.id = mapped.track_id;
      out2d.cls = mapped.cls;
      out2d.top_left.x = mapped.box2d_top_left[0];
      out2d.top_left.y = mapped.box2d_top_left[1];
      out2d.bottom_right.x = mapped.box2d_bottom_right[0];
      out2d.bottom_right.y = mapped.box2d_bottom_right[1];
      frame->objects2d_len++;
    }
  }
}
