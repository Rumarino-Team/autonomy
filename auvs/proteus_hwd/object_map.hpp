#pragma once

#include "../../include/auv.h"

#include <cstddef>
#include <cstdint>

// Stabilizes per-frame ZED detections into a bounded world map: class allowlist,
// distance-based association, median smoothing, and eviction by expected count.
struct ObjectMapConfig {
  float match_distance_m;
  size_t history_size;
};

void object_map_init(const ObjectMapConfig *config);
void object_map_reset(void);

struct MappedDetection {
  AuvObjectCls cls;
  float conf;
  float pos[3];
  float size[3];
  float distance;
  uint32_t box2d_top_left[2];
  uint32_t box2d_bottom_right[2];
  bool has_box2d;
};

void object_map_update(const MappedDetection *detections, size_t detection_count);

void object_map_fill_frame(AuvFrame *frame);

bool object_map_class_enabled(AuvObjectCls cls);
