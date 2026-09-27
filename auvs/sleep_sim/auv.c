#include "../../include/auv.h"
#include <assert.h>
#include <stdint.h>
#include <time.h>

#define THRUSTOR_VALUES_COUNT 6
#define FRAME_INTERVAL_NS (1000000000ULL / 60)

static uint64_t next_frame;
static uint64_t next_reactive_frame;

void auv_init(void) {
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);

  next_frame =
      (uint64_t)now.tv_sec * 1000000000ULL +
      (uint64_t)now.tv_nsec +
      FRAME_INTERVAL_NS;
  next_reactive_frame = next_frame;
}

void auv_yield_until_next_frame(AuvFrame *frame) {
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);

  uint64_t current =
      (uint64_t)now.tv_sec * 1000000000ULL +
      (uint64_t)now.tv_nsec;

  if (current < next_frame) {
    uint64_t remaining = next_frame - current;

    struct timespec ts = {
        .tv_sec = remaining / 1000000000ULL,
        .tv_nsec = remaining % 1000000000ULL,
    };

    nanosleep(&ts, NULL);
  }

  clock_gettime(CLOCK_MONOTONIC, &now);

  uint64_t timestamp =
      (uint64_t)now.tv_sec * 1000000000ULL +
      (uint64_t)now.tv_nsec;

  next_frame += FRAME_INTERVAL_NS;

  if (next_frame <= timestamp)
    next_frame = timestamp + FRAME_INTERVAL_NS;

  frame->objects_len = 0;
  frame->camera_pose = (MathPose){
      .pos = {0, 0, 0},
      .quat = {0, 0, 0, 1},
  };
  frame->timestamp = timestamp;
  frame->tracking_ok = true;
}

void auv_yield_until_reactive_frame(AuvReactiveFrame *frame) {
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);

  uint64_t current =
      (uint64_t)now.tv_sec * 1000000000ULL +
      (uint64_t)now.tv_nsec;

  if (current < next_reactive_frame) {
    uint64_t remaining = next_reactive_frame - current;
    struct timespec ts = {
        .tv_sec = remaining / 1000000000ULL,
        .tv_nsec = remaining % 1000000000ULL,
    };
    nanosleep(&ts, NULL);
  }

  clock_gettime(CLOCK_MONOTONIC, &now);
  uint64_t timestamp =
      (uint64_t)now.tv_sec * 1000000000ULL +
      (uint64_t)now.tv_nsec;
  next_reactive_frame += FRAME_INTERVAL_NS;
  if (next_reactive_frame <= timestamp)
    next_reactive_frame = timestamp + FRAME_INTERVAL_NS;

  *frame = (AuvReactiveFrame){
      .quat = {.buf = {0, 0, 0, 1}},
      .object_len = 0,
      .timestamp = timestamp,
      .image_width = 1280,
      .image_height = 720,
  };
}

void auv_set_thrustor_values(
    const float *thrustor_values,
    uint8_t thrustor_values_len)
{
  assert(thrustor_values_len == THRUSTOR_VALUES_COUNT);
  (void)thrustor_values;
}

void auv_deinit(void) {
}
