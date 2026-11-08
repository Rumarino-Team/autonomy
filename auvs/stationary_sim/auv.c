#include "../../include/auv.h"
#include <assert.h>
#include <stdint.h>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <time.h>
#endif

#define THRUSTOR_VALUES_COUNT 8
#define FRAME_INTERVAL_NS (1000000000ULL / 60)

static uint64_t next_frame;

static uint64_t now_ns(void) {
#ifdef _WIN32
  static LARGE_INTEGER frequency;
  LARGE_INTEGER counter;
  if (frequency.QuadPart == 0)
    QueryPerformanceFrequency(&frequency);
  QueryPerformanceCounter(&counter);
  return (uint64_t)((counter.QuadPart * 1000000000ULL) / (uint64_t)frequency.QuadPart);
#else
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return (uint64_t)now.tv_sec * 1000000000ULL + (uint64_t)now.tv_nsec;
#endif
}

static void sleep_ns(uint64_t remaining) {
#ifdef _WIN32
  DWORD milliseconds = (DWORD)((remaining + 999999ULL) / 1000000ULL);
  if (milliseconds == 0)
    milliseconds = 1;
  Sleep(milliseconds);
#else
  struct timespec ts = {
      .tv_sec = remaining / 1000000000ULL,
      .tv_nsec = remaining % 1000000000ULL,
  };
  nanosleep(&ts, NULL);
#endif
}

void auv_init(void) {
  next_frame = now_ns() + FRAME_INTERVAL_NS;
}

void auv_yield_until_next_frame(AuvFrame *frame) {
  uint64_t current = now_ns();

  if (current < next_frame)
    sleep_ns(next_frame - current);

  uint64_t timestamp = now_ns();

  next_frame += FRAME_INTERVAL_NS;

  if (next_frame <= timestamp)
    next_frame = timestamp + FRAME_INTERVAL_NS;

  frame->objects_len = 0;
  frame->objects2d_len = 0;
  frame->camera_pose = (MathPose){
      .pos = {0, 0, 0},
      .quat = {0, 0, 0, 1},
  };
  frame->timestamp = timestamp;
  frame->position_ok = (AuvAxisOk){true, true, true};
  frame->attitude_ok = (AuvAxisOk){true, true, true};
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