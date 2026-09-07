#include "../../include/auv.h"
#include <assert.h>
#include <time.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdatomic.h>

static _Atomic bool global_stop = false;

static AuvFrame global_frame;
static bool global_frame_ready = false;
static pthread_mutex_t global_frame_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t global_frame_cond = PTHREAD_COND_INITIALIZER;

#define THRUSTOR_VALUES_COUNT 6
float global_thrustor_values[THRUSTOR_VALUES_COUNT] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
static pthread_mutex_t global_thrustor_values_mutex = PTHREAD_MUTEX_INITIALIZER;

void auv_loop(void) {
  while (!atomic_load(&global_stop)) {
    struct timespec ts = {
        .tv_sec = 0,
        .tv_nsec = 16 * 1000000,
    };

    nanosleep(&ts, NULL);

    pthread_mutex_lock(&global_frame_mutex);
    global_frame_ready = true;
    pthread_cond_signal(&global_frame_cond);
    pthread_mutex_unlock(&global_frame_mutex);
  }
}

void auv_yield_until_next_frame(AuvFrame *frame) {
  (void)frame;

  pthread_mutex_lock(&global_frame_mutex);
  while (!global_frame_ready) {
    pthread_cond_wait(&global_frame_cond, &global_frame_mutex);
  }
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  uint64_t ns = (uint64_t)now.tv_sec * 1000000000LL + now.tv_nsec;
  frame->objects_len = 0;
  frame->camera_pose = (AuvPose){.pos = {0, 0, 0}, .quat = {0, 0, 0, 1}};
  frame->timestamp = ns;
  global_frame_ready = false;
  pthread_mutex_unlock(&global_frame_mutex);
}

void auv_set_thrustor_values(const float *thrustor_values, uint8_t thrustor_values_len) {
  assert(thrustor_values_len == THRUSTOR_VALUES_COUNT);
  pthread_mutex_lock(&global_frame_mutex);
  for (int i = 0; i < thrustor_values_len; i += 1) {
    global_thrustor_values[i] = thrustor_values[i];
  }
  pthread_mutex_unlock(&global_frame_mutex);
}

void auv_set_stop(void) {
  atomic_store(&global_stop, true);
}
