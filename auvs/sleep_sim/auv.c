#include "../../include/auv.h"
#include <time.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>

static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
static bool ready = false;

void auv_loop(void) {
  while (true) {
    struct timespec ts = {
        .tv_sec = 0,
        .tv_nsec = 16 * 1000000,
    };

    nanosleep(&ts, NULL);

    pthread_mutex_lock(&mutex);
    ready = true;
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&mutex);
  }
  // __builtin_trap();
}

void auv_yield_until_next_frame(AuvFrame *frame) {
  (void)frame;

  pthread_mutex_lock(&mutex);
  while (!ready) {
    pthread_cond_wait(&cond, &mutex);
  }
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  uint64_t ns = (uint64_t)now.tv_sec * 1000000000LL + now.tv_nsec;
  frame->objects_len = 0;
  frame->camera_pose = (AuvPose){.pos = {20, 1, 0}, .quat = {0, 0, 0, 1}};
  frame->timestamp = ns;
  ready = false;
  pthread_mutex_unlock(&mutex);
  // __builtin_trap();
}

void auv_set_thrustor_values(const float *thrustor_values, uint8_t thrustor_values_len) {
  (void)thrustor_values;
  (void)thrustor_values_len;
  // __builtin_trap();
}
