#include "../../include/auv.h"
#include <assert.h>
#include <stdbool.h>
#include <stdatomic.h>

void auv_loop(void) {
  __builtin_trap();
}

void auv_yield_until_next_frame(AuvFrame *frame) {
  (void)frame;
  __builtin_trap();
}

void auv_set_thrustor_values(const float *thrustor_values, uint8_t thrustor_values_len) {
  (void)thrustor_values;
  (void)thrustor_values_len;
  __builtin_trap();
}

void auv_set_stop(void) {
  __builtin_trap();
}
