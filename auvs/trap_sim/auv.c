#include "../../include/auv.h"

void auv_init(void) {
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

void auv_deinit(void) {
  __builtin_trap();
}
