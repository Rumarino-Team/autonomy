#include "sim_backend.h"
_Static_assert(sizeof(SimVec3) == 3 * sizeof(double), "vector layout");
_Static_assert(sizeof(SimQuat) == 4 * sizeof(double), "quaternion layout");
_Static_assert(offsetof(SimQuat, w) == 3 * sizeof(double), "xyzw quaternion");
_Static_assert(sizeof(SimTriangle) == 3 * sizeof(uint32_t), "triangle layout");
int main(void) {
  SimBackendApi api = {0};
  return api.abi_version != 0;
}
