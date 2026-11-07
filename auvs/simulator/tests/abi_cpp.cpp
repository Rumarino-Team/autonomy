#include "layout.h"
extern "C" void simulator_c_layout(size_t *);
static_assert(
    sizeof(SimWrench) == 48 && offsetof(SimWrench, force) == 16 && offsetof(SimWrench, torque) == 32
);
static_assert(sizeof(SimBodyState) == 96 && offsetof(SimBodyState, pose) == 16);

int main() {
    size_t from_c[SIM_LAYOUT_COUNT];
    const size_t from_cpp[] = {SIM_LAYOUT_VALUES};
    simulator_c_layout(from_c);
    for (unsigned i = 0; i < SIM_LAYOUT_COUNT; ++i)
        if (from_c[i] != from_cpp[i])
            return 1;
    return 0;
}
