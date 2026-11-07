#include "arena_allocator.h"
#include "sim.h"
#include "sim_backend.h"
#include "simulation_types.h"

int abi_other_translation_unit(void) {
    return PHYSICS_BACKEND_ABI_VERSION;
}

#include "layout.h"

void simulator_c_layout(size_t *output) {
    const size_t values[] = {SIM_LAYOUT_VALUES};
    for (unsigned i = 0; i < SIM_LAYOUT_COUNT; ++i)
        output[i] = values[i];
}
