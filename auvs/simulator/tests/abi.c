#include "sim.h"
#include <stddef.h>
_Static_assert(sizeof(MathVector3f) == 16, "float vector ABI");
_Static_assert(_Alignof(MathVector6f) == 32, "spatial alignment");
_Static_assert(offsetof(ObjectBlock, id) == 0, "intrusive header first");
_Static_assert(offsetof(IntrusiveDataNode, link) == 0, "link first");
_Static_assert(sizeof(DataKind) == 4, "explicit enum storage");
_Static_assert(sizeof(SimTriangle) == 12, "triangle layout");
int abi_other_translation_unit(void);

int main(void) {
    SimulationConfig c;
    simulation_config_default(&c);
    return c.object_capacity != 32 || abi_other_translation_unit() != 5;
}
