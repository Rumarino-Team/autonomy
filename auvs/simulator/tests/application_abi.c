#include "application.h"
#include <stddef.h>
_Static_assert(AUTONOMY_APPLICATION_ABI == 1u, "application ABI");
_Static_assert(offsetof(AutonomyApplicationApi, struct_size) == 0, "descriptor header");
_Static_assert(sizeof(((AutonomyLaunch *)0)->frame_ticks) == 8, "tick width");
_Static_assert(sizeof(((AutonomyHostServices *)0)->argc) == 4, "argument width");
int application_abi_other(void);

int main(void) {
    return application_abi_other() != AUTONOMY_APPLICATION_ABI;
}
