#ifndef SIM_TEST_LAYOUT_H
#define SIM_TEST_LAYOUT_H
#include "sim.h"
#include "auv.h"
#include <stdalign.h>
#define SIM_LAYOUT_VALUES                                                                          \
    sizeof(Simulation), alignof(Simulation), sizeof(ObjectBlock), alignof(ObjectBlock),            \
        offsetof(ObjectBlock, body), offsetof(ObjectBlock, colliders), sizeof(Collider),           \
        sizeof(UnderwaterBody), sizeof(SimBodyState), sizeof(SimWrench),                           \
        offsetof(SimWrench, force), offsetof(SimWrench, torque), sizeof(PhysicsBackendApi),        \
        offsetof(PhysicsBackendApi, world_create), sizeof(SensorRec), alignof(SensorRec),          \
        offsetof(SensorRec, settings), sizeof(SensorSample), alignof(SensorSample),                \
        offsetof(SensorSample, value), sizeof(AuvFrame), offsetof(AuvFrame, timestamp),            \
        sizeof(AuvSimulationStatus)
#define SIM_LAYOUT_COUNT 23u
#endif
