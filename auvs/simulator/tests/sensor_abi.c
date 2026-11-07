#include "simulation_types.h"
#include "auv_simulator.h"

size_t simulator_sensor_abi_size(void) {
    return sizeof(SensorRec);
}

size_t simulator_sample_abi_size(void) {
    return sizeof(SensorSample);
}

size_t simulator_sensor_abi_alignment(void) {
    return _Alignof(SensorRec);
}

size_t simulator_sample_abi_alignment(void) {
    return _Alignof(SensorSample);
}

size_t simulator_sensor_settings_offset(void) {
    return offsetof(SensorRec, settings);
}

size_t simulator_sample_value_offset(void) {
    return offsetof(SensorSample, value);
}

size_t simulator_frame_abi_size(void) {
    return sizeof(AuvFrame);
}

size_t simulator_status_abi_size(void) {
    return sizeof(AuvSimulationStatus);
}

size_t simulator_options_size(void) {
    return sizeof(AuvSimulatorOptions);
}

size_t simulator_result_size(void) {
    return sizeof(AuvSimulatorResult);
}

size_t simulator_result_robot_offset(void) {
    return offsetof(AuvSimulatorResult, robot);
}
