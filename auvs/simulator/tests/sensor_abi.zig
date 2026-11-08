const std = @import("std");
const c = @import("sim_c");
extern fn simulator_sensor_abi_size() usize;
extern fn simulator_sample_abi_size() usize;
extern fn simulator_sensor_abi_alignment() usize;
extern fn simulator_sample_abi_alignment() usize;
extern fn simulator_sensor_settings_offset() usize;
extern fn simulator_sample_value_offset() usize;
extern fn simulator_frame_abi_size() usize;
extern fn simulator_status_abi_size() usize;

test "shared C sensor and AUV layouts match Zig C imports" {
    try std.testing.expectEqual(@sizeOf(c.SensorRec), simulator_sensor_abi_size());
    try std.testing.expectEqual(@sizeOf(c.SensorSample), simulator_sample_abi_size());
    try std.testing.expectEqual(@alignOf(c.SensorRec), simulator_sensor_abi_alignment());
    try std.testing.expectEqual(@alignOf(c.SensorSample), simulator_sample_abi_alignment());
    try std.testing.expectEqual(@offsetOf(c.SensorRec, "settings"), simulator_sensor_settings_offset());
    try std.testing.expectEqual(@offsetOf(c.SensorSample, "value"), simulator_sample_value_offset());
    try std.testing.expectEqual(@sizeOf(c.AuvFrame), simulator_frame_abi_size());
    try std.testing.expectEqual(@sizeOf(c.AuvSimulationStatus), simulator_status_abi_size());
    try std.testing.expectEqual(@as(c_uint, 5), c.SIM_MODEL_ABI_VERSION);
}

extern fn simulator_options_size() usize;
extern fn simulator_result_size() usize;
extern fn simulator_result_robot_offset() usize;
test "simulator extension C/Zig layout" {
    try std.testing.expectEqual(@sizeOf(c.AuvSimulatorOptions), simulator_options_size());
    try std.testing.expectEqual(@sizeOf(c.AuvSimulatorResult), simulator_result_size());
    try std.testing.expectEqual(@offsetOf(c.AuvSimulatorResult, "robot"), simulator_result_robot_offset());
}
