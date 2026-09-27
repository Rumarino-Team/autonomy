const std = @import("std");
const Auv = @import("../Auv.zig");
const math = @import("../math.zig");
const controls = @import("../controls.zig");
const ConfigLoader = @import("../ConfigLoader.zig");

pub const yaw_gate_default: f32 = std.math.pi / 8.0;

/// SLAM frame, tracked 3D objects, goal, and global PID memory.
pub const GlobalState = struct {
    frame: Auv.Frame = undefined,
    seen_objects: [Auv.Frame.max_objects]Auv.Object = undefined,
    seen_objects_len: u8 = 0,
    goal: math.Vector6f = @splat(0),
    prev_timestamp_ns: ?u64 = null,
    controller: controls.ControllerState = undefined,
};

pub fn initController(state: *GlobalState, config: ConfigLoader.Config) void {
    state.controller = .{
        .ErrorConstant = config.kp,
        .IntegralConstant = config.ki,
        .DerivativeConstant = config.kd,
    };
}

pub fn syncControllerGains(state: *GlobalState, config: ConfigLoader.Config) void {
    state.controller.ErrorConstant = config.kp;
    state.controller.IntegralConstant = config.ki;
    state.controller.DerivativeConstant = config.kd;
}
