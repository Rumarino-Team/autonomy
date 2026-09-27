const Auv = @import("../Auv.zig");
const controls = @import("../controls.zig");
const ConfigLoader = @import("../ConfigLoader.zig");

pub const ReactiveGains = struct {
    k_surge: f32 = 3.0,
    k_heave: f32 = 2.0,
    k_yaw: f32 = 2.0,
    center_deadband: f32 = 0.05,
};

/// IMU / 2D detection frame, tracked boxes, and reactive PID memory.
pub const ReactiveState = struct {
    frame: Auv.ReactiveFrame = undefined,
    seen_objects2d: [Auv.ReactiveFrame.max_objects]Auv.Object2DYolo = undefined,
    seen_objects2d_len: u8 = 0,
    image_width: u32 = 1280,
    image_height: u32 = 720,
    prev_timestamp_ns: ?u64 = null,
    controller: controls.ControllerState = undefined,
};

pub fn initController(state: *ReactiveState, config: ConfigLoader.Config) void {
    state.controller = .{
        .ErrorConstant = config.kp,
        .IntegralConstant = config.ki,
        .DerivativeConstant = config.kd,
    };
}

pub fn syncControllerGains(state: *ReactiveState, config: ConfigLoader.Config) void {
    state.controller.ErrorConstant = config.kp;
    state.controller.IntegralConstant = config.ki;
    state.controller.DerivativeConstant = config.kd;
}
