//! Application settings are compiled Zig values; this file contains types only.
const std = @import("std");
const math = @import("math.zig");
/// Image-based visual servoing. The interaction matrix maps image error into a body error.
pub const Ibvs = struct {
    center_deadband: f32,
    target_height: f32,
    /// Rows are body X, Y, Z, roll, pitch, then yaw. Row count must match [`math.interaction_output_len`].
    /// Each row length must match the runtime error vector passed to [`math.mapInteraction`].
    interaction: []const []const f32,
};

pub const OdometryPid = struct {
    /// Body attitude error magnitude (rad) above which horizontal translation is zeroed.
    rotation_gate: f32 = std.math.pi / 8.0,
    kp: math.Vector6f,
    ki: math.Vector6f,
    kd: math.Vector6f,
    /// Use body velocity and gyro for D, avoiding kicks at waypoint changes.
    derivative_on_measurement: bool = false,
    /// World coordinates keep depth independent of horizontal waypoint error
    /// when the hull tilts. Body coordinates preserve legacy gain conventions.
    translation_frame: enum { body, world } = .body,
    /// Bounds on the I contribution and total PID output, in command units.
    integral_limit: math.Vector6f = @splat(std.math.inf(f32)),
    output_limit: math.Vector6f = @splat(std.math.inf(f32)),
    /// Known steady force to apply in world axes (e.g. weight minus buoyancy).
    feedforward_world_force: math.Vector3f = @splat(0),
};

pub const Controller = struct {
    teleop_port: u16 = 8080,
    /// Body direction the camera looks along, such as `[0, 1, 0]` for body +Y.
    camera_forward: math.Vector3f,
    /// Max |rotation| about body X, Y, and Z (rad) when building a look from a world direction. Zero holds that axis at the reference.
    rotation_limit: math.Vector3f = .{ std.math.inf(f32), std.math.inf(f32), std.math.inf(f32) },
    tam: []const math.Vector6f,
    thruster_output_scale: f32 = 5,
    /// RPM-controlled propellers produce signed quadratic thrust at zero inflow.
    thruster_curve: enum { linear, quadratic_rpm } = .linear,
    odometry: OdometryPid,
    ibvs: Ibvs,
};

pub const Application = struct {
    controller: Controller,
    mission: enum { prequalify_coco, prequalify_robosub, teleop } = .prequalify_robosub,
    goal_dist_threshold: f32 = 1,
    plugin: [:0]const u8 = "libauv_simulator.so",
    backend: [:0]const u8 = "libsimulator_backend_mujoco.so",
    robot: [:0]const u8 = "hydrus",
    scene: [:0]const u8 = "robosub_course",
    frame_ticks: u64 = 6,
    max_ticks: u64 = 21600,
    realtime: bool = false,
    physics: struct { dt_numerator: u32 = 1, dt_denominator: u32 = 360, hydro_period: u32 = 7 } = .{},
    telemetry: bool = true,
    blender: struct { enabled: bool = false, port: u16 = 9760 } = .{},
    run_result_path: ?[:0]const u8 = null,
    verbose: bool = true,
    telemetry_address: [:0]const u8 = "127.0.0.1:5600",
    reload: struct { watch: bool = true, preserve_state: bool = true } = .{},
    fuzz: struct {
        seed: u64 = 1,
        cases: usize = 100,
        workers: usize = 8,
        results: []const u8 = "fuzz-results",
        nominal: bool = false,
        profile: @import("SceneContext.zig").FuzzProfile = .{},
    } = .{},
};
pub fn validate(c: Controller) !void {
    if (c.tam.len == 0 or c.tam.len > 8 or !std.math.isFinite(c.thruster_output_scale) or c.thruster_output_scale <= 0)
        return error.InvalidControlConfiguration;
    for (c.tam) |row| inline for (0..6) |i| {
        if (!std.math.isFinite(row[i])) return error.InvalidControlConfiguration;
    };
    if (c.teleop_port == 0 or !std.math.isFinite(c.odometry.rotation_gate) or c.odometry.rotation_gate < 0 or
        !std.math.isFinite(c.ibvs.center_deadband) or c.ibvs.center_deadband < 0 or
        !std.math.isFinite(c.ibvs.target_height) or c.ibvs.target_height <= 0)
        return error.InvalidControlConfiguration;
    inline for (0..6) |i| if (!std.math.isFinite(c.odometry.kp[i]) or !std.math.isFinite(c.odometry.ki[i]) or !std.math.isFinite(c.odometry.kd[i])) return error.InvalidControlConfiguration;
    inline for (0..3) |i| {
        if (!std.math.isFinite(c.camera_forward[i]) or !std.math.isFinite(c.odometry.feedforward_world_force[i]) or
            std.math.isNan(c.rotation_limit[i]) or c.rotation_limit[i] < 0) return error.InvalidControlConfiguration;
    }
    if (math.length3f(c.camera_forward) <= 1e-6) return error.InvalidControlConfiguration;
    inline for (0..6) |i| {
        if (std.math.isNan(c.odometry.integral_limit[i]) or c.odometry.integral_limit[i] < 0 or
            std.math.isNan(c.odometry.output_limit[i]) or c.odometry.output_limit[i] < 0) return error.InvalidControlConfiguration;
    }
    if (c.ibvs.interaction.len != 6) return error.InvalidControlConfiguration;
    for (c.ibvs.interaction) |row| {
        if (row.len != 3) return error.InvalidControlConfiguration;
        for (row) |v| if (!std.math.isFinite(v)) return error.InvalidControlConfiguration;
    }
}
pub fn validateApplication(a: Application) !void {
    try validate(a.controller);
    try a.fuzz.profile.validate();
    if (a.plugin.len == 0 or a.backend.len == 0 or a.robot.len == 0 or a.scene.len == 0 or (if (a.run_result_path) |p| p.len == 0 else false)) return error.InvalidApplicationConfiguration;
    if (a.blender.port == 0 or a.physics.dt_numerator == 0 or a.physics.dt_denominator == 0 or a.physics.hydro_period == 0 or a.frame_ticks == 0 or a.max_ticks == 0 or a.max_ticks % a.frame_ticks != 0 or
        a.fuzz.cases == 0 or a.fuzz.workers == 0 or a.fuzz.results.len == 0 or
        !std.math.isFinite(a.goal_dist_threshold) or a.goal_dist_threshold <= 0)
        return error.InvalidApplicationConfiguration;
}
