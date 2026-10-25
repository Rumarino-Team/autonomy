const std = @import("std");
const math = @import("math.zig");

pub const Options = struct {
    /// Position error -> world velocity, s^-1.
    position_gain: math.Vector3f = .{ 1, 1, 1 },
    max_velocity: math.Vector3f = .{ 0.6, 0.6, 0.4 },
    velocity_kp: math.Vector3f = .{ 6, 6, 10 },
    velocity_ki: math.Vector3f = .{ 0, 0, 0.5 },
    velocity_kd: math.Vector3f = .{ 0, 0, 0 },
    /// Rotation vector -> body angular rate, s^-1.
    attitude_gain: math.Vector3f = .{ 2, 2, 2 },
    max_angular_rate: math.Vector3f = .{ 1.2, 1.2, 1.2 },
    max_angular_acceleration: math.Vector3f = .{ 2, 2, 2 },
    rate_kp: math.Vector3f = .{ 1.5, 1.5, 1.5 },
    rate_ki: math.Vector3f = .{ 0.1, 0.1, 0.1 },
    rate_kd: math.Vector3f = .{ 0.02, 0.02, 0.02 },
    derivative_filter_time: f32 = 0.04,
    integral_limit: f32 = 2,
    /// Per-axis clamp for the world and depth velocity integrators, in newtons.
    /// Zero on an axis keeps `integral_limit`. Surge needs a larger clamp because
    /// skin drag is a steady force, not a velocity error.
    velocity_integral_limit: math.Vector3f = .{ 0, 0, 0 },
    anti_windup_gain: f32 = 2,
    /// Net submerged weight, positive down, in newtons: dry weight minus buoyancy.
    /// Added to the depth command so the velocity loop trims around neutral.
    submerged_weight: f32 = 0,
    /// N per unit normalized body effort.
    body_force_scale: math.Vector3f = .{ 5, 5, 5 },
    /// Thruster command limit in (0, 1]. Zero keeps each motion's own limit.
    max_effort: f32 = 0,
    /// Physical body wrench -> old TAM convention; only used without allocation.
    legacy_wrench_scale: math.Vector6f = .{ 1, 1, 1, -1, 1, 1 },

    pub fn validate(options: Options) !void {
        inline for (.{ "position_gain", "max_velocity", "attitude_gain", "max_angular_rate", "max_angular_acceleration", "body_force_scale" }) |field| {
            inline for (0..3) |axis| if (!positive(@field(options, field)[axis])) return error.InvalidControlOptions;
        }
        inline for (.{ "velocity_kp", "velocity_ki", "velocity_kd", "rate_kp", "rate_ki", "rate_kd" }) |field| {
            inline for (0..3) |axis| if (!nonnegative(@field(options, field)[axis])) return error.InvalidControlOptions;
        }
        inline for (0..6) |axis| if (!std.math.isFinite(options.legacy_wrench_scale[axis]) or options.legacy_wrench_scale[axis] == 0) return error.InvalidControlOptions;
        if (!nonnegative(options.derivative_filter_time) or !positive(options.integral_limit) or !nonnegative(options.anti_windup_gain)) return error.InvalidControlOptions;
        if (!std.math.isFinite(options.submerged_weight)) return error.InvalidControlOptions;
        inline for (0..3) |axis| if (!nonnegative(options.velocity_integral_limit[axis])) return error.InvalidControlOptions;
        if (options.max_effort != 0 and !effortLimit(options.max_effort)) return error.InvalidControlOptions;
    }

    pub fn velocityIntegralLimit(options: Options) math.Vector3f {
        var limit: math.Vector3f = @splat(options.integral_limit);
        inline for (0..3) |axis| {
            if (options.velocity_integral_limit[axis] > 0) limit[axis] = options.velocity_integral_limit[axis];
        }
        return limit;
    }
};
fn effortLimit(v: f32) bool {
    return std.math.isFinite(v) and v > 0 and v <= 1;
}
fn positive(v: f32) bool {
    return std.math.isFinite(v) and v > 0;
}
fn nonnegative(v: f32) bool {
    return std.math.isFinite(v) and v >= 0;
}
