//! Ackermann bicycle-model actuation. This module is independent of the AUV
//! simulator and can be connected to a car driver with its own command API.
const std = @import("std");

/// Signed curvature is in 1/metre; positive curvature turns left. Speed is m/s.
pub const Intent = struct { speed: f32, curvature: f32 };
/// Virtual front-wheel steering angle, in radians, and target speed in m/s.
pub const Command = struct { speed: f32, steering_angle: f32 };

/// The reference point is supplied by a path planner or lookahead selector.
pub const TrackingInput = struct {
    x: f32,
    y: f32,
    heading: f32,
    target_x: f32,
    target_y: f32,
    target_speed: f32,
};

/// Pure pursuit at the rear axle: direct the vehicle along the circle through
/// the chosen lookahead point. Supports signed forward and reverse speeds.
pub const PathTracker = struct {
    pub const Input = TrackingInput;
    pub const Intent = ThisModule.Intent;

    pub fn update(_: *@This(), input: TrackingInput) !ThisModule.Intent {
        if (!std.math.isFinite(input.x) or !std.math.isFinite(input.y) or
            !std.math.isFinite(input.heading) or !std.math.isFinite(input.target_x) or
            !std.math.isFinite(input.target_y) or !std.math.isFinite(input.target_speed))
            return error.InvalidAckermannReference;
        const dx = @as(f64, input.target_x) - @as(f64, input.x);
        const dy = @as(f64, input.target_y) - @as(f64, input.y);
        const distance_squared = dx * dx + dy * dy;
        if (distance_squared <= 1e-12) return error.AckermannLookaheadTooClose;
        const heading: f64 = input.heading;
        const lateral = -@sin(heading) * dx + @cos(heading) * dy;
        return .{ .speed = input.target_speed, .curvature = @floatCast(2.0 * lateral / distance_squared) };
    }
};

pub const Config = struct {
    wheelbase: f32,
    max_steering_angle: f32,
    max_forward_speed: f32,
    max_reverse_speed: f32,

    pub fn validate(self: Config) !void {
        if (!std.math.isFinite(self.wheelbase) or self.wheelbase <= 0 or
            !std.math.isFinite(self.max_steering_angle) or self.max_steering_angle < 0 or
            self.max_steering_angle >= std.math.pi / 2.0 or
            !std.math.isFinite(self.max_forward_speed) or self.max_forward_speed < 0 or
            !std.math.isFinite(self.max_reverse_speed) or self.max_reverse_speed < 0)
            return error.InvalidAckermannConfiguration;
    }
};

pub const Steering = struct {
    pub const Intent = ThisModule.Intent;
    pub const Command = ThisModule.Command;
    config: Config,

    pub fn map(self: *@This(), intent: ThisModule.Intent) !ThisModule.Command {
        try self.config.validate();
        if (!std.math.isFinite(intent.speed) or !std.math.isFinite(intent.curvature))
            return error.InvalidAckermannIntent;
        // f64 multiplication avoids overflow for finite f32 input values.
        const angle: f32 = @floatCast(std.math.atan2(@as(f64, self.config.wheelbase) * @as(f64, intent.curvature), 1.0));
        return .{
            .speed = std.math.clamp(intent.speed, -self.config.max_reverse_speed, self.config.max_forward_speed),
            .steering_angle = std.math.clamp(angle, -self.config.max_steering_angle, self.config.max_steering_angle),
        };
    }
};

const ThisModule = @This();

/// Kinematic bicycle state at the rear axle, assuming no lateral tire slip.
pub const State = struct { x: f32, y: f32, heading: f32, speed: f32 };
pub const Input = struct { acceleration: f32, steering_angle: f32 };

/// Time derivative, in world coordinates; steering is the virtual wheel angle.
pub fn derivative(config: Config, state: State, input: Input) !State {
    try config.validate();
    if (!std.math.isFinite(state.x) or !std.math.isFinite(state.y) or
        !std.math.isFinite(state.heading) or !std.math.isFinite(state.speed) or
        !std.math.isFinite(input.acceleration) or !std.math.isFinite(input.steering_angle) or
        @abs(input.steering_angle) > config.max_steering_angle)
        return error.InvalidAckermannState;
    const result: State = .{
        .x = state.speed * @cos(state.heading),
        .y = state.speed * @sin(state.heading),
        .heading = state.speed / config.wheelbase * (@sin(input.steering_angle) / @cos(input.steering_angle)),
        .speed = input.acceleration,
    };
    if (!std.math.isFinite(result.x) or !std.math.isFinite(result.y) or !std.math.isFinite(result.heading))
        return error.InvalidAckermannState;
    return result;
}

test "Ackermann pipeline maps curvature and respects steering and speed limits" {
    const Control = @import("ControlSystem.zig");
    const config: Config = .{ .wheelbase = 2, .max_steering_angle = 0.5, .max_forward_speed = 4, .max_reverse_speed = 2 };
    var system: Control.System(Control.Direct(Intent), Steering) = .{ .controller = .{}, .actuation = .{ .config = config } };
    const left = try system.step(.{ .speed = 8, .curvature = 0.1 });
    try std.testing.expectApproxEqAbs(@as(f32, 4), left.speed, 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0.19739556), left.steering_angle, 1e-6);
    const right = try system.step(.{ .speed = -3, .curvature = -10 });
    try std.testing.expectApproxEqAbs(@as(f32, -2), right.speed, 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, -0.5), right.steering_angle, 1e-6);
    try std.testing.expectError(error.InvalidAckermannIntent, system.step(.{ .speed = 0, .curvature = std.math.nan(f32) }));
    system.actuation.config.wheelbase = 0;
    try std.testing.expectError(error.InvalidAckermannConfiguration, system.step(.{ .speed = 0, .curvature = 0 }));
}

test "bicycle model advances heading only while moving" {
    const config: Config = .{ .wheelbase = 2, .max_steering_angle = 0.5, .max_forward_speed = 4, .max_reverse_speed = 2 };
    const moving = try derivative(config, .{ .x = 0, .y = 0, .heading = 0, .speed = 3 }, .{ .acceleration = 1, .steering_angle = 0.2 });
    try std.testing.expectApproxEqAbs(@as(f32, 3), moving.x, 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0.30406505), moving.heading, 1e-6);
    const stopped = try derivative(config, .{ .x = 0, .y = 0, .heading = 0, .speed = 0 }, .{ .acceleration = 0, .steering_angle = 0.2 });
    try std.testing.expectEqual(@as(f32, 0), stopped.heading);
}

test "path tracker produces steering from a lookahead point" {
    const Control = @import("ControlSystem.zig");
    var system: Control.System(PathTracker, Steering) = .{
        .controller = .{},
        .actuation = .{ .config = .{ .wheelbase = 2, .max_steering_angle = 1, .max_forward_speed = 4, .max_reverse_speed = 2 } },
    };
    const left = try system.step(.{ .x = 0, .y = 0, .heading = 0, .target_x = 2, .target_y = 2, .target_speed = 3 });
    try std.testing.expectApproxEqAbs(@as(f32, 0.78539816), left.steering_angle, 1e-6);
    try std.testing.expectEqual(@as(f32, 3), left.speed);
    const straight = try system.step(.{ .x = 0, .y = 0, .heading = std.math.pi / 2.0, .target_x = 0, .target_y = 4, .target_speed = 1 });
    try std.testing.expectApproxEqAbs(@as(f32, 0), straight.steering_angle, 1e-6);
    try std.testing.expectError(error.AckermannLookaheadTooClose, system.step(.{ .x = 1, .y = 2, .heading = 0, .target_x = 1, .target_y = 2, .target_speed = 1 }));
}
