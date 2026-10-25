const std = @import("std");
const math = @import("math.zig");
const Motion = @import("Motion.zig");
const Executor = @import("MotionExecutor.zig");
const Navigation = @import("Navigation.zig");
const ConfigLoader = @import("ConfigLoader.zig");
const Control = @import("ControlOptions.zig");
const MotionController = @This();

const Pid = struct {
    /// Integral contribution in output units, not an unbounded sum of error.
    integral: math.Vector3f = @splat(0),
    previous_measurement: math.Vector3f = @splat(0),
    derivative: math.Vector3f = @splat(0),
    initialized: bool = false,

    fn run(pid: *Pid, err: math.Vector3f, measurement: math.Vector3f, kp: math.Vector3f, ki: math.Vector3f, kd: math.Vector3f, dt: f32, options: Control.Options, limit: math.Vector3f) math.Vector3f {
        if (dt > 0) {
            const alpha = dt / (options.derivative_filter_time + dt);
            if (pid.initialized) {
                const derivative = -(measurement - pid.previous_measurement) / @as(math.Vector3f, @splat(dt));
                pid.derivative += (derivative - pid.derivative) * @as(math.Vector3f, @splat(alpha));
            }
            pid.integral += ki * err * @as(math.Vector3f, @splat(dt));
        }
        inline for (0..3) |axis| {
            if (ki[axis] == 0) pid.integral[axis] = 0;
            pid.integral[axis] = std.math.clamp(pid.integral[axis], -limit[axis], limit[axis]);
        }
        pid.previous_measurement = measurement;
        pid.initialized = dt > 0;
        return kp * err + pid.integral + kd * pid.derivative;
    }

    fn backCalculate(pid: *Pid, difference: math.Vector3f, ki: math.Vector3f, dt: f32, options: Control.Options, limit: math.Vector3f) void {
        inline for (0..3) |axis| {
            if (ki[axis] > 0) pid.integral[axis] = std.math.clamp(pid.integral[axis] + options.anti_windup_gain * difference[axis] * dt, -limit[axis], limit[axis]);
        }
    }
};

world: Pid = .{},
depth: Pid = .{},
rate: Pid = .{},
visual: Pid = .{},
last_mode: ?Motion.Mode = null,
last_pressure_controls_z: ?bool = null,
commanded_rate: math.Vector3f = @splat(0),
rate_initialized: bool = false,
last_orientation: math.Quaternionf = .{ 0, 0, 0, 1 },
last_world_active: bool = false,
last_depth_active: bool = false,
last_visual_active: bool = false,
visual_axes: math.Vector3f = @splat(0),
feedback_available: bool = false,
adaptive_pitch: f32 = 0,
pitch_adapting: bool = false,
pitch_limited: bool = false,
adaptation_normal: ?math.Vector3f = null,
planned_force_difference_world: math.Vector3f = @splat(0),

pub fn reset(controller: *MotionController) void {
    controller.* = .{};
}

/// Returns a physical body wrench [N, N, N, Nm, Nm, Nm]. Translation and depth
/// are combined in world axes before rotating all three force components.
/// Quaternion attitude errors request body rates; the inner loop uses gyro
/// feedback (or Navigation's fresh orientation-derived fallback).
pub fn calculate(controller: *MotionController, nav: *const Navigation, target: Executor.Target, config: ConfigLoader.Config, elapsed: f32) math.Vector6f {
    if (controller.last_mode == null or controller.last_mode.? != target.mode or target.stop) controller.reset();
    controller.last_mode = target.mode;
    if (target.stop) return @splat(0);
    if (elapsed < 0 or elapsed > 0.25 or !std.math.isFinite(elapsed)) {
        controller.reset();
        return @splat(0);
    }
    const snapshot = nav.snapshot;
    const orientation = snapshot.orientation orelse {
        controller.reset();
        return @splat(0);
    };
    const angular_velocity = snapshot.angular_velocity_body orelse {
        controller.reset();
        return @splat(0);
    };
    const options = config.control;
    const dt = if (elapsed > 0 and elapsed <= 0.25) elapsed else 0;
    controller.feedback_available = true;
    controller.last_orientation = orientation;
    const pressure_controls_z = target.pressure_depth != null and snapshot.pressure_depth != null and
        (target.visual == null or !target.visual.?.align_vertical);
    if (controller.last_pressure_controls_z == null or controller.last_pressure_controls_z.? != pressure_controls_z) {
        controller.depth = .{};
        controller.world = .{};
    }
    controller.last_pressure_controls_z = pressure_controls_z;
    controller.last_world_active = false;
    controller.last_depth_active = false;
    controller.last_visual_active = false;
    var world_force: math.Vector3f = @splat(0);
    if (target.world_position) |goal| {
        if (snapshot.position != null and snapshot.velocity_world != null) {
            const position_error = goal - snapshot.position.?;
            var desired_velocity = position_error * options.position_gain;
            inline for (0..3) |axis| desired_velocity[axis] = std.math.clamp(desired_velocity[axis], -options.max_velocity[axis], options.max_velocity[axis]);
            desired_velocity[0] *= target.translation_scale;
            desired_velocity[1] *= target.translation_scale;
            var error_velocity = desired_velocity - snapshot.velocity_world.?;
            var measurement = snapshot.velocity_world.?;
            if (pressure_controls_z) {
                error_velocity[2] = 0;
                measurement[2] = 0;
                controller.world.integral[2] = 0;
            }
            world_force = controller.world.run(error_velocity, measurement, options.velocity_kp, options.velocity_ki, options.velocity_kd, dt, options, options.velocityIntegralLimit());
            if (pressure_controls_z) world_force[2] = 0;
            controller.last_world_active = true;
        } else controller.world = .{};
    } else controller.world = .{};
    if (pressure_controls_z and snapshot.depth_rate != null) {
        const desired_rate = std.math.clamp(options.position_gain[2] * (target.pressure_depth.? - snapshot.pressure_depth.?), -options.max_velocity[2], options.max_velocity[2]);
        const error_velocity: math.Vector3f = .{ 0, 0, desired_rate - snapshot.depth_rate.? };
        const measurement: math.Vector3f = .{ 0, 0, snapshot.depth_rate.? };
        const force = controller.depth.run(error_velocity, measurement, options.velocity_kp, options.velocity_ki, options.velocity_kd, dt, options, options.velocityIntegralLimit());
        world_force[2] += force[2];
        controller.last_depth_active = true;
    } else controller.depth = .{};
    // Constant weight is not a velocity error. Cancel it in world axes so the
    // integrator only trims the residual, including while pitch adaptation
    // redirects the same force.
    if (controller.last_world_active or controller.last_depth_active) world_force[2] -= options.submerged_weight;
    // Euler input packets are converted to a quaternion once; no Euler error
    // or Euler-rate derivative is used inside the attitude/rate feedback loops.
    var desired = target.orientation orelse math.quaternionFromEuler(.{ target.roll_pitch[0], target.roll_pitch[1], target.heading orelse snapshot.yaw().? });
    const was_adapting = controller.pitch_adapting;
    controller.pitch_adapting = false;
    controller.pitch_limited = false;
    controller.adaptation_normal = null;
    controller.planned_force_difference_world = @splat(0);
    if (target.attitude_policy == .allow_pitch and target.world_position != null and target.orientation == null) {
        if (target.translation_plane_normal) |normal| {
            if (@abs(normal[0]) >= 0.999 and @abs(target.roll_pitch[0]) <= 0.05) {
                const heading = target.heading orelse snapshot.yaw().?;
                const plan = pitchForForce(world_force, heading, target.max_pitch);
                if (!was_adapting) controller.adaptive_pitch = std.math.clamp(math.quaternionToEuler(orientation)[1], -target.max_pitch, target.max_pitch);
                const step = target.pitch_rate_limit * dt;
                controller.adaptive_pitch = std.math.clamp(controller.adaptive_pitch + std.math.clamp(plan.pitch - controller.adaptive_pitch, -step, step), -target.max_pitch, target.max_pitch);
                controller.pitch_adapting = true;
                controller.pitch_limited = plan.limited;
                controller.adaptation_normal = normal;
                controller.planned_force_difference_world = plan.force - world_force;
                world_force = plan.force;
                desired = math.quaternionFromEuler(.{ target.roll_pitch[0], controller.adaptive_pitch, heading });
            }
        }
    }
    var body_force = math.quaternionRotate(math.quaternionConjugate(orientation), world_force) + target.body_effort * options.body_force_scale;
    var requested_rate = options.attitude_gain * math.quaternionError(orientation, desired);
    if (target.visual) |visual| {
        const axes: math.Vector3f = .{ 0, if (visual.approach_active) 1 else 0, if (visual.align_vertical) 1 else 0 };
        if (@reduce(.Or, axes != controller.visual_axes)) controller.visual = .{};
        controller.visual_axes = axes;
        const error_image: math.Vector3f = .{ 0, visual.approach, if (visual.align_vertical) visual.cy else 0 };
        const gains = config.non_odometry;
        body_force += controller.visual.run(error_image, -error_image, first3(gains.kp) * axes, first3(gains.ki) * axes, first3(gains.kd) * axes, dt, options, @splat(options.integral_limit));
        // Image centering owns body yaw rate; depth/tilt stabilization continues.
        requested_rate[2] = visual.cx * @abs(gains.kp[5]);
        controller.last_visual_active = true;
    } else controller.visual = .{};
    if (!controller.rate_initialized) {
        inline for (0..3) |axis| controller.commanded_rate[axis] = std.math.clamp(angular_velocity[axis], -options.max_angular_rate[axis], options.max_angular_rate[axis]);
        controller.rate_initialized = true;
    }
    inline for (0..3) |axis| {
        const limit = @min(options.max_angular_rate[axis], if (axis == 2) target.yaw_rate_limit else options.max_angular_rate[axis]);
        const requested = std.math.clamp(requested_rate[axis], -limit, limit);
        const step = options.max_angular_acceleration[axis] * dt;
        controller.commanded_rate[axis] += std.math.clamp(requested - controller.commanded_rate[axis], -step, step);
    }
    const torque = controller.rate.run(controller.commanded_rate - angular_velocity, angular_velocity, options.rate_kp, options.rate_ki, options.rate_kd, dt, options, @splat(options.integral_limit));
    const result: math.Vector6f = .{ body_force[0], body_force[1], body_force[2], torque[0], torque[1], torque[2] };
    inline for (0..6) |axis| if (!std.math.isFinite(result[axis])) {
        controller.reset();
        return @splat(0);
    };
    return result;
}

/// Allocation feedback is in physical body units. Rotate force residual back
/// to world axes for world/depth integrators, retain body axes for visual/rate.
pub fn allocationFeedback(controller: *MotionController, requested: math.Vector6f, achieved: math.Vector6f, config: ConfigLoader.Config, elapsed: f32) void {
    if (!controller.feedback_available or elapsed <= 0 or elapsed > 0.25) return;
    const options = config.control;
    const difference = achieved - requested;
    const body_difference = first3(difference);
    var translation_difference = body_difference;
    if (controller.adaptation_normal) |normal| {
        // During the pitch ramp, attitude is responsible for recovering the
        // missing force direction. Do not unwind that demand before it tilts.
        translation_difference -= normal * @as(math.Vector3f, @splat(math.dot3f(translation_difference, normal)));
    }
    const world_difference = math.quaternionRotate(controller.last_orientation, translation_difference) + controller.planned_force_difference_world;
    if (controller.last_world_active) {
        var residual = world_difference;
        if (controller.last_pressure_controls_z == true) residual[2] = 0;
        controller.world.backCalculate(residual, options.velocity_ki, elapsed, options, options.velocityIntegralLimit());
    }
    if (controller.last_depth_active) controller.depth.backCalculate(.{ 0, 0, world_difference[2] }, options.velocity_ki, elapsed, options, options.velocityIntegralLimit());
    if (controller.last_visual_active) controller.visual.backCalculate(body_difference, first3(config.non_odometry.ki) * controller.visual_axes, elapsed, options, @splat(options.integral_limit));
    controller.rate.backCalculate(.{ difference[3], difference[4], difference[5] }, options.rate_ki, elapsed, options, @splat(options.integral_limit));
}

fn first3(v: math.Vector6f) math.Vector3f {
    return .{ v[0], v[1], v[2] };
}

const PitchPlan = struct { pitch: f32, force: math.Vector3f, limited: bool };

/// At fixed heading, rotation around body Y redirects vertical thrust into
/// sideways thrust. Depth has priority: Fx = Fz*tan(pitch). Cap sideways force
/// to the permitted pitch instead of sacrificing the requested world depth.
fn pitchForForce(force: math.Vector3f, heading: f32, max_pitch: f32) PitchPlan {
    const yaw = math.quaternionFromEuler(.{ 0, 0, heading });
    var local = math.quaternionRotate(math.quaternionConjugate(yaw), force);
    const capacity = @abs(local[2]) * @tan(max_pitch);
    const sideways = std.math.clamp(local[0], -capacity, capacity);
    const limited = @abs(sideways - local[0]) > 1e-4;
    local[0] = sideways;
    var pitch: f32 = 0;
    if (@abs(local[2]) > 1e-4) {
        pitch = std.math.atan2(sideways, local[2]);
        if (pitch > std.math.pi / 2.0) pitch -= std.math.pi;
        if (pitch < -std.math.pi / 2.0) pitch += std.math.pi;
        pitch = std.math.clamp(pitch, -max_pitch, max_pitch);
    }
    return .{ .pitch = pitch, .force = math.quaternionRotate(yaw, local), .limited = limited };
}

test "pitch planning keeps depth and heading and caps infeasible lateral force" {
    const yaw: f32 = 0.7;
    const heading = math.quaternionFromEuler(.{ 0, 0, yaw });
    for ([_]f32{ -1, 1 }) |side| {
        const force = math.quaternionRotate(heading, .{ side, 2, -4 });
        const plan = pitchForForce(force, yaw, std.math.pi / 6.0);
        const attitude = math.quaternionFromEuler(.{ 0, plan.pitch, yaw });
        const body = math.quaternionRotate(math.quaternionConjugate(attitude), plan.force);
        try std.testing.expectApproxEqAbs(@as(f32, 0), body[0], 1e-5);
        try std.testing.expectApproxEqAbs(force[2], plan.force[2], 1e-5);
        try std.testing.expectApproxEqAbs(yaw, math.quaternionToEuler(attitude)[2], 1e-5);
        try std.testing.expect(!plan.limited);
    }
    const limited = pitchForForce(.{ 20, 0, -4 }, 0, std.math.pi / 6.0);
    try std.testing.expect(limited.limited);
    try std.testing.expectApproxEqAbs(@as(f32, -4), limited.force[2], 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, -std.math.pi / 6.0), limited.pitch, 1e-5);
    const neutral = pitchForForce(.{ 20, 0, 0 }, 0, std.math.pi / 6.0);
    try std.testing.expectEqual(@as(f32, 0), neutral.force[0]);
    try std.testing.expectEqual(@as(f32, 0), neutral.pitch);
}

const TestAllocation = @import("ThrusterAllocator.zig");
const test_proteus_thrusters = blk: {
    const model: TestAllocation.Thruster = .{ .forward_force = 143, .reverse_force = 143 };
    var thrusters = [_]TestAllocation.Thruster{model} ** 6;
    thrusters[0].position = .{ 0.205, 0, 0.13 };
    thrusters[1].position = .{ -0.205, 0, 0.13 };
    const mounts = [_]math.Vector3f{ .{ 0.165, 0.265, 0 }, .{ -0.165, 0.265, 0 }, .{ 0.165, -0.265, 0 }, .{ -0.165, -0.265, 0 } };
    for (mounts, 2..) |position, i| {
        thrusters[i].position = position;
        thrusters[i].direction = .{ 0, 0, 1 };
    }
    break :blk thrusters;
};

fn testProteusAllocator() !TestAllocation {
    return TestAllocation.init(.{ .thrusters = &test_proteus_thrusters, .require_full_rank = false }, null);
}

const TestResult = struct { distance: f32, depth_error: f32, yaw_error: f32, max_pitch: f32 };
fn simulateSideways(policy: Motion.AttitudePolicy, sideways: f32) !TestResult {
    var allocator = try testProteusAllocator();
    try std.testing.expect(allocator.supportsPitchAdaptation());
    const yaw: f32 = 0.7;
    var orientation = math.quaternionFromEuler(.{ 0, 0, yaw });
    var position: math.Vector3f = .{ 0, 0, 2 };
    var velocity: math.Vector3f = @splat(0);
    var angular_velocity: math.Vector3f = @splat(0);
    const goal = position + math.quaternionRotate(orientation, .{ sideways, 0, 0 });
    var controller: MotionController = .{};
    var nav: Navigation = .{};
    const config: ConfigLoader.Config = .{
        .non_odometry = .{ .kp = @splat(0), .ki = @splat(0), .kd = @splat(0), .center_deadband = 0.05, .target_height = 0.9 },
        .control = .{ .velocity_kp = .{ 12, 12, 18 }, .velocity_ki = .{ 2, 2, 2 }, .integral_limit = 8 },
    };
    const dt: f32 = 0.02;
    var max_pitch: f32 = 0;
    for (0..6000) |_| {
        nav.snapshot = .{ .position = position, .orientation = orientation, .velocity_world = velocity, .angular_velocity_body = angular_velocity, .pressure_depth = position[2], .depth_rate = velocity[2], .measured_position = true };
        const input = controller.calculate(&nav, .{ .mode = .world, .world_position = goal, .heading = yaw, .pressure_depth = 2, .attitude_policy = policy, .translation_plane_normal = allocator.translationPlaneNormal() }, config, dt);
        const result = allocator.allocate(input, 1, dt);
        controller.allocationFeedback(input, result.achieved, config, dt);
        const body_velocity = math.quaternionRotate(math.quaternionConjugate(orientation), velocity);
        const force = math.quaternionRotate(orientation, first3(result.achieved) - body_velocity * @as(math.Vector3f, .{ 18, 12, 14 }));
        velocity += (force + @as(math.Vector3f, .{ 0, 0, 3.6 })) * @as(math.Vector3f, @splat(dt / 5.323));
        position += velocity * @as(math.Vector3f, @splat(dt));
        const torque: math.Vector3f = .{ result.achieved[3], result.achieved[4], result.achieved[5] };
        angular_velocity += (torque - angular_velocity * @as(math.Vector3f, @splat(3))) / @as(math.Vector3f, .{ 0.656, 0.700, 1.175 }) * @as(math.Vector3f, @splat(dt));
        const rotation = angular_velocity * @as(math.Vector3f, @splat(dt * 0.5));
        const increment = math.normalize4f(.{ rotation[0], rotation[1], rotation[2], 1 });
        orientation = math.normalize4f(math.quaternionMul(orientation, increment));
        const angles = math.quaternionToEuler(orientation);
        max_pitch = @max(max_pitch, @abs(angles[1]));
        if (!Navigation.validPoint(position) or !Navigation.validQuat(orientation)) return error.NonFiniteFeedback;
    }
    return .{ .distance = math.length3f(goal - position), .depth_error = @abs(position[2] - 2), .yaw_error = @abs(math.wrapAngle(math.quaternionToEuler(orientation)[2] - yaw)), .max_pitch = max_pitch };
}

test "surge integral can cancel steady drag above the shared limit" {
    var controller: MotionController = .{};
    var nav: Navigation = .{};
    nav.snapshot = .{
        .position = .{ 0, 0, 2 },
        .orientation = .{ 0, 0, 0, 1 },
        .velocity_world = @splat(0),
        .angular_velocity_body = @splat(0),
        .pressure_depth = 2,
        .depth_rate = 0,
        .measured_position = true,
    };
    const config: ConfigLoader.Config = .{
        .non_odometry = .{ .kp = @splat(0), .ki = @splat(0), .kd = @splat(0), .center_deadband = 0.05, .target_height = 0.9 },
        .control = .{
            .velocity_kp = .{ 0, 0, 0 },
            .velocity_ki = .{ 0, 10, 0 },
            .max_velocity = .{ 0.6, 0.6, 0.4 },
            .integral_limit = 3,
            .velocity_integral_limit = .{ 40, 40, 0 },
            .position_gain = .{ 1, 1, 1 },
        },
    };
    // One metre of along-track error asks for max forward speed. With kp = 0 the
    // whole force is the integrator, which must be allowed past the 3 N shared cap.
    var force: math.Vector6f = @splat(0);
    for (0..50) |_| {
        force = controller.calculate(&nav, .{ .mode = .world, .world_position = .{ 0, 1, 2 }, .heading = 0, .pressure_depth = 2 }, config, 0.02);
    }
    try std.testing.expectApproxEqAbs(@as(f32, 6), force[1], 0.05);
    try std.testing.expect(@abs(force[2]) < 0.01);
}

test "submerged weight cancels in world axes while depth error is zero" {
    var controller: MotionController = .{};
    var nav: Navigation = .{};
    nav.snapshot = .{
        .position = .{ 0, 0, 2 },
        .orientation = .{ 0, 0, 0, 1 },
        .velocity_world = @splat(0),
        .angular_velocity_body = @splat(0),
        .pressure_depth = 2,
        .depth_rate = 0,
        .measured_position = true,
    };
    const config: ConfigLoader.Config = .{
        .non_odometry = .{ .kp = @splat(0), .ki = @splat(0), .kd = @splat(0), .center_deadband = 0.05, .target_height = 0.9 },
        .control = .{ .submerged_weight = 11.1885 },
    };
    const input = controller.calculate(&nav, .{ .mode = .world, .world_position = .{ 0, 0, 2 }, .heading = 0, .pressure_depth = 2 }, config, 0.02);
    try std.testing.expectApproxEqAbs(@as(f32, 0), input[0], 1e-4);
    try std.testing.expectApproxEqAbs(@as(f32, 0), input[1], 1e-4);
    try std.testing.expectApproxEqAbs(@as(f32, -11.1885), input[2], 1e-4);
    const stopped = controller.calculate(&nav, .{ .stop = true }, config, 0.02);
    try std.testing.expectEqual(@as(f32, 0), stopped[2]);
}

test "five-axis vehicle moves sideways with heading held when pitch is released" {
    for ([_]f32{ -0.8, 0.8 }) |side| {
        const result = try simulateSideways(.allow_pitch, side);
        try std.testing.expect(result.distance < 0.08);
        try std.testing.expect(result.depth_error < 0.04);
        try std.testing.expect(result.yaw_error < 0.01);
        try std.testing.expect(result.max_pitch > 0.1);
        try std.testing.expect(result.max_pitch < std.math.pi / 6.0 + 0.02);
    }
    const fixed = try simulateSideways(.fixed, 0.8);
    try std.testing.expect(fixed.distance > 0.7);
    try std.testing.expect(fixed.max_pitch < 0.02);
}
