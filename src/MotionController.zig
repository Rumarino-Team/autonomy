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

    fn run(pid: *Pid, err: math.Vector3f, measurement: math.Vector3f, kp: math.Vector3f, ki: math.Vector3f, kd: math.Vector3f, dt: f32, options: Control.Options) math.Vector3f {
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
            pid.integral[axis] = std.math.clamp(pid.integral[axis], -options.integral_limit, options.integral_limit);
        }
        pid.previous_measurement = measurement;
        pid.initialized = dt > 0;
        return kp * err + pid.integral + kd * pid.derivative;
    }

    fn backCalculate(pid: *Pid, difference: math.Vector3f, ki: math.Vector3f, dt: f32, options: Control.Options) void {
        inline for (0..3) |axis| {
            if (ki[axis] > 0) pid.integral[axis] = std.math.clamp(pid.integral[axis] + options.anti_windup_gain * difference[axis] * dt, -options.integral_limit, options.integral_limit);
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
            world_force = controller.world.run(error_velocity, measurement, options.velocity_kp, options.velocity_ki, options.velocity_kd, dt, options);
            if (pressure_controls_z) world_force[2] = 0;
            controller.last_world_active = true;
        } else controller.world = .{};
    } else controller.world = .{};
    if (pressure_controls_z and snapshot.depth_rate != null) {
        const desired_rate = std.math.clamp(options.position_gain[2] * (target.pressure_depth.? - snapshot.pressure_depth.?), -options.max_velocity[2], options.max_velocity[2]);
        const error_velocity: math.Vector3f = .{ 0, 0, desired_rate - snapshot.depth_rate.? };
        const measurement: math.Vector3f = .{ 0, 0, snapshot.depth_rate.? };
        const force = controller.depth.run(error_velocity, measurement, options.velocity_kp, options.velocity_ki, options.velocity_kd, dt, options);
        world_force[2] += force[2];
        controller.last_depth_active = true;
    } else controller.depth = .{};
    var body_force = math.quaternionRotate(math.quaternionConjugate(orientation), world_force) + target.body_effort * options.body_force_scale;

    // Euler input packets are converted to a quaternion once; no Euler error
    // or Euler-rate derivative is used inside the attitude/rate feedback loops.
    const desired = target.orientation orelse math.quaternionFromEuler(.{ target.roll_pitch[0], target.roll_pitch[1], target.heading orelse snapshot.yaw().? });
    var requested_rate = options.attitude_gain * math.quaternionError(orientation, desired);
    if (target.visual) |visual| {
        const axes: math.Vector3f = .{ 0, if (visual.approach_active) 1 else 0, if (visual.align_vertical) 1 else 0 };
        if (@reduce(.Or, axes != controller.visual_axes)) controller.visual = .{};
        controller.visual_axes = axes;
        const error_image: math.Vector3f = .{ 0, visual.approach, if (visual.align_vertical) visual.cy else 0 };
        const gains = config.non_odometry;
        body_force += controller.visual.run(error_image, -error_image, first3(gains.kp) * axes, first3(gains.ki) * axes, first3(gains.kd) * axes, dt, options);
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
    const torque = controller.rate.run(controller.commanded_rate - angular_velocity, angular_velocity, options.rate_kp, options.rate_ki, options.rate_kd, dt, options);
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
    const world_difference = math.quaternionRotate(controller.last_orientation, body_difference);
    if (controller.last_world_active) {
        var residual = world_difference;
        if (controller.last_pressure_controls_z == true) residual[2] = 0;
        controller.world.backCalculate(residual, options.velocity_ki, elapsed, options);
    }
    if (controller.last_depth_active) controller.depth.backCalculate(.{ 0, 0, world_difference[2] }, options.velocity_ki, elapsed, options);
    if (controller.last_visual_active) controller.visual.backCalculate(body_difference, first3(config.non_odometry.ki) * controller.visual_axes, elapsed, options);
    controller.rate.backCalculate(.{ difference[3], difference[4], difference[5] }, options.rate_ki, elapsed, options);
}

fn first3(v: math.Vector6f) math.Vector3f {
    return .{ v[0], v[1], v[2] };
}
