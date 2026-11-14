//! AUV pose control and thruster actuation implement the generic control pipeline.
const std = @import("std");
const math = @import("math.zig");
const bc = @import("BodyControl.zig");
const Config = @import("Configuration.zig").Controller;
const print_stuff = true;
const clear_print_stuff = true;

fn slew3(current: math.Vector3f, desired: math.Vector3f, max_step: f32) math.Vector3f {
    if (max_step <= 0) return current;
    const delta = desired - current;
    const len = math.length3f(delta);
    if (len <= max_step or len <= 1e-6) return desired;
    return current + delta / @as(math.Vector3f, @splat(len / max_step));
}

fn quaternionFromRotationVector(rot: math.Vector3f) math.Quaternionf {
    const angle = math.length3f(rot);
    if (angle <= 1e-6) return bc.identity;
    return bc.forwardRotation(rot, angle) catch bc.identity;
}

/// The mission owns the state so its existing reload snapshot remains valid.
pub fn Controller(comptime Context: type) type {
    return struct {
        const Self = @This();
        pub const Input = [6]f32;
        pub const Intent = math.Vector6f;
        context: *Context,

        pub fn update(self: *Self, body_error: Input) !Intent {
            const ctx = self.context;
            const timestamp_ns = ctx.frame.timestamp;
            const dt: ?f32 = if (ctx.pid_prev_timestamp_ns) |previous| blk: {
                const elapsed_ns = timestamp_ns - previous;
                ctx.pid_prev_timestamp_ns = timestamp_ns;
                break :blk @as(f32, @floatFromInt(elapsed_ns)) * 1e-9;
            } else blk: {
                ctx.pid_prev_timestamp_ns = timestamp_ns;
                break :blk null;
            };

            const thrust_direction = bc.ThrustDirections.fromTamLinearForces(ctx.config.tam) catch unreachable;
            const current = math.vec4(ctx.frame.camera_pose.quat);
            const error_body: math.Vector3f = .{ body_error[0], body_error[1], body_error[2] };
            const rot: math.Vector3f = .{ body_error[3], body_error[4], body_error[5] };
            const delta = quaternionFromRotationVector(rot);
            const current_q = bc.normalizedQuat(current) catch bc.identity;
            var target_q = bc.normalizedQuat(math.quaternionMul(current_q, delta)) catch current_q;
            const odom = ctx.config.odometry;
            var velocity_body: math.Vector3f = @splat(0);
            if (dt) |t| {
                if (t > 0) {
                    if (ctx.previous_position) |position| {
                        var velocity_world = (math.vec3(ctx.frame.camera_pose.pos) - position) / @as(math.Vector3f, @splat(t));
                        if (!ctx.frame.position_ok.x) velocity_world[0] = 0;
                        if (!ctx.frame.position_ok.y) velocity_world[1] = 0;
                        if (!ctx.frame.position_ok.z) velocity_world[2] = 0;
                        velocity_body = math.quaternionRotate(math.quaternionConjugate(current_q), velocity_world);
                    }
                }
            }
            ctx.previous_position = if (ctx.frame.position_ok.x and ctx.frame.position_ok.y and ctx.frame.position_ok.z) math.vec3(ctx.frame.camera_pose.pos) else null;
            var desired_tilt: math.Vector3f = .{ 0, 0, 0 };
            var force_error: math.Vector3f = .{ 0, 0, 0 };
            const moments = bc.ThrustDirections.fromTamMoments(ctx.config.tam) catch unreachable;
            // Coverage swings a thruster onto a force component the map cannot produce.
            const translation_command: math.Vector3f = blk: {
                force_error = error_body;
                const error_control = if (odom.translation_frame == .world) math.quaternionRotate(current_q, error_body) else error_body;
                const velocity_control = if (odom.translation_frame == .world) math.quaternionRotate(current_q, velocity_body) else velocity_body;
                var force_control: math.Vector3f = .{ 0, 0, 0 };
                inline for (0..3) |i| {
                    force_control[i] = ctx.pid[i].step(error_control[i], if (odom.derivative_on_measurement) velocity_control[i] else null, dt, odom.kp[i], odom.ki[i], odom.kd[i], odom.integral_limit[i], odom.output_limit[i]);
                }
                const force_body = if (odom.translation_frame == .world) math.quaternionRotate(math.quaternionConjugate(current_q), force_control) else force_control;
                const coverage = bc.coverForce(thrust_direction, moments, force_body, ctx.config.rotation_limit, ctx.config.camera_forward);
                if (coverage.tilt) |tilt| desired_tilt = tilt.axis * @as(math.Vector3f, @splat(tilt.angle));
                break :blk coverage.force;
            };
            // The tilt follows the reshaped force. Slew it so the attitude loop does not see a step.
            const tilt_step = if (dt) |t| 0.8 * t else 0;
            ctx.tilt_correction = slew3(ctx.tilt_correction, desired_tilt, tilt_step);
            const tilt_len = math.length3f(ctx.tilt_correction);
            if (tilt_len > 1e-4) {
                const tilt_q = bc.forwardRotation(ctx.tilt_correction, tilt_len) catch unreachable;
                target_q = bc.normalizedQuat(math.quaternionMul(target_q, tilt_q)) catch unreachable;
            }
            var attitude_error = bc.attitudeError(current, target_q) catch unreachable;
            const attitude_axis = [3]bool{ ctx.frame.attitude_ok.x, ctx.frame.attitude_ok.y, ctx.frame.attitude_ok.z };
            inline for (0..3) |i| {
                if (!attitude_axis[i]) attitude_error[i] = 0;
            }
            const translation_body: math.Vector3f = blk: {
                var force = thrust_direction.alongThrust(translation_command);
                if (math.length3f(attitude_error) > odom.rotation_gate) {
                    if (odom.translation_frame == .world) force = math.quaternionRotate(current_q, force);
                    force[0] = 0;
                    force[1] = 0;
                    if (odom.translation_frame == .world) force = math.quaternionRotate(math.quaternionConjugate(current_q), force);
                }
                force += math.quaternionRotate(math.quaternionConjugate(current_q), odom.feedforward_world_force);
                break :blk force;
            };
            ctx.force_error = force_error;

            var rotation: math.Vector3f = .{ 0, 0, 0 };
            inline for (0..3) |i| {
                const lane = i + 3;
                if (attitude_axis[i]) {
                    rotation[i] = ctx.pid[lane].step(attitude_error[i], if (odom.derivative_on_measurement) ctx.frame.gyro.buf[i] else null, dt, odom.kp[lane], odom.ki[lane], odom.kd[lane], odom.integral_limit[lane], odom.output_limit[lane]);
                } else {
                    ctx.pid[lane] = .{};
                }
            }
            const input: math.Vector6f = .{
                translation_body[0], translation_body[1], translation_body[2],
                rotation[0],         rotation[1],         rotation[2],
            };
            if (ctx.verbose and print_stuff and ctx.log_counter % ctx.log_freq_div == 0) {
                if (clear_print_stuff) {
                    std.debug.print("\x1b[2J\x1b[H", .{});
                }
                std.log.debug("control:", .{});
                math.debug6f("\tkp", odom.kp);
                math.debug6f("\tki", odom.ki);
                math.debug6f("\tkd", odom.kd);
                for (ctx.config.tam) |row| {
                    math.debug6f("\ttam", row);
                }
                std.log.debug("dt = {d:6.2} ms", .{(dt orelse 0) * 1000});
                std.log.debug("timestamp_ns = {d}", .{timestamp_ns});
                math.debug6f("pose", math.poseTo6f(ctx.frame.camera_pose));
                std.log.debug("position ok: {any} attitude ok: {any}", .{ ctx.frame.position_ok, ctx.frame.attitude_ok });
                math.debug3f("translation error", error_body);
                std.log.debug("target orientation: {any}", .{ctx.target_orientation});
                std.log.debug("attitude error body: {any}", .{attitude_error});
                math.debug3f("rotation limit", ctx.config.rotation_limit);
                std.log.debug("target quaternion: {any}", .{target_q});
                math.debug6f("\tinput", input);
            }
            return input;
        }
    };
}

/// Converts a body-frame wrench request to normalized thruster commands.
pub const ThrusterAllocator = struct {
    const Self = @This();
    pub const Intent = math.Vector6f;
    pub const Command = []f32;
    config: *const Config,
    buffer: *[math.max_thrusters]f32,

    pub fn map(self: *Self, input: Intent) !Command {
        const thruster_values = math.tamToThrusters(
            self.config.tam,
            input,
            self.config.thruster_output_scale,
            self.config.thruster_output_scale,
            self.buffer,
        );
        if (self.config.thruster_curve == .quadratic_rpm) {
            for (thruster_values) |*value| value.* = std.math.sign(value.*) * @sqrt(@abs(value.*));
        }
        return thruster_values;
    }
};
