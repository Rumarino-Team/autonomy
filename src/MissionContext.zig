const std = @import("std");
const Io = std.Io;
const MissionArgs = @import("MissionArgs.zig");
const math = @import("math.zig");
const bc = @import("BodyControl.zig");

const Auv = @import("Auv.zig");
const AuvLoader = @import("AuvLoader.zig");
const ConfigLoader = @import("ConfigLoader.zig");
const FileWatcher = @import("FileWatcher.zig");
const SerialBinaryWriter = @import("SerialBinaryWriter.zig");

const MissionContext = @This();
const print_stuff = true;
const clear_print_stuff = true;
const telemetry_log_path: ?[]const u8 = null;
const telemetry_addr: []const u8 = "127.0.0.1:5600";

var stop_requested = std.atomic.Value(bool).init(false);

fn requestStop(_: std.posix.SIG) callconv(.c) void {
    stop_requested.store(true, .monotonic);
}

io: Io,

frame: Auv.Frame,

seen_objects: [Auv.Frame.max_objects]Auv.Object,
seen_objects_len: u8,

goal_dist_threshold: f32,
thrustor_saturate: f32,

log_counter: u32,
log_freq_div: u16,

target_position: math.Vector3f,
/// World position latched on the first frame of a hold episode. Null outside hold.
hold_position: ?math.Vector3f = null,
/// Target attitude. Null holds the current orientation through the yield.
target_orientation: ?math.Quaternionf = null,
/// Motion chosen before the frame. The yield turns it into `command`.
motion: Motion = .{ .hold = .{} },
/// Numeric command filled from the current frame. Only the yield writes this.
command: Command = .{},
/// Odometry PID memory in TAM order. Lanes 0..2 are body translation, lanes 3..5 are attitude.
pid_sum_err: math.Vector6f = @splat(0),
pid_prev_err: math.Vector6f = @splat(0),
/// Slew-limited tilt that swings a controllable force onto a missing translation.
tilt_correction: math.Vector3f = .{ 0, 0, 0 },
/// Orientation held while `target_orientation` is null.
held_q: ?math.Quaternionf = null,
/// Last explicit target orientation. A mission-look change clears the attitude lanes of the PID.
explicit_q: ?math.Quaternionf = null,
/// Previous frame's position validity. A falling edge clears the translation integrators.
position_axes: ?Auv.AxisOk = null,
pid_prev_timestamp_ns: ?u64,

step: []const u8,
telemetry_file: ?Io.File = null,
telemetry_socket: Io.net.Socket = undefined,
telemetry_dest: Io.net.IpAddress = undefined,

seen_objects2d: [Auv.Frame.max_objects]Auv.Object2d = undefined,
seen_objects2d_len: u8 = 0,

auv: Auv,
auv_loader: AuvLoader,
auv_watcher: FileWatcher,

config: ConfigLoader.Config,
config_loader: ConfigLoader,
config_watcher: FileWatcher,

pub fn init(arena: std.mem.Allocator, io: Io, args: MissionArgs) !MissionContext {
    const stop: std.posix.Sigaction = .{ .handler = .{ .handler = requestStop }, .mask = std.posix.sigemptyset(), .flags = 0 };
    std.posix.sigaction(.INT, &stop, null);
    std.posix.sigaction(.TERM, &stop, null);

    var config_loader: ConfigLoader = try .init(arena, args.live_config_path);
    const config = config_loader.load(io) catch |err| {
        std.log.err("failed to load config: {s}", .{config_loader.config_path});
        return err;
    };
    if (print_stuff) {
        std.log.debug("succesfully loaded config: {s}", .{config_loader.config_path});
        std.log.debug("{any}", .{config});
    }

    var auv_loader: AuvLoader = try .init(arena, args.auv_dynlib_path);
    const auv = auv_loader.load(io) catch |err| {
        const dl_err = std.mem.span(std.c.dlerror()) orelse "NO_DL_ERROR";
        std.log.err("dlerror: `{s}`", .{dl_err});
        std.log.err("failed to load auv: {s}", .{auv_loader.auv_path});
        return err;
    };
    auv.init();
    if (print_stuff) {
        std.log.debug("succesfully loaded auv: {s}", .{auv_loader.auv_path});
        std.log.debug("{any}", .{auv});
    }

    const telemetry_dest = try Io.net.IpAddress.parseLiteral(telemetry_addr);
    const telemetry_bind: Io.net.IpAddress = .{ .ip4 = .unspecified(0) };
    const telemetry_socket = try telemetry_bind.bind(io, .{ .mode = .dgram });
    const telemetry_file: ?Io.File = if (telemetry_log_path) |path|
        try Io.Dir.cwd().createFile(io, path, .{})
    else
        null;

    return .{
        .frame = undefined,

        .seen_objects = undefined,
        .seen_objects_len = 0,

        .goal_dist_threshold = args.goal_dist_threshold,
        .thrustor_saturate = 5,

        .log_counter = 0,
        .log_freq_div = 120,

        .target_position = @splat(0),

        .pid_prev_timestamp_ns = null,

        .step = "",
        .telemetry_file = telemetry_file,
        .telemetry_socket = telemetry_socket,
        .telemetry_dest = telemetry_dest,

        .auv = auv,
        .auv_loader = auv_loader,
        .auv_watcher = try .init(arena, args.auv_dynlib_path),

        .config = config,
        .config_loader = config_loader,
        .config_watcher = try .init(arena, args.live_config_path),

        .io = io,
    };
}

/// Invalidates `seen_objects`
pub fn reset(ctx: *MissionContext) void {
    ctx.seen_objects_len = 0;
    ctx.resetOdometryPid();
    ctx.held_q = null;
    ctx.explicit_q = null;
    ctx.target_orientation = null;
    ctx.motion = .{ .hold = .{} };
    ctx.command = .{};
    ctx.pid_prev_timestamp_ns = null;
    ctx.hold_position = null;
    ctx.position_axes = null;
}

pub fn yieldUntilNextFrameAndUpdate(ctx: *MissionContext) !void {
    const start = Io.Clock.Timestamp.now(ctx.io, .awake);
    ctx.auv.yieldUntilNextFrame(&ctx.frame);
    if (stop_requested.load(.monotonic)) return error.Stopped;


    // std.log.debug("updating seen_objects...", .{});
    for (ctx.frame.objects[0..ctx.frame.objects_len]) |frame_object| {
        const maybe_seen_object = for (ctx.seen_objects[0..ctx.seen_objects_len]) |*seen_object| {
            if (frame_object.id == seen_object.id) {
                break seen_object;
            }
        } else null;

        if (maybe_seen_object) |seen_object| {
            seen_object.* = frame_object;
        } else {
            ctx.seen_objects[ctx.seen_objects_len] = frame_object;
            ctx.seen_objects_len += 1;
        }
    }

    for (ctx.frame.objects2d[0..ctx.frame.objects2d_len]) |frame_object| {
        const maybe_seen_object = for (ctx.seen_objects2d[0..ctx.seen_objects2d_len]) |*seen_object| {
            if (frame_object.id == seen_object.id) {
                break seen_object;
            }
        } else null;

        if (maybe_seen_object) |seen_object| {
            seen_object.* = frame_object;
        } else {
            ctx.seen_objects2d[ctx.seen_objects2d_len] = frame_object;
            ctx.seen_objects2d_len += 1;
        }
    }

    if (ctx.config_watcher.changed() catch |err| blk: {
        std.log.err("{s}", .{@errorName(err)});
        break :blk false;
    }) {
        if (ctx.config_loader.load(ctx.io)) |config| {
            if (print_stuff) {
                std.log.debug("succesfully reloaded config: {s}/{s}", .{ ctx.auv_watcher.dir, ctx.auv_watcher.name });
                std.log.debug("{any}", .{config});
            }
            ctx.config = config;
        } else |err| {
            std.log.err("failed to reload config: {s}/{s}", .{ ctx.auv_watcher.dir, ctx.auv_watcher.name });
            std.log.err("{s}", .{@errorName(err)});
            std.log.info("kept previous config", .{});
        }
    }

    if (ctx.auv_watcher.changed() catch |err| blk: {
        std.log.err("{s}", .{@errorName(err)});
        break :blk false;
    }) {
        ctx.auv.deinit();
        if (ctx.auv_loader.load(ctx.io)) |auv| {
            if (print_stuff) {
                std.log.debug("succesfully reloaded auv: {s}/{s}", .{ ctx.auv_watcher.dir, ctx.auv_watcher.name });
                std.log.debug("{any}", .{auv});
            }
            ctx.auv = auv;
            if (print_stuff) {
                std.log.debug("restarted auv loop", .{});
            }
            return error.RestartMission;
        } else |err| {
            std.log.err("failed to reload auv: {s}/{s}", .{ ctx.auv_watcher.dir, ctx.auv_watcher.name });
            std.log.err("{s}", .{@errorName(err)});
            std.log.info("kept previous auv", .{});
        }
        ctx.auv.init();
    }

    ctx.applyMotion();
    ctx.controlStep();

    const elapsed = start.untilNow(ctx.io);
    if (print_stuff and ctx.log_counter % ctx.log_freq_div == 0) {
        std.log.debug("done in {} us\n", .{elapsed.raw.toMicroseconds()});
    }
    ctx.log_counter +%= 1;
}

// Field order is the little-endian packet.
const TelemetryPacket = struct {
    magic: [4]u8 = "AUV1".*,
    version: u8 = 2,
    timestamp: u64,
    camera_pose: math.Pose,
    position_ok: Auv.AxisOk,
    attitude_ok: Auv.AxisOk,
    depth_ok: bool,
    depth: f32,
    objects: []const Auv.Object,
    boxes: []const Auv.Object2d,
    thrusters: []const f32,
    step: []const u8,
};

fn publishToUdp(ctx: *MissionContext, thrusters: []const f32) void {
    const nobj = @min(@as(usize, ctx.seen_objects_len), ctx.seen_objects.len);
    const nbox = @min(@as(usize, ctx.seen_objects2d_len), ctx.seen_objects2d.len);

    var buf: [64 * 1024]u8 = undefined;
    var w: Io.Writer = .fixed(&buf);
    SerialBinaryWriter.write(&w, TelemetryPacket{
        .timestamp = ctx.frame.timestamp,
        .camera_pose = ctx.frame.camera_pose,
        .position_ok = ctx.frame.position_ok,
        .attitude_ok = ctx.frame.attitude_ok,
        .depth_ok = ctx.frame.pressure_depth_ok,
        .depth = ctx.frame.pressure_depth,
        .objects = ctx.seen_objects[0..nobj],
        .boxes = ctx.seen_objects2d[0..nbox],
        .thrusters = thrusters[0..@min(thrusters.len, 255)],
        .step = ctx.step[0..@min(ctx.step.len, 255)],
    }) catch return;
    const packet = w.buffered();

    if (ctx.telemetry_file) |file| file.writeStreamingAll(ctx.io, packet) catch |err| {
        std.log.err("telemetry log write failed: {s}", .{@errorName(err)});
    };
    // udp never blocks, even with the laptop unplugged
    ctx.telemetry_socket.send(ctx.io, &ctx.telemetry_dest, packet) catch {};
}

pub fn resetOdometryPid(ctx: *MissionContext) void {
    ctx.pid_sum_err = @splat(0);
    ctx.pid_prev_err = @splat(0);
    ctx.tilt_correction = .{ 0, 0, 0 };
    ctx.hold_position = null;
}

fn clearOdometryAttitude(ctx: *MissionContext) void {
    ctx.pid_sum_err = .{ ctx.pid_sum_err[0], ctx.pid_sum_err[1], ctx.pid_sum_err[2], 0, 0, 0 };
    ctx.pid_prev_err = .{ ctx.pid_prev_err[0], ctx.pid_prev_err[1], ctx.pid_prev_err[2], 0, 0, 0 };
}

pub const Motion = union(enum) {
    /// Hold the position latched in `hold_position`. Optional orientation is the teleop look.
    hold: struct {
        orientation: ?math.Quaternionf = null,
    },
    /// Body thrust and a world orientation. Coverage may tilt off that quaternion to produce a missing force.
    direct_thrust: struct {
        translation: math.Vector3f,
        orientation: math.Quaternionf,
    },
    /// Position error toward `target_position`, and the look already chosen.
    position,
    /// Surge, sway, and yaw scaled by the 2D box with `id` on this frame.
    look_forward_heading: struct {
        id: u32,
        surge: f32 = 1,
        sway: f32 = 0,
        yaw: f32 = 1,
    },
};

const Command = struct {
    translation: math.Vector3f = .{ 0, 0, 0 },
    /// Target attitude. Null holds orientation.// TODO: WHYYY
    target_orientation: ?math.Quaternionf = null,
    /// `translation` is a world position error. The PID rotates it into body first.
    transport: bool = false,
    /// `translation` is already thrust. Teleop only.
    direct_thrust: bool = false,
    /// Mission looks clear attitude lanes when the quaternion jumps. Box yaw does not.
    clear_on_look_change: bool = true,
};

/// Default surge, sway, and yaw for look-forward-and-heading. The 2D box error only scales these.
const LookForwardHeading = struct {
    surge: f32 = 1,
    sway: f32 = 0,
    yaw: f32 = 1,
};

/// World direction of body +Y at a level heading. Yaw 0 looks along +Y.
fn lookAtHeading(heading: f32) math.Vector3f {
    return .{ -@sin(heading), @cos(heading), 0 };
}

fn object2dOnFrame(ctx: *const MissionContext, id: u32) ?Auv.Object2d {
    for (ctx.frame.objects2d[0..ctx.frame.objects2d_len]) |object| {
        if (object.id == id) return object;
    }
    return null;
}

fn clearTranslationIntegral(ctx: *MissionContext) void {
    ctx.pid_sum_err[0] = 0;
    ctx.pid_sum_err[1] = 0;
    ctx.pid_sum_err[2] = 0;
}

/// Fill `command` from `motion` and this frame.
fn applyMotion(ctx: *MissionContext) void {
    switch (ctx.motion) {
        .hold => |hold| commandHold(ctx, hold.orientation),
        .direct_thrust => |thrust| commandDirectThrust(ctx, thrust.translation, thrust.orientation),
        .position => commandPosition(ctx),
        .look_forward_heading => |heading| {
            if (object2dOnFrame(ctx, heading.id)) |box| {
                commandLookForwardAndHeading(ctx, box, .{
                    .surge = heading.surge,
                    .sway = heading.sway,
                    .yaw = heading.yaw,
                });
            } else {
                clearTranslationIntegral(ctx);
                commandHold(ctx, null);
            }
        },
    }
}

/// A position axis that just became invalid must not keep pushing through the old body integral.
fn notePositionAxes(ctx: *MissionContext) void {
    const ok = ctx.frame.position_ok;
    if (ctx.position_axes) |previous| {
        if ((previous.x and !ok.x) or (previous.y and !ok.y) or (previous.z and !ok.z))
            clearTranslationIntegral(ctx);
    }
    ctx.position_axes = ok;
}

/// Point the camera at `target` from the current pose, clamped by `rotation_limit`.
pub fn setLookAtTarget(ctx: *MissionContext, target: math.Vector3f) !void {
    if (try bc.lookDirection(ctx.frame.camera_pose.pos, target)) |toward| {
        ctx.target_orientation = try bc.lookWithinLimits(ctx.config.camera_forward, toward, bc.identity, ctx.config.rotation_limit);
    } else {
        ctx.target_orientation = null;
    }
}

/// Position error toward the latched pose. A null orientation holds attitude.
fn commandHold(ctx: *MissionContext, orientation: ?math.Quaternionf) void {
    if (ctx.hold_position == null) ctx.hold_position = ctx.frame.camera_pose.pos;
    const target = ctx.hold_position.?;
    ctx.command = .{
        .translation = bc.positionError(ctx.frame.camera_pose.pos, target) catch unreachable,
        .target_orientation = orientation,
        .transport = true,
        .direct_thrust = false,
        .clear_on_look_change = true,
    };
}

/// Direct thrust and a world orientation. The quaternion is the full attitude target.
fn commandDirectThrust(
    ctx: *MissionContext,
    translation: math.Vector3f,
    orientation: math.Quaternionf,
) void {
    ctx.hold_position = null;
    ctx.command = .{
        .translation = translation,
        .target_orientation = orientation,
        .transport = false,
        .direct_thrust = true,
        .clear_on_look_change = true,
    };
}

fn slew3(current: math.Vector3f, desired: math.Vector3f, max_step: f32) math.Vector3f {
    if (max_step <= 0) return current;
    const delta = desired - current;
    const len = math.length3f(delta);
    if (len <= max_step or len <= 1e-6) return desired;
    return current + delta / @as(math.Vector3f, @splat(len / max_step));
}

/// World-frame error toward `target_position`, and the look already chosen.
fn commandPosition(ctx: *MissionContext) void {
    ctx.hold_position = null;
    ctx.command = .{
        .translation = bc.positionError(ctx.frame.camera_pose.pos, ctx.target_position) catch unreachable,
        .target_orientation = ctx.target_orientation,
        .transport = true,
        .direct_thrust = false,
        .clear_on_look_change = true,
    };
}

/// Scale the surge, sway, and yaw defaults by the 2D box error. Body Z stays zero.
fn commandLookForwardAndHeading(ctx: *MissionContext, box: Auv.Object2d, motion: LookForwardHeading) void {
    ctx.hold_position = null;
    const metrics = math.boxMetrics(
        ctx.frame.image_width,
        ctx.frame.image_height,
        .{ box.top_left.x, box.top_left.y },
        .{ box.bottom_right.x, box.bottom_right.y },
    );
    const vision = ctx.config.non_odometry;
    const cx = math.withinDeadband(metrics.cx, vision.center_deadband);
    const height_err = vision.target_height - metrics.height;
    const current = bc.normalizedQuat(ctx.frame.camera_pose.quat) catch unreachable;
    const heading = math.quaternionToEuler(current)[2] + motion.yaw * cx;
    const look_q = bc.lookWithinLimits(
        ctx.config.camera_forward,
        lookAtHeading(heading),
        bc.identity,
        ctx.config.rotation_limit,
    ) catch unreachable;
    ctx.command = .{
        .translation = .{ motion.sway * cx, motion.surge * height_err, 0 },
        .target_orientation = look_q,
        .transport = false,
        .direct_thrust = false,
        .clear_on_look_change = false,
    };
}

fn resolveLook(ctx: *MissionContext, current: math.Quaternionf, look: ?math.Quaternionf, clear_on_change: bool) math.Quaternionf {
    if (look) |q| {
        const target = bc.normalizedQuat(q) catch unreachable;
        if (clear_on_change) {
            const changed = if (ctx.explicit_q) |previous| @abs(@reduce(.Add, previous * target)) < 0.9999 else true;
            if (changed) ctx.clearOdometryAttitude();
        }
        ctx.explicit_q = target;
        return target;
    }
    if (ctx.explicit_q != null) {
        ctx.explicit_q = null;
        ctx.held_q = null;
        ctx.clearOdometryAttitude();
    }
    if (ctx.held_q == null) ctx.held_q = current;
    return ctx.held_q.?;
}

/// Track `ctx.command` and send the resulting thruster values. Invalid pose axes contribute no error.
fn controlStep(ctx: *MissionContext) void {
    ctx.notePositionAxes();
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
    const current = ctx.frame.camera_pose.quat;
    const command = ctx.command;
    var target_q = ctx.resolveLook(current, command.target_orientation, command.clear_on_look_change);
    const odom = ctx.config.odometry;
    var desired_tilt: math.Vector3f = .{ 0, 0, 0 };
    const moments = bc.ThrustDirections.fromTamMoments(ctx.config.tam) catch unreachable;
    // Direct thrust skips the translation PID. Coverage still swings a thruster onto a component the map cannot produce.
    const translation_command: math.Vector3f = if (command.direct_thrust) blk: {
        const coverage = bc.coverForce(thrust_direction, moments, command.translation, ctx.config.rotation_limit, ctx.config.camera_forward);
        if (coverage.tilt) |tilt| desired_tilt = tilt.axis * @as(math.Vector3f, @splat(tilt.angle));
        break :blk coverage.force;
    } else blk: {
        const body_q = bc.normalizedQuat(current) catch unreachable;
        if (command.transport) {
            const ok = ctx.frame.position_ok;
            if (!ok.x) ctx.command.translation[0] = 0;
            if (!ok.y) ctx.command.translation[1] = 0;
            if (!ok.z) ctx.command.translation[2] = 0;
        }
        const error_body = if (command.transport)
            math.quaternionRotate(math.quaternionConjugate(body_q), ctx.command.translation)
        else
            ctx.command.translation;
        const available = thrust_direction.alongThrust(error_body);
        var force_body: math.Vector3f = .{ 0, 0, 0 };
        inline for (0..3) |i| {
            var rate: f32 = 0;
            if (dt) |t| {
                ctx.pid_sum_err[i] += available[i] * t;
                rate = (error_body[i] - ctx.pid_prev_err[i]) / t;
            }
            force_body[i] = odom.kp[i] * error_body[i] + odom.ki[i] * ctx.pid_sum_err[i] + odom.kd[i] * rate;
            ctx.pid_prev_err[i] = error_body[i];
        }
        const coverage = bc.coverForce(thrust_direction, moments, force_body, ctx.config.rotation_limit, ctx.config.camera_forward);
        if (coverage.tilt) |tilt| desired_tilt = tilt.axis * @as(math.Vector3f, @splat(tilt.angle));
        force_body = coverage.force;
        break :blk force_body;
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
        var force = translation_command;
        if (command.direct_thrust) {
            if (command.target_orientation) |q| {
                const body_q = bc.normalizedQuat(current) catch unreachable;
                const command_q = bc.normalizedQuat(q) catch unreachable;
                const into_body = math.quaternionMul(math.quaternionConjugate(body_q), command_q);
                force = math.quaternionRotate(into_body, force);
            }
        }
        force = thrust_direction.alongThrust(force);
        if (math.length3f(attitude_error) > odom.rotation_gate) {
            force[0] = 0;
            force[1] = 0;
            //we doing zero out force[2] beause we always want to maintain the z axis.
        }
        break :blk force;
    };

    var rotation: math.Vector3f = .{ 0, 0, 0 };
    inline for (0..3) |i| {
        const lane = i + 3;
        if (attitude_axis[i]) {
            var rate: f32 = 0;
            if (dt) |t| {
                rate = (attitude_error[i] - ctx.pid_prev_err[lane]) / t;
                ctx.pid_sum_err[lane] += attitude_error[i] * t;
            }
            ctx.pid_prev_err[lane] = attitude_error[i];
            rotation[i] = odom.kp[lane] * attitude_error[i] + odom.ki[lane] * ctx.pid_sum_err[lane] + odom.kd[lane] * rate;
        } else {
            ctx.pid_sum_err[lane] = 0;
            ctx.pid_prev_err[lane] = 0;
        }
    }
    ctx.held_q = target_q;
    const input: math.Vector6f = .{
        translation_body[0], translation_body[1], translation_body[2],
        rotation[0],         rotation[1],         rotation[2],
    };
    if (print_stuff and ctx.log_counter % ctx.log_freq_div == 0) {
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
        if (command.direct_thrust) {
            math.debug3f("translation direction", command.translation);
        } else if (ctx.hold_position) |held| {
            math.debug3f("hold", held);
        } else if (command.transport) {
            math.debug3f("target_position", ctx.target_position);
        } else {
            math.debug3f("translation error", command.translation);
        }
        if (command.target_orientation) |orient_q| {
            std.log.debug("target orientation: {any}", .{orient_q});
        }
        std.log.debug("attitude error body: {any}", .{attitude_error});
        math.debug3f("rotation limit", ctx.config.rotation_limit);
        std.log.debug("target quaternion: {any}", .{target_q});
        math.debug6f("\tinput", input);
    }
    var thruster_buffer: [math.max_thrusters]f32 = undefined;
    const thruster_values = math.tamToThrusters(
        ctx.config.tam,
        input,
        ctx.thrustor_saturate,
        5.0,
        &thruster_buffer,
    );
    if (print_stuff and ctx.log_counter % ctx.log_freq_div == 0) {
        std.log.debug("\tthruster_values = {any}", .{thruster_values});
    }
    ctx.auv.setThrustorValues(thruster_values.ptr, @intCast(thruster_values.len));
    ctx.publishToUdp(thruster_values);
}

fn yieldUntilObjectWithCls(ctx: *MissionContext, clss: []const u8, start: usize) !*const Auv.Object {
    ctx.motion = .position;
    var seen = start;

    while (true) {
        for (ctx.seen_objects[seen..ctx.seen_objects_len]) |*reacted_object| {
            for (clss) |cls| {
                if (reacted_object.cls == cls) {
                    return reacted_object;
                }
            }
            seen += 1;
        }

        try ctx.yieldUntilNextFrameAndUpdate();
    }
}

/// yield until getting first object with any of the `cls` in `clss`
pub fn yieldUntilFirstObjectWithAnyCls(ctx: *MissionContext, clss: []const u8) !*const Auv.Object {
    return try yieldUntilObjectWithCls(ctx, clss, 0);
}

/// yield until getting first object wit `cls`
pub fn yieldUntilFirstObjectWithCls(ctx: *MissionContext, cls: u8) !*const Auv.Object {
    return try yieldUntilFirstObjectWithAnyCls(ctx, &.{cls});
}

/// yield until getting next object with any of the `cls` in `clss` (ignoring any seen before)
pub fn yieldUntilNewObjectWithAnyCls(ctx: *MissionContext, clss: []const u8) !*const Auv.Object {
    return try yieldUntilObjectWithCls(ctx, clss, ctx.seen_objects_len);
}

/// yield until getting next object wit `cls` (ignoring any seen before)
pub fn yieldUntilNewObjectWithCls(ctx: *MissionContext, cls: u8) !*const Auv.Object {
    return try yieldUntilNewObjectWithAnyCls(ctx, &.{cls});
}

/// Drive to `target_pos` while looking at `look_at`. The look is refreshed from the current pose each frame.
pub fn yieldUntilReachGoal(ctx: *MissionContext, target_pos: math.Vector3f, look_at: math.Vector3f) !void {
    ctx.target_position = target_pos;
    ctx.motion = .position;
    while (true) {
        try ctx.setLookAtTarget(look_at);
        try ctx.yieldUntilNextFrameAndUpdate();
        const ok = ctx.frame.position_ok;
        if (ok.x and ok.y and ok.z) {
            const camera_pos = ctx.frame.camera_pose.pos;
            const delta = target_pos - camera_pos;
            const dist = @sqrt(@reduce(.Add, delta * delta));
            if (dist <= ctx.goal_dist_threshold) {
                break;
            }
        }
    }
}
