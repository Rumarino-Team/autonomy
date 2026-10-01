const std = @import("std");
const Io = std.Io;
const MissionArgs = @import("MissionArgs.zig");

const Auv = @import("Auv.zig");
const AuvLoader = @import("AuvLoader.zig");
const ConfigLoader = @import("ConfigLoader.zig");
const FileWatcher = @import("FileWatcher.zig");

const MissionContext = @This();
const print_stuff = true;
const clear_print_stuff = true;

io: Io,

frame: Auv.Frame,

seen_objects: [Auv.Frame.max_objects]Auv.Object,
seen_objects_len: u8,

goal_dist_threshold: f32,
close_enough: f32,
thrustor_saturate: f32,

log_counter: u32,
log_freq_div: u16,

goal: math.Vector6f,
pid_sum_err: math.Vector6f,
pid_prev_pose_err: math.Vector6f,
pid_prev_timestamp_ns: ?u64,


//ReactiveState 
seen_objects2d: [Auv.Frame.max_objects]Auv.Object2d = undefined,
seen_objects2d_len: u8 = 0,
reactive_pid_sum_err: math.Vector6f,
reactive_pid_prev_pose_err: math.Vector6f,
reactive_pid_prev_timestamp_ns : ?u64,
reactive_prev_tracked_id: i32 = -1,

tracked_id: i32 = -1,
search_yaw: f32 = 0,
hold_depth: f32 = 2,
// 



auv: Auv,
auv_loader: AuvLoader,
auv_watcher: FileWatcher,

config: ConfigLoader.Config,
config_loader: ConfigLoader,
config_watcher: FileWatcher,

pub fn init(arena: std.mem.Allocator, io: Io, args: MissionArgs) !MissionContext {
    var config_loader: ConfigLoader = try .init(arena, args.live_config_path);
    const config = config_loader.load(io) catch |err| {
        std.log.err("failed to load config: {s}", .{config_loader.config_path});
        return err;
    };
    std.log.debug("succesfully loaded config: {s}", .{config_loader.config_path});
    std.log.debug("{any}", .{config});

    var auv_loader: AuvLoader = try .init(arena, args.auv_dynlib_path);
    const auv = auv_loader.load(io) catch |err| {
        const dl_err = std.mem.span(std.c.dlerror()) orelse "NO_DL_ERROR";
        std.log.err("dlerror: `{s}`", .{dl_err});
        std.log.err("failed to load auv: {s}", .{auv_loader.auv_path});
        return err;
    };
    auv.init();
    std.log.debug("succesfully loaded auv: {s}", .{auv_loader.auv_path});
    std.log.debug("{any}", .{auv});

    return .{
        .frame = undefined,

        .seen_objects = undefined,
        .seen_objects_len = 0,

        .goal_dist_threshold = args.goal_dist_threshold,
        .close_enough = 1,
        .thrustor_saturate = 5,

        .log_counter = 0,
        .log_freq_div = 120,

        .goal = @splat(0),

        .pid_sum_err = @splat(0),
        .pid_prev_pose_err = @splat(0),
        .pid_prev_timestamp_ns = null,

        .reactive_pid_sum_err = @splat(0),
        .reactive_pid_prev_pose_err = @splat(0),
        .reactive_pid_prev_timestamp_ns = null,

        .auv = auv,
        .auv_loader = auv_loader,
        .auv_watcher = try .init(arena, args.auv_dynlib_path),

        .config = config,
        .config_loader = config_loader,
        .config_watcher = try .init(arena, args.live_config_path),

        .io = io,
    };
}

pub fn yieldUntilNextFrameAndUpdate(ctx: *MissionContext) void {
    ctx.auv.yieldUntilNextFrame(&ctx.frame);
    const start = Io.Clock.Timestamp.now(ctx.io, .awake);

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


    // std.log.debug("setting thruster_values with pid controller...", .{});



    if (ctx.frame.tracking_ok) {
        ctx.globalControllerStep();
    } else {
        ctx.reactiveControllerStep();
    }

    if (ctx.config_watcher.changed() catch |err| blk: {
        std.log.err("{s}", .{@errorName(err)});
        break :blk false;
    }) {
        if (ctx.config_loader.load(ctx.io)) |config| {
            std.log.debug("succesfully reloaded config: {s}/{s}", .{ ctx.auv_watcher.dir, ctx.auv_watcher.name });
            std.log.debug("{any}", .{config});
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
            std.log.debug("succesfully reloaded auv: {s}/{s}", .{ ctx.auv_watcher.dir, ctx.auv_watcher.name });
            std.log.debug("{any}", .{auv});
            ctx.auv = auv;
            std.log.debug("restarted auv loop", .{});
        } else |err| {
            std.log.err("failed to reload auv: {s}/{s}", .{ ctx.auv_watcher.dir, ctx.auv_watcher.name });
            std.log.err("{s}", .{@errorName(err)});
            std.log.info("kept previous auv", .{});
        }
        ctx.auv.init();
    }

    const elapsed = start.untilNow(ctx.io);
    if (print_stuff and ctx.log_counter % ctx.log_freq_div == 0) {
        std.log.debug("done in {} us\n", .{elapsed.raw.toMicroseconds()});
    }
    ctx.log_counter +%= 1;
}

const math = @import("math.zig");

fn globalControllerStep(ctx: *MissionContext) void {
    ControllerStep(ctx);
}

fn setThrustersFromTamInput(ctx: *MissionContext, input: math.Vector6f) void {
    var thruster_buffer: [math.max_thrusters]f32 = undefined;
    const thruster_values = math.tamToThrusters(
        ctx.config.tam,
        input,
        ctx.thrustor_saturate,
        5.0,
        &thruster_buffer,
    );
    ctx.auv.setThrustorValues(thruster_values.ptr, @intCast(thruster_values.len));
}

fn ControllerStep(ctx: *MissionContext) void {
    const pose = ctx.frame.camera_pose;
    const timestamp_ns = ctx.frame.timestamp;
    const dt: ?f32 = if (ctx.pid_prev_timestamp_ns) |previous| blk: {
        const elapsed_ns = timestamp_ns - previous;
        if (elapsed_ns > 0) {
            break :blk @as(f32, @floatFromInt(elapsed_ns)) * 1e-9;
        }
        std.log.warn(
            "odometry stamp not increasing (prev={} ns, now={} ns); skipping I/D",
            .{ previous, timestamp_ns },
        );
        break :blk null;
    } else null;
    ctx.pid_prev_timestamp_ns = timestamp_ns;

    const goal = ctx.goal;
    const current_pose = math.poseTo6f(pose);

    var pose_err = goal - current_pose;

    // Current orientation.
    const rot = math.normalize4f(pose.quat);

    // Vehicle forward is +Y in body frame.
    const forward = math.quaternionRotate(rot, .{ 0.0, 1.0, 0.0 });
    const current_yaw = std.math.atan2(forward[1], forward[0]);

    const dir = math.Vector3f{
        pose_err[0],
        pose_err[1],
        0.0,
    };

    const xy_distance = math.length3f(dir);


    const target_yaw = if (xy_distance > ctx.close_enough)
        std.math.atan2(dir[1], dir[0])
    else
        goal[5];

    const yaw_error = math.wrapAngle(target_yaw - current_yaw);

    pose_err[5] = yaw_error;

    const vel_err: math.Vector6f = if (dt) |delta_t| blk: {
        ctx.pid_sum_err += pose_err * @as(math.Vector6f, @splat(delta_t));

        break :blk (pose_err - ctx.pid_prev_pose_err) / @as(math.Vector6f, @splat(delta_t));
    } else @splat(0.0);

    const kp = ctx.config.kp;
    const ki = ctx.config.ki;
    const kd = ctx.config.kd;

    const wrench =
        kp * pose_err +
        ki * ctx.pid_sum_err +
        kd * vel_err;

    const input = math.wrenchToThrusterInput(wrench, pose.quat, yaw_error, std.math.pi / 8.0);
    const max_thrusters = 8;
    var thruster_buffer: [max_thrusters]f32 = undefined;
    const thruster_values = math.tamMul(ctx.config.tam, input, &thruster_buffer);

    for (thruster_values) |*value| {
        value.* = std.math.clamp(
            value.*,
            -ctx.thrustor_saturate,
            ctx.thrustor_saturate,
        );

        value.* /= 5.0;
    }

    if (print_stuff and ctx.log_counter % ctx.log_freq_div == 0) {
        if (clear_print_stuff) {
            std.debug.print("\x1b[2J\x1b[H", .{});
        }
        std.log.debug("config:", .{});
        std.log.debug("\tclose_enough = {}", .{ctx.close_enough});
        math.debug6f("\tkp", kp);
        math.debug6f("\tki", ki);
        math.debug6f("\tkd", kd);
        for (ctx.config.tam) |row| {
            math.debug6f("\ttam", row);
        }
        std.log.debug("dt = {d:6.2} ms", .{(dt orelse 0) * 1000});
        math.debug6f("pose", current_pose);
        math.debug6f("goal", goal);
        std.log.debug("xy_distance = {}", .{xy_distance});
        std.log.debug("thruster mapping:", .{});
        math.debug6f("\twrench  ", wrench);
        math.debug3f("\tforward ", forward);
        math.debug3f("\tdir2d   ", dir);
        std.log.debug(
            "\tcurrent_yaw={d:3.2} target_yaw={d:3.2} yaw_error={d:3.2}",
            .{ current_yaw, target_yaw, yaw_error },
        );
        // math.debug3f("\trotated ", rotated);
        // math.debug3f("\tbody_force ", body_force);
        math.debug6f("\tinput   ", input);
        std.log.debug("\tthruster_values = {any}", .{thruster_values});
    }
    ctx.auv.setThrustorValues(thruster_values.ptr, @intCast(thruster_values.len));
    
    ctx.pid_prev_pose_err = pose_err;
}

/// Body command [Fx, Fy, Fz, Mx, My, Mz]. Image error is already camera-relative.
/// +Y is surge, +yaw turns right. A box below center (positive cy) dives.
/// Body +Z is down, so dive is +Fz. Surge/heave/yaw use reactive `kp`/`ki`/`kd`
/// indices 1, 2, and 5. Roll and pitch (indices 3 and 4) level toward `goal`
/// using the camera pose, with the same roll sign as `wrenchToThrusterInput`.
fn reactiveControllerStep(ctx: *MissionContext) void {
    ctx.trackFirstSeenObject2d();
    const timestamp_ns = ctx.frame.timestamp;
    const dt: ?f32 = if (ctx.reactive_pid_prev_timestamp_ns) |previous| blk: {
        const elapsed_ns = timestamp_ns - previous;
        if (elapsed_ns > 0) {
            break :blk @as(f32, @floatFromInt(elapsed_ns)) * 1e-9;
        }
        std.log.warn(
            "reactive stamp not increasing (prev={} ns, now={} ns); skipping I/D",
            .{ previous, timestamp_ns },
        );
        break :blk null;
    } else null;
    ctx.reactive_pid_prev_timestamp_ns = timestamp_ns;
    const reactive = ctx.config.reactive;
    const dive_sign: f32 = 1;
    const current_pose = math.poseTo6f(ctx.frame.camera_pose);
    const roll_err = ctx.goal[3] - current_pose[3];
    const pitch_err = ctx.goal[4] - current_pose[4];

    const tracked_box: ?Auv.Object2d = if (ctx.tracked_id < 0)
        null
    else
        for (ctx.seen_objects2d[0..ctx.seen_objects2d_len]) |object| {
            if (object.id == @as(u32, @intCast(ctx.tracked_id))) break object;
        } else null;

    const input: math.Vector6f = if (ctx.tracked_id >= 0 and tracked_box == null) blk: {
        ctx.reactive_pid_sum_err = @splat(0);
        ctx.reactive_pid_prev_pose_err = @splat(0);
        ctx.reactive_pid_prev_timestamp_ns = null;
        break :blk @splat(0);
    } else if (ctx.tracked_id < 0) blk: {
        var err: math.Vector6f = @splat(0);
        if (ctx.frame.pressure_depth_ok) {
            // `pressure_depth` is +down; positive error means too shallow and the sub should dive.
            err[2] = ctx.hold_depth - ctx.frame.pressure_depth;
        }
        err[3] = roll_err;
        err[4] = pitch_err;
        const wrench = math.pidWrench(
            &ctx.reactive_pid_sum_err,
            &ctx.reactive_pid_prev_pose_err,
            reactive.kp,
            reactive.ki,
            reactive.kd,
            err,
            dt,
        );
        break :blk .{ 0, 0, dive_sign * wrench[2], -wrench[3], wrench[4], ctx.search_yaw };
    } else blk: {
        const tracked = tracked_box orelse break :blk @as(math.Vector6f, @splat(0));
        const metrics = math.boxMetrics(
            ctx.frame.image_width,
            ctx.frame.image_height,
            .{ tracked.top_left.x, tracked.top_left.y },
            .{ tracked.bottom_right.x, tracked.bottom_right.y },
        );
        const cx = math.withinDeadband(metrics.cx, reactive.center_deadband);
        const cy = math.withinDeadband(metrics.cy, reactive.center_deadband);

        var err: math.Vector6f = @splat(0);
        err[1] = reactive.target_height - metrics.height;
        err[2] = cy;
        err[3] = roll_err;
        err[4] = pitch_err;
        err[5] = cx;

        const wrench = math.pidWrench(
            &ctx.reactive_pid_sum_err,
            &ctx.reactive_pid_prev_pose_err,
            reactive.kp,
            reactive.ki,
            reactive.kd,
            err,
            dt,
        );
        break :blk .{ 0, wrench[1], dive_sign * wrench[2], -wrench[3], wrench[4], wrench[5] };
    };

    setThrustersFromTamInput(ctx, input);

    if (print_stuff and ctx.log_counter % ctx.log_freq_div == 0) {
        if (clear_print_stuff) {
            std.debug.print("\x1b[2J\x1b[H", .{});
        }
        std.log.debug("reactive objects2d = {}", .{ctx.frame.objects2d_len});
        for (ctx.frame.objects2d[0..ctx.frame.objects2d_len]) |object| {
            std.log.debug(
                "  id={} cls={s} box=({},{})-({},{})",
                .{
                    object.id,
                    @tagName(object.cls),
                    object.top_left.x,
                    object.top_left.y,
                    object.bottom_right.x,
                    object.bottom_right.y,
                },
            );
        }
        math.debug6f("reactive tam input", input);
    }
}

/// Stop search and follow the 2D object `id`. The next reactive step uses the tracking branch.
pub fn trackObject2d(ctx: *MissionContext, id: u32) void {
    const tracked_id: i32 = @intCast(id);
    if (ctx.tracked_id == tracked_id) return;
    ctx.tracked_id = tracked_id;
    ctx.reactive_pid_sum_err = @splat(0);
    ctx.reactive_pid_prev_pose_err = @splat(0);
    ctx.reactive_pid_prev_timestamp_ns = null;
}

fn trackFirstSeenObject2d(ctx: *MissionContext) void {
    if (ctx.tracked_id >= 0 or ctx.seen_objects2d_len == 0) return;
    ctx.trackObject2d(ctx.seen_objects2d[0].id);
}

fn yieldUntilObjectWithCls(ctx: *MissionContext, clss: []const Auv.ObjectCls, start: usize) *const Auv.Object {
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

        ctx.yieldUntilNextFrameAndUpdate();
    }
}

/// yield until getting first object with any of the `cls` in `clss`
pub fn yieldUntilFirstObjectWithAnyCls(ctx: *MissionContext, clss: []const Auv.ObjectCls) *const Auv.Object {
    return yieldUntilObjectWithCls(ctx, clss, 0);
}

/// yield until getting first object wit `cls`
pub fn yieldUntilFirstObjectWithCls(ctx: *MissionContext, cls: Auv.ObjectCls) *const Auv.Object {
    return yieldUntilFirstObjectWithAnyCls(ctx, &.{cls});
}

/// yield until getting next object with any of the `cls` in `clss` (ignoring any seen before)
pub fn yieldUntilNewObjectWithAnyCls(ctx: *MissionContext, clss: []const Auv.ObjectCls) *const Auv.Object {
    return yieldUntilObjectWithCls(ctx, clss, ctx.seen_objects_len);
}

/// yield until getting next object wit `cls` (ignoring any seen before)
pub fn yieldUntilNewObjectWithCls(ctx: *MissionContext, cls: Auv.ObjectCls) *const Auv.Object {
    return yieldUntilNewObjectWithAnyCls(ctx, &.{cls});
}

/// yield until at `ctx.frame.camera_pose.pos` is at `goal_threshold` distance from `goal_pos`
pub fn yieldUntilReachGoal(ctx: *MissionContext, goal_pos: math.Vector3f) void {
    ctx.goal[0] = goal_pos[0];
    ctx.goal[1] = goal_pos[1];
    ctx.goal[2] = goal_pos[2];
    while (true) {
        const camera_pos = ctx.frame.camera_pose.pos;
        const goal_delta = goal_pos - camera_pos;
        const goal_dist = @sqrt(@reduce(.Add, goal_delta * goal_delta));
        if (goal_dist <= ctx.goal_dist_threshold) {
            break;
        }

        ctx.yieldUntilNextFrameAndUpdate();
    }
}
