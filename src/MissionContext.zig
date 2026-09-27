const std = @import("std");
const Io = std.Io;
const MissionArgs = @import("MissionArgs.zig");

const Auv = @import("Auv.zig");
const AuvLoader = @import("AuvLoader.zig");
const ConfigLoader = @import("ConfigLoader.zig");
const FileWatcher = @import("FileWatcher.zig");
const controls = @import("controls.zig");
const math = @import("math.zig");
const global_nav = @import("navigation/global.zig");
const reactive_nav = @import("navigation/reactive.zig");

const MissionContext = @This();
const print_stuff = true;
const clear_print_stuff = true;

io: Io,

/// SLAM / 3D object navigation. Missions use this for global yields.
global: global_nav.GlobalState,
/// IMU + 2D detection navigation. Missions use this for reactive yields.
reactive: reactive_nav.ReactiveState,
reactive_gains: reactive_nav.ReactiveGains,

goal_dist_threshold: f32,
close_enough: f32,
thrustor_saturate: f32,
thrustor_output_scale: f32,

log_counter: u32,
log_freq_div: u16,

auv: Auv,
auv_loader: AuvLoader,
auv_watcher: FileWatcher,

config: ConfigLoader.Config,
config_loader: ConfigLoader,
config_watcher: FileWatcher,

pub fn init(gpa: std.mem.Allocator, io: Io, args: MissionArgs) !MissionContext {
    var config_loader: ConfigLoader = try .init(gpa, args.live_config_path);
    const config = config_loader.load(io) catch |err| {
        std.log.err("failed to load config: {s}", .{config_loader.config_path});
        return err;
    };
    std.log.debug("succesfully loaded config: {s}", .{config_loader.config_path});
    std.log.debug("{any}", .{config});

    var auv_loader: AuvLoader = try .init(gpa, args.auv_dynlib_path);
    const auv = auv_loader.load(io) catch |err| {
        std.log.err("failed to load auv: {s}", .{auv_loader.auv_path});
        return err;
    };
    auv.init();
    std.log.debug("succesfully loaded auv: {s}", .{auv_loader.auv_path});
    std.log.debug("{any}", .{auv});

    var global_state: global_nav.GlobalState = .{};
    global_nav.initController(&global_state, config);
    var reactive_state: reactive_nav.ReactiveState = .{};
    reactive_nav.initController(&reactive_state, config);

    return .{
        .global = global_state,
        .reactive = reactive_state,
        .reactive_gains = .{},

        .goal_dist_threshold = args.goal_dist_threshold,
        .close_enough = 1,
        .thrustor_saturate = 5,
        .thrustor_output_scale = 5,

        .log_counter = 0,
        .log_freq_div = 120,

        .auv = auv,
        .auv_loader = auv_loader,
        .auv_watcher = try .init(gpa, args.auv_dynlib_path),

        .config = config,
        .config_loader = config_loader,
        .config_watcher = try .init(gpa, args.live_config_path),

        .io = io,
    };
}

fn updateSeenObjects(ctx: *MissionContext) void {
    const state = &ctx.global;
    for (state.frame.objects[0..state.frame.objects_len]) |frame_object| {
        const maybe_seen_object = for (state.seen_objects[0..state.seen_objects_len]) |*seen_object| {
            if (frame_object.id == seen_object.id) {
                break seen_object;
            }
        } else null;

        if (maybe_seen_object) |seen_object| {
            seen_object.* = frame_object;
        } else {
            state.seen_objects[state.seen_objects_len] = frame_object;
            state.seen_objects_len += 1;
        }
    }
}

const linux = std.os.linux;
pub fn yieldUntilNextFrameAndUpdate(ctx: *MissionContext) void {
    var start: linux.timespec = undefined;
    var end: linux.timespec = undefined;
    ctx.auv.yieldUntilNextFrame(&ctx.global.frame);
    _ = linux.clock_gettime(.MONOTONIC, &start);
    ctx.updateSeenObjects();
    ctx.pidStep();
    ctx.runHotReload();

    _ = linux.clock_gettime(.MONOTONIC, &end);
    const ns = (end.sec - start.sec) * 1000000000 + (end.nsec - start.nsec);
    if (print_stuff and ctx.log_counter % ctx.log_freq_div == 0) {
        std.log.debug("done in {} us\n", .{@divTrunc(ns, 1000)});
    }
    ctx.log_counter +%= 1;
}

fn runHotReload(ctx: *MissionContext) void {
    if (ctx.config_watcher.changed() catch |err| blk: {
        std.log.err("{s}", .{@errorName(err)});
        break :blk false;
    }) {
        if (ctx.config_loader.load(ctx.io)) |config| {
            std.log.debug("succesfully reloaded config: {s}/{s}", .{ ctx.config_watcher.dir, ctx.config_watcher.name });
            std.log.debug("{any}", .{config});
            ctx.config = config;
            global_nav.syncControllerGains(&ctx.global, config);
            reactive_nav.syncControllerGains(&ctx.reactive, config);
        } else |err| {
            std.log.err("failed to reload config: {s}/{s}", .{ ctx.config_watcher.dir, ctx.config_watcher.name });
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
}

fn pidStep(ctx: *MissionContext) void {
    const pose = ctx.global.frame.camera_pose;
    const timestamp_ns = ctx.global.frame.timestamp;

    const dt: ?f32 = if (ctx.global.prev_timestamp_ns) |previous| blk: {
        if (timestamp_ns > previous) {
            break :blk @as(f32, @floatFromInt(timestamp_ns - previous)) * 1e-9;
        }

        std.log.warn(
            "odometry stamp not increasing (prev={} ns, now={} ns); skipping I/D",
            .{ previous, timestamp_ns },
        );
        break :blk null;
    } else null;

    ctx.global.prev_timestamp_ns = timestamp_ns;

    const goal = ctx.global.goal;
    const current_pose = math.poseTo6f(pose);
    var pose_err = goal - current_pose;

    const rot = math.normalize4f(pose.quat);
    const forward = math.quaternionRotate(rot, .{ 0.0, 1.0, 0.0 });
    const current_rpy = math.quaternionToEuler(rot);
    _, _, const current_yaw = current_rpy;

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

    const wrench = controls.pidStep(&ctx.global.controller, pose_err, dt);
    const input = controls.wrenchToThrusterInput(wrench, pose.quat, yaw_error, global_nav.yaw_gate_default);

    var thruster_buffer: [controls.max_thrusters]f32 = undefined;
    const thruster_values = controls.tamToThrusters(
        ctx.config.tam,
        input,
        ctx.thrustor_saturate,
        ctx.thrustor_output_scale,
        &thruster_buffer,
    );

    if (print_stuff and ctx.log_counter % ctx.log_freq_div == 0) {
        if (clear_print_stuff) {
            std.debug.print("\x1b[2J\x1b[H", .{});
        }
        std.log.debug("config:", .{});
        std.log.debug("\tclose_enough = {}", .{ctx.close_enough});
        math.debug6f("\tkp", ctx.global.controller.ErrorConstant);
        math.debug6f("\tki", ctx.global.controller.IntegralConstant);
        math.debug6f("\tkd", ctx.global.controller.DerivativeConstant);
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
        math.debug6f("\tinput   ", input);
        std.log.debug("\tthruster_values = {any}", .{thruster_values});
    }
    ctx.auv.setThrustorValues(thruster_values.ptr, @intCast(thruster_values.len));
}

fn yieldUntilObjectWithCls(ctx: *MissionContext, clss: []const Auv.ObjectCls, start: usize) *const Auv.Object {
    var seen = start;

    while (true) {
        for (ctx.global.seen_objects[seen..ctx.global.seen_objects_len]) |*reacted_object| {
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
    return yieldUntilObjectWithCls(ctx, clss, ctx.global.seen_objects_len);
}

/// yield until getting next object wit `cls` (ignoring any seen before)
pub fn yieldUntilNewObjectWithCls(ctx: *MissionContext, cls: Auv.ObjectCls) *const Auv.Object {
    return yieldUntilNewObjectWithAnyCls(ctx, &.{cls});
}

/// yield until `ctx.global.frame.camera_pose.pos` is within `goal_dist_threshold` of `goal_pos`
pub fn yieldUntilReachGoal(ctx: *MissionContext, goal_pos: math.Vector3f) void {
    ctx.global.goal[0] = goal_pos[0];
    ctx.global.goal[1] = goal_pos[1];
    ctx.global.goal[2] = goal_pos[2];
    while (true) {
        const camera_pos = ctx.global.frame.camera_pose.pos;
        const goal_delta = goal_pos - camera_pos;
        const goal_dist = @sqrt(@reduce(.Add, goal_delta * goal_delta));
        if (goal_dist <= ctx.goal_dist_threshold) {
            break;
        }

        ctx.yieldUntilNextFrameAndUpdate();
    }
}
