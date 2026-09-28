const std = @import("std");
const Io = std.Io;
const MissionArgs = @import("MissionArgs.zig");

const Auv = @import("Auv.zig");
const AuvLoader = @import("AuvLoader.zig");
const ConfigLoader = @import("ConfigLoader.zig");
const FileWatcher = @import("FileWatcher.zig");
const math = @import("math.zig");
const global_nav = @import("navigation/global.zig");
const reactive_nav = @import("navigation/reactive.zig");
const Policy = @import("policy.zig").Policy;

const MissionContext = @This();
const print_stuff = true;
const clear_print_stuff = true;

gpa: std.mem.Allocator,
io: Io,
/// Loaded from config.policy; replaces the global PID while set.
policy: ?Policy,

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
    const reactive_state: reactive_nav.ReactiveState = .{};

    return .{
        .global = global_state,
        .reactive = reactive_state,
        .reactive_gains = .{},

        .goal_dist_threshold = args.goal_dist_threshold,
        .close_enough = 1,
        .thrustor_saturate = 5,
        .thrustor_output_scale = 1,

        .log_counter = 0,
        .log_freq_div = 120,

        .auv = auv,
        .auv_loader = auv_loader,
        .auv_watcher = try .init(gpa, args.auv_dynlib_path),

        .config = config,
        .config_loader = config_loader,
        .config_watcher = try .init(gpa, args.live_config_path),

        .gpa = gpa,
        .io = io,
        .policy = try loadPolicy(gpa, io, config),
    };
}

fn loadPolicy(gpa: std.mem.Allocator, io: Io, config: ConfigLoader.Config) !?Policy {
    const path = config.policy orelse return null;
    const policy = Policy.load(gpa, io, path) catch |err| {
        std.log.err("failed to load policy {s}: {s}", .{ path, @errorName(err) });
        return err;
    };
    std.log.info("global navigation uses policy {s} at {d} Hz", .{ path, policy.policy_hz });
    return policy;
}

pub fn deinit(ctx: *MissionContext, gpa: std.mem.Allocator) void {
    if (ctx.policy) |*policy| policy.deinit(gpa);
    ctx.config_watcher.deinit(gpa);
    ctx.config_loader.deinit(gpa);
    ctx.auv_watcher.deinit(gpa);
    ctx.auv_loader.deinit(gpa);
}

const linux = std.os.linux;
pub fn fetchFrameAndUpdate(ctx: *MissionContext) void {
    var start: linux.timespec = undefined;
    var end: linux.timespec = undefined;
    ctx.auv.yieldUntilNextFrame(&ctx.global.frame);
    _ = linux.clock_gettime(.MONOTONIC, &start);
    global_nav.updateSeenObjects(&ctx.global);
    global_nav.pidStep(&ctx.global, .{
        .auv = &ctx.auv,
        .tam = ctx.config.tam,
        .close_enough = ctx.close_enough,
        .thrustor_saturate = ctx.thrustor_saturate,
        .thrustor_output_scale = ctx.thrustor_output_scale,
        .policy = if (ctx.policy) |*policy| policy else null,
        .log = print_stuff and ctx.log_counter % ctx.log_freq_div == 0 and clear_print_stuff,
    });
    ctx.runHotReload();

    _ = linux.clock_gettime(.MONOTONIC, &end);
    const ns = (end.sec - start.sec) * 1000000000 + (end.nsec - start.nsec);
    if (print_stuff and ctx.log_counter % ctx.log_freq_div == 0) {
        std.log.debug("done in {} us\n", .{@divTrunc(ns, 1000)});
    }
    ctx.log_counter +%= 1;
}

pub fn fetchReactiveFrameAndUpdate(ctx: *MissionContext) void {
    var start: linux.timespec = undefined;
    var end: linux.timespec = undefined;
    ctx.auv.yieldUntilReactiveFrame(&ctx.reactive.frame);
    _ = linux.clock_gettime(.MONOTONIC, &start);
    reactive_nav.updateSeenObjects(&ctx.reactive);
    reactive_nav.step(&ctx.reactive, .{
        .auv = &ctx.auv,
        .tam = ctx.config.tam,
        .gains = ctx.reactive_gains,
        .thrustor_saturate = ctx.thrustor_saturate,
        .thrustor_output_scale = ctx.thrustor_output_scale,
        .z_down = ctx.config.z_down,
        .log = print_stuff and ctx.log_counter % ctx.log_freq_div == 0 and clear_print_stuff,
    });
    ctx.runHotReload();

    _ = linux.clock_gettime(.MONOTONIC, &end);
    const ns = (end.sec - start.sec) * 1000000000 + (end.nsec - start.nsec);
    if (print_stuff and ctx.log_counter % ctx.log_freq_div == 0) {
        std.log.debug("reactive done in {} us\n", .{@divTrunc(ns, 1000)});
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
            if (loadPolicy(ctx.gpa, ctx.io, config)) |policy| {
                if (ctx.policy) |*old| old.deinit(ctx.gpa);
                ctx.policy = policy;
                ctx.global.policy_runner = .{};
            } else |_| {
                std.log.info("kept previous policy", .{});
            }
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
            ctx.global.policy_runner = .{};
            std.log.debug("restarted auv loop", .{});
        } else |err| {
            std.log.err("failed to reload auv: {s}/{s}", .{ ctx.auv_watcher.dir, ctx.auv_watcher.name });
            std.log.err("{s}", .{@errorName(err)});
            std.log.info("kept previous auv", .{});
        }
        ctx.auv.init();
    }
}

fn globalYieldUntilObject(ctx: *MissionContext, cls: []const Auv.ObjectCls, start: usize) *const Auv.Object {
    while (true) {
        if (global_nav.findObject(&ctx.global, cls, start)) |object| return object;
        ctx.fetchFrameAndUpdate();
    }
}

pub fn globalYieldUntilFirstObjectWithCls(ctx: *MissionContext, cls: Auv.ObjectCls) *const Auv.Object {
    return ctx.globalYieldUntilObject(&.{cls}, 0);
}

pub fn globalYieldUntilFirstObjectWithAnyCls(ctx: *MissionContext, cls: []const Auv.ObjectCls) *const Auv.Object {
    return ctx.globalYieldUntilObject(cls, 0);
}

pub fn globalYieldUntilNewObjectWithCls(ctx: *MissionContext, cls: Auv.ObjectCls) *const Auv.Object {
    return ctx.globalYieldUntilObject(&.{cls}, ctx.global.seen_objects_len);
}

pub fn globalYieldUntilNewObjectWithAnyCls(ctx: *MissionContext, cls: []const Auv.ObjectCls) *const Auv.Object {
    return ctx.globalYieldUntilObject(cls, ctx.global.seen_objects_len);
}

pub fn globalYieldUntilReachGoal(ctx: *MissionContext, goal_pos: math.Vector3f) global_nav.MissionError!void {
    global_nav.setGoalPosition(&ctx.global, goal_pos);
    while (true) {
        ctx.fetchFrameAndUpdate();
        if (!ctx.global.frame.tracking_ok) return error.TrackingLost;
        if (global_nav.cameraWithin(&ctx.global, goal_pos, ctx.goal_dist_threshold)) return;
    }
}

fn reactiveYieldUntilObject(ctx: *MissionContext, cls: []const Auv.ObjectCls, start: usize) *const Auv.Object2DYolo {
    ctx.reactive.tracked_id = null;
    ctx.reactive.target_height = null;
    ctx.reactive.search_yaw = null;
    while (true) {
        if (reactive_nav.findObject(&ctx.reactive, cls, start)) |object| return object;
        ctx.fetchReactiveFrameAndUpdate();
    }
}

pub fn reactiveYieldUntilFirstObjectWithCls(ctx: *MissionContext, cls: Auv.ObjectCls) *const Auv.Object2DYolo {
    return ctx.reactiveYieldUntilObject(&.{cls}, 0);
}

pub fn reactiveYieldUntilFirstObjectWithAnyCls(ctx: *MissionContext, cls: []const Auv.ObjectCls) *const Auv.Object2DYolo {
    return ctx.reactiveYieldUntilObject(cls, 0);
}

pub fn reactiveYieldUntilNewObjectWithCls(ctx: *MissionContext, cls: Auv.ObjectCls) *const Auv.Object2DYolo {
    return ctx.reactiveYieldUntilObject(&.{cls}, ctx.reactive.seen_objects2d_len);
}

pub fn reactiveYieldUntilNewObjectWithAnyCls(ctx: *MissionContext, cls: []const Auv.ObjectCls) *const Auv.Object2DYolo {
    return ctx.reactiveYieldUntilObject(cls, ctx.reactive.seen_objects2d_len);
}

fn reactiveYieldUntilYawFind(ctx: *MissionContext, cls: []const Auv.ObjectCls) *const Auv.Object2DYolo {
    ctx.reactive.tracked_id = null;
    ctx.reactive.target_height = null;
    ctx.reactive.search_yaw = ctx.reactive_gains.search_yaw;
    ctx.reactive.hold_depth = null;
    defer {
        ctx.reactive.search_yaw = null;
        ctx.reactive.hold_depth = null;
    }
    while (true) {
        ctx.fetchReactiveFrameAndUpdate();
        const frame_object = reactive_nav.findFrameObjectWithCls(&ctx.reactive, cls) orelse continue;
        return reactive_nav.findSeenById(&ctx.reactive, frame_object.id) orelse continue;
    }
}

/// Yaw in place until a box of `cls` is in the current camera frame.
pub fn reactiveYieldUntilYawFindCls(ctx: *MissionContext, cls: Auv.ObjectCls) *const Auv.Object2DYolo {
    return ctx.reactiveYieldUntilYawFind(&.{cls});
}

/// Yaw in place until a box of any class in `cls` is in the current camera frame.
pub fn reactiveYieldUntilYawFindAnyCls(ctx: *MissionContext, cls: []const Auv.ObjectCls) *const Auv.Object2DYolo {
    return ctx.reactiveYieldUntilYawFind(cls);
}

pub fn reactiveYieldUntilCentered(ctx: *MissionContext, id: u32) void {
    ctx.reactive.tracked_id = id;
    ctx.reactive.target_height = null;
    while (true) {
        ctx.fetchReactiveFrameAndUpdate();
        if (reactive_nav.findFrameObject(&ctx.reactive, id)) |box| {
            if (reactive_nav.isCentered(&ctx.reactive, box, ctx.reactive_gains.center_deadband)) return;
        }
    }
}

pub fn reactiveYieldUntilHeight(ctx: *MissionContext, id: u32, target_height: f32) void {
    ctx.reactive.tracked_id = id;
    ctx.reactive.target_height = target_height;
    while (true) {
        ctx.fetchReactiveFrameAndUpdate();
        if (reactive_nav.findFrameObject(&ctx.reactive, id)) |box| {
            if (reactive_nav.heightReached(&ctx.reactive, box, target_height)) return;
        }
    }
}
