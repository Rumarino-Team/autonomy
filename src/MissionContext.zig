const std = @import("std");
const Io = std.Io;
const MissionArgs = @import("MissionArgs.zig");

const Auv = @import("Auv.zig");
const AuvLoader = @import("AuvLoader.zig");
const ConfigLoader = @import("ConfigLoader.zig");
const FileWatcher = @import("FileWatcher.zig");
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





pub const Mode = enum { global, reactive };
const Clock = std.Io.Clock;

fn globalActuation(ctx: *MissionContext) global_nav.Actuation {
    return .{
        .auv = &ctx.auv,
        .tam = ctx.config.tam,
        .close_enough = ctx.close_enough,
        .thrustor_saturate = ctx.thrustor_saturate,
        .thrustor_output_scale = ctx.thrustor_output_scale,
        .policy = if (ctx.policy) |*policy| policy else null,
        .log = print_stuff and ctx.log_counter % ctx.log_freq_div == 0 and clear_print_stuff,
    };
}

fn reactiveActuation(ctx: *MissionContext) reactive_nav.Actuation {
    return .{
        .auv = &ctx.auv,
        .tam = ctx.config.tam,
        .thrustor_saturate = ctx.thrustor_saturate,
        .thrustor_output_scale = ctx.thrustor_output_scale,
        .z_down = ctx.config.z_down,
        .log = print_stuff and ctx.log_counter % ctx.log_freq_div == 0 and clear_print_stuff,
    };
}

pub fn fetchFrameAndUpdate(ctx: *MissionContext, mode: Mode) void {
    const start = Clock.now(.awake, ctx.io);
    switch (mode) {
        .global => ctx.auv.yieldUntilNextFrame(&ctx.global.frame),
        .reactive => ctx.auv.yieldUntilReactiveFrame(&ctx.reactive.frame),
    }

    switch (mode) {
        .global =>{
            global_nav.updateSeenObjects(&ctx.global);
            global_nav.globalControllerUpdate(&ctx.global, globalActuation(ctx));
        },
        .reactive => {
            reactive_nav.updateSeenObjects(&ctx.reactive);
            reactive_nav.reactiveControllerUpdate(&ctx.reactive, reactiveActuation(ctx));
        }
    }

    ctx.runHotReload();


    const end = Clock.now(.awake, ctx.io);
    if (print_stuff and ctx.log_counter % ctx.log_freq_div == 0) {
        const us = start.durationTo(end).toMicroseconds();
        std.log.debug("done in {} us\n", .{us});
    }
    ctx.log_counter +%= 1;
}


pub fn loopUntil(ctx: *MissionContext, mode: Mode, comptime fetch_first: bool, pred: anytype) !void {
    if (fetch_first) ctx.fetchFrameAndUpdate(mode);
    while (true) {
        const finished = pred.done(ctx) catch |err| return err;
        if (finished) break;
        ctx.fetchFrameAndUpdate(mode);
    }


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