const std = @import("std");
const math = @import("math.zig");
const MissionContext = @import("MissionContext.zig");
const MissionArgs = @import("MissionArgs.zig");
const Auv = @import("Auv.zig");

const batch_envs: c_int = 32;
const record_fps: f64 = 24;

pub fn main(init: std.process.Init) !void {
    const args_slice = try init.minimal.args.toSlice(init.arena.allocator());
    const args: MissionArgs = try .init(args_slice);

    std.log.info("args = {f}", .{args});

    if (args.mission_id == .batch) {
        try runBatch(init.io, args);
        return;
    }
    if (args.mission_id == .fuzz) {
        try runFuzz(init.io, args);
        return;
    }
    if (args.mission_id == .record) {
        try runRecord(init.arena.allocator(), init.io, args);
        return;
    }
    try runMission(init.arena.allocator(), init.io, args);
}

/// One headless world on the simulator clock. The recording is the product, so a
/// mission failure still ends with a complete file and its reason.
fn runRecord(allocator: std.mem.Allocator, io: std.Io, args: MissionArgs) !void {
    var dynlib = try std.DynLib.open(args.auv_dynlib_path);
    defer dynlib.close();
    const prepare = dynlib.lookup(*const Auv.PrepareBatchFunc, "auv_prepare_batch") orelse {
        std.log.err("record requires a plugin that exports auv_prepare_batch", .{});
        return error.RecordUnavailable;
    };
    if (!prepare(1)) {
        std.log.err("auv_prepare_batch(1) failed", .{});
        return error.BatchPrepareFailed;
    }

    var ctx: MissionContext = try .init(allocator, io, args);
    defer ctx.auv.deinit();
    const begin = ctx.auv.beginFuzz orelse return error.RecordUnavailable;
    const start = ctx.auv.recordStart orelse return error.RecordUnavailable;
    const stop = ctx.auv.recordStop orelse return error.RecordUnavailable;

    if (!begin(args.record_seed)) return error.WorldResetFailed;
    if (!start(args.record_path.?.ptr, record_fps)) {
        std.log.err("cannot record to {s}", .{args.record_path.?});
        return error.RecordStartFailed;
    }
    ctx.stopThrusters();

    const result = executeMission(&ctx, args);
    var reason_buf: [64]u8 = undefined;
    const reason = if (result) |_| "complete" else |err| @errorName(err);
    const reason_z = std.fmt.bufPrintZ(&reason_buf, "{s}", .{reason}) catch "unknown";
    stop(reason_z.ptr);
    std.log.info("record ended: {s}", .{reason});
}

fn runBatch(io: std.Io, args: MissionArgs) !void {
    var dynlib = try std.DynLib.open(args.auv_dynlib_path);
    defer dynlib.close();
    const prepare = dynlib.lookup(*const Auv.PrepareBatchFunc, "auv_prepare_batch") orelse {
        std.log.err("batch requires a plugin that exports auv_prepare_batch", .{});
        return error.BatchUnavailable;
    };
    if (!prepare(batch_envs)) {
        std.log.err("auv_prepare_batch({d}) failed", .{batch_envs});
        return error.BatchPrepareFailed;
    }

    var outcomes: [batch_envs]anyerror!void = @splat({});
    var threads: [batch_envs]std.Thread = undefined;
    var started: usize = 0;
    errdefer for (threads[0..started]) |thread| thread.join();
    for (&threads, &outcomes) |*thread, *outcome| {
        thread.* = try std.Thread.spawn(.{}, threadMain, .{ io, args, outcome });
        started += 1;
    }
    for (threads) |thread| thread.join();
    // errdefer also joins. Clear this first so a mission error does not join twice.
    started = 0;
    for (outcomes) |outcome| try outcome;
}

fn runFuzz(io: std.Io, args: MissionArgs) !void {
    const count: u64 = if (args.replay_seed != null) 1 else args.case_count;
    const workers: usize = @min(count, batch_envs);
    var dynlib = try std.DynLib.open(args.auv_dynlib_path);
    defer dynlib.close();
    const prepare = dynlib.lookup(*const Auv.PrepareBatchFunc, "auv_prepare_batch") orelse {
        std.log.err("fuzz requires a plugin that exports auv_prepare_batch", .{});
        return error.FuzzUnavailable;
    };
    if (dynlib.lookup(*const Auv.BeginFuzzFunc, "auv_begin_fuzz") == null) {
        std.log.err("fuzz requires auv_begin_fuzz", .{});
        return error.FuzzUnavailable;
    }
    if (!prepare(@intCast(workers))) {
        std.log.err("auv_prepare_batch({d}) failed", .{workers});
        return error.BatchPrepareFailed;
    }

    var next: std.atomic.Value(u64) = .init(0);
    var failures: std.atomic.Value(u32) = .init(0);
    var threads: [batch_envs]std.Thread = undefined;
    var started: usize = 0;
    errdefer for (threads[0..started]) |thread| thread.join();
    for (threads[0..workers]) |*thread| {
        thread.* = try std.Thread.spawn(.{}, fuzzWorker, .{ io, args, &next, &failures });
        started += 1;
    }
    for (threads[0..started]) |thread| thread.join();
    started = 0;

    const failed = failures.load(.monotonic);
    if (failed != 0) {
        std.log.err("fuzz failures={d}", .{failed});
        return error.FuzzFailures;
    }
    std.log.info("fuzz passed {d} cases", .{count});
}

fn splitmix64(x: u64) u64 {
    var z = x +% 0x9E3779B97F4A7C15;
    z = (z ^ (z >> 30)) *% 0xBF58476D1CE4E5B9;
    z = (z ^ (z >> 27)) *% 0x94D049BB133111EB;
    return z ^ (z >> 31);
}

fn caseSeed(master: u64, index: u64) u64 {
    return splitmix64(master +% index *% 0x9E3779B97F4A7C15);
}

fn fuzzWorker(io: std.Io, args: MissionArgs, next: *std.atomic.Value(u64), failures: *std.atomic.Value(u32)) void {
    const count: u64 = if (args.replay_seed != null) 1 else args.case_count;
    while (true) {
        const index = next.fetchAdd(1, .monotonic);
        if (index >= count) return;
        const seed = args.replay_seed orelse caseSeed(args.master_seed, index);
        var arena_impl: std.heap.ArenaAllocator = .init(std.heap.page_allocator);
        defer arena_impl.deinit();
        var ctx = MissionContext.init(arena_impl.allocator(), io, args) catch |err| {
            std.log.err("fuzz worker failed: {s}", .{@errorName(err)});
            _ = failures.fetchAdd(1, .monotonic);
            return;
        };
        defer ctx.auv.deinit();
        const begin = ctx.auv.beginFuzz orelse {
            std.log.err("fuzz seed={d} error={s}", .{ seed, @errorName(error.FuzzUnavailable) });
            _ = failures.fetchAdd(1, .monotonic);
            return;
        };
        if (!begin(seed)) {
            std.log.err("fuzz seed={d} error={s}", .{ seed, @errorName(error.WorldResetFailed) });
            _ = failures.fetchAdd(1, .monotonic);
            continue;
        }
        ctx.stopThrusters();
        executeMission(&ctx, args) catch |err| {
            const time = @as(f64, @floatFromInt(ctx.frame.timestamp)) * 1e-9;
            std.log.err("fuzz seed={d} error={s} time={d:.3}", .{ seed, @errorName(err), time });
            _ = failures.fetchAdd(1, .monotonic);
        };
    }
}

fn threadMain(io: std.Io, args: MissionArgs, outcome: *anyerror!void) void {
    var arena_impl: std.heap.ArenaAllocator = .init(std.heap.page_allocator);
    defer arena_impl.deinit();
    outcome.* = runMission(arena_impl.allocator(), io, args);
}

fn runMission(allocator: std.mem.Allocator, io: std.Io, args: MissionArgs) !void {
    var ctx: MissionContext = try .init(allocator, io, args);
    defer if (args.mission_id == .batch) ctx.auv.deinit();
    try executeMission(&ctx, args);
}

fn executeMission(ctx: *MissionContext, args: MissionArgs) !void {
    const sim_clock = args.mission_id == .batch or args.mission_id == .fuzz or args.mission_id == .record;
    mission_loop: while (true) {
        ctx.yieldUntilNextFrameAndUpdate() catch |err| switch (err) {
            error.RestartMission => continue,
            else => return err,
        };
        var warmup_seconds: f32 = 0;
        while (ctx.navigation.snapshot.orientation == null) {
            warmup_seconds += if (sim_clock) ctx.controller_dt else @max(ctx.controller_dt, ctx.execution_dt);
            if (warmup_seconds >= 2) {
                ctx.stopThrusters();
                return error.SensorUnavailable;
            }
            ctx.yieldUntilNextFrameAndUpdate() catch |err| switch (err) {
                error.RestartMission => continue :mission_loop,
                else => return err,
            };
        }
        ctx.holdPosition();

        const mission_result = switch (args.mission_id) {
            .prequalify, .batch, .fuzz, .record => @import("missions/prequalify.zig").mission(ctx),
            .teleop => @import("missions/prequalify.zig").teleopMission(ctx),
            .teleop_command => @import("missions/teleop.zig").teleopCommandMission(ctx),
        };

        mission_result catch |err| switch (err) {
            error.RestartMission => {
                std.log.info("restarting mission context", .{});
                ctx.reset();
                std.log.info("restarting mission", .{});
                continue;
            },
            else => {
                ctx.stopThrusters();
                return err;
            },
        };

        std.log.info("succesfully completed mission", .{});
        break;
    }
}

test {
    // The test build does not execute main, so explicitly discover the motion
    // and controller modules' regression tests.
    std.testing.refAllDecls(@import("Motion.zig"));
    std.testing.refAllDecls(@import("MotionController.zig"));
    std.testing.refAllDecls(@import("ThrusterAllocator.zig"));
    std.testing.refAllDecls(MissionContext);
    std.testing.refAllDecls(@import("missions/prequalify.zig"));
    std.testing.refAllDecls(math);
}

test "splitmix64 is the published mix" {
    try std.testing.expectEqual(@as(u64, 0x910a2dec89025cc1), splitmix64(1));
    try std.testing.expectEqual(splitmix64(1), caseSeed(1, 0));
    try std.testing.expect(caseSeed(1, 0) != caseSeed(1, 1));
}
