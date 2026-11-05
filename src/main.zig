const std = @import("std");
const MissionContext = @import("MissionContext.zig");
const MissionArgs = @import("MissionArgs.zig");

test {
    _ = @import("Pid.zig");
    _ = @import("Simulation.zig");
    std.testing.refAllDecls(@import("BodyControl.zig"));
    std.testing.refAllDecls(@import("MissionContext.zig"));
    std.testing.refAllDecls(@import("ConfigLoader.zig"));
}

const batch_envs: usize = 32;

pub fn main(init: std.process.Init) !void {
    const args_slice = try init.minimal.args.toSlice(init.arena.allocator());
    const args: MissionArgs = try .init(args_slice);

    std.log.info("args = {f}", .{args});

    if (args.mission_id == .fuzz) {
        try runFuzz(init.io, args);
        return;
    }

    var ctx: MissionContext = try .init(init.arena.allocator(), init.io, args);
    defer ctx.auv.deinit();
    try executeMission(&ctx, args);
}

fn executeMission(ctx: *MissionContext, args: MissionArgs) !void {
    const course = if (args.mission_id == .fuzz) MissionArgs.MissionId.prequalify_robosub else args.mission_id;
    while (true) {
        try ctx.yieldUntilNextFrameAndUpdate();
        // Make sure the target position starts with the initial odometry measurement.
        ctx.target_position = ctx.frame.camera_pose.pos;

        const mission_result = switch (course) {
            .prequalify_coco => @import("missions/prequalify.zig").cocoMission(ctx),
            .prequalify_robosub, .fuzz => @import("missions/prequalify.zig").robosubMission(ctx),
            .teleop => @import("missions/prequalify.zig").teleopMission(ctx),
        };

        mission_result catch |err| switch (err) {
            error.RestartMission => {
                std.log.info("restaring mission context", .{});
                ctx.reset();
                std.log.info("restaring mission", .{});
                continue;
            },
            error.Stopped => {
                std.log.info("stopped", .{});
                break;
            },
            else => return err,
        };

        std.log.info("succesfully completed mission", .{});
        break;
    }
}

fn runFuzz(io: std.Io, args: MissionArgs) !void {
    const count: u64 = if (args.replay_seed != null) 1 else args.case_count;
    const workers: usize = @min(count, batch_envs);
    var dynlib = try std.DynLib.open(args.auv_dynlib_path);
    defer dynlib.close();
    const prepare = dynlib.lookup(*const @import("Auv.zig").PrepareBatchFunc, "auv_prepare_batch") orelse {
        std.log.err("fuzz requires a plugin that exports auv_prepare_batch", .{});
        return error.FuzzUnavailable;
    };
    if (dynlib.lookup(*const @import("Auv.zig").BeginFuzzFunc, "auv_begin_fuzz") == null) {
        std.log.err("fuzz requires auv_begin_fuzz", .{});
        return error.FuzzUnavailable;
    }
    if (!prepare(@intCast(workers))) {
        std.log.err("auv_prepare_batch({d}) failed", .{workers});
        return error.BatchPrepareFailed;
    }

    var next: std.atomic.Value(u64) = .init(0);
    var failures: std.atomic.Value(u32) = .init(0);
    const threads = try std.heap.page_allocator.alloc(std.Thread, workers);
    defer std.heap.page_allocator.free(threads);
    var started: usize = 0;
    errdefer for (threads[0..started]) |thread| thread.join();
    for (threads) |*thread| {
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

test "splitmix64 is the published mix" {
    try std.testing.expectEqual(@as(u64, 0x910a2dec89025cc1), splitmix64(1));
    try std.testing.expectEqual(splitmix64(1), caseSeed(1, 0));
    try std.testing.expect(caseSeed(1, 0) != caseSeed(1, 1));
}
