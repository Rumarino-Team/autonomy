//! A bounded OS-thread pool. Each worker owns all mutable simulation/controller
//! state. Only counters and uniquely indexed result slots cross thread boundaries.
const std = @import("std");
const Io = std.Io;
const Scene = @import("SceneContext.zig");
const Mission = @import("MissionContext.zig");
const Args = @import("MissionArgs.zig");
const execute = @import("MissionRunner.zig").execute;
const c = Scene.c;
const Outcome = enum { pending, passed, failed, cancelled };
const Result = struct {
    index: usize,
    seed: u64,
    outcome: Outcome = .pending,
    error_name: []const u8 = "",
    stage: [64]u8 = @splat(0),
    detail: [512]u8 = @splat(0),
    stats: c.AuvSimulatorResult = std.mem.zeroes(c.AuvSimulatorResult),
};
const Provenance = struct { plugin_sha256: [64]u8, core_sha256: [64]u8, backend_sha256: [64]u8, zig_sha256: [64]u8, config_sha256: [64]u8 };
const Settings = struct { backend: [:0]const u8, robot: [:0]const u8, scene: [:0]const u8, max_ticks: u64, frame_ticks: u64 };
const Shared = struct {
    io: Io,
    args: Args,
    api: *const Scene.Api,
    settings: Settings,
    provenance: Provenance,
    config_contents: []const u8,
    results: []Result,
    next: std.atomic.Value(usize) = .init(0),
};
fn env(allocator: std.mem.Allocator, name: [:0]const u8, fallback: []const u8) ![:0]const u8 {
    const value = std.c.getenv(name);
    return allocator.dupeZ(u8, if (value) |v| std.mem.span(v) else fallback);
}
fn number(allocator: std.mem.Allocator, name: [:0]const u8, fallback: []const u8) !u64 {
    return std.fmt.parseInt(u64, try env(allocator, name, fallback), 10);
}
fn digest(io: Io, allocator: std.mem.Allocator, path: []const u8) ![64]u8 {
    const contents = try Io.Dir.cwd().readFileAlloc(io, path, allocator, .unlimited);
    defer allocator.free(contents);
    var hash: [32]u8 = undefined;
    std.crypto.hash.sha2.Sha256.hash(contents, &hash, .{});
    return std.fmt.bytesToHex(hash, .lower);
}
fn writeJson(io: Io, allocator: std.mem.Allocator, path: []const u8, value: anytype) !void {
    const encoded = try std.json.Stringify.valueAlloc(allocator, value, .{ .emit_null_optional_fields = true });
    defer allocator.free(encoded);
    const file = try Io.Dir.cwd().createFile(io, path, .{ .exclusive = true });
    defer file.close(io);
    try file.writeStreamingAll(io, encoded);
    try file.writeStreamingAll(io, "\n");
}
fn copy(dest: []u8, source: []const u8) void {
    @memset(dest, 0);
    @memcpy(dest[0..@min(dest.len - 1, source.len)], source[0..@min(dest.len - 1, source.len)]);
}
fn fail(result: *Result, err: anyerror) void {
    result.outcome = if (err == error.Stopped or err == error.Canceled) .cancelled else .failed;
    result.error_name = @errorName(err);
}
fn report(shared: *Shared, allocator: std.mem.Allocator, scene: ?*Scene, result: *Result) !void {
    const path = try std.fmt.allocPrint(allocator, "{s}/case-{d:0>6}.json", .{ shared.args.results_path, result.index });
    try writeJson(shared.io, allocator, path, .{
        .schema_version = 1,
        .index = result.index,
        .seed = result.seed,
        .generator = "xoshiro256++/splitmix64",
        .profile_version = Scene.FuzzProfile.version,
        .profile = if (shared.args.fuzz_nominal) Scene.FuzzProfile.nominal() else Scene.FuzzProfile{},
        .provenance = shared.provenance,
        .robot = shared.settings.robot,
        .scene = shared.settings.scene,
        .backend = "mujoco",
        .backend_version = std.mem.sliceTo(&shared.api.backend.api.version, 0),
        .asset_sha256 = if (scene) |s| std.fmt.bytesToHex(s.baseline.robot.asset_hash, .lower) else @as([64]u8, @splat('0')),
        .max_ticks = shared.settings.max_ticks,
        .frame_ticks = shared.settings.frame_ticks,
        .outcome = result.outcome,
        .error_name = result.error_name,
        .mission_stage = std.mem.sliceTo(&result.stage, 0),
        .detail = std.mem.sliceTo(&result.detail, 0),
        .tick = result.stats.tick,
        .hash = result.stats.hash,
        .status = result.stats.status,
        .min_depth = result.stats.min_depth,
        .max_depth = result.stats.max_depth,
        .final_state = if (result.stats.has_final_state != 0) result.stats.robot else null,
        .resolved = if (scene) |s| if (s.scene.initialized != 0) try s.snapshot(allocator) else null else null,
    });
}
fn runCase(shared: *Shared, scene: *Scene, allocator: std.mem.Allocator, result: *Result) void {
    var random = std.Random.Xoshiro256.init(result.seed);
    scene.prepare(random.random(), if (shared.args.fuzz_nominal) Scene.FuzzProfile.nominal() else .{}) catch |err| {
        fail(result, err);
        copy(&result.detail, std.mem.sliceTo(&scene.detail, 0));
        return;
    };
    scene.start(shared.settings.backend, shared.settings.max_ticks, shared.settings.frame_ticks) catch |err| {
        fail(result, err);
        copy(&result.detail, std.mem.sliceTo(&scene.detail, 0));
        return;
    };
    var ctx = Mission.initBatch(allocator, shared.io, shared.args, shared.api.auv, shared.config_contents) catch |err| {
        fail(result, err);
        return;
    };
    defer ctx.deinit();
    result.outcome = .passed;
    execute(&ctx, shared.args) catch |err| fail(result, err);
    if (Mission.stopRequested()) {
        result.outcome = .cancelled;
        result.error_name = "Stopped";
    }
    copy(&result.stage, ctx.step);
}
fn worker(shared: *Shared) void {
    var scene: Scene = undefined;
    var baseline_error: ?anyerror = null;
    scene.init(shared.api, shared.settings.scene, shared.settings.robot) catch |err| {
        baseline_error = err;
    };
    defer scene.deinit();
    while (!Mission.stopRequested()) {
        const index = shared.next.fetchAdd(1, .monotonic);
        if (index >= shared.results.len) return;
        const result = &shared.results[index];
        var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
        defer arena.deinit();
        const allocator = arena.allocator();
        if (baseline_error) |err| {
            fail(result, err);
            copy(&result.detail, std.mem.sliceTo(&scene.detail, 0));
        } else {
            runCase(shared, &scene, allocator, result);
            if (scene.active) result.stats = scene.readResult() catch |err| blk: {
                fail(result, err);
                break :blk std.mem.zeroes(c.AuvSimulatorResult);
            };
            if (result.detail[0] == 0 and result.stats.message[0] != 0) copy(&result.detail, std.mem.sliceTo(&result.stats.message, 0));
        }
        report(shared, allocator, if (baseline_error == null) &scene else null, result) catch |err| {
            fail(result, err);
        };
        scene.endCase();
    }
}
const DefaultLauncher = struct {
    pub fn spawn(shared: anytype) std.Thread.SpawnError!std.Thread {
        return spawnWorker(shared);
    }
};
pub fn spawnWorker(shared: *Shared) std.Thread.SpawnError!std.Thread {
    return std.Thread.spawn(.{}, worker, .{shared});
}
pub fn run(io: Io, args: Args) !void {
    return runWithLauncher(io, args, DefaultLauncher);
}
// A compile-time launcher seam exercises partial startup without a production
// environment switch or relying on the host's thread quota.
pub fn runWithLauncher(io: Io, args: Args, comptime Launcher: type) !void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();
    var api = try Scene.Api.load(args.auv_dynlib_path);
    defer api.deinit();
    const directory = std.fs.path.dirname(args.auv_dynlib_path) orelse ".";
    const default_backend = try std.fs.path.join(allocator, &.{ directory, "libsimulator_backend_mujoco.so" });
    const settings: Settings = .{
        .backend = try env(allocator, "AUTONOMY_SIM_BACKEND", default_backend),
        .robot = try env(allocator, "AUTONOMY_SIM_ROBOT", "hydrus"),
        .scene = try env(allocator, "AUTONOMY_SIM_SCENE", "robosub_course"),
        .max_ticks = try number(allocator, "AUTONOMY_SIM_MAX_TICKS", "21600"),
        .frame_ticks = try number(allocator, "AUTONOMY_SIM_FRAME_TICKS", "6"),
    };
    if (settings.frame_ticks == 0 or settings.max_ticks == 0 or settings.max_ticks % settings.frame_ticks != 0) return error.InvalidFuzzBudget;
    try api.pinBackend(settings.backend);
    const core = try std.fs.path.join(allocator, &.{ directory, "libsimulator_core.so" });
    const config_contents = try Io.Dir.cwd().readFileAlloc(io, args.live_config_path, allocator, .limited(16384));
    var config_hash: [32]u8 = undefined;
    std.crypto.hash.sha2.Sha256.hash(config_contents, &config_hash, .{});
    const provenance: Provenance = .{
        .plugin_sha256 = try digest(io, allocator, args.auv_dynlib_path),
        .core_sha256 = try digest(io, allocator, core),
        .backend_sha256 = try digest(io, allocator, settings.backend),
        .zig_sha256 = try digest(io, allocator, "/proc/self/exe"),
        .config_sha256 = std.fmt.bytesToHex(config_hash, .lower),
    };
    try Io.Dir.cwd().createDirPath(io, args.results_path);
    const campaign_path = try std.fs.path.join(allocator, &.{ args.results_path, "campaign.json" });
    try writeJson(io, allocator, campaign_path, .{ .schema_version = 1, .seed = args.master_seed, .cases = args.case_count, .workers = @min(args.case_count, args.workers), .replay_seed = args.replay_seed, .provenance = provenance, .settings = settings });
    const results = try allocator.alloc(Result, args.case_count);
    for (results, 0..) |*result, i| result.* = .{ .index = i, .seed = args.replay_seed orelse Scene.caseSeed(args.master_seed, i) };
    var shared: Shared = .{
        .io = io,
        .args = args,
        .api = &api,
        .settings = settings,
        .provenance = provenance,
        .results = results,
        .config_contents = config_contents,
    };
    const threads = try allocator.alloc(std.Thread, @min(args.case_count, args.workers));
    var started: usize = 0;
    var startup_error: ?anyerror = null;
    for (threads) |*thread| {
        thread.* = Launcher.spawn(&shared) catch |err| {
            startup_error = err;
            Mission.requestCancellation();
            break;
        };
        started += 1;
    }
    for (threads[0..started]) |thread| thread.join();
    var passed: usize = 0;
    var failed: usize = 0;
    var cancelled: usize = 0;
    const summary_path = try std.fs.path.join(allocator, &.{ args.results_path, "summary.jsonl" });
    const summary = try Io.Dir.cwd().createFile(io, summary_path, .{ .exclusive = true });
    defer summary.close(io);
    for (results) |*result| {
        if (result.outcome == .pending) {
            result.outcome = .cancelled;
            result.error_name = "Stopped";
            try report(&shared, allocator, null, result);
        }
        switch (result.outcome) {
            .passed => passed += 1,
            .failed => failed += 1,
            .cancelled => cancelled += 1,
            .pending => unreachable,
        }
        const json = try std.json.Stringify.valueAlloc(allocator, .{ .index = result.index, .seed = result.seed, .outcome = result.outcome, .error_name = result.error_name, .mission_stage = std.mem.sliceTo(&result.stage, 0), .tick = result.stats.tick, .hash = result.stats.hash }, .{});
        try summary.writeStreamingAll(io, json);
        try summary.writeStreamingAll(io, "\n");
    }
    const totals_path = try std.fs.path.join(allocator, &.{ args.results_path, "totals.json" });
    try writeJson(io, allocator, totals_path, .{ .passed = passed, .failed = failed, .cancelled = cancelled });
    std.log.info("fuzz: passed={d}, failed={d}, cancelled={d}; results={s}", .{ passed, failed, cancelled, args.results_path });
    if (startup_error) |err| return err;
    if (cancelled != 0) return error.FuzzInterrupted;
    if (failed != 0) return error.FuzzFailures;
}
