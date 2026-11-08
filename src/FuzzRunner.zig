//! A bounded OS-thread pool. Each worker owns all mutable simulation/controller
//! state. Only counters and uniquely indexed result slots cross thread boundaries.
const std = @import("std");
const Io = std.Io;
const Scene = @import("SceneContext.zig");
const Mission = @import("MissionContext.zig");
const Args = @import("MissionArgs.zig");
const execute = @import("MissionRunner.zig").execute;
const c = Scene.c;
const Binary = @import("BinaryData.zig");
const Config = @import("ConfigLoader.zig").Config;
const compiled_config = @import("../config/application.zig").config;
const Configuration = @import("Configuration.zig");
pub const Outcome = enum(u32) { pending, passed, failed, cancelled };
const Result = struct {
    index: usize,
    seed: u64,
    outcome: Outcome = .pending,
    error_name: []const u8 = "",
    stage: [64]u8 = @splat(0),
    detail: [512]u8 = @splat(0),
    stats: c.AuvSimulatorResult = std.mem.zeroes(c.AuvSimulatorResult),
};
pub const Provenance = struct { application_sha256: [64]u8, plugin_sha256: [64]u8, core_sha256: [64]u8, backend_sha256: [64]u8, zig_sha256: [64]u8, config_sha256: [64]u8 };
const Settings = struct { backend: [:0]const u8, robot: [:0]const u8, scene: [:0]const u8, max_ticks: u64, frame_ticks: u64 };
const Shared = struct {
    io: Io,
    args: Args,
    api: *const Scene.Api,
    settings: Settings,
    provenance: Provenance,
    config_contents: Config,
    config: Configuration.Application,
    results: []Result,
    next: std.atomic.Value(usize) = .init(0),
};
fn digest(io: Io, allocator: std.mem.Allocator, path: []const u8) ![64]u8 {
    const contents = try Io.Dir.cwd().readFileAlloc(io, path, allocator, .unlimited);
    defer allocator.free(contents);
    var hash: [32]u8 = undefined;
    std.crypto.hash.sha2.Sha256.hash(contents, &hash, .{});
    return std.fmt.bytesToHex(hash, .lower);
}
fn writeBinary(io: Io, allocator: std.mem.Allocator, path: []const u8, value: anytype) !void {
    const kind: Binary.Kind = if (std.mem.endsWith(u8, path, "campaign.bin")) .campaign else if (std.mem.endsWith(u8, path, "totals.bin")) .totals else .case;
    try Binary.writeFile(io, allocator, path, kind, value);
}
fn copy(dest: []u8, source: []const u8) void {
    @memset(dest, 0);
    @memcpy(dest[0..@min(dest.len - 1, source.len)], source[0..@min(dest.len - 1, source.len)]);
}
fn fail(result: *Result, err: anyerror) void {
    result.outcome = if (err == error.Stopped or err == error.Canceled) .cancelled else .failed;
    result.error_name = @errorName(err);
}
pub const CaseRecord = struct {
    schema_version: u32 = 1,
    index: usize,
    seed: u64,
    generator: []const u8 = "xoshiro256++/splitmix64",
    profile_version: u32 = Scene.FuzzProfile.version,
    profile: Scene.FuzzProfile,
    configuration: Configuration.Application,
    provenance: Provenance,
    robot: []const u8,
    scene: []const u8,
    backend: []const u8 = "mujoco",
    backend_version: []const u8,
    asset_sha256: [64]u8,
    max_ticks: u64,
    frame_ticks: u64,
    outcome: Outcome,
    error_name: []const u8,
    mission_stage: []const u8,
    detail: []const u8,
    tick: u64,
    hash: u64,
    status: u32,
    min_depth: f64,
    max_depth: f64,
    final_state: ?c.SimBodyState,
    resolved: ?Scene.Snapshot,
};
fn report(shared: *Shared, allocator: std.mem.Allocator, scene: ?*Scene, result: *Result) !void {
    const path = try std.fmt.allocPrint(allocator, "{s}/case-{d:0>6}.bin", .{ shared.args.results_path, result.index });
    try writeBinary(shared.io, allocator, path, CaseRecord{
        .index = result.index,
        .seed = result.seed,
        .profile = if (shared.args.fuzz_nominal) Scene.FuzzProfile.nominal() else shared.config.fuzz.profile,
        .configuration = shared.config,
        .provenance = shared.provenance,
        .robot = shared.settings.robot,
        .scene = shared.settings.scene,
        .backend_version = std.mem.sliceTo(&shared.api.backend.api.version, 0),
        .asset_sha256 = if (scene) |s| std.fmt.bytesToHex(s.baseline.robot.asset_hash, .lower) else @splat('0'),
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
    scene.prepare(random.random(), if (shared.args.fuzz_nominal) Scene.FuzzProfile.nominal() else shared.config.fuzz.profile) catch |err| {
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
    if (baseline_error == null) scene.configureBaseline(shared.config.physics.dt_numerator, shared.config.physics.dt_denominator, shared.config.physics.hydro_period) catch |err| {
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
pub fn runWithLauncher(io: Io, input_args: Args, comptime Launcher: type) !void {
    var args = input_args;
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();
    var config = compiled_config;
    if (args.replay_file) |path| {
        const bytes = try Io.Dir.cwd().readFileAlloc(io, path, allocator, .limited(Binary.max_bytes + 28));
        const recorded = try Binary.decode(allocator, CaseRecord, bytes, .case, 1);
        if (recorded.profile_version != Scene.FuzzProfile.version) return error.UnsupportedFuzzProfileVersion;
        config = recorded.configuration;
        config.fuzz.profile = recorded.profile;
        // Output location belongs to this invocation; do not overwrite the original campaign.
        config.fuzz.results = compiled_config.fuzz.results;
        args.replay_seed = recorded.seed;
        args.fuzz_nominal = false;
        args.mission_id = @enumFromInt(@intFromEnum(config.mission));
        args.goal_dist_threshold = config.goal_dist_threshold;
        args.case_count = 1;
        args.workers = 1;
    }
    var api = try Scene.Api.load(args.auv_dynlib_path);
    defer api.deinit();
    const directory = std.fs.path.dirname(args.auv_dynlib_path) orelse ".";
    const default_backend = try std.fs.path.join(allocator, &.{ directory, if (@import("builtin").os.tag == .windows) "libsimulator_backend_mujoco.dll" else "libsimulator_backend_mujoco.so" });
    const settings: Settings = .{
        .backend = try allocator.dupeSentinel(u8, if (args.backend_path.len > 0) args.backend_path else default_backend, 0),
        .robot = config.robot,
        .scene = config.scene,
        .max_ticks = config.max_ticks,
        .frame_ticks = config.frame_ticks,
    };
    if (settings.frame_ticks == 0 or settings.max_ticks == 0 or settings.max_ticks % settings.frame_ticks != 0) return error.InvalidFuzzBudget;
    try api.pinBackend(settings.backend);
    const core = try std.fs.path.join(allocator, &.{ directory, if (@import("builtin").os.tag == .windows) "libsimulator_core.dll" else "libsimulator_core.so" });
    const config_contents = config.controller;
    const config_bytes = try Binary.pack(allocator, .metadata, 1, config);
    var config_hash: [32]u8 = undefined;
    std.crypto.hash.sha2.Sha256.hash(config_bytes, &config_hash, .{});
    const provenance: Provenance = .{
        .application_sha256 = try digest(io, allocator, args.application_path),
        .plugin_sha256 = try digest(io, allocator, args.auv_dynlib_path),
        .core_sha256 = try digest(io, allocator, core),
        .backend_sha256 = try digest(io, allocator, settings.backend),
        .zig_sha256 = try digest(io, allocator, args.executable_path),
        .config_sha256 = std.fmt.bytesToHex(config_hash, .lower),
    };
    try Io.Dir.cwd().createDirPath(io, args.results_path);
    {
        var directory_view = try Io.Dir.cwd().openDir(io, args.results_path, .{ .iterate = true });
        defer directory_view.close(io);
        var entries = directory_view.iterate();
        if (try entries.next(io) != null) return error.ResultsDirectoryNotEmpty;
    }
    const campaign_path = try std.fs.path.join(allocator, &.{ args.results_path, "campaign.bin" });
    writeBinary(io, allocator, campaign_path, .{ .schema_version = 1, .seed = args.master_seed, .cases = args.case_count, .workers = @min(args.case_count, args.workers), .replay_seed = args.replay_seed, .provenance = provenance, .settings = settings }) catch |err| {
        if (err == error.PathAlreadyExists) {
            std.log.err("fuzz: `{s}` already contains campaign.bin — change config.fuzz.results to a fresh directory", .{args.results_path});
        }
        return err;
    };
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
        .config = config,
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
    const summary_path = try std.fs.path.join(allocator, &.{ args.results_path, "summary.bin" });
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
    }
    try Binary.writeFile(io, allocator, summary_path, .summary, results);
    const totals_path = try std.fs.path.join(allocator, &.{ args.results_path, "totals.bin" });
    try writeBinary(io, allocator, totals_path, .{ .passed = passed, .failed = failed, .cancelled = cancelled });
    std.log.info("fuzz: passed={d}, failed={d}, cancelled={d}; results={s}", .{ passed, failed, cancelled, args.results_path });
    if (startup_error) |err| return err;
    if (cancelled != 0) return error.FuzzInterrupted;
    if (failed != 0) return error.FuzzFailures;
}
