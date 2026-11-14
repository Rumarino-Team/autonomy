//! Reloadable application. A generation never owns the retained native world.
const std = @import("std");
const Abi = @import("ApplicationApi.zig");
const Mission = @import("MissionContext.zig");
const Runner = @import("MissionRunner.zig");
const Args = @import("MissionArgs.zig");
const Binary = @import("BinaryData.zig");
const config = @import("../config/application.zig").config;
const State = struct { structural: [32]u8, mission: Mission.Saved, runner: Runner.State };
// Schema 1 is intentionally retained as an explicit migration path. It did not
// preserve the controller diagnostic counter; schema 2 does through Mission.Saved.
const SavedV1 = struct {
    frame: @import("Auv.zig").Frame,
    seen_objects: []const @import("Auv.zig").Object,
    seen_objects2d: []const @import("Auv.zig").Object2d,
    goals: []const Mission.Goal,
    target_position: @import("math.zig").Vector3f,
    target_orientation: @import("math.zig").Quaternionf,
    use_position: bool,
    use_orientation: bool,
    position_axes: @import("Auv.zig").AxisOk,
    desired: @import("math.zig").Vector2f,
    desired_height: ?f32,
    translation: ?@import("math.zig").Vector3f,
    pid: [6]@import("Pid.zig"),
    previous_position: ?@import("math.zig").Vector3f,
    tilt_correction: @import("math.zig").Vector3f,
    force_error: @import("math.zig").Vector3f,
    pid_prev_timestamp_ns: ?u64,
};
const StateV1 = struct { structural: [32]u8, mission: SavedV1, runner: Runner.State };
const App = struct {
    arena: std.heap.ArenaAllocator,
    threaded: std.Io.Threaded,
    services: *const Abi.Services,
    args: Args,
    mission: Mission,
    runner: Runner.State = .{},
    active: bool = false,
    batch_done: bool = false,
    fn destroy(self: *App) void {
        self.mission.deinit();
        self.threaded.deinit();
        self.arena.deinit();
        std.heap.page_allocator.destroy(self);
    }
};
fn app(ptr: *anyopaque) *App {
    return @ptrCast(@alignCast(ptr));
}
fn failure(err: anyerror, buffer: [*]u8, size: usize) i32 {
    if (size > 0) {
        const s = @errorName(err);
        const n = @min(s.len, size - 1);
        @memcpy(buffer[0..n], s[0..n]);
        buffer[n] = 0;
    }
    return -1;
}
fn structural(allocator: std.mem.Allocator) ![32]u8 {
    const bytes = try Binary.pack(allocator, .state, 1, .{
        .scene_builder = @embedFile("scenes/ApplicationScene.zig"),
        .mission = config.mission,
        .plugin = config.plugin,
        .backend = config.backend,
        .robot = config.robot,
        .scene = config.scene,
        .frame_ticks = config.frame_ticks,
        .max_ticks = config.max_ticks,
        .realtime = config.realtime,
        .physics = config.physics,
        .blender = config.blender,
        .teleop_port = config.controller.teleop_port,
        .telemetry = config.telemetry,
        .telemetry_address = config.telemetry_address,
        .motor_count = @as(u32, @intCast(config.controller.tam.len)),
    });
    defer allocator.free(bytes);
    var hash: [32]u8 = undefined;
    std.crypto.hash.sha2.Sha256.hash(bytes, &hash, .{});
    return hash;
}
fn create(services: *const Abi.Services, out: *?*anyopaque, error_buf: [*]u8, size: usize) callconv(.c) i32 {
    out.* = null;
    const instance = createInner(services) catch |err| return failure(err, error_buf, size);
    out.* = instance;
    return 0;
}
fn createInner(services: *const Abi.Services) !*App {
    if (services.abi_version != Abi.version or services.struct_size != @sizeOf(Abi.Services)) return error.HostAbiMismatch;
    try @import("Configuration.zig").validateApplication(config);
    const p = try std.heap.page_allocator.create(App);
    errdefer std.heap.page_allocator.destroy(p);
    p.arena = .init(std.heap.page_allocator);
    errdefer p.arena.deinit();
    // This DLL must not install process signal handlers or spawn Io workers.
    // Explicit OS fuzz threads join before destruction; synchronous Io remains private.
    p.threaded = .init_single_threaded;
    errdefer p.threaded.deinit();
    p.services = services;
    const allocator = p.arena.allocator();
    const argv = try allocator.alloc([:0]const u8, services.argc);
    for (argv, services.argv[0..services.argc]) |*s, raw| s.* = std.mem.span(raw);
    p.args = try Args.init(argv);
    p.args.auv_dynlib_path = std.mem.span(services.plugin_path);
    p.args.backend_path = std.mem.span(services.backend_path);
    p.args.executable_path = std.mem.span(services.executable_path);
    p.mission = try Mission.initBatch(allocator, p.threaded.io(), p.args, services.auv.to(), config.controller);
    p.runner = .{};
    p.active = false;
    p.batch_done = false;
    Mission.cancellation = services.cancelled;
    return p;
}
fn destroy(ptr: *anyopaque) callconv(.c) void {
    app(ptr).destroy();
}
fn describe(_: *anyopaque, out: *Abi.Launch) callconv(.c) void {
    // String literals remain borrowed until the host copies this descriptor.
    out.* = .{ .plugin = config.plugin, .backend = config.backend, .robot = config.robot, .scene = config.scene, .telemetry_address = config.telemetry_address.ptr, .frame_ticks = config.frame_ticks, .max_ticks = config.max_ticks, .dt_numerator = config.physics.dt_numerator, .dt_denominator = config.physics.dt_denominator, .hydro_period = config.physics.hydro_period, .result_path = if (config.run_result_path) |path| path.ptr else null, .blender_port = config.blender.port, .blender_enabled = @intFromBool(config.blender.enabled), .teleop_port = config.controller.teleop_port, .realtime = @intFromBool(config.realtime), .telemetry = @intFromBool(config.telemetry), .fuzz = 0, .teleop_enabled = @intFromBool(config.mission == .teleop) };
}
// Called only while creating a fresh native session. The host does not retain
// this callback; geometry/records in the returned root are core-owned copies.
fn buildScene(ptr: *anyopaque, root: *anyopaque, error_buf: [*]u8, size: usize) callconv(.c) i32 {
    const Scene = @import("SceneContext.zig");
    var scene_api = Scene.Api.load(std.mem.span(app(ptr).services.plugin_path)) catch |err| return failure(err, error_buf, size);
    defer scene_api.deinit();
    const output: *Scene.c.Simulation = @ptrCast(@alignCast(root));
    @import("scenes/ApplicationScene.zig").construct(&scene_api, output, config.scene, config.robot, config.physics, error_buf[0..size]) catch |err| return failure(err, error_buf, size);
    return 0;
}
fn flags(ptr: *anyopaque) callconv(.c) u32 {
    return @as(u32, @intFromBool(config.reload.watch)) | (@as(u32, @intFromBool(config.reload.preserve_state)) << 1) | (@as(u32, @intFromBool(app(ptr).args.fuzz)) << 2);
}
fn update(ptr: *anyopaque, error_buf: [*]u8, size: usize) callconv(.c) i32 {
    const a = app(ptr);
    if (!a.active) return failure(error.InactiveApplication, error_buf, size);
    if (a.args.fuzz) {
        if (a.batch_done) return 1;
        a.batch_done = true;
        @import("FuzzRunner.zig").run(a.threaded.io(), a.args) catch |err| return failure(err, error_buf, size);
        return 1;
    }
    const done = Runner.step(&a.mission, &a.runner, a.args) catch |err| {
        if (err == error.Stopped) return 2;
        return failure(err, error_buf, size);
    };
    if (done) {
        std.log.info("successfully completed mission", .{});
        return 1;
    }
    return 0;
}
fn capture(ptr: *anyopaque, buffer: ?[*]u8, capacity: usize, needed: *usize, error_buf: [*]u8, size: usize) callconv(.c) i32 {
    const a = app(ptr);
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    const alloc = arena.allocator();
    const state: State = .{ .structural = structural(alloc) catch |err| return failure(err, error_buf, size), .mission = a.mission.save(alloc) catch |err| return failure(err, error_buf, size), .runner = a.runner };
    const bytes = Binary.pack(alloc, .state, 2, state) catch |err| return failure(err, error_buf, size);
    needed.* = bytes.len;
    if (buffer) |b| {
        if (capacity < bytes.len) return failure(error.StateBufferCapacity, error_buf, size);
        @memcpy(b[0..bytes.len], bytes);
    }
    return 0;
}
fn restore(ptr: *anyopaque, buffer: [*]const u8, len: usize, error_buf: [*]u8, size: usize) callconv(.c) i32 {
    restoreInner(app(ptr), buffer[0..len]) catch |err| return failure(err, error_buf, size);
    return 0;
}
fn restoreInner(a: *App, bytes: []const u8) !void {
    if (a.active or a.args.fuzz) return error.InvalidRestore;
    const allocator = a.arena.allocator();
    const payload = try Binary.unpack(bytes);
    const state: State = switch (payload.schema) {
        1 => blk: {
            const old = try Binary.decode(allocator, StateV1, bytes, .state, 1);
            var migrated: State = undefined;
            migrated.structural = old.structural;
            migrated.runner = old.runner;
            const info = @typeInfo(SavedV1).@"struct";
            if (@hasField(@TypeOf(info), "fields")) {
                inline for (info.fields) |field| @field(migrated.mission, field.name) = @field(old.mission, field.name);
            } else {
                inline for (info.field_names) |name| @field(migrated.mission, name) = @field(old.mission, name);
            }
            migrated.mission.log_counter = 0;
            break :blk migrated;
        },
        2 => try Binary.decode(allocator, State, bytes, .state, 2),
        else => return error.UnsupportedStateMigration,
    };
    if (!std.mem.eql(u8, &state.structural, &(try structural(allocator)))) return error.StructuralChangeRequiresRestart;
    try a.mission.restore(state.mission);
    a.runner = state.runner;
    a.mission.step = a.runner.name(a.args.mission_id);
}
fn activate(ptr: *anyopaque) callconv(.c) void {
    const a = app(ptr);
    a.active = true;
    a.mission.auv = a.services.auv.to();
    a.args.application_path = std.mem.span(a.services.application_path);
    a.args.auv_dynlib_path = std.mem.span(a.services.plugin_path);
    a.args.backend_path = std.mem.span(a.services.backend_path);
    a.mission.batch = false;
    a.mission.verbose = config.verbose;
    a.mission.telemetry = if (config.telemetry and !a.args.fuzz) a.services.telemetry else null;
    a.mission.teleop = a.services.teleop;
    if (a.args.fuzz) {
        a.mission.batch = true;
        a.mission.verbose = false;
    }
}
const api: Abi.Api = .{ .create = create, .destroy = destroy, .describe = describe, .build_scene = buildScene, .flags = flags, .update = update, .capture = capture, .restore = restore, .activate = activate };
pub export fn autonomy_application_api(version: u32) callconv(.c) ?*const Abi.Api {
    return if (version == Abi.version) &api else null;
}

test {
    _ = @import("AckermannControl.zig");
    _ = @import("Pid.zig");
    _ = @import("BinaryData.zig");
    _ = @import("SceneContext.zig");
    std.testing.refAllDecls(@import("BodyControl.zig"));
    std.testing.refAllDecls(Mission);
    std.testing.refAllDecls(Runner);
}
