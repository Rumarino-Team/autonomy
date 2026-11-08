//! Permanent host: native resources survive replacement of every Zig application generation.
const std = @import("std");
const builtin = @import("builtin");
const Io = std.Io;
const Abi = @import("ApplicationApi.zig");
const Session = @import("HostSession.zig");
const DynLib = @import("DynLib.zig").DynLib;
const options = @import("host_options");
pub const std_options: std.Options = .{
    .signal_stack_size = if (@import("runtime_options").external_asan) null else 1 << 18,
    .enable_segfault_handler = !@import("runtime_options").external_asan,
};
var stop = std.atomic.Value(bool).init(false);
var session: ?*Session = null;
fn cancelled() callconv(.c) bool {
    return stop.load(.monotonic);
}
fn requestStop(_: std.posix.SIG) callconv(.c) void {
    stop.store(true, .monotonic);
}
fn telemetry(p: [*]const u8, n: usize) callconv(.c) void {
    if (session) |s| s.telemetry(p[0..n]);
}
fn commands(values: [*c]const f32, count: u8) callconv(.c) void {
    if (session) |s| {
        s.command_count = count;
        s.auv.setThrustorValues(values, count);
    }
}
fn teleop(p: [*]f32) callconv(.c) bool {
    return if (session) |s| s.teleop(p) else false;
}
const Win = struct {
    const W = std.os.windows;
    extern "kernel32" fn SetConsoleCtrlHandler(?*const fn (W.DWORD) callconv(.winapi) W.BOOL, W.BOOL) callconv(.winapi) W.BOOL;
    fn handler(code: W.DWORD) callconv(.winapi) W.BOOL {
        if (code > 1) return .FALSE;
        stop.store(true, .monotonic);
        return .TRUE;
    }
};
fn signals() void {
    if (builtin.os.tag == .windows) {
        _ = Win.SetConsoleCtrlHandler(&Win.handler, .TRUE);
        return;
    }
    const action: std.posix.Sigaction = .{ .handler = .{ .handler = requestStop }, .mask = std.posix.sigemptyset(), .flags = 0 };
    std.posix.sigaction(.INT, &action, null);
    std.posix.sigaction(.TERM, &action, null);
}
const Generation = struct {
    library: DynLib,
    api: *const Abi.Api,
    instance: *anyopaque,
    path: [:0]const u8,
    fn open(io: Io, allocator: std.mem.Allocator, source: []const u8, number: u64, services: *const Abi.Services) !Generation {
        const path = try std.fmt.allocPrintSentinel(allocator, "{s}.generation-{d}-{d}", .{ source, if (builtin.os.tag == .windows) std.os.windows.GetCurrentProcessId() else std.c.getpid(), number }, 0);
        errdefer allocator.free(path);
        const data = try Io.Dir.cwd().readFileAlloc(io, source, allocator, .limited(128 * 1024 * 1024));
        defer allocator.free(data);
        const file = try Io.Dir.cwd().createFile(io, path, .{ .exclusive = true });
        {
            defer file.close(io);
            try file.writeStreamingAll(io, data);
        }
        errdefer Io.Dir.cwd().deleteFile(io, path) catch {};
        var lib = try DynLib.open(path);
        errdefer lib.close();
        const get = lib.lookup(Abi.GetApi, "autonomy_application_api") orelse return error.MissingApplicationApi;
        const api = get(Abi.version) orelse return error.ApplicationAbiMismatch;
        if (api.struct_size != @sizeOf(Abi.Api) or api.abi_version != Abi.version) return error.ApplicationAbiMismatch;
        var error_buf: [512]u8 = @splat(0);
        var instance: ?*anyopaque = null;
        if (api.create(services, &instance, &error_buf, error_buf.len) != 0) {
            std.log.err("application create: {s}", .{std.mem.sliceTo(&error_buf, 0)});
            return error.ApplicationCreation;
        }
        return .{ .library = lib, .api = api, .instance = instance orelse return error.NullApplication, .path = path };
    }
    fn close(g: *Generation, io: Io, allocator: std.mem.Allocator) void {
        g.api.destroy(g.instance);
        g.library.close();
        Io.Dir.cwd().deleteFile(io, g.path) catch {};
        allocator.free(g.path);
    }
};
fn nativePath(io: Io, allocator: std.mem.Allocator, dir: []const u8, name: []const u8) ![:0]u8 {
    if (builtin.os.tag != .windows or !std.mem.endsWith(u8, name, ".so")) {
        if (std.fs.path.isAbsolute(name) or std.fs.path.dirname(name) != null) return allocator.dupeSentinel(u8, name, 0);
        return std.fmt.allocPrintSentinel(allocator, "{s}/{s}", .{ dir, name }, 0);
    }
    const stem = name[0 .. name.len - 3];
    const unprefixed = if (std.mem.startsWith(u8, stem, "lib")) stem[3..] else stem;
    for ([_][]const u8{ stem, unprefixed }) |base| {
        for ([_][]const u8{ dir, "" }) |directory| {
            const path = if (directory.len > 0) try std.fmt.allocPrintSentinel(allocator, "{s}/{s}.dll", .{ directory, base }, 0) else try std.fmt.allocPrintSentinel(allocator, "{s}/../bin/{s}.dll", .{ dir, base }, 0);
            if (Io.Dir.cwd().openFile(io, path, .{})) |file| {
                file.close(io);
                return path;
            } else |_| allocator.free(path);
        }
    }
    return std.fmt.allocPrintSentinel(allocator, "{s}/{s}.dll", .{ dir, stem }, 0);
}
fn sourceDigest(io: Io, allocator: std.mem.Allocator) !u64 {
    var hash = std.hash.Wyhash.init(0);
    for ([_][]const u8{ "src", "config", "include" }) |directory| {
        const path = try std.fs.path.join(allocator, &.{ options.source_root, directory });
        defer allocator.free(path);
        var dir = try Io.Dir.cwd().openDir(io, path, .{ .iterate = true });
        defer dir.close(io);
        var walker = try dir.walk(allocator);
        defer walker.deinit();
        while (try walker.next(io)) |entry| {
            if (entry.kind != .file) continue;
            const data = try entry.dir.readFileAlloc(io, entry.basename, allocator, .limited(8 * 1024 * 1024));
            defer allocator.free(data);
            hash.update(entry.path);
            hash.update(data);
        }
    }
    for ([_][]const u8{ "application.zig", "build.zig" }) |name| {
        const path = try std.fs.path.join(allocator, &.{ options.source_root, name });
        defer allocator.free(path);
        const data = try Io.Dir.cwd().readFileAlloc(io, path, allocator, .limited(1024 * 1024));
        defer allocator.free(data);
        hash.update(data);
    }
    return hash.final();
}
fn hostDigest(io: Io, allocator: std.mem.Allocator) !u64 {
    var hash = std.hash.Wyhash.init(0);
    for ([_][]const u8{ "src/main.zig", "src/HostSession.zig", "src/ApplicationApi.zig", "src/Auv.zig", "src/MathTypes.zig", "src/SimulatorApi.zig", "src/DynLib.zig", "include/application.h", "include/auv.h", "include/math.h", "build.zig" }) |name| {
        const path = try std.fs.path.join(allocator, &.{ options.source_root, name });
        defer allocator.free(path);
        const bytes = try Io.Dir.cwd().readFileAlloc(io, path, allocator, .limited(1024 * 1024));
        defer allocator.free(bytes);
        hash.update(bytes);
    }
    return hash.final();
}
fn build(io: Io) !bool {
    var child = try std.process.spawn(io, .{ .argv = &.{ options.zig_exe, "build", "--prefix", options.install_prefix }, .cwd = .{ .path = options.source_root } });
    return switch (try child.wait(io)) {
        .exited => |code| code == 0,
        else => false,
    };
}
// Native simulation pauses while building. A hardware plugin must keep its
// existing controller running until a candidate is ready for the frame-boundary swap.
fn buildWhileRunning(io: Io, current: *Generation, native: *Session) !bool {
    if (native.api != null) return build(io);
    const Job = struct {
        io: Io,
        done: std.atomic.Value(bool) = .init(false),
        success: bool = false,
        fn run(job: *@This()) void {
            job.success = build(job.io) catch false;
            job.done.store(true, .release);
        }
    };
    var job: Job = .{ .io = io };
    const thread = try std.Thread.spawn(.{}, Job.run, .{&job});
    var error_buf: [512]u8 = @splat(0);
    while (!job.done.load(.acquire) and !cancelled()) {
        const status = current.api.update(current.instance, &error_buf, error_buf.len);
        if (status != 0) {
            if (status < 0) std.log.err("application during build: {s}", .{std.mem.sliceTo(&error_buf, 0)});
            stop.store(true, .monotonic);
            break;
        }
    }
    thread.join();
    return job.success and !cancelled();
}
fn replace(io: Io, allocator: std.mem.Allocator, g: *Generation, candidate: *Generation, services: *Abi.Services, native: *Session, lib_dir: []const u8) !void {
    const preserving = candidate.api.flags(candidate.instance) & 2 != 0;
    var error_buf: [512]u8 = @splat(0);
    if (preserving) {
        var needed: usize = 0;
        if (g.api.capture(g.instance, null, 0, &needed, &error_buf, error_buf.len) != 0 or needed > 64 * 1024 * 1024) return error.StateCapture;
        const bytes = try allocator.alloc(u8, needed);
        defer allocator.free(bytes);
        if (g.api.capture(g.instance, bytes.ptr, bytes.len, &needed, &error_buf, error_buf.len) != 0) return error.StateCapture;
        if (candidate.api.restore(candidate.instance, bytes.ptr, needed, &error_buf, error_buf.len) != 0) {
            std.log.err("reload rejected: {s}", .{std.mem.sliceTo(&error_buf, 0)});
            return error.StateRestore;
        }
    } else {
        var launch: Abi.Launch = undefined;
        candidate.api.describe(candidate.instance, &launch);
        const plugin = try nativePath(io, allocator, lib_dir, std.mem.span(launch.plugin));
        const backend = try nativePath(io, allocator, lib_dir, std.mem.span(launch.backend));
        // A fresh restart ends the previous native session. If creation fails,
        // caller stops instead of pretending the old world can be restored.
        native.deinit();
        services.plugin_path = plugin.ptr;
        services.backend_path = backend.ptr;
        native.init(launch, plugin, backend, false, candidate.api.build_scene, candidate.instance) catch return error.RestartFailed;
        native.callbacks.commands = commands;
    }
    g.close(io, allocator);
    g.* = candidate.*;
    services.application_path = g.path.ptr;
    g.api.activate(g.instance);
    std.log.info("application reloaded ({s})", .{if (preserving) "state preserved" else "fresh world"});
}
fn writeResult(io: Io, current: *Generation, native: *Session) !void {
    var launch: Abi.Launch = undefined;
    current.api.describe(current.instance, &launch);
    const path = launch.result_path orelse return;
    if (native.api) |*api| {
        if (!native.active) return;
        var result: @import("SimulatorApi.zig").c.AuvSimulatorResult = undefined;
        var error_buf: [512]u8 = @splat(0);
        if (api.result(&result, @sizeOf(@TypeOf(result)), &error_buf, error_buf.len) != 0) return error.ResultUnavailable;
        try @import("BinaryData.zig").writeFile(io, std.heap.page_allocator, std.mem.span(path), .run, result);
    }
}
pub fn main(init: std.process.Init) !void {
    signals();
    const allocator = init.arena.allocator();
    const io = init.io;
    const argv = try init.minimal.args.toSlice(allocator);
    if (argv.len == 3 and std.mem.eql(u8, argv[1], "inspect")) return @import("BinaryData.zig").inspect(io, allocator, argv[2]);
    const exe = try std.process.executablePathAlloc(io, allocator);
    const bin_dir = std.fs.path.dirname(exe) orelse ".";
    const lib_dir = try std.fs.path.join(allocator, &.{ bin_dir, "..", "lib" });
    const application_path = try nativePath(io, allocator, lib_dir, "libautonomy_application.so");
    const plugin = try nativePath(io, allocator, lib_dir, "libauv_simulator.so");
    const backend = try nativePath(io, allocator, lib_dir, "libsimulator_backend_mujoco.so");
    const raw_args = try allocator.alloc([*:0]const u8, argv.len);
    for (raw_args, argv) |*a, b| a.* = b.ptr;
    var native: Session = .{ .allocator = allocator, .io = io };
    session = &native;
    defer session = null;
    defer native.deinit();
    var services: Abi.Services = .{ .argc = @intCast(argv.len), .argv = raw_args.ptr, .cancelled = cancelled, .telemetry = telemetry, .teleop = teleop, .auv = &native.callbacks, .plugin_path = plugin.ptr, .backend_path = backend.ptr, .executable_path = exe.ptr, .application_path = application_path.ptr };
    var generation: u64 = 1;
    var current = try Generation.open(io, std.heap.page_allocator, application_path, generation, &services);
    defer current.close(io, std.heap.page_allocator);
    var launch: Abi.Launch = undefined;
    current.api.describe(current.instance, &launch);
    const actual_plugin = try nativePath(io, allocator, lib_dir, std.mem.span(launch.plugin));
    const actual_backend = try nativePath(io, allocator, lib_dir, std.mem.span(launch.backend));
    services.plugin_path = actual_plugin;
    services.backend_path = actual_backend;
    try native.init(launch, actual_plugin, actual_backend, current.api.flags(current.instance) & 4 != 0, current.api.build_scene, current.instance);
    native.callbacks.commands = commands;
    services.application_path = current.path.ptr;
    current.api.activate(current.instance);
    std.log.info("application ready", .{});
    const host_digest = hostDigest(io, std.heap.page_allocator) catch 0;
    var digest = sourceDigest(io, std.heap.page_allocator) catch 0;
    var checked = Io.Clock.Timestamp.now(io, .awake);
    var error_buf: [512]u8 = @splat(0);
    while (!cancelled()) {
        if (current.api.flags(current.instance) & 1 != 0 and current.api.flags(current.instance) & 4 == 0 and checked.untilNow(io).raw.toMilliseconds() >= 100) {
            checked = Io.Clock.Timestamp.now(io, .awake);
            const next = sourceDigest(io, std.heap.page_allocator) catch digest;
            if (next != digest) {
                if ((hostDigest(io, std.heap.page_allocator) catch host_digest) != host_digest) {
                    std.log.err("host/ABI sources changed; restart the executable to apply them", .{});
                    digest = next;
                    continue;
                }
                // Native physics is idle throughout compilation and replacement.
                std.log.info("application sources changed; rebuilding", .{});
                // Debounce editor save bursts while physics is idle.
                Io.sleep(io, .fromMilliseconds(100), .awake) catch {};
                digest = sourceDigest(io, std.heap.page_allocator) catch next;
                if (native.api) |*api| {
                    var result: @import("SimulatorApi.zig").c.AuvSimulatorResult = undefined;
                    var detail: [512]u8 = undefined;
                    if (native.active and api.result(&result, @sizeOf(@TypeOf(result)), &detail, detail.len) == 0) std.log.info("application paused tick={d} hash={d}", .{ result.tick, result.hash });
                }
                if (buildWhileRunning(io, &current, &native) catch false) {
                    const built_digest = sourceDigest(io, std.heap.page_allocator) catch digest;
                    if (built_digest != digest) {
                        // A newer edit arrived during compilation. Rebuild before
                        // activating a generation from an indeterminate source set.
                        if (native.rebase) |rebase| rebase();
                        continue;
                    }
                    generation += 1;
                    if (Generation.open(io, std.heap.page_allocator, application_path, generation, &services)) |candidate_value| {
                        var candidate = candidate_value;
                        replace(io, std.heap.page_allocator, &current, &candidate, &services, &native, lib_dir) catch |err| {
                            candidate.close(io, std.heap.page_allocator);
                            if (err == error.RestartFailed) return err;
                            std.log.err("kept previous application: {s}", .{@errorName(err)});
                        };
                    } else |err| std.log.err("kept previous application: {s}", .{@errorName(err)});
                } else std.log.err("build failed; kept previous application", .{});
                if (native.rebase) |rebase| rebase();
                if (native.api) |*api| {
                    var result: @import("SimulatorApi.zig").c.AuvSimulatorResult = undefined;
                    var detail: [512]u8 = undefined;
                    if (native.active and api.result(&result, @sizeOf(@TypeOf(result)), &detail, detail.len) == 0) std.log.info("application resumed tick={d} hash={d}", .{ result.tick, result.hash });
                }
                checked = Io.Clock.Timestamp.now(io, .awake);
            }
        }
        const result = current.api.update(current.instance, &error_buf, error_buf.len);
        if (result == 1 or result == 2) break;
        if (result != 0) {
            try writeResult(io, &current, &native);
            std.log.err("application: {s}", .{std.mem.sliceTo(&error_buf, 0)});
            return error.ApplicationFailed;
        }
    }
    try writeResult(io, &current, &native);
}
