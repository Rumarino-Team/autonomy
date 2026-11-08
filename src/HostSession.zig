//! Permanent native/I/O ownership. None of these resources point into app DLL code.
const std = @import("std");
const Io = std.Io;
const net = Io.net;
const Auv = @import("Auv.zig");
const Scene = @import("SimulatorApi.zig");
const Abi = @import("ApplicationApi.zig");
const DynLib = @import("DynLib.zig").DynLib;
const Session = @This();
allocator: std.mem.Allocator,
io: Io,
api: ?Scene.Api = null,
library: ?DynLib = null,
model: ?*Scene.c.Simulation = null,
auv: Auv = .{ .init = noop, .deinit = noop, .yieldUntilNextFrame = noFrame, .setThrustorValues = noCommands },
callbacks: Abi.AuvCallbacks = .{ .init = noop, .deinit = noop, .yield_frame = noFrame, .commands = noCommands, .status = null },
active: bool = false,
command_count: u8 = 0,
socket: ?Io.net.Socket = null,
destination: ?Io.net.IpAddress = null,
teleop_socket: ?Io.net.Socket = null,
rebase: ?*const fn () callconv(.c) void = null,
fn noop() callconv(.c) void {}
fn noFrame(_: *Auv.Frame) callconv(.c) void {}
fn noCommands(_: [*c]const f32, _: u8) callconv(.c) void {}
pub fn init(s: *Session, launch: Abi.Launch, plugin_path: [:0]const u8, backend_path: [:0]const u8, fuzz: bool, build_scene: @FieldType(Abi.Api, "build_scene"), instance: *anyopaque) !void {
    errdefer s.deinit();
    var lib = try DynLib.open(plugin_path);
    s.library = lib;
    s.auv = .{
        .init = lib.lookup(*const Auv.InitFunc, "auv_init") orelse return error.MissingAuvInit,
        .deinit = lib.lookup(*const Auv.DeinitFunc, "auv_deinit") orelse return error.MissingAuvDeinit,
        .yieldUntilNextFrame = lib.lookup(*const Auv.YieldUntilNextFrameFunc, "auv_yield_until_next_frame") orelse return error.MissingAuvYield,
        .setThrustorValues = lib.lookup(*const Auv.SetThrustorValuesFunc, "auv_set_thrustor_values") orelse return error.MissingAuvCommands,
        .simulationStatus = lib.lookup(*const Auv.SimulationStatusFunc, "auv_simulation_status"),
    };
    s.callbacks = Abi.AuvCallbacks.from(s.auv);
    s.rebase = lib.lookup(*const fn () callconv(.c) void, "auv_simulator_rebase_pacing");
    if (lib.lookup(*const anyopaque, "auv_simulator_init_scene") != null) {
        s.api = try Scene.Api.load(plugin_path);
        const api = &s.api.?;
        try api.pinBackend(backend_path);
        if (!fuzz) {
            const model = try s.allocator.create(Scene.c.Simulation);
            model.* = std.mem.zeroes(Scene.c.Simulation);
            s.model = model;
            var error_buf: [512]u8 = @splat(0);
            if (build_scene(instance, model, &error_buf, error_buf.len) != 0) {
                std.log.err("scene: {s}", .{std.mem.sliceTo(&error_buf, 0)});
                return error.InvalidScene;
            }
            const options: Scene.c.AuvSimulatorOptions = .{
                .struct_size = @sizeOf(Scene.c.AuvSimulatorOptions),
                .version = Scene.c.AUV_SIMULATOR_EXTENSION_VERSION,
                .backend_path = backend_path,
                .max_ticks = launch.max_ticks,
                .frame_ticks = launch.frame_ticks,
                .realtime = launch.realtime,
                .print_summary = 1,
            };
            try Scene.check(api.init_scene(model, &options, &error_buf, error_buf.len));
            s.active = true;
            if (lib.lookup(*const fn (u32, u16) callconv(.c) void, "auv_simulator_configure_blender")) |configure| configure(launch.blender_enabled, launch.blender_port);
        }
    } else {
        if (fuzz) return error.FuzzRequiresMuJoCo;
        s.auv.init();
        s.active = true;
    }
    if (!fuzz and launch.telemetry != 0) {
        s.destination = try Io.net.IpAddress.parseLiteral(std.mem.span(launch.telemetry_address));
        s.socket = try (Io.net.IpAddress{ .ip4 = .unspecified(0) }).bind(s.io, .{ .mode = .dgram });
    }
    if (!fuzz and launch.teleop_enabled != 0) {
        s.teleop_socket = try (Io.net.IpAddress{ .ip4 = Io.net.Ip4Address.loopback(launch.teleop_port) }).bind(s.io, .{ .mode = .dgram, .protocol = .udp });
        if (@import("builtin").os.tag == .windows) try setAfdNonblocking(s.teleop_socket.?.handle);
    }
}
pub fn deinit(s: *Session) void {
    if (s.active) {
        var zero: [8]f32 = @splat(0);
        const count: u8 = if (s.model) |m| @intCast(m.robot.thruster_count) else s.command_count;
        if (count > 0) s.auv.setThrustorValues(&zero, count);
        s.auv.deinit();
        s.active = false;
        s.command_count = 0;
    }
    if (s.teleop_socket) |socket| socket.close(s.io);
    if (s.socket) |socket| socket.close(s.io);
    s.teleop_socket = null;
    s.socket = null;
    if (s.model) |m| {
        if (s.api) |*api| api.destroy(m);
        s.allocator.destroy(m);
        s.model = null;
    }
    if (s.api) |*api| api.deinit();
    s.api = null;
    if (s.library) |*lib| lib.close();
    s.library = null;
}
pub fn telemetry(s: *Session, bytes: []const u8) void {
    if (s.socket) |socket| if (s.destination) |*dest| socket.send(s.io, dest, bytes) catch {};
}
pub fn teleop(s: *Session, out: [*]f32) bool {
    const socket = if (s.teleop_socket) |*socket| socket else return false;
    var buffer: [256]u8 = undefined;
    const data = (pollTeleop(socket, s.io, &buffer) catch return false) orelse return false;
    if (data.len != 28) return false;
    for (0..7) |i| out[i] = @bitCast(std.mem.readInt(u32, data[i * 4 ..][0..4], .little));
    return true;
}
fn pollTeleop(sock: *const net.Socket, io: std.Io, buffer: []u8) !?[]u8 {
    if (@import("builtin").os.tag == .windows) return pollDatagramWindows(sock.handle, buffer);
    const timeout: std.Io.Timeout = .{ .duration = .{
        .clock = .awake,
        .raw = .fromNanoseconds(0),
    } };
    if (sock.receiveTimeout(io, buffer, timeout)) |msg| {
        return msg.data;
    } else |err| switch (err) {
        error.Timeout => return null,
        else => return err,
    }
}

fn setAfdNonblocking(handle: net.Socket.Handle) !void {
    const windows = std.os.windows;
    var info: windows.AFD.INFORMATION = .{
        .InformationType = .NONBLOCKING_MODE,
        .Information = .{ .Boolean = .TRUE },
    };
    var iosb: windows.IO_STATUS_BLOCK = undefined;
    switch (windows.ntdll.NtDeviceIoControlFile(
        handle,
        null,
        null,
        null,
        &iosb,
        windows.IOCTL.AFD.SET_INFORMATION,
        &info,
        @sizeOf(@TypeOf(info)),
        null,
        0,
    )) {
        .SUCCESS => {},
        else => |status| return windows.unexpectedStatus(status),
    }
}

fn pollDatagramWindows(handle: net.Socket.Handle, buffer: []u8) !?[]u8 {
    const windows = std.os.windows;
    var addr_storage: [128]u8 align(8) = undefined;
    var addr_len: windows.ULONG = addr_storage.len;
    var iov: [1]windows.AFD.WSABUF(.@"var") = .{.{
        .len = @intCast(buffer.len),
        .buf = buffer.ptr,
    }};
    var request = windows.AFD.RECV_DATAGRAM_INFO{
        .BufferArray = &iov,
        .BufferCount = 1,
        .AfdFlags = .{},
        .TdiFlags = .{ .NORMAL = true },
        .Address = &addr_storage,
        .AddressLength = &addr_len,
    };
    var iosb: windows.IO_STATUS_BLOCK = undefined;
    switch (windows.ntdll.NtDeviceIoControlFile(
        handle,
        null,
        null,
        null,
        &iosb,
        windows.IOCTL.AFD.RECEIVE_DATAGRAM,
        &request,
        @sizeOf(@TypeOf(request)),
        null,
        0,
    )) {
        .SUCCESS => return buffer[0..iosb.Information],
        // No datagram is queued. Overlapped AFD flags must stay off, or this
        // returns STATUS_PENDING instead of completing inline.
        .DEVICE_NOT_READY, .CANT_WAIT => return null,
        else => |status| return windows.unexpectedStatus(status),
    }
}
