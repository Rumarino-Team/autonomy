const std = @import("std");
const builtin = @import("builtin");
const math = @import("../math.zig");
const MissionContext = @import("../MissionContext.zig");
const MissionArgs = @import("../MissionArgs.zig");
const Auv = @import("../Auv.zig");
const c = Auv.c;
const net = std.Io.net;

pub fn teleopMission(ctx: *MissionContext) !void {
    const port = ctx.config.teleop_port;
    const addr: net.IpAddress = .{ .ip4 = net.Ip4Address.loopback(port) };

    var sock = addr.bind(ctx.io, .{
        .mode = .dgram,
        .protocol = .udp,
    }) catch |err| switch (err) {
        error.AddressInUse => {
            std.log.err("teleop UDP port {d} is already in use", .{port});
            return error.AddressInUse;
        },
        else => return err,
    };
    defer sock.close(ctx.io);
    // Zig 0.17's Windows Io.operateTimeout cannot receive on a socket
    // (ConcurrencyUnavailable). Poll the AFD handle directly instead.
    if (builtin.os.tag == .windows) try setAfdNonblocking(sock.handle);

    var recv_buf: [256]u8 = undefined;

    var translation: math.Vector3f = .{ 0, 0, 0 };
    var orientation: math.Quaternionf = .{ 0, 0, 0, 1 };

    while (true) {
        if (try pollTeleop(&sock, ctx.io, &recv_buf)) |data| {
            const packet_len = 7 * @sizeOf(f32);
            if (data.len != packet_len) continue;
            var values: [7]f32 = undefined;
            for (0..values.len) |i| {
                const bits = std.mem.readInt(u32, data[i * 4 ..][0..4], .little);
                values[i] = @bitCast(bits);
            }
            const next_translation: math.Vector3f = .{ values[0], values[1], values[2] };
            const next_orientation: math.Quaternionf = .{ values[3], values[4], values[5], values[6] };
            if (!std.math.isFinite(next_translation[0]) or
                !std.math.isFinite(next_translation[1]) or
                !std.math.isFinite(next_translation[2])) continue;
            const n2 = @reduce(.Add, next_orientation * next_orientation);
            if (!std.math.isFinite(n2) or n2 <= 1e-12) continue;
            translation = next_translation;
            orientation = math.normalize4f(next_orientation);
        }

        ctx.use_position = false;
        ctx.use_orientation = true;
        ctx.target_orientation = orientation;
        if (math.length3f(translation) <= 1e-4) {
            ctx.translation = null;
        } else {
            ctx.translation = translation;
        }
        try ctx.yieldUntilNextFrameAndUpdate();
    }
}

fn pollTeleop(sock: *const net.Socket, io: std.Io, buffer: []u8) !?[]u8 {
    if (builtin.os.tag == .windows) return pollDatagramWindows(sock.handle, buffer);
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

pub fn cocoMission(ctx: *MissionContext) !void {
    ctx.step = "find_person";
    const person = try ctx.yieldFind(@intCast(c.AUV_COCO_PERSON));
    ctx.step = "pass_person";
    try ctx.yieldPass(person);

    ctx.step = "find_bottle";
    const bottle = try ctx.yieldFind(@intCast(c.AUV_COCO_BOTTLE));
    ctx.step = "circle_bottle";
    try ctx.yieldCircle(bottle);
}

pub fn robosubMission(ctx: *MissionContext) !void {
    ctx.step = "find_gate";
    const opening = try ctx.yieldFind(@intCast(c.AUV_ROBOSUB_GATE_OPENING));
    ctx.step = "pass_gate";
    try ctx.yieldPass(opening);

    ctx.step = "find_marker";
    const marker = try ctx.yieldFind(@intCast(c.AUV_ROBOSUB_MARKER));
    ctx.step = "circle_marker";
    try ctx.yieldCircle(marker);
}
