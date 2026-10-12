const std = @import("std");
const math = @import("../math.zig");
const MissionContext = @import("../MissionContext.zig");
const MissionArgs = @import("../MissionArgs.zig");
const Auv = @import("../Auv.zig");
const net = std.Io.net;

pub const CommandOP = enum(u8)  {
    MoveToObject,
    MoveToDirection,
    

};

const Direction = enum {
    north,
    south,
    east,
    west,
};

const CommandType = enum(u8) {
    quit = 0,
    move = 1,
    write = 2,
};

const Command = union(CommandOP) {
    MoveToObject: struct{ class: u8, object_id: u8},
    moveToDirection: struct { },
};

const CommandComplete = struct{
    facingDirection: Direction,
    com: Command,

};


pub fn teleopMission(ctx: *MissionContext) !void {
    const port = ctx.config.teleop_port;
    const addr: net.IpAddress = .{ .ip4 = net.Ip4Address.loopback(port) };
    const retry_interval_ms: i64 = 1_000;
    const retry_interval: std.Io.Clock.Duration = .{
        .clock = .awake,
        .raw = .fromMilliseconds(retry_interval_ms),
    };

    var sock = while (true) {
        const bound_sock = addr.bind(ctx.io, .{
            .mode = .dgram,
            .protocol = .udp,
        }) catch |err| {
            std.log.debug(
                "teleop could not bind UDP port {d}: {s}. The port may already be in use; change `teleop_port` in auv.json. Retrying in {d} ms",
                .{ port, @errorName(err), retry_interval_ms },
            );

            const retry_at = std.Io.Clock.Timestamp.now(ctx.io, .awake).addDuration(retry_interval);
            while (std.Io.Clock.Timestamp.compare(
                std.Io.Clock.Timestamp.now(ctx.io, .awake),
                .lt,
                retry_at,
            )) {
                try ctx.yieldUntilNextFrameAndUpdate();
            }
            continue;
        };
        break bound_sock;
    };
    defer sock.close(ctx.io);

    var recv_buf: [256]u8 = undefined;
    const timeout: std.Io.Timeout = .{ .duration = .{
        .clock = .awake,
        .raw = .fromNanoseconds(0),
    } };

    while (true) {
        if (sock.receiveTimeout(ctx.io, &recv_buf, timeout)) |msg| {
            if (msg.data.len != 6 * @sizeOf(f32)) {
                return;
            }
            var values : [6]f32 = @splat(6);
            for (0..6) |i| {
                const bits = std.mem.readInt(u32, msg.data[i * 4 ..][0..4], .little);
                values[i] = @bitCast(bits);
            }
            ctx.goal = values;
        } else |err| switch (err) {
            error.Timeout => {},
            else => return err,
        }

        try ctx.yieldUntilNextFrameAndUpdate();
    }
}






pub fn teleopCommandMission(ctx: *MissionContext) !void{

    const port = ctx.config.teleop_port;
    const addr: net.IpAddress = .{ .ip4 = net.Ip4Address.loopback(port) };
    const retry_interval_ms: i64 = 1_000;
    const retry_interval: std.Io.Clock.Duration = .{
        .clock = .awake,
        .raw = .fromMilliseconds(retry_interval_ms),
    };

    var sock = while (true) {
        const bound_sock = addr.bind(ctx.io, .{
            .mode = .dgram,
            .protocol = .udp,
        }) catch |err| {
            std.log.debug(
                "teleop could not bind UDP port {d}: {s}. The port may already be in use; change `teleop_port` in auv.json. Retrying in {d} ms",
                .{ port, @errorName(err), retry_interval_ms },
            );

            const retry_at = std.Io.Clock.Timestamp.now(ctx.io, .awake).addDuration(retry_interval);
            while (std.Io.Clock.Timestamp.compare(
                std.Io.Clock.Timestamp.now(ctx.io, .awake),
                .lt,
                retry_at,
            )) {
                try ctx.yieldUntilNextFrameAndUpdate();
            }
            continue;
        };
        break bound_sock;
    };
    defer sock.close(ctx.io);

    var recv_buf: [256]u8 = undefined;
    const timeout: std.Io.Timeout = .{ .duration = .{
        .clock = .awake,
        .raw = .fromNanoseconds(0),
    } };

    while (true) {
        if (sock.receiveTimeout(ctx.io, &recv_buf, timeout)) |msg| {
            if (msg.data.len != 6 * @sizeOf(f32)) {
                return;
            }
            var values : [6]f32 = @splat(6);
            for (0..6) |i| {
                const bits = std.mem.readInt(u32, msg.data[i * 4 ..][0..4], .little);
                values[i] = @bitCast(bits);
            }
            ctx.goal = values;
        } else |err| switch (err) {
            error.Timeout => {},
            else => return err,
        }

        try ctx.yieldUntilNextFrameAndUpdate();
    }
}