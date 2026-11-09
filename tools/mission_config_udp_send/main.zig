//! Send one `MCF1` live mission config UDP packet (full tunable snapshot).
const std = @import("std");
const Io = std.Io;
const MissionIo = @import("MissionIo");
const MissionConfig = MissionIo.Config;
const SerialBinary = MissionIo.SerialBinary;

pub fn main(init: std.process.Init) !void {
    const io = init.io;

    const args_slice = try init.minimal.args.toSlice(init.arena.allocator());
    var arg_i: usize = 0;
    const nextArg = blk: {
        break :blk struct {
            fn f(i: *usize, s: []const []const u8) ?[]const u8 {
                if (i.* >= s.len) return null;
                const v = s[i.*];
                i.* += 1;
                return v;
            }
        }.f;
    };

    const host = nextArg(&arg_i, args_slice) orelse "127.0.0.1";
    const port: u16 = if (nextArg(&arg_i, args_slice)) |p| try std.fmt.parseInt(u16, p, 10) else 8080;
    const profile = nextArg(&arg_i, args_slice) orelse "stonefish_sim";

    const base: *const MissionConfig.MissionConfig = blk: {
        const id = MissionConfig.MissionProfileId.fromName(profile) orelse {
            std.log.err("unknown profile `{s}`", .{profile});
            return error.InvalidProfile;
        };
        break :blk switch (id) {
            .stonefish_sim => &MissionConfig.mission_profiles.stonefish_sim,
            .proteus_hwd => &MissionConfig.mission_profiles.proteus_hwd,
        };
    };

    var packet = MissionIo.ReadPacket.fromConfig(base);

    while (nextArg(&arg_i, args_slice)) |arg| {
        if (std.mem.eql(u8, arg, "--kp0")) {
            const v = nextArg(&arg_i, args_slice) orelse return error.MissingArg;
            packet.odometry.kp[0] = try std.fmt.parseFloat(f32, v);
        } else {
            std.log.err("unknown option `{s}`", .{arg});
            return error.InvalidArg;
        }
    }

    var addr_buf: [64]u8 = undefined;
    const dest_text = try std.fmt.bufPrint(&addr_buf, "{s}:{d}", .{ host, port });
    const dest = try Io.net.IpAddress.parseLiteral(dest_text);

    var packet_buf: [4096]u8 = undefined;
    var w: Io.Writer = .fixed(&packet_buf);
    try SerialBinary.write(&w, packet);
    const encoded = w.buffered();

    const bind: Io.net.IpAddress = .{ .ip4 = .unspecified(0) };
    const sock = try bind.bind(io, .{ .mode = .dgram });
    defer sock.close(io);
    try sock.send(io, &dest, encoded);
    std.log.info("sent {d} bytes to {s} (profile {s})", .{ encoded.len, dest_text, profile });
}
