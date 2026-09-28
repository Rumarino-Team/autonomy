const std = @import("std");
const math = @import("math.zig");
const Io = std.Io;
const Auv = @import("Auv.zig");

const MissionLoader = @This();

const max_thrustors = 8;

config_path: []const u8,
use_ping: bool,
ping: std.heap.FixedBufferAllocator,
pong: std.heap.FixedBufferAllocator,
reader_buf: [4098]u8,

pub const Config = struct {
    kp: math.Vector6f,
    ki: math.Vector6f,
    kd: math.Vector6f,
    tam: []math.Vector6f,
    /// World and body +Z point down (Stonefish NED). False for Z-up (ZED RIGHT_HANDED_Z_UP_X_FWD).
    z_down: bool = false,
    /// Policy file from rl/export_policy.py. When set, it replaces the global-navigation PID.
    policy: ?[]const u8 = null,
};

pub fn init(gpa: std.mem.Allocator, config_path: []const u8) !MissionLoader {
    return .{
        .config_path = config_path,
        .use_ping = false,
        .ping = .init(try gpa.alloc(u8, 4098)),
        .pong = .init(try gpa.alloc(u8, 4098)),
        .reader_buf = undefined,
    };
}

pub fn deinit(loader: *MissionLoader, gpa: std.mem.Allocator) void {
    gpa.free(loader.ping.buffer);
    gpa.free(loader.pong.buffer);
}

pub fn load(loader: *MissionLoader, io: Io) !Config {
    const fba = if (loader.use_ping) &loader.ping else &loader.pong; 
    const file = try Io.Dir.cwd().openFile(io, loader.config_path, .{ .mode = .read_only });
    var file_reader = file.reader(io, &loader.reader_buf);
    const file_contents = try file_reader.interface.allocRemaining(fba.allocator(), .unlimited);

    const config = std.json.parseFromSliceLeaky(Config, fba.allocator(), file_contents, .{});

    loader.use_ping = !loader.use_ping;
    fba.reset();

    return config;
}
