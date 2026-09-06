const std = @import("std");
const Io = std.Io;
const Auv = @import("Auv.zig");

const MissionLoader = @This();

const max_thrustors = 8;

config_path: []const u8,
io: Io,
use_ping: bool,
ping: std.heap.FixedBufferAllocator,
pong: std.heap.FixedBufferAllocator,
reader_buf: [4098]u8,

pub const Config = struct {
    kp: Auv.Vector6f,
    ki: Auv.Vector6f,
    kd: Auv.Vector6f,
    tam: []Auv.Vector6f,
};

pub fn init(gpa: std.mem.Allocator, io: Io, config_path: []const u8) !MissionLoader {
    return .{
        .config_path = config_path,
        .io = io,
        .use_ping = false,
        .ping = .init(try gpa.alloc(u8, 4098)),
        .pong = .init(try gpa.alloc(u8, 4098)),
        .reader_buf = undefined,
    };
}

pub fn load(loader: *MissionLoader) !Config {
    const fba = if (loader.use_ping) &loader.ping else &loader.pong; 
    const file = try Io.Dir.cwd().openFile(loader.io, loader.config_path, .{ .mode = .read_only });
    var file_reader = file.reader(loader.io, &loader.reader_buf);
    const file_contents = try file_reader.interface.allocRemaining(fba.allocator(), .unlimited);

    const config = std.json.parseFromSliceLeaky(Config, fba.allocator(), file_contents, .{});

    loader.use_ping = !loader.use_ping;
    fba.reset();

    return config;
}
