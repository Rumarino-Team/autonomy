const std = @import("std");
const math = @import("math.zig");
const Io = std.Io;
const Auv = @import("Auv.zig");
const Navigation = @import("Navigation.zig");
const Allocation = @import("ThrusterAllocator.zig");
const Control = @import("ControlOptions.zig");

const MissionLoader = @This();

const max_thrustors = 8;

config_path: []const u8,
use_ping: bool,
ping: std.heap.FixedBufferAllocator,
pong: std.heap.FixedBufferAllocator,
reader_buf: [16384]u8,
backend_geometry: ?Auv.ThrusterGeometry = null,

pub const NonOdometryPid = struct {
    kp: math.Vector6f,
    ki: math.Vector6f,
    kd: math.Vector6f,
    center_deadband: f32,
    target_height: f32,
};

pub const OdometryPid = struct {
    xy_arrival_radius: f32 = 0.025,
    kp: math.Vector6f,
    ki: math.Vector6f,
    kd: math.Vector6f,
};

pub const Config = struct {
    teleop_port: u16 = 8080,
    tam: []math.Vector6f = &.{},
    /// Accepted for old configuration files; new loops use control gains.
    odometry: OdometryPid = .{ .kp = @splat(0), .ki = @splat(0), .kd = @splat(0) },
    non_odometry: NonOdometryPid,
    navigation: Navigation.Options = .{},
    control: Control.Options = .{},
    allocation: ?Allocation.Options = null,
};

pub fn init(arena: std.mem.Allocator, config_path: []const u8) !MissionLoader {
    return .{
        .config_path = config_path,
        .use_ping = false,
        .ping = .init(try arena.alloc(u8, 65536)),
        .pong = .init(try arena.alloc(u8, 65536)),
        .reader_buf = undefined,
    };
}

pub fn load(loader: *MissionLoader, io: Io) !Config {
    const fba = if (loader.use_ping) &loader.ping else &loader.pong;
    fba.reset();
    errdefer fba.reset();
    const file = try Io.Dir.cwd().openFile(io, loader.config_path, .{ .mode = .read_only });
    defer file.close(io);
    var file_reader = file.reader(io, &loader.reader_buf);
    const file_contents = try file_reader.interface.allocRemaining(fba.allocator(), .unlimited);

    const config = try std.json.parseFromSliceLeaky(Config, fba.allocator(), file_contents, .{});
    try config.navigation.validate();
    try config.control.validate();
    if (config.allocation) |options| {
        try options.validate();
        if (!options.use_backend_geometry or loader.backend_geometry != null)
            _ = try Allocation.init(options, loader.backend_geometry);
    } else {
        if (config.tam.len == 0 or config.tam.len > math.max_thrusters) return error.InvalidTam;
        for (config.tam) |row| inline for (0..6) |axis| {
            if (!std.math.isFinite(row[axis])) return error.InvalidTam;
        };
    }

    loader.use_ping = !loader.use_ping;
    fba.reset();

    return config;
}
