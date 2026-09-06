const std = @import("std");
const Io = std.Io;
const Auv = @import("Auv.zig");

const AuvLoader = @This();

auv_path: []const u8,
auv_tmp_path: []const u8,
prev_dynlib: ?std.DynLib,
use_tmp: bool,
io: Io,

pub fn init(io: Io, auv_path: []const u8) !AuvLoader {
    var tmp_path_buf: [std.Io.Dir.max_path_bytes]u8 = undefined;
    const auv_tmp_path = try std.fmt.bufPrintZ(&tmp_path_buf, "{s}.tmp", .{auv_path});
    return .{
        .auv_path = auv_path,
        .auv_tmp_path = auv_tmp_path,
        .prev_dynlib = null,
        .use_tmp = false,
        .io = io,
    };
}

pub fn load(loader: *AuvLoader) !Auv {
    const path = if (loader.use_tmp) loader.auv_tmp_path else loader.auv_path;

    // If we're loading the .tmp slot, recreate its hard link
    // from the current build output.
    if (loader.use_tmp) {
        const cwd = Io.Dir.cwd();
        cwd.deleteFile(loader.io, loader.auv_tmp_path) catch |err| switch (err) {
            error.FileNotFound => {},
            else => return err,
        };

        try cwd.hardLink(loader.auv_path, cwd, loader.auv_tmp_path, loader.io, .{});
    }

    var dynlib = try std.DynLib.open(path);
    errdefer dynlib.close();

    const auv: Auv = .{
        .loop = dynlib.lookup(
            *const Auv.LoopFunc,
            "auv_loop",
        ) orelse return error.MissingAuvLoop,

        .yieldUntilNextFrame = dynlib.lookup(
            *const Auv.YieldUntilNextFrameFunc,
            "auv_yield_until_next_frame",
        ) orelse return error.MissingAuvYieldNextFrame,

        .setThrustorValues = dynlib.lookup(
            *const Auv.SetThrustorValuesFunc,
            "auv_set_thrustor_values",
        ) orelse return error.MissingAuvSetThrustorsInput,
    };

    if (loader.prev_dynlib) |*prev_dynlib| {
        prev_dynlib.close();
    }

    loader.prev_dynlib = dynlib;
    loader.use_tmp = !loader.use_tmp;

    return auv;
}
