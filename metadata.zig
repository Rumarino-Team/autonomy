//! Build-time conversion of Zig-owned asset metadata to binary interchange.
const std = @import("std");
pub fn main(init: std.process.Init) !void {
    const argv = try init.minimal.args.toSlice(init.arena.allocator());
    if (argv.len != 2) return error.OutputPathRequired;
    const allocator = init.arena.allocator();
    const data = try @import("src/BinaryData.zig").pack(allocator, .metadata, 1, @import("config/robots.zig").manifest);
    defer allocator.free(data);
    // The build runner may regenerate an existing cache output in watch mode.
    const file = try std.Io.Dir.cwd().createFile(init.io, argv[1], .{ .truncate = true });
    defer file.close(init.io);
    try file.writeStreamingAll(init.io, data);
}
