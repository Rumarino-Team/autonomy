const std = @import("std");
const MissionContext = @import("MissionContext.zig");
const MissionArgs = @import("MissionArgs.zig");

pub fn main(init: std.process.Init) !void {
    const args_slice = try init.minimal.args.toSlice(init.arena.allocator());
    const args: MissionArgs = try .init(args_slice);

    std.log.info("args = {f}", .{args});

    var ctx: MissionContext = try .init(init.arena.allocator(), init.io, args);

    while (true) {
        ctx.yieldUntilNextFrameAndUpdate();
        if (ctx.frame.@"error" != .none) continue;

        switch (args.mission_id) {
            .prequalify => @import("missions/prequalify.zig").mission(&ctx),
        }
        if (ctx.frame.@"error" != .none) continue;
        break;
    }
}
