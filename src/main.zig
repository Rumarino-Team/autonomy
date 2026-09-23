const std = @import("std");
const MissionContext = @import("MissionContext.zig");
const MissionArgs = @import("MissionArgs.zig");
const Telemetry = @import("Telemetry.zig");

pub fn main(init: std.process.Init) !void {
    const args_slice = try init.minimal.args.toSlice(init.arena.allocator());
    const args: MissionArgs = try .init(args_slice);

    std.log.info("args = {f}", .{args});

    const telemetry: Telemetry = try .init(
        init.io,
        init.environ_map.get("AUV_TELEMETRY_LOG"),
        init.environ_map.get("AUV_TELEMETRY_ADDR"),
    );
    var ctx: MissionContext = try .init(init.arena.allocator(), init.io, args, telemetry);

    ctx.yieldUntilNextFrameAndUpdate();

    switch (args.mission_id) {
        .prequalify => @import("missions/prequalify.zig").mission(&ctx),
    }

    ctx.yieldUntilNextFrameAndUpdate();
}
