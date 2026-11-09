const std = @import("std");
const MissionContext = @import("MissionContext.zig");
const MissionArgs = @import("MissionArgs.zig");

test {
    std.testing.refAllDecls(@import("BodyControl.zig"));
    std.testing.refAllDecls(@import("MissionContext.zig"));
}

pub fn main(init: std.process.Init) !void {
    const args_slice = try init.minimal.args.toSlice(init.arena.allocator());
    const args: MissionArgs = try .init(args_slice);

    std.log.info("args = {f}", .{args});

    var ctx: MissionContext = undefined;
    try ctx.init(init.arena.allocator(), init.io, args);
    defer ctx.auv.deinit();
    defer ctx.telemetry.deinit(ctx.io);

    while (true) {
        try ctx.yieldUntilNextFrameAndUpdate();
        // Make sure the target position starts with the initial odometry measurement.
        ctx.target_position = ctx.frame.camera_pose.pos;

        const mission_result = switch (args.mission_id) {
            .prequalify_coco => @import("missions.zig").cocoMission(&ctx),
            .prequalify_robosub => @import("missions.zig").robosubMission(&ctx),
            .teleop => @import("missions.zig").teleopMission(&ctx),
            .mission_control => @import("missions.zig").missionControl(&ctx),
        };

        mission_result catch |err| switch (err) {
            error.Stopped => {
                std.log.info("stopped", .{});
                break;
            },
            else => return err,
        };

        std.log.info("succesfully completed mission", .{});
        break;
    }
}
