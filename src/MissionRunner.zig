const std = @import("std");
const math = @import("math.zig");
const MissionContext = @import("MissionContext.zig");
const MissionArgs = @import("MissionArgs.zig");

pub fn execute(ctx: *MissionContext, args: MissionArgs) !void {
    while (true) {
        ctx.yieldUntilNextFrameAndUpdate() catch |err| switch (err) {
            error.Stopped => break,
            else => return err,
        };
        // Make sure the target position starts with the initial odometry measurement.
        ctx.target_position = math.vec3(ctx.frame.camera_pose.pos);

        const mission_result = switch (args.mission_id) {
            .prequalify_coco => @import("missions/prequalify.zig").cocoMission(ctx),
            .prequalify_robosub => @import("missions/prequalify.zig").robosubMission(ctx),
            .teleop => @import("missions/prequalify.zig").teleopMission(ctx),
        };

        mission_result catch |err| switch (err) {
            error.RestartMission => {
                std.log.info("restaring mission context", .{});
                ctx.reset();
                std.log.info("restaring mission", .{});
                continue;
            },
            error.Stopped => {
                std.log.info("stopped", .{});
                break;
            },
            else => return err,
        };

        if (!ctx.batch) std.log.info("succesfully completed mission", .{});
        break;
    }
}
