const std = @import("std");
const MissionContext = @import("MissionContext.zig");
const MissionArgs = @import("MissionArgs.zig");
const Auv = @import("Auv.zig");

fn firstCubeNextRectThenBackMission(ctx: *MissionContext) void {
    const original_camera_pose = ctx.frame.camera_pose;

    const cube_object = ctx.yieldUntilFirstObjectWithCls(.cube);
    ctx.yieldUntilReachGoal(cube_object.pose.pos);

    const rect_object = ctx.yieldUntilNextObjectWithCls(.rect);
    ctx.yieldUntilReachGoal(rect_object.pose.pos);

    ctx.yieldUntilReachGoal(original_camera_pose.pos);
}

pub fn main(init: std.process.Init) !void {
    const args: MissionArgs = try .init(init);

    std.log.info("args = {f}", .{args});

    var ctx: MissionContext = try .init(init.gpa, init.io, args);

    ctx.yieldUntilNextFrameAndUpdate();

    switch (args.mission_id) {
        .first_cube_next_rect_then_back => firstCubeNextRectThenBackMission(&ctx),
    }
}
