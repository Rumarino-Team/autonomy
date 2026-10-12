const MissionContext = @import("../MissionContext.zig");

pub fn teleopMission(ctx: *MissionContext) !void {
    try @import("prequalify.zig").teleopMission(ctx);
}

pub fn teleopCommandMission(ctx: *MissionContext) !void {
    try @import("../TeleopCommandServer.zig").run(ctx);
}
