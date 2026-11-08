//! Reloadable mission definitions. Operations advance one frame at a time in
//! MissionRunner; all progress lives in its versioned State and MissionContext.
const Args = @import("../MissionArgs.zig");
const c = @import("../Auv.zig").c;
pub const Definition = struct {
    first_class: u8,
    second_class: u8,
    find_first: []const u8,
    pass_first: []const u8,
    find_second: []const u8,
    circle_second: []const u8,
};
pub fn definition(mission: Args.MissionId) Definition {
    return switch (mission) {
        .prequalify_robosub => .{
            .first_class = c.AUV_ROBOSUB_GATE_OPENING,
            .second_class = c.AUV_ROBOSUB_MARKER,
            .find_first = "find_gate",
            .pass_first = "pass_gate",
            .find_second = "find_marker",
            .circle_second = "circle_marker",
        },
        .prequalify_coco => .{
            .first_class = c.AUV_COCO_PERSON,
            .second_class = c.AUV_COCO_BOTTLE,
            .find_first = "find_person",
            .pass_first = "pass_person",
            .find_second = "find_bottle",
            .circle_second = "circle_bottle",
        },
        .teleop => .{ .first_class = 0, .second_class = 0, .find_first = "teleop", .pass_first = "teleop", .find_second = "teleop", .circle_second = "teleop" },
    };
}
