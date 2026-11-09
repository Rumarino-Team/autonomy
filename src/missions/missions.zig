const std = @import("std");
const math = @import("../math.zig");
const MissionContext = @import("../MissionContext.zig");
const MissionArgs = @import("../MissionArgs.zig");
const Auv = @import("../Auv.zig");
const c = Auv.c;

pub fn teleopMission(ctx: *MissionContext) !void {
    var translation: math.Vector3f = .{ 0, 0, 0 };
    var orientation: math.Quaternionf = .{ 0, 0, 0, 1 };

    while (true) {
        if (ctx.telemetry.takeTeleopCommand()) |command| {
            const next_translation: math.Vector3f = command.translation;
            const next_orientation: math.Quaternionf = command.orientation;
            const translation_finite = std.math.isFinite(next_translation[0]) and
                std.math.isFinite(next_translation[1]) and
                std.math.isFinite(next_translation[2]);
            const n2 = @reduce(.Add, next_orientation * next_orientation);
            if (translation_finite and std.math.isFinite(n2) and n2 > 1e-12) {
                translation = next_translation;
                orientation = math.normalize4f(next_orientation);
            }
        }

        ctx.use_position = false;
        ctx.use_orientation = true;
        ctx.target_orientation = orientation;
        if (math.length3f(translation) <= 1e-4) {
            ctx.translation = null;
        } else {
            ctx.translation = translation;
        }
        try ctx.yieldUntilNextFrameAndUpdate();
    }
}

pub fn cocoMission(ctx: *MissionContext) !void {
    ctx.step = "find_person";
    const person = try ctx.yieldFind(@intCast(c.AUV_COCO_PERSON));
    ctx.step = "pass_person";
    try ctx.yieldPass(person);

    ctx.step = "find_bottle";
    const bottle = try ctx.yieldFind(@intCast(c.AUV_COCO_BOTTLE));
    ctx.step = "circle_bottle";
    try ctx.yieldCircle(bottle);
}

pub fn robosubMission(ctx: *MissionContext) !void {
    ctx.step = "find_gate";
    const opening = try ctx.yieldFind(@intCast(c.AUV_ROBOSUB_GATE_OPENING));
    ctx.step = "pass_gate";
    try ctx.yieldPass(opening);

    ctx.step = "find_marker";
    const marker = try ctx.yieldFind(@intCast(c.AUV_ROBOSUB_MARKER));
    ctx.step = "circle_marker";
    try ctx.yieldCircle(marker);
}
