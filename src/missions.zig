const std = @import("std");
const math = @import("math.zig");
const Auv = @import("Auv.zig");
const Wire = @import("AuvRobot.zig").Wire;
const MissionContext = @import("MissionContext.zig");
const reflection = @import("ReflectionWriter.zig");
const c = Auv.c;

pub fn teleopMission(ctx: *MissionContext) !void {
    const state = &ctx.app.robot.state;
    var translation: math.Vector3f = .{ 0, 0, 0 };
    var orientation: math.Quaternionf = .{ 0, 0, 0, 1 };

    while (true) {
        if (takeTeleop(ctx)) |command| {
            const next_translation = command.teleop_translation orelse math.Vector3f{ 0, 0, 0 };
            const next_orientation = command.target_orientation;
            const translation_finite = std.math.isFinite(next_translation[0]) and
                std.math.isFinite(next_translation[1]) and
                std.math.isFinite(next_translation[2]);
            const n2 = @reduce(.Add, next_orientation * next_orientation);
            if (translation_finite and std.math.isFinite(n2) and n2 > 1e-12) {
                translation = next_translation;
                orientation = math.normalize4f(next_orientation);
            }
        }

        state.use_position = false;
        state.use_orientation = true;
        state.target_orientation = orientation;
        if (math.length3f(translation) <= 1e-4) {
            state.teleop_translation = null;
        } else {
            state.teleop_translation = translation;
        }
        try ctx.yieldFrame();
    }
}

pub fn thrusterControl(ctx: *MissionContext) !void {
    ctx.step = "direct_thruster_control";
    while (true) try ctx.yieldFrame();
}

pub fn missionControl(ctx: *MissionContext) !void {
    while (true) {
        ctx.step = "waiting_for_mission_command";
        try ctx.yieldFrame();
        const command = ctx.app.telemetry.takeMissionCommand() orelse continue;

        switch (command.kind) {
            .point => {
                ctx.step = "move_to_point";
                try ctx.yieldPoint(command.position, command.look_direction);
            },
            .object => {
                ctx.step = "find_object";
                const object = try ctx.yieldFind(command.object_class);
                switch (command.maneuver) {
                    .move => {
                        ctx.step = "move_to_object";
                        try ctx.yieldMoveTo(object);
                    },
                    .pass => {
                        ctx.step = "pass_object";
                        try ctx.yieldPass(object);
                    },
                    .circle => {
                        ctx.step = "circle_object";
                        try ctx.yieldCircle(object);
                    },
                }
            },
        }
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

fn takeTeleop(ctx: *MissionContext) ?Wire.TeleopWire {
    const bytes = ctx.app.telemetry.takeTeleopPacket() orelse return null;
    const doc = reflection.read(Wire.TeleopDoc, bytes) catch return null;
    return doc.command;
}

