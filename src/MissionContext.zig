//! Per-run state, cooperative maneuver helpers, and mission scripts.
const std = @import("std");
const Auv = @import("Auv.zig");
const AuvRobot = @import("AuvRobot.zig");
const MissionConfig = @import("MissionConfig.zig");
const MissionIO = @import("MissionIO.zig");
const math = @import("math.zig");
const bc = @import("BodyControl.zig");
const Wire = @import("AuvRobot.zig").Wire;
const reflection = @import("ReflectionWriter.zig");
const runtime = @import("runtime.zig").App;
const Goal = MissionIO.Goal;
const c = Auv.c;

const object_clearance: f64 = 2;
const look_done: f64 = 0.1;
const circle_corners: [4]math.Vector2f = .{
    .{ 0.5, 0.5 },
    .{ 0.5, -0.5 },
    .{ -0.5, -0.5 },
    .{ -0.5, 0.5 },
};

var goal_queue_buf: [MissionConfig.queue_capacity_max]Goal = undefined;
var queue: std.Deque(Goal) = undefined;
var step: []const u8 = "";

pub fn init() void {
    queue = .initBuffer(&goal_queue_buf);
    step = "";
}

pub fn yieldFrame() !void {
    try runtime.runtimeLoop(&queue, step);
}

pub fn prime() !void {
    try yieldFrame();
    runtime.robot.state.target_position = runtime.frame.camera_pose.pos;
}

fn setLookAtTarget(target: math.Vector3f) !void {
    const offset = target - runtime.frame.camera_pose.pos;
    if (math.length3f(offset) <= 1e-6) return;
    const toward = math.normalize3f(offset);
    runtime.robot.state.target_orientation = try bc.lookWithinLimits(
        runtime.config.camera_forward,
        toward,
        bc.identity,
        runtime.config.rotation_limit,
    );
}

pub fn yieldFind(cls: u8) !*const Auv.Object {
    runtime.robot.state.use_position = true;
    runtime.robot.state.use_orientation = true;
    runtime.robot.state.teleop_translation = null;
    var seen: usize = 0;

    while (true) {
        for (runtime.robot.state.seen_objects[seen..runtime.robot.state.seen_objects_len]) |*object| {
            if (object.cls == cls) return object;
            seen += 1;
        }
        try yieldFrame();
    }
}

fn reached(target: math.Vector3f) bool {
    const ok = runtime.frame.position_ok;
    if (!ok.x or !ok.y or !ok.z) return false;
    return math.length3f(target - runtime.frame.camera_pose.pos) <= runtime.config.goal_dist_threshold;
}

fn approachDirection(object: *const Auv.Object) math.Vector2f {
    const delta = math.xy(object.bbox.pose.pos) - math.xy(runtime.frame.camera_pose.pos);
    if (math.length2f(delta) <= 1e-4) return .{ 0, 1 };
    return math.normalize2f(delta);
}

fn horizontalHalfExtent(object: *const Auv.Object, direction: math.Vector2f) f64 {
    const rot = math.normalize4f(object.bbox.pose.quat);
    const world_dir: math.Vector3f = .{ direction[0], direction[1], 0 };
    const size = object.bbox.size;
    const axes = [3]math.Vector3f{
        math.quaternionRotate(rot, .{ 1, 0, 0 }),
        math.quaternionRotate(rot, .{ 0, 1, 0 }),
        math.quaternionRotate(rot, .{ 0, 0, 1 }),
    };
    var extent: f64 = 0;
    inline for (axes, 0..) |axis, i| extent += @abs(math.dot3f(world_dir, axis)) * size[i];
    return 0.5 * extent;
}

fn standoffAlong(object: *const Auv.Object, direction: math.Vector2f, sign: f64) math.Vector3f {
    const reach = horizontalHalfExtent(object, direction) + object_clearance;
    const offset = direction * @as(math.Vector2f, @splat(sign * reach));
    const center = object.bbox.pose.pos;
    return .{ center[0] + offset[0], center[1] + offset[1], center[2] };
}

fn clearQueue() void {
    queue.head = 0;
    queue.len = 0;
}

pub fn yieldLookAt(object: *const Auv.Object) !void {
    runtime.robot.state.target_position = runtime.frame.camera_pose.pos;
    runtime.robot.state.use_position = true;
    runtime.robot.state.use_orientation = true;
    runtime.robot.state.teleop_translation = null;
    while (true) {
        try setLookAtTarget(object.bbox.pose.pos);
        try yieldFrame();
        const err = AuvRobot.attitudeToward(&runtime.frame, runtime.robot.state.target_orientation) orelse continue;
        if (math.length3f(err) <= look_done) return;
    }
}

pub fn yieldMoveTo(object: *const Auv.Object) !void {
    clearQueue();
    const direction = approachDirection(object);
    const far = standoffAlong(object, direction, 1);
    try queue.pushBackBounded(.{
        .position = standoffAlong(object, direction, -1),
        .look_at = far,
    });
    try yieldQueue();
}

pub fn yieldPoint(position: math.Vector3f, look_direction: math.Vector3f) !void {
    clearQueue();
    try queue.pushBackBounded(.{
        .position = position,
        .look_at = position + look_direction,
    });
    try yieldQueue();
}

pub fn yieldPass(object: *const Auv.Object) !void {
    clearQueue();
    const direction = approachDirection(object);
    const far = standoffAlong(object, direction, 1);
    try queue.pushBackBounded(.{
        .position = standoffAlong(object, direction, -1),
        .look_at = far,
    });
    try queue.pushBackBounded(.{
        .position = far,
        .look_at = far,
    });
    try yieldQueue();
}

pub fn yieldQueue() !void {
    runtime.robot.state.use_position = true;
    runtime.robot.state.use_orientation = true;
    runtime.robot.state.teleop_translation = null;
    while (queue.len != 0) {
        const goal = queue.front().?;
        runtime.robot.state.target_position = goal.position;
        try setLookAtTarget(goal.look_at);
        try yieldFrame();
        if (reached(goal.position)) {
            _ = queue.popFront();
        }
    }
}

fn circleCorner(object: *const Auv.Object, index: usize, depth: f64) math.Vector3f {
    const local_2d = circle_corners[index] * math.xy(object.bbox.size);
    const rotated = math.xy(math.quaternionRotate(math.normalize4f(object.bbox.pose.quat), .{ local_2d[0], local_2d[1], 0 }));
    const outward = if (math.length2f(rotated) <= 1e-4) math.Vector2f{ 1, 0 } else math.normalize2f(rotated);
    const pos = math.xy(object.bbox.pose.pos) + rotated + outward * @as(math.Vector2f, @splat(object_clearance));
    return .{ pos[0], pos[1], depth };
}

pub fn yieldCircle(object: *const Auv.Object) !void {
    clearQueue();
    const start = runtime.frame.camera_pose.pos;
    const look_at = object.bbox.pose.pos;
    var corners: [circle_corners.len]math.Vector3f = undefined;
    for (0..circle_corners.len) |i| corners[i] = circleCorner(object, i, start[2]);

    var first: usize = 0;
    var best = std.math.inf(f64);
    const sub = math.xy(start);
    for (corners, 0..) |corner, i| {
        const dist = math.length2f(math.xy(corner) - sub);
        if (dist < best) {
            best = dist;
            first = i;
        }
    }

    for (0..corners.len) |corner_i| {
        const index = (first + corner_i) % corners.len;
        try queue.pushBackBounded(.{
            .position = corners[index],
            .look_at = look_at,
        });
    }
    try queue.pushBackBounded(.{ .position = start, .look_at = look_at });
    try yieldQueue();
}

fn takeTeleop() ?Wire.TeleopWire {
    const bytes = runtime.telemetry.takeTeleopPacket() orelse return null;
    const doc = reflection.read(Wire.TeleopDoc, bytes) catch return null;
    return doc.command;
}

fn teleopMission() !void {
    const state = &runtime.robot.state;
    var translation: math.Vector3f = .{ 0, 0, 0 };
    var orientation: math.Quaternionf = .{ 0, 0, 0, 1 };

    while (true) {
        if (takeTeleop()) |command| {
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
        try yieldFrame();
    }
}

fn thrusterControl() !void {
    step = "direct_thruster_control";
    while (true) try yieldFrame();
}

fn missionControl() !void {
    while (true) {
        step = "waiting_for_mission_command";
        try yieldFrame();
        const command = runtime.telemetry.takeMissionCommand() orelse continue;

        switch (command.kind) {
            .point => {
                step = "move_to_point";
                try yieldPoint(command.position, command.look_direction);
            },
            .object => {
                step = "find_object";
                const object = try yieldFind(command.object_class);
                switch (command.maneuver) {
                    .move => {
                        step = "move_to_object";
                        try yieldMoveTo(object);
                    },
                    .pass => {
                        step = "pass_object";
                        try yieldPass(object);
                    },
                    .circle => {
                        step = "circle_object";
                        try yieldCircle(object);
                    },
                }
            },
        }
    }
}

fn cocoMission() !void {
    step = "find_person";
    const person = try yieldFind(@intCast(c.AUV_COCO_PERSON));
    step = "pass_person";
    try yieldPass(person);

    step = "find_bottle";
    const bottle = try yieldFind(@intCast(c.AUV_COCO_BOTTLE));
    step = "circle_bottle";
    try yieldCircle(bottle);
}

fn robosubMission() !void {
    step = "find_gate";
    const opening = try yieldFind(@intCast(c.AUV_ROBOSUB_GATE_OPENING));
    step = "pass_gate";
    try yieldPass(opening);

    step = "find_marker";
    const marker = try yieldFind(@intCast(c.AUV_ROBOSUB_MARKER));
    step = "circle_marker";
    try yieldCircle(marker);
}

pub fn run(id: MissionConfig.MissionId) !void {
    switch (id) {
        .prequalify_coco => try cocoMission(),
        .prequalify_robosub => try robosubMission(),
        .teleop => try teleopMission(),
        .mission_control => try missionControl(),
        .thruster_control => try thrusterControl(),
    }
}
