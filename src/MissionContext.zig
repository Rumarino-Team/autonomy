//! Per-run state and cooperative maneuver helpers for the AUV missions.
const std = @import("std");
const Auv = @import("Auv.zig");
const AuvRobot = @import("AuvRobot.zig");
const MissionConfig = @import("MissionConfig.zig");
const MissionIO = @import("MissionIO.zig");
const AuvApplication = @import("application/Application.zig").AuvApplication;
const math = @import("math.zig");
const bc = @import("BodyControl.zig");
const MissionContext = @This();
const Goal = MissionIO.Goal;

app: *AuvApplication,
goal_queue_buf: [MissionConfig.queue_capacity_max]Goal = undefined,
queue: std.Deque(Goal) = undefined,
step: []const u8 = "",

pub fn init(self: *MissionContext, app: *AuvApplication) void {
    self.* = .{ .app = app };
    self.queue = .initBuffer(&self.goal_queue_buf);
}

pub fn yieldFrame(self: *MissionContext) !void {
    try self.app.runtimeLoop(&self.queue, self.step);
}

pub fn prime(self: *MissionContext) !void {
    try self.yieldFrame();
    self.app.robot.state.target_position = self.app.frame.camera_pose.pos;
}

fn setLookAtTarget(self: *MissionContext, target: math.Vector3f) !void {
    const platform = self.app;
    const offset = target - platform.frame.camera_pose.pos;
    if (math.length3f(offset) <= 1e-6) return;
    const toward = math.normalize3f(offset);
    platform.robot.state.target_orientation = try bc.lookWithinLimits(
        platform.config.camera_forward,
        toward,
        bc.identity,
        platform.config.rotation_limit,
    );
}

pub fn yieldFind(ctx: *MissionContext, cls: u8) !*const Auv.Object {
    ctx.app.robot.state.use_position = true;
    ctx.app.robot.state.use_orientation = true;
    ctx.app.robot.state.teleop_translation = null;
    var seen: usize = 0;

    while (true) {
        for (ctx.app.robot.state.seen_objects[seen..ctx.app.robot.state.seen_objects_len]) |*object| {
            if (object.cls == cls) return object;
            seen += 1;
        }
        try ctx.yieldFrame();
    }
}

const object_clearance: f64 = 2;
const look_done: f64 = 0.1;

fn reached(ctx: *const MissionContext, target: math.Vector3f) bool {
    const platform = ctx.app;
    const ok = platform.frame.position_ok;
    if (!ok.x or !ok.y or !ok.z) return false;
    return math.length3f(target - platform.frame.camera_pose.pos) <= platform.config.goal_dist_threshold;
}

fn approachDirection(ctx: *const MissionContext, object: *const Auv.Object) math.Vector2f {
    const delta = math.xy(object.bbox.pose.pos) - math.xy(ctx.app.frame.camera_pose.pos);
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

fn clearQueue(ctx: *MissionContext) void {
    ctx.queue.head = 0;
    ctx.queue.len = 0;
}

pub fn yieldLookAt(ctx: *MissionContext, object: *const Auv.Object) !void {
    ctx.app.robot.state.target_position = ctx.app.frame.camera_pose.pos;
    ctx.app.robot.state.use_position = true;
    ctx.app.robot.state.use_orientation = true;
    ctx.app.robot.state.teleop_translation = null;
    while (true) {
        try ctx.setLookAtTarget(object.bbox.pose.pos);
        try ctx.yieldFrame();
        const err = AuvRobot.attitudeToward(&ctx.app.frame, ctx.app.robot.state.target_orientation) orelse continue;
        if (math.length3f(err) <= look_done) return;
    }
}

pub fn yieldMoveTo(ctx: *MissionContext, object: *const Auv.Object) !void {
    ctx.clearQueue();
    const direction = approachDirection(ctx, object);
    const far = standoffAlong(object, direction, 1);
    try ctx.queue.pushBackBounded(.{
        .position = standoffAlong(object, direction, -1),
        .look_at = far,
    });
    try ctx.yieldQueue();
}

pub fn yieldPoint(ctx: *MissionContext, position: math.Vector3f, look_direction: math.Vector3f) !void {
    ctx.clearQueue();
    try ctx.queue.pushBackBounded(.{
        .position = position,
        .look_at = position + look_direction,
    });
    try ctx.yieldQueue();
}

pub fn yieldPass(ctx: *MissionContext, object: *const Auv.Object) !void {
    ctx.clearQueue();
    const direction = approachDirection(ctx, object);
    const far = standoffAlong(object, direction, 1);
    try ctx.queue.pushBackBounded(.{
        .position = standoffAlong(object, direction, -1),
        .look_at = far,
    });
    try ctx.queue.pushBackBounded(.{
        .position = far,
        .look_at = far,
    });
    try ctx.yieldQueue();
}

pub fn yieldQueue(ctx: *MissionContext) !void {
    ctx.app.robot.state.use_position = true;
    ctx.app.robot.state.use_orientation = true;
    ctx.app.robot.state.teleop_translation = null;
    while (ctx.queue.len != 0) {
        const goal = ctx.queue.front().?;
        ctx.app.robot.state.target_position = goal.position;
        try ctx.setLookAtTarget(goal.look_at);
        try ctx.yieldFrame();
        if (reached(ctx, goal.position)) {
            _ = ctx.queue.popFront();
        }
    }
}

const circle_corners: [4]math.Vector2f = .{
    .{ 0.5, 0.5 },
    .{ 0.5, -0.5 },
    .{ -0.5, -0.5 },
    .{ -0.5, 0.5 },
};

fn circleCorner(object: *const Auv.Object, index: usize, depth: f64) math.Vector3f {
    const local_2d = circle_corners[index] * math.xy(object.bbox.size);
    const rotated = math.xy(math.quaternionRotate(math.normalize4f(object.bbox.pose.quat), .{ local_2d[0], local_2d[1], 0 }));
    const outward = if (math.length2f(rotated) <= 1e-4) math.Vector2f{ 1, 0 } else math.normalize2f(rotated);
    const pos = math.xy(object.bbox.pose.pos) + rotated + outward * @as(math.Vector2f, @splat(object_clearance));
    return .{ pos[0], pos[1], depth };
}

pub fn yieldCircle(ctx: *MissionContext, object: *const Auv.Object) !void {
    ctx.clearQueue();
    const start = ctx.app.frame.camera_pose.pos;
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

    for (0..corners.len) |step| {
        const index = (first + step) % corners.len;
        try ctx.queue.pushBackBounded(.{
            .position = corners[index],
            .look_at = look_at,
        });
    }
    try ctx.queue.pushBackBounded(.{ .position = start, .look_at = look_at });
    try ctx.yieldQueue();
}
