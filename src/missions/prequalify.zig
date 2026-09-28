const math = @import("../math.zig");
const MissionContext = @import("../MissionContext.zig");
const Auv = @import("../Auv.zig");
const global_nav = @import("../navigation/global.zig");
const reactive_nav = @import("../navigation/reactive.zig");

pub fn mission(ctx: *MissionContext) void {
    const gate_object = firstObject(ctx, &.{.gate});
    goThrough(ctx, gate_object);

    const cube_or_rect_object = firstObject(ctx, &.{ .cube, .rect });
    goAround(ctx, cube_or_rect_object);
}

const FAR_ENOUGH: f32 = 2.0;
const OVERSHOOT: f32 = 2.0;
const REACTIVE_TARGET_HEIGHT: f32 = 0.4;

fn reachGoal(ctx: *MissionContext, goal_pos: math.Vector3f) !void {
    global_nav.setGoalPosition(&ctx.global, goal_pos);
    return ctx.loopUntil(.global, true, AtGoal{
        .goal = goal_pos,
        .threshold = ctx.goal_dist_threshold,
    });
}

fn approachDirection(ctx: *MissionContext, object: *const Auv.Object) math.Vector2f {
    const object_xy = math.xy(object.bbox.pose.pos);
    const sub_xy = math.xy(ctx.global.frame.camera_pose.pos);
    return math.normalize2f(object_xy - sub_xy);
}

/// `FAR_ENOUGH` meters before the object, at the object's depth.
fn pointInFront(ctx: *MissionContext, object: *const Auv.Object) math.Vector3f {
    const object_pos = object.bbox.pose.pos;
    const before_xy = math.xy(object_pos) - approachDirection(ctx, object) * @as(math.Vector2f, @splat(FAR_ENOUGH));
    return .{ before_xy[0], before_xy[1], object_pos[2] };
}

/// `OVERSHOOT` meters past the object, at the object's depth.
fn overshoot(ctx: *MissionContext, object: *const Auv.Object) math.Vector3f {
    const object_pos = object.bbox.pose.pos;
    const past_xy = math.xy(object_pos) + approachDirection(ctx, object) * @as(math.Vector2f, @splat(OVERSHOOT));
    return .{ past_xy[0], past_xy[1], object_pos[2] };
}

fn goThrough(ctx: *MissionContext, object: *const Auv.Object) void {
    reachGoal(ctx, pointInFront(ctx, object)) catch {
        reactiveFallback(ctx, .gate);
        return;
    };
    reachGoal(ctx, overshoot(ctx, object)) catch {
        reactiveFallback(ctx, .gate);
    };
}

fn reactiveFallback(ctx: *MissionContext, cls: Auv.ObjectCls) void {
    ctx.reactive.tracked_id = null;
    ctx.reactive.target_height = null;
    ctx.reactive.search_yaw = ctx.reactive.controller.search_yaw;
    ctx.reactive.hold_depth = null;
    defer {
        ctx.reactive.search_yaw = null;
        ctx.reactive.hold_depth = null;
    }

    ctx.loopUntil(.reactive, true, YawFind{ .cls = &.{cls} }) catch |err| switch (err) {};
    const frame_object = reactive_nav.findFrameObjectWithCls(&ctx.reactive, &.{cls}).?;
    const box = reactive_nav.findSeenById(&ctx.reactive, frame_object.id).?;

    ctx.reactive.tracked_id = box.id;
    ctx.reactive.target_height = null;
    ctx.loopUntil(.reactive, true, Centered{ .id = box.id }) catch |err| switch (err) {};

    ctx.reactive.target_height = REACTIVE_TARGET_HEIGHT;
    ctx.loopUntil(.reactive, true, HeightReached{
        .id = box.id,
        .target_height = REACTIVE_TARGET_HEIGHT,
    }) catch |err| switch (err) {};
}

fn firstObject(ctx: *MissionContext, cls: []const Auv.ObjectCls) *const Auv.Object {
    ctx.loopUntil(.global, false, HasCls{ .cls = cls, .start = 0 }) catch |err| switch (err) {};
    return global_nav.findObject(&ctx.global, cls, 0).?;
}

const HasCls = struct {
    cls: []const Auv.ObjectCls,
    start: usize,
    pub fn done(self: @This(), ctx: *MissionContext) !bool {
        return global_nav.findObject(&ctx.global, self.cls, self.start) != null;
    }
};

const AtGoal = struct {
    goal: math.Vector3f,
    threshold: f32,
    pub fn done(self: @This(), ctx: *MissionContext) global_nav.MissionError!bool {
        if (!ctx.global.frame.tracking_ok) return error.TrackingLost;
        return global_nav.cameraWithin(&ctx.global, self.goal, self.threshold);
    }
};

const YawFind = struct {
    cls: []const Auv.ObjectCls,
    pub fn done(self: @This(), ctx: *MissionContext) !bool {
        const frame_object = reactive_nav.findFrameObjectWithCls(&ctx.reactive, self.cls) orelse return false;
        return reactive_nav.findSeenById(&ctx.reactive, frame_object.id) != null;
    }
};

const Centered = struct {
    id: u32,
    pub fn done(self: @This(), ctx: *MissionContext) !bool {
        const box = reactive_nav.findFrameObject(&ctx.reactive, self.id) orelse return false;
        return reactive_nav.isCentered(&ctx.reactive, box, ctx.reactive.controller.center_deadband);
    }
};

const HeightReached = struct {
    id: u32,
    target_height: f32,
    pub fn done(self: @This(), ctx: *MissionContext) !bool {
        const box = reactive_nav.findFrameObject(&ctx.reactive, self.id) orelse return false;
        return reactive_nav.heightReached(&ctx.reactive, box, self.target_height);
    }
};

fn goAround(ctx: *MissionContext, object: *const Auv.Object) void {
    const object_pos = object.bbox.pose.pos;
    const object_rot = object.bbox.pose.quat;

    // corners of a square centered in 0, with area 1
    const square_corners: [4]math.Vector2f = .{
        .{0.5, 0.5},
        .{0.5, -0.5},
        .{-0.5, -0.5},
        .{-0.5, 0.5},
    };

    // absolute distance from any corner of the object
    const DISTANCE_TO_CORNER: f32 = 2.0;
    // a big number so that any corner is always closer
    const HUGE_NUMBER: f32 = 10000000.0;

    var corner_pluss: [4]math.Vector3f = undefined;
    var nearest_dist: f32 = HUGE_NUMBER;
    var starting_i: usize = 0;
    const initial_sub_pos = ctx.global.frame.camera_pose.pos;
    for (square_corners, 0..) |square_corner, i| {
        const sub_pose = ctx.global.frame.camera_pose;
        const rot_unit = math.normalize4f(object_rot);
        const actual_corner_2d = square_corner * math.xy(object.bbox.size);
        const actual_corner: math.Vector3f = .{actual_corner_2d[0], actual_corner_2d[1], 0};
        const rotated_corner = math.quaternionRotate(rot_unit, actual_corner);
        const rotated_corner_2d = math.xy(rotated_corner);
        const pos2d = math.xy(object_pos);
        const sub2d = math.xy(sub_pose.pos);
        const corner_plus =
            pos2d + rotated_corner_2d + math.normalize2f(rotated_corner_2d) * @as(math.Vector2f, @splat(DISTANCE_TO_CORNER));
        corner_pluss[i] = .{corner_plus[0], corner_plus[1], sub_pose.pos[2]};

        const dist = math.length2f(corner_plus - sub2d);
        if (dist < nearest_dist) {
            starting_i = i;
            nearest_dist = dist;
        }
    }

    for (0..corner_pluss.len) |i| {
        reachGoal(ctx, corner_pluss[(starting_i + i) % corner_pluss.len]) catch return;
    }
    reachGoal(ctx, initial_sub_pos) catch return;
}
