const std = @import("std");
const math = @import("../math.zig");
const MissionContext = @import("../MissionContext.zig");
const MissionArgs = @import("../MissionArgs.zig");
const Auv = @import("../Auv.zig");

pub fn mission(ctx: *MissionContext) void {
    const gate_object = ctx.globalYieldUntilFirstObjectWithCls(.gate);
    goThrough(ctx, gate_object);

    const cube_or_rect_object = ctx.globalYieldUntilFirstObjectWithAnyCls(&.{ .cube, .rect });
    goAround(ctx, cube_or_rect_object);
}

const FAR_ENOUGH: f32 = 2.0;
const OVERSHOOT: f32 = 2.0;

fn reachGoal(ctx: *MissionContext, goal_pos: math.Vector3f) !void {
    return ctx.globalYieldUntilReachGoal(goal_pos);
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
    const box = findObject2d(ctx, cls) orelse {
        std.log.err("tracking lost and no 2D {s} is in the reactive frame", .{@tagName(cls)});
        return;
    };
    std.log.warn("tracking lost; holding on 2D {s} id {}", .{ @tagName(cls), box.id });
}

fn findObject2d(ctx: *MissionContext, cls: Auv.ObjectCls) ?*const Auv.Object2DYolo {
    for (ctx.reactive.frame.objects2d[0..ctx.reactive.frame.object_len]) |*obj| {
        if (obj.cls == cls) return obj;
    }
    return null;
}

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
    var starting_corner: math.Vector2f = .{HUGE_NUMBER, HUGE_NUMBER};
    var starting_i: usize = std.math.maxInt(usize);
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

        if (math.length2f(corner_plus - sub2d) < math.length2f(starting_corner)) {
            starting_i = i;
            starting_corner = corner_plus;
        }
    }

    for (0..corner_pluss.len) |i| {
        reachGoal(ctx, corner_pluss[(starting_i + i) % corner_pluss.len]) catch return;
    }
    reachGoal(ctx, initial_sub_pos) catch return;
}
