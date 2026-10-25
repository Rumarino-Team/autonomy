const std = @import("std");
const math = @import("../math.zig");
const MissionContext = @import("../MissionContext.zig");
const Motion = @import("../Motion.zig");
const MissionArgs = @import("../MissionArgs.zig");
const Auv = @import("../Auv.zig");
const net = std.Io.net;

pub fn teleopMission(ctx: *MissionContext) !void {
    const port = ctx.config.teleop_port;
    const addr: net.IpAddress = .{ .ip4 = net.Ip4Address.loopback(port) };
    const retry_interval_ms: i64 = 1_000;
    const retry_interval: std.Io.Clock.Duration = .{
        .clock = .awake,
        .raw = .fromMilliseconds(retry_interval_ms),
    };

    var sock = while (true) {
        const bound_sock = addr.bind(ctx.io, .{
            .mode = .dgram,
            .protocol = .udp,
        }) catch |err| {
            std.log.debug(
                "teleop could not bind UDP port {d}: {s}. The port may already be in use; change `teleop_port` in auv.json. Retrying in {d} ms",
                .{ port, @errorName(err), retry_interval_ms },
            );

            const retry_at = std.Io.Clock.Timestamp.now(ctx.io, .awake).addDuration(retry_interval);
            while (std.Io.Clock.Timestamp.compare(
                std.Io.Clock.Timestamp.now(ctx.io, .awake),
                .lt,
                retry_at,
            )) {
                try ctx.yieldUntilNextFrameAndUpdate();
            }
            continue;
        };
        break bound_sock;
    };
    defer sock.close(ctx.io);

    var recv_buf: [256]u8 = undefined;
    const timeout: std.Io.Timeout = .{ .duration = .{
        .clock = .awake,
        .raw = .fromNanoseconds(0),
    } };

    while (true) {
        if (sock.receiveTimeout(ctx.io, &recv_buf, timeout)) |msg| {
            if (msg.data.len != 6 * @sizeOf(f32)) {
                return;
            }
            var values: [6]f32 = @splat(6);
            for (0..6) |i| {
                const bits = std.mem.readInt(u32, msg.data[i * 4 ..][0..4], .little);
                values[i] = @bitCast(bits);
            }
            ctx.setPoseGoal(values);
        } else |err| switch (err) {
            error.Timeout => {},
            else => return err,
        }

        try ctx.yieldUntilNextFrameAndUpdate();
    }
}

pub fn mission(ctx: *MissionContext) !void {
    ctx.recordEvent("search gate");
    try ctx.startSearch(.{ .target_class = .gate, .require_position = true, .duration = 60 });
    const gate_object = try ctx.yieldUntilFirstObjectWithCls(.gate);
    ctx.recordEvent("go through gate");
    try goThrough(ctx, gate_object);
    ctx.recordEvent("pitch turn");
    try fullTurn(ctx, .pitch);
    ctx.recordEvent("roll turn");
    try fullTurn(ctx, .roll);

    ctx.recordEvent("search marker");
    try ctx.startSearch(.{ .duration = 60 });
    const cube_or_rect_object = try ctx.yieldUntilFirstObjectWithAnyCls(&.{ .cube, .rect });
    ctx.recordEvent("go around marker");
    try goAround(ctx, cube_or_rect_object, .{
        .facing = .{ .look_at_object = cube_or_rect_object.id },
        .attitude_policy = .allow_pitch,
        .translation = .simultaneous,
        .position_tolerance = 0.2,
        .recovery = .{ .strategy = .search_for_tracking, .max_duration = 15 },
    });
}

const FAR_ENOUGH: f64 = 2.0;
const OVERSHOOT: f64 = 2.0;

const RotationAxis = enum { pitch, roll };

fn fullTurn(ctx: *MissionContext, axis: RotationAxis) !void {
    errdefer |err| if (err != error.RestartMission) ctx.stopThrusters();
    const position = ctx.navigation.snapshot.position orelse return error.TrackingUnavailable;
    const initial = ctx.navigation.snapshot.orientation orelse return error.SensorUnavailable;
    try ctx.startFollowPath(&.{position}, .{
        .move = .{
            .facing = .hold_current,
            .attitude = initial,
            .translation = .simultaneous,
        },
        .corner_radius = 0,
    });

    std.log.info("after gate: starting 360-degree {s} rotation", .{@tagName(axis)});
    // A quaternion endpoint at 2*pi is identical to the initial orientation.
    // Consecutive targets below pi preserve a positive turn about the chosen
    // body axis: Y for pitch, X for roll, matching the controller convention.
    for (1..9) |step| {
        const angle = @as(f32, @floatFromInt(step)) * (std.math.pi / 4.0);
        const sine = @sin(angle / 2);
        const cosine = @cos(angle / 2);
        const turn: math.Quaternionf = switch (axis) {
            .pitch => .{ 0, sine, 0, cosine },
            .roll => .{ sine, 0, 0, cosine },
        };
        const desired = math.normalize4f(math.quaternionMul(initial, turn));
        ctx.motion.?.options.attitude = desired;
        ctx.motion.?.completed = false;
        ctx.motion.?.settling = .{};
        var settling: Motion.Settling = .{};
        while (true) {
            try ctx.yieldUntilNextFrameAndUpdate();
            try ctx.checkMotionFailure();
            const snapshot = ctx.navigation.snapshot;
            const actual = snapshot.orientation orelse continue;
            const rate = snapshot.angular_velocity_body orelse continue;
            const final = step == 8;
            const tolerance: f32 = if (final) std.math.pi / 180.0 else std.math.pi / 36.0;
            const aligned = math.length3f(math.quaternionError(actual, desired)) <= tolerance;
            const slow = !final or math.length3f(rate) <= ctx.motion.?.options.angular_rate_tolerance;
            if (settling.update(aligned, slow, ctx.controller_dt, if (final) 0.3 else 0)) break;
        }
        std.log.info("{s} rotation progress: {d}/360 degrees", .{ @tagName(axis), step * 45 });
    }
    // Position control is best effort while tilted: at some angles Proteus
    // cannot supply world vertical force. Continue from the measured end pose.
    ctx.holdPosition();
    std.log.info("360-degree {s} rotation complete; continuing mission", .{@tagName(axis)});
}

fn goThrough(ctx: *MissionContext, object: *const Auv.Object) !void {
    const sub_pose: math.Pose = .{ .pos = ctx.navigation.snapshot.position orelse return error.TrackingUnavailable, .quat = ctx.navigation.snapshot.orientation orelse return error.SensorUnavailable };

    const object_pos = object.bbox.pose.pos;
    const object_pos_2d = math.xy(object_pos);
    const sub_pos_2d = math.xy(sub_pose.pos);

    const approach = object_pos_2d - sub_pos_2d;
    if (math.length2f(approach) <= 1e-4) return error.InvalidGateApproach;
    const direction_2d = math.normalize2f(approach);

    const before_2d = object_pos_2d - direction_2d * @as(math.Vector2f, @splat(FAR_ENOUGH));
    const before: math.Vector3f = .{ before_2d[0], before_2d[1], object_pos[2] };

    std.log.info("before object_pos {any}", .{object_pos_2d});
    std.log.info("before sub_pos {any}", .{sub_pos_2d});
    std.log.info("before {any}", .{before});

    const overshoot_2d = object_pos_2d + direction_2d * @as(math.Vector2f, @splat(OVERSHOOT));
    const overshoot: math.Vector3f = .{ overshoot_2d[0], overshoot_2d[1], object_pos[2] };

    // Keep the crossing heading after passing the gate; looking at its centre
    // would command a turn back toward it on the exit side.
    const direction: math.Vector3f = .{ direction_2d[0], direction_2d[1], 0 };
    try ctx.yieldUntilFollowPath(&.{ before, overshoot }, .{
        .move = .{
            .facing = .{ .fixed_yaw = @import("../Motion.zig").yawForDirection(direction, 0) },
            .translation = .align_first,
            .position_tolerance = 0.2,
            .recovery = .{ .strategy = .search_for_tracking, .max_duration = 15 },
        },
        .corner_radius = 0,
    });
}

fn goAround(ctx: *MissionContext, object: *const Auv.Object, options: MissionContext.MoveOptions) !void {
    const waypoints = orbitWaypoints(object, ctx.navigation.snapshot.position orelse return error.TrackingUnavailable);
    try ctx.yieldUntilFollowPath(&waypoints, .{ .move = options });
}

fn orbitWaypoints(object: *const Auv.Object, initial_sub_pos: math.Vector3f) [6]math.Vector3f {
    return orbitWaypointsWithClearance(object, initial_sub_pos, 2);
}

pub fn orbitWaypointsWithClearance(object: *const Auv.Object, initial_sub_pos: math.Vector3f, clearance: f32) [6]math.Vector3f {
    const object_pos = object.bbox.pose.pos;
    const object_rot = object.bbox.pose.quat;

    // corners of a square centered in 0, with area 1
    const square_corners: [4]math.Vector2f = .{
        .{ 0.5, 0.5 },
        .{ 0.5, -0.5 },
        .{ -0.5, -0.5 },
        .{ -0.5, 0.5 },
    };

    // Expand each local side by 2m. Rounded corners then remain outside the
    // object, unlike an unconstrained spline through its original corners.

    var corner_pluss: [4]math.Vector3f = undefined;
    var nearest_distance: f32 = std.math.inf(f32);
    var starting_i: usize = 0;
    for (square_corners, 0..) |square_corner, i| {
        const rot_unit = math.normalize4f(object_rot);
        const actual_corner_2d = square_corner * (math.xy(object.bbox.size) + @as(math.Vector2f, @splat(2 * clearance)));
        const actual_corner: math.Vector3f = .{ actual_corner_2d[0], actual_corner_2d[1], 0 };
        const rotated_corner = math.quaternionRotate(rot_unit, actual_corner);
        const rotated_corner_2d = math.xy(rotated_corner);
        const pos2d = math.xy(object_pos);
        const sub2d = math.xy(initial_sub_pos);
        const corner_plus = pos2d + rotated_corner_2d;
        corner_pluss[i] = .{ corner_plus[0], corner_plus[1], initial_sub_pos[2] };

        const distance = math.length2f(corner_plus - sub2d);
        if (distance < nearest_distance) {
            starting_i = i;
            nearest_distance = distance;
        }
    }

    var waypoints: [6]math.Vector3f = undefined;
    for (0..5) |i| {
        waypoints[i] = corner_pluss[(starting_i + i) % corner_pluss.len];
    }
    // Complete the loop through the starting corner before retracing the entry
    // leg. Returning directly from corner four can cut across the object.
    waypoints[5] = initial_sub_pos;
    return waypoints;
}

test "orbit starts at the nearest world corner and closes before returning" {
    const object: Auv.Object = .{
        .id = 7,
        .cls = .cube,
        .bbox = .{ .pose = .{ .pos = .{ 100, 100, 2 }, .quat = .{ 0, 0, 0, 1 } }, .size = .{ 2, 2, 2 } },
    };
    const route = orbitWaypoints(&object, .{ 95, 101, 2 });
    try std.testing.expect(math.length3f(route[0] - @as(math.Vector3f, .{ 97, 103, 2 })) < 1e-5);
    try std.testing.expect(math.length3f(route[4] - route[0]) < 1e-5);
    try std.testing.expect(math.length3f(route[5] - @as(math.Vector3f, .{ 95, 101, 2 })) < 1e-5);
}
