const std = @import("std");
const math = @import("../math.zig");
const MissionContext = @import("../MissionContext.zig");
const MissionArgs = @import("../MissionArgs.zig");
const Auv = @import("../Auv.zig");
const c = Auv.c;
const net = std.Io.net;

pub fn teleopMission(ctx: *MissionContext) !void {
    const port = ctx.config.teleop_port;
    const addr: net.IpAddress = .{ .ip4 = net.Ip4Address.loopback(port) };

    var sock = addr.bind(ctx.io, .{
        .mode = .dgram,
        .protocol = .udp,
    }) catch |err| switch (err) {
        error.AddressInUse => {
            std.log.err("teleop UDP port {d} is already in use", .{port});
            return error.AddressInUse;
        },
        else => return err,
    };
    defer sock.close(ctx.io);

    var recv_buf: [256]u8 = undefined;
    const timeout: std.Io.Timeout = .{ .duration = .{
        .clock = .awake,
        .raw = .fromNanoseconds(0),
    } };

    var translation: math.Vector3f = .{ 0, 0, 0 };
    var orientation: math.Quaternionf = .{ 0, 0, 0, 1 };

    while (true) {
        if (sock.receiveTimeout(ctx.io, &recv_buf, timeout)) |msg| {
            const packet_len = 7 * @sizeOf(f32);
            if (msg.data.len != packet_len) continue;
            var values: [7]f32 = undefined;
            for (0..values.len) |i| {
                const bits = std.mem.readInt(u32, msg.data[i * 4 ..][0..4], .little);
                values[i] = @bitCast(bits);
            }
            const next_translation: math.Vector3f = .{ values[0], values[1], values[2] };
            const next_orientation: math.Quaternionf = .{ values[3], values[4], values[5], values[6] };
            if (!std.math.isFinite(next_translation[0]) or
                !std.math.isFinite(next_translation[1]) or
                !std.math.isFinite(next_translation[2])) continue;
            const n2 = @reduce(.Add, next_orientation * next_orientation);
            if (!std.math.isFinite(n2) or n2 <= 1e-12) continue;
            translation = next_translation;
            orientation = math.normalize4f(next_orientation);
        } else |err| switch (err) {
            error.Timeout => {},
            else => return err,
        }

        if (math.length3f(translation) <= 1e-4) {
            ctx.motion = .{ .hold = .{ .orientation = orientation } };
        } else {
            ctx.motion = .{ .direct_thrust = .{
                .translation = translation,
                .orientation = orientation,
            } };
        }
        try ctx.yieldUntilNextFrameAndUpdate();
    }
}

pub fn cocoMission(ctx: *MissionContext) !void {
    ctx.step = "find_person";
    const person = try ctx.yieldUntilFirstObjectWithCls(@intCast(c.AUV_COCO_PERSON));
    ctx.step = "go_through_person";
    try goThrough(ctx, person);

    ctx.step = "find_bottle";
    const bottle = try ctx.yieldUntilFirstObjectWithCls(@intCast(c.AUV_COCO_BOTTLE));
    ctx.step = "go_around_bottle";
    try goAround(ctx, bottle);
}

pub fn robosubMission(ctx: *MissionContext) !void {
    ctx.step = "find_gate";
    const gate = try ctx.yieldUntilFirstObjectWithCls(@intCast(c.AUV_ROBOSUB_GATE));
    ctx.step = "go_through_gate";
    try goThrough(ctx, gate);

    ctx.step = "find_marker";
    const marker = try ctx.yieldUntilFirstObjectWithCls(@intCast(c.AUV_ROBOSUB_MARKER));
    ctx.step = "go_around_marker";
    try goAround(ctx, marker);
}

/// Surge, sway, and yaw from the 2D box with `id` until horizontal position is valid again.
pub fn followBoxUntilTracking(ctx: *MissionContext, id: u32) !void {
    ctx.resetOdometryPid();
    ctx.motion = .{ .look_forward_heading = .{ .id = id } };
    while (!ctx.frame.position_ok.x or !ctx.frame.position_ok.y) {
        try ctx.yieldUntilNextFrameAndUpdate();
    }
    ctx.resetOdometryPid();
}

const FAR_ENOUGH: f64 = 2.0;
const OVERSHOOT: f64 = 2.0;

fn goThrough(ctx: *MissionContext, object: *const Auv.Object) !void {
    const sub_pose = ctx.frame.camera_pose;

    const object_pos = object.bbox.pose.pos;
    const object_pos_2d = math.xy(object_pos);
    const sub_pos_2d = math.xy(sub_pose.pos);

    const direction_2d = math.normalize2f(object_pos_2d - sub_pos_2d);

    const before_2d = object_pos_2d - direction_2d * @as(math.Vector2f, @splat(FAR_ENOUGH));
    const before: math.Vector3f = .{ before_2d[0], before_2d[1], object_pos[2] };

    std.log.info("before object_pos {any}", .{object_pos_2d});
    std.log.info("before sub_pos {any}", .{sub_pos_2d});
    std.log.info("before {any}", .{before});

    try ctx.yieldUntilReachGoal(before, before);

    const overshoot_2d = object_pos_2d + direction_2d * @as(math.Vector2f, @splat(OVERSHOOT));
    const overshoot: math.Vector3f = .{ overshoot_2d[0], overshoot_2d[1], object_pos[2] };

    try ctx.yieldUntilReachGoal(overshoot, overshoot);
}

fn goAround(ctx: *MissionContext, object: *const Auv.Object) !void {
    const object_pos = object.bbox.pose.pos;
    const object_rot = object.bbox.pose.quat;

    // corners of a square centered in 0, with area 1
    const square_corners: [4]math.Vector2f = .{
        .{ 0.5, 0.5 },
        .{ 0.5, -0.5 },
        .{ -0.5, -0.5 },
        .{ -0.5, 0.5 },
    };

    // absolute distance from any corner of the object
    const DISTANCE_TO_CORNER: f32 = 2.0;
    // a big number so that any corner is always closer
    const HUGE_NUMBER: f32 = 10000000.0;

    var corner_pluss: [4]math.Vector3f = undefined;
    var starting_corner: math.Vector2f = .{ HUGE_NUMBER, HUGE_NUMBER };
    var starting_i: usize = std.math.maxInt(usize);
    const initial_sub_pos = ctx.frame.camera_pose.pos;
    for (square_corners, 0..) |square_corner, i| {
        const sub_pose = ctx.frame.camera_pose;
        const rot_unit = math.normalize4f(object_rot);
        const actual_corner_2d = square_corner * math.xy(object.bbox.size);
        const actual_corner: math.Vector3f = .{ actual_corner_2d[0], actual_corner_2d[1], 0 };
        const rotated_corner = math.quaternionRotate(rot_unit, actual_corner);
        const rotated_corner_2d = math.xy(rotated_corner);
        const pos2d = math.xy(object_pos);
        const sub2d = math.xy(sub_pose.pos);
        const corner_plus =
            pos2d + rotated_corner_2d + math.normalize2f(rotated_corner_2d) * @as(math.Vector2f, @splat(DISTANCE_TO_CORNER));
        corner_pluss[i] = .{ corner_plus[0], corner_plus[1], sub_pose.pos[2] };

        if (math.length2f(corner_plus - sub2d) < math.length2f(starting_corner)) {
            starting_i = i;
            starting_corner = corner_plus;
        }
    }

    for (0..corner_pluss.len) |i| {
        const corner = corner_pluss[(starting_i + i) % corner_pluss.len];
        try ctx.yieldUntilReachGoal(corner, corner);
    }
    try ctx.yieldUntilReachGoal(initial_sub_pos, initial_sub_pos);
}
