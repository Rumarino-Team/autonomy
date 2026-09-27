const std = @import("std");
const Auv = @import("../Auv.zig");
const math = @import("../math.zig");
const controls = @import("../controls.zig");

pub const ReactiveGains = struct {
    k_surge: f32 = 3.0,
    k_heave: f32 = 2.0,
    k_yaw: f32 = 2.0,
    /// Constant yaw moment while searching for a box that is not in frame. +yaw turns right.
    search_yaw: f32 = 0.5,
    center_deadband: f32 = 0.05,
};

/// IMU / 2D detection frame and tracked boxes.
/// `tracked_id` selects the box the body command follows.
/// `target_height` is a fraction of image height; null holds surge at zero while centering.
/// `search_yaw` is a constant yaw moment used while no box is selected.
pub const ReactiveState = struct {
    frame: Auv.ReactiveFrame = undefined,
    seen_objects2d: [Auv.ReactiveFrame.max_objects]Auv.Object2DYolo = undefined,
    seen_objects2d_len: u8 = 0,
    tracked_id: ?u32 = null,
    target_height: ?f32 = null,
    search_yaw: ?f32 = null,
};

pub const BoxMetrics = struct {
    /// Image center is the origin. +x is right, +y is down. Each axis is about [-0.5, 0.5].
    cx: f32,
    cy: f32,
    /// Box height as a fraction of the image height.
    height: f32,
};

pub const Actuation = struct {
    auv: *Auv,
    tam: []const math.Vector6f,
    gains: ReactiveGains,
    thrustor_saturate: f32,
    thrustor_output_scale: f32,
    log: bool = false,
};

pub fn updateSeenObjects(state: *ReactiveState) void {
    for (state.frame.objects2d[0..state.frame.object_len]) |frame_object| {
        const maybe_seen_object = for (state.seen_objects2d[0..state.seen_objects2d_len]) |*seen_object| {
            if (frame_object.id == seen_object.id) break seen_object;
        } else null;

        if (maybe_seen_object) |seen_object| {
            seen_object.* = frame_object;
        } else {
            state.seen_objects2d[state.seen_objects2d_len] = frame_object;
            state.seen_objects2d_len += 1;
        }
    }
}

/// First box at or after `start` whose class is in `clss`.
pub fn findObject(state: *const ReactiveState, clss: []const Auv.ObjectCls, start: usize) ?*const Auv.Object2DYolo {
    for (state.seen_objects2d[start..state.seen_objects2d_len]) |*object| {
        for (clss) |cls| {
            if (object.cls == cls) return object;
        }
    }
    return null;
}

pub fn findFrameObjectWithCls(state: *const ReactiveState, clss: []const Auv.ObjectCls) ?*const Auv.Object2DYolo {
    for (state.frame.objects2d[0..state.frame.object_len]) |*object| {
        for (clss) |cls| {
            if (object.cls == cls) return object;
        }
    }
    return null;
}

pub fn findSeenById(state: *const ReactiveState, id: u32) ?*const Auv.Object2DYolo {
    for (state.seen_objects2d[0..state.seen_objects2d_len]) |*object| {
        if (object.id == id) return object;
    }
    return null;
}

pub fn findFrameObject(state: *const ReactiveState, id: u32) ?*const Auv.Object2DYolo {
    for (state.frame.objects2d[0..state.frame.object_len]) |*object| {
        if (object.id == id) return object;
    }
    return null;
}

pub fn boxMetrics(state: *const ReactiveState, box: *const Auv.Object2DYolo) BoxMetrics {
    const width: f32 = @floatFromInt(state.frame.image_width);
    const height: f32 = @floatFromInt(state.frame.image_height);
    const cx_px: f32 = @floatFromInt(box.top_left.x + box.bottom_right.x);
    const cy_px: f32 = @floatFromInt(box.top_left.y + box.bottom_right.y);
    const box_h: u32 = if (box.bottom_right.y >= box.top_left.y)
        box.bottom_right.y - box.top_left.y
    else
        0;
    return .{
        .cx = cx_px * 0.5 / width - 0.5,
        .cy = cy_px * 0.5 / height - 0.5,
        .height = @as(f32, @floatFromInt(box_h)) / height,
    };
}

pub fn isCentered(state: *const ReactiveState, box: *const Auv.Object2DYolo, deadband: f32) bool {
    if (state.frame.image_width == 0 or state.frame.image_height == 0) return false;
    const metrics = boxMetrics(state, box);
    return @abs(metrics.cx) <= deadband and @abs(metrics.cy) <= deadband;
}

pub fn heightReached(state: *const ReactiveState, box: *const Auv.Object2DYolo, target_height: f32) bool {
    if (state.frame.image_height == 0) return false;
    return boxMetrics(state, box).height >= target_height;
}

fn withinDeadband(err: f32, deadband: f32) f32 {
    if (@abs(err) <= deadband) return 0;
    return err;
}

/// Body command [Fx, Fy, Fz, Mx, My, Mz]. Image error is already camera-relative.
/// +Y is surge, +Z is up, +yaw turns right. A box below center (positive cy) dives.
pub fn bodyCommand(state: *const ReactiveState, gains: ReactiveGains) math.Vector6f {
    const id = state.tracked_id orelse {
        const yaw = state.search_yaw orelse return @splat(0);
        return .{ 0, 0, 0, 0, 0, yaw };
    };
    const box = findFrameObject(state, id) orelse return @splat(0);
    if (state.frame.image_width == 0 or state.frame.image_height == 0) return @splat(0);

    const metrics = boxMetrics(state, box);
    const cx = withinDeadband(metrics.cx, gains.center_deadband);
    const cy = withinDeadband(metrics.cy, gains.center_deadband);
    const surge = if (state.target_height) |target| gains.k_surge * (target - metrics.height) else 0;
    const heave = -gains.k_heave * cy;
    const yaw = gains.k_yaw * cx;
    return .{ 0, surge, heave, 0, 0, yaw };
}

pub fn step(state: *ReactiveState, act: Actuation) void {
    const input = bodyCommand(state, act.gains);
    var thruster_buffer: [controls.max_thrusters]f32 = undefined;
    const thruster_values = controls.tamToThrusters(
        act.tam,
        input,
        act.thrustor_saturate,
        act.thrustor_output_scale,
        &thruster_buffer,
    );

    if (act.log) {
        std.debug.print("\x1b[2J\x1b[H", .{});
        std.log.debug("reactive objects = {}", .{state.frame.object_len});
        std.log.debug(
            "image = {}x{} tracked_id = {?} target_height = {?} search_yaw = {?}",
            .{
                state.frame.image_width,
                state.frame.image_height,
                state.tracked_id,
                state.target_height,
                state.search_yaw,
            },
        );
        math.debug6f("\tinput   ", input);
        std.log.debug("\tthruster_values = {any}", .{thruster_values});
    }
    act.auv.setThrustorValues(thruster_values.ptr, @intCast(thruster_values.len));
}
