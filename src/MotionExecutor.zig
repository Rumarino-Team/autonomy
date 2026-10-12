const std = @import("std");
const math = @import("math.zig");
const Auv = @import("Auv.zig");
const Motion = @import("Motion.zig");
const Navigation = @import("Navigation.zig");

pub const VisualError = struct { cx: f32, cy: f32 = 0, approach: f32 = 0, approach_active: bool = false, align_vertical: bool = false };
pub const Target = struct {
    mode: Motion.Mode = .paused,
    world_position: ?math.Vector3f = null,
    heading: ?f32 = null,
    orientation: ?math.Quaternionf = null,
    yaw_rate_limit: f32 = 0.6,
    roll_pitch: math.Vector2f = .{ 0, 0 },
    pressure_depth: ?f32 = null,
    body_effort: math.Vector3f = @splat(0),
    visual: ?VisualError = null,
    translation_scale: f32 = 1,
    max_effort: f32 = 1,
    stop: bool = false,
};

pub fn box(frame: *const Auv.Frame, observations_valid: bool, id: u32) ?math.BoxMetrics2d {
    if (!observations_valid or frame.image_width == 0 or frame.image_height == 0) return null;
    for (frame.objects2d[0..@min(frame.objects2d_len, Auv.Frame.max_objects)]) |object| {
        if (object.id != id) continue;
        if (object.bottom_right.x <= object.top_left.x or object.bottom_right.y <= object.top_left.y or
            object.bottom_right.x > frame.image_width or object.bottom_right.y > frame.image_height) continue;
        return math.boxMetrics(frame.image_width, frame.image_height, .{ object.top_left.x, object.top_left.y }, .{ object.bottom_right.x, object.bottom_right.y });
    }
    return null;
}

fn visualError(metrics: math.BoxMetrics2d, options: Motion.VisualOptions) VisualError {
    return .{
        .cx = math.withinDeadband(metrics.cx, options.center_tolerance),
        .cy = math.withinDeadband(metrics.cy, options.center_tolerance),
        .approach = if (options.target_height) |height| math.withinDeadband(height - metrics.height, options.height_tolerance) else 0,
        .approach_active = options.target_height != null,
        .align_vertical = options.align_vertical,
    };
}

fn worldObject(nav: *const Navigation, frame: *const Auv.Frame, id: u32) ?math.Vector3f {
    if (!nav.snapshot.measured_position or !nav.snapshot.observations_valid) return null;
    for (frame.objects[0..@min(frame.objects_len, Auv.Frame.max_objects)]) |object| {
        if (object.id == id and Navigation.validPoint(object.bbox.pose.pos)) return nav.toMission(object.bbox.pose.pos);
    }
    return null;
}

fn fail(state: *Motion.State, failure: Motion.Failure) Target {
    state.failure = failure;
    state.mode = .failed;
    state.paused = true;
    state.settling = .{};
    return .{ .mode = .failed, .stop = true };
}

fn base(state: *const Motion.State, nav: *const Navigation, goal: math.Vector6f) Target {
    const world_depth = if (nav.pressure_to_world_offset) |offset| goal[2] - offset else state.pressure_goal;
    return .{
        .heading = if (nav.snapshot.orientation != null) state.commanded_yaw else null,
        .roll_pitch = .{ goal[3], goal[4] },
        .orientation = state.options.attitude,
        .yaw_rate_limit = state.options.yaw_rate_limit,
        .pressure_depth = world_depth,
    };
}

fn sweep(center: f32, seconds: f64, options: Motion.SearchOptions) f32 {
    // Triangle sweep starts at the captured heading, then visits both sides.
    const phase: f32 = @floatCast(@mod(seconds * options.sweep_rate + options.sweep_angle, 4 * options.sweep_angle));
    const offset = if (phase <= 2 * options.sweep_angle) phase - options.sweep_angle else 3 * options.sweep_angle - phase;
    return math.wrapAngle(center + offset);
}

fn detected(frame: *const Auv.Frame, snapshot: Navigation.Snapshot, options: Motion.SearchOptions) bool {
    if (!snapshot.observations_valid or (options.require_position and !snapshot.measured_position)) return false;
    if (options.target_id == null and options.target_class == null) return false;
    for (frame.objects2d[0..@min(frame.objects2d_len, Auv.Frame.max_objects)]) |object| {
        if (options.target_id) |id| if (object.id != id) continue;
        if (options.target_class) |cls| if (object.cls != cls) continue;
        if (box(frame, true, object.id) != null) return true;
    }
    if (snapshot.measured_position) {
        for (frame.objects[0..@min(frame.objects_len, Auv.Frame.max_objects)]) |object| {
            if (options.target_id) |id| if (object.id != id) continue;
            if (options.target_class) |cls| if (object.cls != cls) continue;
            if (Navigation.validPoint(object.bbox.pose.pos)) return true;
        }
    }
    return false;
}

fn recoveryTarget(state: *Motion.State, nav: *const Navigation, frame: *const Auv.Frame, goal: math.Vector6f, dt: f32) Target {
    var target = base(state, nav, goal);
    target.max_effort = state.options.recovery.max_effort;
    target.mode = .paused;
    switch (state.options.recovery.strategy) {
        .search_for_tracking => {
            const options = state.options.recovery.search;
            state.desired_yaw = sweep(state.recovery_yaw, state.recovery_seconds, options);
            state.commanded_yaw = Motion.slewYaw(state.commanded_yaw, state.desired_yaw, @min(options.sweep_rate, state.options.yaw_rate_limit), dt);
            target.heading = if (nav.snapshot.orientation != null) state.commanded_yaw else null;
            if (target.heading != null) target.body_effort = .{ options.body_effort[0], options.body_effort[1], 0 };
            target.max_effort = @min(target.max_effort, options.max_effort);
            if (options.depth) |depth| target.pressure_depth = depth;
            target.mode = .search;
        },
        .visual_target => {
            const options = state.options.recovery.visual.?;
            if (box(frame, nav.snapshot.observations_valid, options.object_id)) |metrics| {
                target.visual = visualError(metrics, options);
                if (state.options.translation == .align_first and @abs(metrics.cx) > options.center_tolerance) {
                    target.visual.?.approach = 0;
                    target.visual.?.approach_active = false;
                }
                target.heading = null;
                target.max_effort = @min(target.max_effort, options.max_effort);
                if (options.depth) |depth| target.pressure_depth = depth;
                target.mode = .visual;
            }
        },
        .pause_and_resume, .estimate => {
            // Object-facing may retain fresh image centering, without approach.
            if (state.options.facing == .look_at_object) {
                if (box(frame, nav.snapshot.observations_valid, state.options.facing.look_at_object)) |metrics| {
                    target.visual = .{ .cx = math.withinDeadband(metrics.cx, 0.05) };
                    target.heading = null;
                }
            }
        },
    }
    state.mode = target.mode;
    return target;
}

/// Every command uses this executor. Only measured world pose can complete a
/// route; recovery/search movement never increments its saved distance.
pub fn update(state: *Motion.State, nav: *const Navigation, frame: *const Auv.Frame, goal: *math.Vector6f, tolerance: f32, elapsed: f32) Target {
    const dt = std.math.clamp(elapsed, 0, 0.1);
    const snapshot = nav.snapshot;
    state.elapsed += @max(0, elapsed);
    if (state.failure) |failure| return fail(state, failure);
    if (!state.completed and state.command != .hold) {
        if (state.options.timeout) |timeout| if (state.elapsed >= timeout) return fail(state, .move_timeout);
    }
    if (snapshot.orientation == null or snapshot.angular_velocity_body == null) {
        state.mode = .paused;
        state.missing_feedback_seconds += @max(0, elapsed);
        state.settling = .{};
        if (state.missing_feedback_seconds >= state.options.observation_timeout) return fail(state, .sensor_unavailable);
        return .{ .mode = .paused, .stop = true };
    }
    state.missing_feedback_seconds = 0;

    switch (state.command) {
        .search => |options| {
            if (state.completed) return .{ .stop = true };
            state.mode = .search;
            state.desired_yaw = sweep(state.captured_yaw, state.elapsed, options);
            state.commanded_yaw = Motion.slewYaw(state.commanded_yaw, state.desired_yaw, @min(options.sweep_rate, state.options.yaw_rate_limit), dt);
            var target = base(state, nav, goal.*);
            target.mode = .search;
            target.pressure_depth = options.depth orelse state.pressure_goal;
            target.max_effort = options.max_effort;
            if (snapshot.orientation != null) target.body_effort = .{ options.body_effort[0], options.body_effort[1], 0 };
            const found = detected(frame, snapshot, options);
            if (state.settling.update(found, true, dt, options.detection_settle_time)) {
                state.completed = true;
                return .{ .stop = true };
            }
            if (state.elapsed >= options.duration) {
                if (options.target_id != null or options.target_class != null) return fail(state, .search_timeout);
                if (snapshot.orientation == null) return fail(state, .sensor_unavailable);
                state.completed = true;
                return .{ .stop = true };
            }
            return target;
        },
        .visual => |options| {
            state.mode = .visual;
            var target = base(state, nav, goal.*);
            target.mode = .visual;
            target.heading = null;
            target.pressure_depth = options.depth orelse state.pressure_goal;
            target.max_effort = options.max_effort;
            if (box(frame, snapshot.observations_valid, options.object_id)) |metrics| {
                state.missing_object_seconds = 0;
                target.visual = visualError(metrics, options);
                if (state.options.translation == .align_first and @abs(metrics.cx) > options.center_tolerance) {
                    target.visual.?.approach = 0;
                    target.visual.?.approach_active = false;
                }
                const centered = @abs(metrics.cx) <= options.center_tolerance and
                    (!options.align_vertical or @abs(metrics.cy) <= options.center_tolerance);
                const height_ok = if (options.target_height) |height| @abs(height - metrics.height) <= options.height_tolerance else true;
                if (state.settling.update(centered, height_ok, dt, state.options.settle_time)) state.completed = true;
            } else {
                state.missing_object_seconds += @max(0, elapsed);
                state.settling = .{};
                if (state.missing_object_seconds >= state.options.observation_timeout) return fail(state, .object_lost);
            }
            return target;
        },
        .body => |options| {
            if (state.completed) return .{ .stop = true };
            state.mode = .body;
            state.update(snapshot.position orelse state.destination, null, snapshot.measured_position, dt);
            var target = base(state, nav, goal.*);
            target.mode = .body;
            target.max_effort = options.max_effort;
            target.pressure_depth = if (options.effort[2] == 0) options.depth orelse state.pressure_goal else null;
            if (snapshot.yaw()) |yaw| {
                state.paused = false;
                state.recovery_seconds = 0;
                const scale = translationScale(state, nav, yaw);
                if (state.active_seconds < options.duration) {
                    target.body_effort = .{ options.effort[0] * scale, options.effort[1] * scale, options.effort[2] };
                    if (scale > 0 or options.effort[2] != 0) state.active_seconds += @max(0, elapsed);
                }
            } else {
                state.recovery_seconds += @max(0, elapsed);
                if (state.recovery_seconds >= (state.options.recovery.max_duration orelse state.options.observation_timeout)) return fail(state, .sensor_unavailable);
            }
            const yaw_ok = if (snapshot.yaw()) |yaw| attitudeReached(state, nav, yaw) else !state.options.require_yaw;
            if (state.settling.update(state.active_seconds >= options.duration, yaw_ok, dt, state.options.settle_time)) {
                state.completed = true;
                return .{ .stop = true };
            }
            return target;
        },
        else => {},
    }

    if (snapshot.localization_changed) return fail(state, .localization_changed);
    // A cached world target keeps aging during recovery. Image-only facing
    // may continue, but reacquiring pose cannot revive an expired 3D target.
    if (!snapshot.measured_position and state.options.facing == .look_at_object and
        !(state.options.recovery.strategy == .estimate and snapshot.estimated_position))
    {
        state.missing_object_seconds += @max(0, elapsed);
        if (state.missing_object_seconds >= state.options.observation_timeout) state.last_object_position = null;
    }
    if (!snapshot.measured_position and !state.recovering) {
        state.recovering = true;
        state.recovery_position = nav.last_measured_position;
        state.recovery_yaw = snapshot.yaw() orelse state.commanded_yaw;
        state.last_recovery_yaw = snapshot.yaw();
        state.last_recovery_orientation = snapshot.orientation;
        state.recovery_seconds = 0;
        state.tracking_confirm_seconds = 0;
        state.settling = .{};
    }
    if (state.recovering) {
        state.recovery_seconds += @max(0, elapsed);
        if (state.command != .hold and state.recovery_seconds >= (state.options.recovery.max_duration orelse state.options.observation_timeout))
            return fail(state, .tracking_lost);
        if (snapshot.measured_position) {
            if (state.recovery_position) |position| {
                if (math.length3f(snapshot.position.? - position) > state.options.recovery.max_rejoin_distance)
                    return fail(state, .rejoin_too_far);
            }
            if (state.options.attitude != null and state.last_recovery_orientation != null) {
                if (math.length3f(math.quaternionError(state.last_recovery_orientation.?, snapshot.orientation.?)) > state.options.recovery.max_heading_jump)
                    return fail(state, .localization_changed);
            } else if (state.last_recovery_yaw) |yaw| {
                if (@abs(math.wrapAngle(snapshot.yaw().? - yaw)) > state.options.recovery.max_heading_jump)
                    return fail(state, .localization_changed);
            }
            state.tracking_confirm_seconds += dt;
            if (state.tracking_confirm_seconds < state.options.recovery.confirm_tracking) {
                if (state.options.facing == .look_at_object) {
                    if (worldObject(nav, frame, state.options.facing.look_at_object)) |point| {
                        state.last_object_position = point;
                        state.missing_object_seconds = 0;
                    } else {
                        state.missing_object_seconds += @max(0, elapsed);
                        if (state.missing_object_seconds >= state.options.observation_timeout) state.last_object_position = null;
                    }
                }
                state.mode = .confirming;
                var target = base(state, nav, goal.*);
                target.mode = .confirming;
                target.max_effort = state.options.recovery.max_effort;
                return target;
            }
            if (state.command == .route) {
                const route = &state.command.route;
                route.distance = route.path.rejoin(snapshot.position.?, route.distance, route.max_tracking_error);
            }
            state.recovering = false;
            state.recovery_seconds = 0;
            state.resume_scale = 0;
        } else {
            state.tracking_confirm_seconds = 0;
            state.last_recovery_yaw = snapshot.yaw();
            state.last_recovery_orientation = snapshot.orientation;
            if (!(state.options.recovery.strategy == .estimate and snapshot.estimated_position)) {
                // A search owns the heading ramp during recovery. Updating
                // hold_current first would pull each sweep step back to zero.
                if (state.options.recovery.strategy != .search_for_tracking) state.update(state.destination, null, false, dt);
                state.paused = true;
                return recoveryTarget(state, nav, frame, goal.*, dt);
            }
        }
    }

    const position = snapshot.position orelse return base(state, nav, goal.*);
    const yaw = snapshot.yaw() orelse return base(state, nav, goal.*);
    state.mode = if (snapshot.estimated_position) .estimated else .world;
    const object_position = if (state.options.facing == .look_at_object) worldObject(nav, frame, state.options.facing.look_at_object) else null;
    if (state.command == .route) state.path_direction = state.command.route.path.sample(state.command.route.distance).direction;
    state.update(position, object_position, true, @max(0, elapsed));
    if (state.failure) |failure| return fail(state, failure);
    if (state.options.recovery.resume_ramp == 0) state.resume_scale = 1 else state.resume_scale = @min(1, state.resume_scale + dt / state.options.recovery.resume_ramp);
    const scale = translationScale(state, nav, yaw) * state.resume_scale;
    if (state.command == .route) {
        const route = &state.command.route;
        const reference = route.path.sample(route.distance);
        if (!state.paused and math.length3f(reference.position - position) <= route.max_tracking_error) {
            route.distance = @min(route.path.total_length, route.distance + route.reference_speed * dt * scale);
        }
        const next = route.path.sample(route.distance);
        goal.*[0] = next.position[0];
        goal.*[1] = next.position[1];
        goal.*[2] = next.position[2];
        state.path_direction = next.direction;
        const position_reached = snapshot.measured_position and !state.paused and route.distance >= route.path.total_length and
            math.length3f(route.path.end - position) <= (state.options.position_tolerance orelse tolerance);
        if (state.settling.update(position_reached, attitudeReached(state, nav, yaw), dt, state.options.settle_time)) state.completed = true;
    }
    var target = base(state, nav, goal.*);
    target.mode = state.mode;
    target.world_position = .{ goal.*[0], goal.*[1], goal.*[2] };
    target.translation_scale = scale;
    if (snapshot.estimated_position) target.max_effort = state.options.recovery.max_effort;
    return target;
}

fn attitudeReached(state: *const Motion.State, nav: *const Navigation, yaw: f32) bool {
    if (!state.options.require_yaw) return true;
    const rate = nav.snapshot.angular_velocity_body orelse return false;
    if (math.length3f(rate) > state.options.angular_rate_tolerance) return false;
    if (state.options.attitude) |desired| {
        const actual = nav.snapshot.orientation orelse return false;
        return math.length3f(math.quaternionError(actual, desired)) <= state.options.attitude_tolerance;
    }
    return state.yawReached(yaw);
}

fn translationScale(state: *const Motion.State, nav: *const Navigation, yaw: f32) f32 {
    if (state.options.attitude) |desired| {
        if (state.paused) return 0;
        if (state.options.translation == .simultaneous) return 1;
        const actual = nav.snapshot.orientation orelse return 0;
        const error_angle = math.length3f(math.quaternionError(actual, desired));
        const gate = state.options.alignment_gate;
        const blend = std.math.clamp((gate - error_angle) / (gate * 0.5), 0, 1);
        return blend * blend * (3 - 2 * blend);
    }
    return state.translationScale(yaw);
}
