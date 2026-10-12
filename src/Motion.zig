const std = @import("std");
const math = @import("math.zig");
const Auv = @import("Auv.zig");

/// All public yaw angles use quaternion/Euler yaw in radians. The vehicle's
/// nose is body +Y, so yaw zero faces world +Y, not world +X.
pub const Facing = union(enum) {
    hold_current,
    face_goal,
    look_at_point: math.Vector3f,
    look_at_object: u32,
    fixed_yaw: f32,
    face_path,
};

pub const Translation = enum { align_first, simultaneous };

/// Fixed preserves the requested attitude. Allow_pitch preserves the facing
/// heading while the controller chooses pitch to make translation feasible.
pub const AttitudePolicy = enum { fixed, allow_pitch };

pub const SearchOptions = struct {
    target_class: ?Auv.ObjectCls = null,
    target_id: ?u32 = null,
    require_position: bool = false,
    duration: f32 = 10,
    sweep_angle: f32 = std.math.pi / 3.0,
    sweep_rate: f32 = 0.3,
    detection_settle_time: f32 = 0.2,
    body_effort: math.Vector2f = .{ 0, 0 },
    max_effort: f32 = 0.2,
    /// Pressure depth in metres, positive down; null captures current depth.
    depth: ?f32 = null,

    pub fn validate(options: SearchOptions) !void {
        if (!positive(options.duration) or !positive(options.sweep_angle) or options.sweep_angle > std.math.pi or
            !positive(options.sweep_rate) or !nonnegative(options.detection_settle_time) or !effortLimit(options.max_effort) or
            !std.math.isFinite(options.body_effort[0]) or !std.math.isFinite(options.body_effort[1]) or
            @abs(options.body_effort[0]) > 1 or @abs(options.body_effort[1]) > 1)
            return error.InvalidSearchOptions;
        if (options.depth) |depth| if (!nonnegative(depth)) return error.InvalidSearchOptions;
    }
};

pub const VisualOptions = struct {
    object_id: u32,
    /// Normalized image height. Null centers the object without approaching.
    target_height: ?f32 = null,
    height_tolerance: f32 = 0.05,
    center_tolerance: f32 = 0.05,
    align_vertical: bool = false,
    max_effort: f32 = 0.2,
    depth: ?f32 = null,

    pub fn validate(options: VisualOptions) !void {
        if (!positive(options.height_tolerance) or !positive(options.center_tolerance) or
            options.center_tolerance > 1 or !effortLimit(options.max_effort)) return error.InvalidVisualOptions;
        if (options.target_height) |height| if (!positive(height) or height > 1) return error.InvalidVisualOptions;
        if (options.depth) |depth| if (!nonnegative(depth)) return error.InvalidVisualOptions;
        if (options.align_vertical and options.depth != null) return error.InvalidVisualOptions;
    }
};

pub const BodyOptions = struct {
    /// Normalized body force demand, scaled by control.body_force_scale (N).
    /// This is not a measured velocity or a direct motor command.
    effort: math.Vector3f,
    duration: f32,
    max_effort: f32 = 0.2,
    depth: ?f32 = null,

    pub fn validate(options: BodyOptions) !void {
        if (!finitePoint(options.effort) or @abs(options.effort[0]) > 1 or @abs(options.effort[1]) > 1 or
            @abs(options.effort[2]) > 1 or !positive(options.duration) or !effortLimit(options.max_effort))
            return error.InvalidBodyOptions;
        if (options.depth) |depth| if (!nonnegative(depth) or options.effort[2] != 0) return error.InvalidBodyOptions;
    }
};

pub const RecoveryOptions = struct {
    pub const Strategy = enum { pause_and_resume, search_for_tracking, visual_target, estimate };
    strategy: Strategy = .pause_and_resume,
    /// Null preserves the existing observation_timeout deadline.
    max_duration: ?f32 = null,
    confirm_tracking: f32 = 0.3,
    resume_ramp: f32 = 0.5,
    max_rejoin_distance: f32 = 2.0,
    max_heading_jump: f32 = std.math.pi / 4.0,
    max_effort: f32 = 0.2,
    search: SearchOptions = .{},
    visual: ?VisualOptions = null,

    pub fn validate(options: RecoveryOptions) !void {
        if (!nonnegative(options.confirm_tracking) or !nonnegative(options.resume_ramp) or
            !positive(options.max_rejoin_distance) or !positive(options.max_heading_jump) or
            options.max_heading_jump > std.math.pi or !effortLimit(options.max_effort)) return error.InvalidRecoveryOptions;
        if (options.max_duration) |duration| if (!positive(duration)) return error.InvalidRecoveryOptions;
        try options.search.validate();
        if (options.visual) |visual| try visual.validate();
        if (options.strategy == .visual_target and options.visual == null) return error.InvalidRecoveryOptions;
    }
};

pub const MoveOptions = struct {
    facing: Facing,
    /// Fixed full body-to-world attitude. Use facing=hold_current when supplied.
    /// Quaternions are [x,y,z,w]; normalized when consumed.
    attitude: ?math.Quaternionf = null,
    /// World moves only: preserve full attitude or release pitch to translate.
    attitude_policy: AttitudePolicy = .fixed,
    /// Absolute adaptive pitch reference bound; at most pi/4 radians.
    max_pitch: f32 = std.math.pi / 6.0,
    /// Maximum change of the adaptive pitch reference in radians per second.
    pitch_rate_limit: f32 = 0.25,
    attitude_tolerance: f32 = std.math.pi / 36.0,
    angular_rate_tolerance: f32 = 0.1,
    translation: Translation = .simultaneous,
    yaw_rate_limit: f32 = 0.6,
    yaw_tolerance: f32 = std.math.pi / 36.0,
    alignment_gate: f32 = std.math.pi / 8.0,
    position_tolerance: ?f32 = null,
    require_yaw: bool = true,
    settle_time: f32 = 0.3,
    observation_timeout: f32 = 2.0,
    timeout: ?f32 = null,
    recovery: RecoveryOptions = .{},

    pub fn validate(options: MoveOptions) !void {
        try options.recovery.validate();
        if (!positive(options.max_pitch) or options.max_pitch > std.math.pi / 4.0 or
            !positive(options.pitch_rate_limit)) return error.InvalidMoveOptions;
        if (!positive(options.attitude_tolerance) or !positive(options.angular_rate_tolerance)) return error.InvalidMoveOptions;
        if (options.attitude) |q| {
            inline for (0..4) |axis| if (!std.math.isFinite(q[axis])) return error.InvalidMoveOptions;
            const norm2 = @reduce(.Add, q * q);
            if (!std.math.isFinite(norm2) or norm2 < 0.000001 or options.facing != .hold_current or options.attitude_policy != .fixed or
                options.recovery.strategy == .search_for_tracking or options.recovery.strategy == .visual_target) return error.InvalidMoveOptions;
        }
        if (!positive(options.yaw_rate_limit) or !positive(options.yaw_tolerance) or
            !positive(options.alignment_gate) or options.alignment_gate > std.math.pi or
            !nonnegative(options.settle_time) or !positive(options.observation_timeout))
            return error.InvalidMoveOptions;
        if (options.position_tolerance) |value| if (!positive(value)) return error.InvalidMoveOptions;
        if (options.timeout) |value| if (!positive(value)) return error.InvalidMoveOptions;
        switch (options.facing) {
            .fixed_yaw => |yaw| if (!std.math.isFinite(yaw)) return error.InvalidMoveOptions,
            .look_at_point => |point| if (!finitePoint(point)) return error.InvalidMoveOptions,
            else => {},
        }
    }
};

pub const PathOptions = struct {
    move: MoveOptions,
    /// Distance trimmed on each side of a corner (metres).
    corner_radius: f32 = 0.6,
    reference_speed: f32 = 0.5,
    /// Freeze the reference until the vehicle catches up (metres).
    max_tracking_error: f32 = 0.5,

    pub fn validate(options: PathOptions) !void {
        try options.move.validate();
        if (!nonnegative(options.corner_radius) or !positive(options.reference_speed) or
            !positive(options.max_tracking_error)) return error.InvalidPathOptions;
    }
};

fn positive(value: f32) bool {
    return std.math.isFinite(value) and value > 0;
}

fn nonnegative(value: f32) bool {
    return std.math.isFinite(value) and value >= 0;
}

fn effortLimit(value: f32) bool {
    return positive(value) and value <= 1;
}

fn finitePoint(point: math.Vector3f) bool {
    return std.math.isFinite(point[0]) and std.math.isFinite(point[1]) and std.math.isFinite(point[2]);
}

pub fn yawForDirection(direction: math.Vector3f, fallback: f32) f32 {
    if (math.length2f(math.xy(direction)) <= 0.05) return fallback;
    return math.wrapAngle(std.math.atan2(direction[1], direction[0]) - std.math.pi / 2.0);
}

pub fn slewYaw(previous: f32, desired: f32, rate_limit: f32, dt: f32) f32 {
    const limit = rate_limit * std.math.clamp(dt, 0.0, 0.1);
    return math.wrapAngle(previous + std.math.clamp(math.wrapAngle(desired - previous), -limit, limit));
}

pub const Failure = enum { object_lost, tracking_lost, move_timeout, search_timeout, localization_changed, rejoin_too_far, sensor_unavailable };
pub const Mode = enum { world, estimated, paused, search, visual, body, confirming, failed };

pub const Route = struct {
    path: Path,
    distance: f32 = 0,
    reference_speed: f32,
    max_tracking_error: f32,
};

pub const Command = union(enum) {
    pose,
    hold,
    route: Route,
    visual: VisualOptions,
    search: SearchOptions,
    body: BodyOptions,
};

/// One state per command/route: holding yaw is captured once, and changing the
/// path reference never recaptures it or restarts the yaw ramp.
pub const State = struct {
    options: MoveOptions,
    command: Command = .pose,
    mode: Mode = .world,
    elapsed: f64 = 0,
    active_seconds: f64 = 0,
    completed: bool = false,
    settling: Settling = .{},
    recovery_seconds: f64 = 0,
    tracking_confirm_seconds: f32 = 0,
    recovering: bool = false,
    recovery_position: ?math.Vector3f = null,
    recovery_yaw: f32 = 0,
    last_recovery_yaw: ?f32 = null,
    last_recovery_orientation: ?math.Quaternionf = null,
    resume_scale: f32 = 1,
    pressure_goal: ?f32 = null,
    destination: math.Vector3f,
    path_direction: math.Vector3f = @splat(0),
    captured_yaw: f32,
    desired_yaw: f32,
    commanded_yaw: f32,
    last_object_position: ?math.Vector3f = null,
    missing_object_seconds: f64 = 0,
    missing_feedback_seconds: f64 = 0,
    paused: bool = false,
    operator_paused: bool = false,
    failure: ?Failure = null,

    pub fn init(destination: math.Vector3f, yaw: f32, options: MoveOptions) State {
        return .{
            .options = options,
            .destination = destination,
            .captured_yaw = yaw,
            .desired_yaw = yaw,
            .commanded_yaw = yaw,
        };
    }

    /// Resolve facing only. Sensor health and recovery belong to the executor.
    pub fn update(state: *State, position: math.Vector3f, object_position: ?math.Vector3f, position_ok: bool, dt: f32) void {
        const elapsed = @max(0, dt);
        state.failure = null;
        state.paused = false;
        if (!position_ok) {
            state.paused = true;
            state.desired_yaw = switch (state.options.facing) {
                .hold_current => state.captured_yaw,
                .fixed_yaw => |yaw| math.wrapAngle(yaw),
                .face_path => yawForDirection(state.path_direction, state.desired_yaw),
                else => state.commanded_yaw,
            };
            state.commanded_yaw = slewYaw(state.commanded_yaw, state.desired_yaw, state.options.yaw_rate_limit, dt);
            return;
        }

        state.desired_yaw = switch (state.options.facing) {
            .hold_current => state.captured_yaw,
            .fixed_yaw => |yaw| math.wrapAngle(yaw),
            .face_goal => yawForDirection(state.destination - position, state.desired_yaw),
            .face_path => blk: {
                // Tangent magnitude depends on segment length, not distance to
                // a target. Normalize it before applying the XY deadband.
                const length = math.length3f(state.path_direction);
                const direction = if (length > 1e-6)
                    state.path_direction / @as(math.Vector3f, @splat(length))
                else
                    @as(math.Vector3f, @splat(0));
                break :blk yawForDirection(direction, state.desired_yaw);
            },
            .look_at_point => |point| yawForDirection(point - position, state.desired_yaw),
            .look_at_object => blk: {
                if (object_position) |point| {
                    state.last_object_position = point;
                    state.missing_object_seconds = 0;
                } else {
                    state.missing_object_seconds += elapsed;
                }
                if (state.missing_object_seconds >= state.options.observation_timeout) {
                    state.paused = true;
                    state.failure = .object_lost;
                    // Hold the last commanded yaw after the cached target expires.
                    break :blk state.commanded_yaw;
                }
                const point = state.last_object_position orelse {
                    state.paused = true;
                    break :blk state.commanded_yaw;
                };
                break :blk yawForDirection(point - position, state.desired_yaw);
            },
        };
        state.commanded_yaw = slewYaw(state.commanded_yaw, state.desired_yaw, state.options.yaw_rate_limit, dt);
    }

    pub fn translationScale(state: *const State, current_yaw: f32) f32 {
        if (state.paused) return 0;
        if (state.options.translation == .simultaneous) return 1;
        // Use the actual desired heading, not the slowly ramping command, so
        // align_first cannot begin translating before it has really aligned.
        const error_angle = @abs(math.wrapAngle(state.desired_yaw - current_yaw));
        const gate = state.options.alignment_gate;
        const blend = std.math.clamp((gate - error_angle) / (gate * 0.5), 0, 1);
        return blend * blend * (3.0 - 2.0 * blend);
    }

    pub fn yawReached(state: *const State, current_yaw: f32) bool {
        return !state.options.require_yaw or
            @abs(math.wrapAngle(state.desired_yaw - current_yaw)) <= state.options.yaw_tolerance;
    }
};

pub const Settling = struct {
    seconds: f32 = 0,

    pub fn update(settling: *Settling, position_reached: bool, yaw_reached: bool, dt: f32, required: f32) bool {
        if (!position_reached or !yaw_reached) {
            settling.seconds = 0;
            return false;
        }
        settling.seconds += @max(0, dt);
        return settling.seconds >= required;
    }
};

/// Allocation-free route made of lines and quadratic Bezier corners. The line
/// and curve tangents agree at joins. Arc lengths are sampled to advance the
/// reference at approximately constant speed rather than constant curve t.
pub const Path = struct {
    pub const max_waypoints = 64;
    const arc_samples = 32;
    const Segment = struct {
        start: math.Vector3f,
        control: ?math.Vector3f,
        end: math.Vector3f,
        arc: [arc_samples + 1]f32,

        fn point(segment: Segment, t: f32) math.Vector3f {
            const u = 1.0 - t;
            if (segment.control) |control|
                return segment.start * @as(math.Vector3f, @splat(u * u)) +
                    control * @as(math.Vector3f, @splat(2.0 * u * t)) +
                    segment.end * @as(math.Vector3f, @splat(t * t));
            return segment.start + (segment.end - segment.start) * @as(math.Vector3f, @splat(t));
        }

        fn tangent(segment: Segment, t: f32) math.Vector3f {
            if (segment.control) |control|
                return (control - segment.start) * @as(math.Vector3f, @splat(2.0 * (1.0 - t))) +
                    (segment.end - control) * @as(math.Vector3f, @splat(2.0 * t));
            return segment.end - segment.start;
        }

        fn length(segment: Segment) f32 {
            return segment.arc[arc_samples];
        }
    };

    segments: [max_waypoints * 2]Segment = undefined,
    len: usize = 0,
    total_length: f32 = 0,
    end: math.Vector3f,

    pub fn init(start: math.Vector3f, waypoints: []const math.Vector3f, corner_radius: f32) !Path {
        if (waypoints.len == 0 or waypoints.len > max_waypoints or !finitePoint(start) or !nonnegative(corner_radius))
            return error.InvalidPath;
        var vertices: [max_waypoints + 1]math.Vector3f = undefined;
        vertices[0] = start;
        var count: usize = 1;
        for (waypoints) |point| {
            if (!finitePoint(point)) return error.InvalidPath;
            if (math.length3f(point - vertices[count - 1]) <= 1e-5) continue;
            vertices[count] = point;
            count += 1;
        }
        var path: Path = .{ .end = vertices[count - 1] };
        var cursor = start;
        for (1..count) |i| {
            const vertex = vertices[i];
            if (i + 1 < count and corner_radius > 0) {
                const incoming = vertex - vertices[i - 1];
                const outgoing = vertices[i + 1] - vertex;
                const trim = @min(corner_radius, @min(math.length3f(incoming), math.length3f(outgoing)) * 0.45);
                const entry = vertex - math.normalize3f(incoming) * @as(math.Vector3f, @splat(trim));
                const exit = vertex + math.normalize3f(outgoing) * @as(math.Vector3f, @splat(trim));
                path.addSegment(cursor, null, entry);
                path.addSegment(entry, vertex, exit);
                cursor = exit;
            } else {
                path.addSegment(cursor, null, vertex);
                cursor = vertex;
            }
        }
        return path;
    }

    fn addSegment(path: *Path, start: math.Vector3f, control: ?math.Vector3f, end: math.Vector3f) void {
        if (control == null and math.length3f(end - start) <= 1e-5) return;
        var segment: Segment = .{ .start = start, .control = control, .end = end, .arc = undefined };
        segment.arc[0] = 0;
        var previous = start;
        for (1..arc_samples + 1) |i| {
            const point = segment.point(@as(f32, @floatFromInt(i)) / arc_samples);
            segment.arc[i] = segment.arc[i - 1] + math.length3f(point - previous);
            previous = point;
        }
        path.segments[path.len] = segment;
        path.len += 1;
        path.total_length += segment.length();
    }

    pub const Sample = struct { position: math.Vector3f, direction: math.Vector3f };

    pub fn sample(path: *const Path, distance: f32) Sample {
        if (path.len == 0) return .{ .position = path.end, .direction = @splat(0) };
        var remaining = std.math.clamp(distance, 0, path.total_length);
        for (path.segments[0..path.len], 0..) |segment, index| {
            if (remaining > segment.length() and index + 1 < path.len) {
                remaining -= segment.length();
                continue;
            }
            for (1..arc_samples + 1) |i| {
                if (remaining > segment.arc[i] and i < arc_samples) continue;
                const span = segment.arc[i] - segment.arc[i - 1];
                const fraction = if (span > 1e-6) (remaining - segment.arc[i - 1]) / span else 0;
                const t = (@as(f32, @floatFromInt(i - 1)) + std.math.clamp(fraction, 0, 1)) / arc_samples;
                return .{ .position = segment.point(t), .direction = segment.tangent(t) };
            }
        }
        unreachable;
    }

    /// Project only near saved progress, so crossing routes cannot jump branches.
    pub fn rejoin(path: *const Path, position: math.Vector3f, distance: f32, window: f32) f32 {
        const start = @max(0, distance - window);
        const end = @min(path.total_length, distance + window);
        var best_distance = distance;
        var best_error = math.length3f(path.sample(distance).position - position);
        for (0..65) |i| {
            const candidate = start + (end - start) * @as(f32, @floatFromInt(i)) / 64.0;
            const err = math.length3f(path.sample(candidate).position - position);
            if (err < best_error) {
                best_error = err;
                best_distance = candidate;
            }
        }
        return best_distance;
    }
};

test "yaw presets use +Y forward and retain heading for zero XY directions" {
    try std.testing.expectApproxEqAbs(@as(f32, 0), yawForDirection(.{ 0, 1, 0 }, 1), 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, -std.math.pi / 2.0), yawForDirection(.{ 1, 0, 0 }, 1), 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 1), yawForDirection(.{ 0, 0, 3 }, 1), 1e-5);
}

test "hold_current survives changing path references and position" {
    var state = State.init(.{ 5, 0, 2 }, 0.7, .{ .facing = .hold_current });
    state.update(.{ 0, 0, 0 }, null, true, 0.05);
    state.path_direction = .{ -3, 4, 0 };
    state.update(.{ 4, 0, 2 }, null, true, 0.05);
    try std.testing.expectApproxEqAbs(@as(f32, 0.7), state.desired_yaw, 1e-5);
}

test "look_at_point changes bearing while strafing and face_path uses tangent" {
    var state = State.init(.{ 5, 0, 0 }, 0, .{ .facing = .{ .look_at_point = .{ 0, 2, 0 } } });
    state.update(.{ 0, 0, 0 }, null, true, 0.05);
    try std.testing.expectApproxEqAbs(@as(f32, 0), state.desired_yaw, 1e-5);
    state.update(.{ 2, 2, 0 }, null, true, 0.05);
    try std.testing.expectApproxEqAbs(@as(f32, std.math.pi / 2.0), state.desired_yaw, 1e-5);
    state.options.facing = .face_path;
    state.path_direction = .{ 0.001, 0, 0 };
    state.update(.{ 2, 2, 0 }, null, true, 0.05);
    try std.testing.expectApproxEqAbs(@as(f32, -std.math.pi / 2.0), state.desired_yaw, 1e-5);
}

test "yaw ramps by the shortest arc across pi" {
    const deg = std.math.pi / 180.0;
    const result = slewYaw(179 * deg, -179 * deg, 1, 0.01);
    try std.testing.expectApproxEqAbs(@as(f32, 0.01), math.wrapAngle(result - 179 * deg), 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 0.1), slewYaw(0, 2, 1, 10), 1e-5);
}

test "gate heading stays fixed after crossing and face_goal freezes at arrival" {
    var crossing = State.init(.{ 2, 0, 1 }, 0, .{ .facing = .{ .fixed_yaw = -std.math.pi / 2.0 } });
    crossing.update(.{ -2, 0, 1 }, null, true, 0.05);
    const heading = crossing.desired_yaw;
    crossing.update(.{ 1, 0, 1 }, null, true, 0.05);
    try std.testing.expectEqual(heading, crossing.desired_yaw);

    var approach = State.init(.{ 2, 0, 1 }, 0, .{ .facing = .face_goal });
    approach.update(.{ 0, 0, 1 }, null, true, 0.05);
    approach.update(.{ 2, 0, 1 }, null, true, 0.05);
    try std.testing.expectApproxEqAbs(@as(f32, -std.math.pi / 2.0), approach.desired_yaw, 1e-5);
}

test "align_first gates actual heading while simultaneous strafes" {
    var state = State.init(.{ 5, 0, 0 }, 0, .{ .facing = .face_goal, .translation = .align_first });
    state.update(.{ 0, 0, 0 }, null, true, 0.05);
    try std.testing.expectEqual(@as(f32, 0), state.translationScale(0));
    try std.testing.expectEqual(@as(f32, 1), state.translationScale(-std.math.pi / 2.0));
    const partially_aligned = state.desired_yaw + state.options.alignment_gate * 0.75;
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), state.translationScale(partially_aligned), 1e-5);
    state.options.translation = .simultaneous;
    try std.testing.expectEqual(@as(f32, 1), state.translationScale(0));
}

test "object cache expires, pauses and recovers using a fresh observation" {
    var state = State.init(.{ 5, 0, 0 }, 0, .{ .facing = .{ .look_at_object = 7 }, .observation_timeout = 0.5 });
    state.update(.{ 0, 0, 0 }, null, true, 0.1);
    try std.testing.expect(state.paused);
    state.update(.{ 0, 0, 0 }, .{ 1, 0, 0 }, true, 0.1);
    state.update(.{ 0, 0, 0 }, null, true, 0.3);
    try std.testing.expect(!state.paused);
    state.update(.{ 0, 0, 0 }, null, true, 0.21);
    try std.testing.expectEqual(Failure.object_lost, state.failure.?);
    try std.testing.expectEqual(@as(f32, 0), state.translationScale(0));
    state.update(.{ 0, 0, 0 }, .{ 0, 1, 0 }, true, 0.1);
    try std.testing.expectEqual(@as(?Failure, null), state.failure);
    try std.testing.expect(!state.paused);
}

test "unavailable position freezes a goal-dependent heading without executor failures" {
    var state = State.init(.{ 5, 0, 0 }, 0.7, .{ .facing = .face_goal, .observation_timeout = 0.5 });
    state.update(.{ 0, 0, 0 }, null, false, 0.6);
    try std.testing.expect(state.paused);
    try std.testing.expectEqual(@as(?Failure, null), state.failure);
    try std.testing.expectApproxEqAbs(@as(f32, 0.7), state.commanded_yaw, 1e-6);
    state.update(.{ 0, 0, 0 }, null, true, 0.05);
    try std.testing.expect(!state.paused);
}

test "settling requires position and desired yaw continuously" {
    var settling: Settling = .{};
    try std.testing.expect(!settling.update(true, false, 0.2, 0.3));
    try std.testing.expect(!settling.update(true, true, 0.2, 0.3));
    try std.testing.expect(!settling.update(false, true, 0.2, 0.3));
    try std.testing.expect(!settling.update(true, true, 0.2, 0.3));
    try std.testing.expect(settling.update(true, true, 0.11, 0.3));
}

test "rounded route has continuous tangents and bounded reference steps" {
    const path = try Path.init(.{ 0, 0, 0 }, &.{ .{ 4, 0, 0 }, .{ 4, 4, 0 }, .{ 4, 4, 2 } }, 1);
    try std.testing.expect(path.len == 5);
    for (1..path.len) |i| {
        const before = path.segments[i - 1];
        const after = path.segments[i];
        try std.testing.expect(math.length3f(before.end - after.start) < 1e-5);
        const a = math.normalize3f(before.tangent(1));
        const b = math.normalize3f(after.tangent(0));
        try std.testing.expect(math.dot3f(a, b) > 0.999);
    }
    var previous = path.sample(0).position;
    var distance: f32 = 0.05;
    while (distance < path.total_length) : (distance += 0.05) {
        const sample = path.sample(distance);
        try std.testing.expect(math.length3f(sample.position - previous) < 0.051);
        previous = sample.position;
    }
    try std.testing.expect(math.length3f(path.sample(path.total_length).position - path.end) < 1e-5);
}

test "rounded orbit stays outside object clearance and closes before returning" {
    const path = try Path.init(.{ 5, 0, 0 }, &.{ .{ 3, 3, 0 }, .{ 3, -3, 0 }, .{ -3, -3, 0 }, .{ -3, 3, 0 }, .{ 3, 3, 0 }, .{ 5, 0, 0 } }, 0.6);
    var distance: f32 = 0;
    while (distance <= path.total_length) : (distance += 0.02) {
        const point = path.sample(distance).position;
        // Object occupies [-1, 1]^2. Distance to its boundary stays >= 2m.
        const delta: math.Vector2f = .{ @max(0, @abs(point[0]) - 1), @max(0, @abs(point[1]) - 1) };
        try std.testing.expect(math.length2f(delta) >= 1.99);
    }
}

test "duplicate and zero length routes are finite and invalid inputs rejected" {
    const path = try Path.init(.{ 0, 0, 0 }, &.{ .{ 0, 0, 0 }, .{ 0, 0, 2 }, .{ 0, 0, 2 } }, 0.5);
    try std.testing.expectApproxEqAbs(@as(f32, 2), path.total_length, 1e-5);
    const stationary = try Path.init(.{ 1, 2, 3 }, &.{.{ 1, 2, 3 }}, 0.5);
    try std.testing.expect(math.length3f(stationary.sample(0).position - stationary.end) == 0);
    try std.testing.expectError(error.InvalidPath, Path.init(.{ 0, 0, 0 }, &.{}, 0.5));
    try std.testing.expectError(error.InvalidPath, Path.init(.{ 0, 0, 0 }, &.{.{ 1, 2, 3 }}, -1));
    try std.testing.expectError(error.InvalidMoveOptions, (MoveOptions{ .facing = .hold_current, .yaw_rate_limit = 0 }).validate());
}
