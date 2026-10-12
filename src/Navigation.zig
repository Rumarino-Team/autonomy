const std = @import("std");
const math = @import("math.zig");
const Auv = @import("Auv.zig");
const Navigation = @This();

pub const Options = struct {
    max_sensor_age: f32 = 0.25,
    /// Rotation of IMU vectors into vehicle body axes.
    body_from_imu: math.Quaternionf = .{ 0, 0, 0, 1 },
    /// Camera origin/orientation in body axes; identity for a body-pose backend.
    body_from_camera: math.Pose = .{ .pos = @splat(0), .quat = .{ 0, 0, 0, 1 } },
    prediction_duration: f32 = 1.0,
    prediction_error_limit: f32 = 0.5,
    /// Conservative error envelope, not an estimator covariance.
    prediction_error_growth: f32 = 0.5,
    max_prediction_speed: f32 = 2.0,

    pub fn validate(options: Options) !void {
        if (!positive(options.max_sensor_age) or !validQuat(options.body_from_imu) or
            !validPoint(options.body_from_camera.pos) or !validQuat(options.body_from_camera.quat) or
            !positive(options.prediction_duration) or !positive(options.prediction_error_limit) or
            !positive(options.prediction_error_growth) or !positive(options.max_prediction_speed))
            return error.InvalidNavigationOptions;
    }
};

pub const Snapshot = struct {
    position: ?math.Vector3f = null,
    orientation: ?math.Quaternionf = null,
    angular_velocity_body: ?math.Vector3f = null,
    angular_velocity_from_gyro: bool = false,
    velocity_world: ?math.Vector3f = null,
    depth_rate: ?f32 = null,
    measured_position: bool = false,
    estimated_position: bool = false,
    position_error: f32 = 0,
    pressure_depth: ?f32 = null,
    observations_valid: bool = false,
    heading_aligned: bool = false,
    localization_changed: bool = false,

    pub fn yaw(snapshot: Snapshot) ?f32 {
        return if (snapshot.orientation) |q| math.quaternionToEuler(q)[2] else null;
    }
};

snapshot: Snapshot = .{},
last_frame_timestamp: ?u64 = null,
epoch: ?u32 = null,
tracking_to_mission: math.Pose = .{ .pos = @splat(0), .quat = .{ 0, 0, 0, 1 } },
mission_from_imu_world: ?math.Quaternionf = null,
pressure_to_world_offset: ?f32 = null,
last_measured_position: ?math.Vector3f = null,
last_position_timestamp: ?u64 = null,
velocity: math.Vector3f = @splat(0),
measured_velocity: ?math.Vector3f = null,
predicted_position: ?math.Vector3f = null,
outage_seconds: f32 = 0,
last_orientation: ?math.Quaternionf = null,
last_orientation_timestamp: ?u64 = null,
derived_angular_velocity: ?math.Vector3f = null,
last_pressure: ?f32 = null,
last_pressure_timestamp: ?u64 = null,
pressure_rate: ?f32 = null,

pub fn fresh(now: u64, timestamp: u64, max_age: f32) bool {
    if (timestamp == 0 or timestamp > now) return false;
    return @as(f32, @floatFromInt(now - timestamp)) * 1e-9 <= max_age;
}

pub fn validPoint(point: math.Vector3f) bool {
    return std.math.isFinite(point[0]) and std.math.isFinite(point[1]) and std.math.isFinite(point[2]);
}

pub fn validQuat(q: math.Quaternionf) bool {
    var norm: f32 = 0;
    inline for (0..4) |i| {
        if (!std.math.isFinite(q[i])) return false;
        norm += q[i] * q[i];
    }
    return std.math.isFinite(norm) and norm > 1e-6;
}

fn positive(value: f32) bool {
    return std.math.isFinite(value) and value > 0;
}

pub fn toMission(nav: *const Navigation, position: math.Vector3f) math.Vector3f {
    return nav.tracking_to_mission.pos + math.quaternionRotate(nav.tracking_to_mission.quat, position);
}

/// Explicitly reconcile a localization-frame reset before resuming its plan.
pub fn reanchor(nav: *Navigation, tracking_to_mission: math.Pose, epoch: u32) !void {
    if (!validPoint(tracking_to_mission.pos) or !validQuat(tracking_to_mission.quat))
        return error.InvalidNavigationTransform;
    nav.tracking_to_mission = .{ .pos = tracking_to_mission.pos, .quat = math.normalize4f(tracking_to_mission.quat) };
    nav.epoch = epoch;
    nav.last_frame_timestamp = null;
    nav.last_measured_position = null;
    nav.last_position_timestamp = null;
    nav.predicted_position = null;
    nav.velocity = @splat(0);
    nav.measured_velocity = null;
    nav.snapshot = .{};
    nav.last_orientation = null;
    nav.last_orientation_timestamp = null;
    nav.derived_angular_velocity = null;
    nav.last_pressure = null;
    nav.last_pressure_timestamp = null;
    nav.pressure_rate = null;
}

pub fn update(nav: *Navigation, frame: *const Auv.Frame, options: Options, elapsed: f32) void {
    const increasing = if (nav.last_frame_timestamp) |last| frame.timestamp > last else frame.timestamp > 0;
    if (increasing) nav.last_frame_timestamp = frame.timestamp;
    nav.snapshot = .{};
    if (nav.epoch == null) nav.epoch = frame.localization_epoch;
    nav.snapshot.localization_changed = nav.epoch.? != frame.localization_epoch;
    const position_ok = increasing and !nav.snapshot.localization_changed and frame.tracking_ok and
        fresh(frame.timestamp, frame.position_timestamp, options.max_sensor_age) and
        validPoint(frame.camera_pose.pos) and validQuat(frame.camera_pose.quat);
    const imu_ok = increasing and frame.imu_ok and
        fresh(frame.timestamp, frame.imu_timestamp, options.max_sensor_age) and validQuat(frame.imu_quat);
    const pressure_ok = increasing and frame.pressure_depth_ok and
        fresh(frame.timestamp, frame.pressure_timestamp, options.max_sensor_age) and std.math.isFinite(frame.pressure_depth);
    nav.snapshot.observations_valid = increasing and
        fresh(frame.timestamp, frame.observations_timestamp, options.max_sensor_age);
    if (pressure_ok) {
        nav.snapshot.pressure_depth = frame.pressure_depth;
        if (nav.last_pressure_timestamp) |previous| {
            if (frame.pressure_timestamp > previous) {
                const dt = @as(f32, @floatFromInt(frame.pressure_timestamp - previous)) * 1e-9;
                nav.pressure_rate = if (dt <= options.max_sensor_age) (frame.pressure_depth - nav.last_pressure.?) / dt else null;
            }
        }
        nav.last_pressure = frame.pressure_depth;
        nav.last_pressure_timestamp = frame.pressure_timestamp;
        nav.snapshot.depth_rate = nav.pressure_rate;
    }
    const gyro_ok = increasing and frame.gyro_ok and fresh(frame.timestamp, frame.imu_timestamp, options.max_sensor_age) and validPoint(frame.gyro);
    if (gyro_ok) {
        nav.snapshot.angular_velocity_body = math.quaternionRotate(math.normalize4f(options.body_from_imu), frame.gyro);
        nav.snapshot.angular_velocity_from_gyro = true;
    }

    const imu_body = if (imu_ok)
        math.normalize4f(math.quaternionMul(math.normalize4f(frame.imu_quat), math.quaternionConjugate(math.normalize4f(options.body_from_imu))))
    else
        null;
    if (position_ok) {
        const camera_orientation = math.normalize4f(math.quaternionMul(nav.tracking_to_mission.quat, math.normalize4f(frame.camera_pose.quat)));
        const orientation = math.normalize4f(math.quaternionMul(camera_orientation, math.quaternionConjugate(math.normalize4f(options.body_from_camera.quat))));
        const position = nav.toMission(frame.camera_pose.pos) - math.quaternionRotate(orientation, options.body_from_camera.pos);
        if (imu_body) |q| nav.mission_from_imu_world = math.quaternionMul(orientation, math.quaternionConjugate(q));
        if (pressure_ok) nav.pressure_to_world_offset = position[2] - frame.pressure_depth;
        if (nav.last_measured_position) |last| {
            const previous = nav.last_position_timestamp.?;
            if (frame.position_timestamp > previous) {
                nav.measured_velocity = null;
                if (nav.outage_seconds == 0) {
                    const dt = @as(f32, @floatFromInt(frame.position_timestamp - previous)) * 1e-9;
                    const velocity = (position - last) / @as(math.Vector3f, @splat(dt));
                    const speed = math.length3f(velocity);
                    nav.velocity = if (speed <= options.max_prediction_speed) velocity else @splat(0);
                    if (dt <= options.max_sensor_age and validPoint(velocity)) nav.measured_velocity = velocity;
                } else nav.velocity = @splat(0);
            }
        }
        nav.last_measured_position = position;
        nav.last_position_timestamp = frame.position_timestamp;
        nav.predicted_position = position;
        nav.outage_seconds = 0;
        nav.snapshot.position = position;
        nav.snapshot.orientation = orientation;
        nav.snapshot.measured_position = true;
        nav.snapshot.heading_aligned = true;
        nav.snapshot.velocity_world = nav.measured_velocity;
    } else {
        nav.outage_seconds += @max(0, elapsed);
        if (imu_body) |q| {
            nav.snapshot.orientation = math.normalize4f(math.quaternionMul(nav.mission_from_imu_world orelse @as(math.Quaternionf, .{ 0, 0, 0, 1 }), q));
            nav.snapshot.heading_aligned = nav.mission_from_imu_world != null;
        }
        const error_bound = 0.03 + options.prediction_error_growth * nav.outage_seconds;
        if (increasing and nav.predicted_position != null and nav.snapshot.heading_aligned and
            !nav.snapshot.localization_changed and nav.outage_seconds <= options.prediction_duration and
            error_bound <= options.prediction_error_limit and elapsed <= options.max_sensor_age)
        {
            nav.predicted_position.? += nav.velocity * @as(math.Vector3f, @splat(@max(0, elapsed)));
            if (nav.snapshot.pressure_depth) |depth| {
                if (nav.pressure_to_world_offset) |offset| nav.predicted_position.?[2] = depth + offset;
            }
            nav.snapshot.position = nav.predicted_position;
            nav.snapshot.estimated_position = true;
            nav.snapshot.position_error = error_bound;
            nav.snapshot.velocity_world = nav.velocity;
        }
    }
    // Camera-only backends can supply a rate estimate from successive fresh
    // orientations. Gyro feedback always takes precedence. Never differentiate
    // a localization reset or repeated observation into an artificial spike.
    if (!nav.snapshot.localization_changed) {
        if (nav.snapshot.orientation) |orientation| {
            const timestamp = if (position_ok) frame.position_timestamp else frame.imu_timestamp;
            if (nav.last_orientation_timestamp) |previous| {
                if (timestamp > previous) {
                    const dt = @as(f32, @floatFromInt(timestamp - previous)) * 1e-9;
                    nav.derived_angular_velocity = if (dt <= options.max_sensor_age)
                        math.quaternionError(nav.last_orientation.?, orientation) / @as(math.Vector3f, @splat(dt))
                    else
                        null;
                }
            }
            nav.last_orientation = orientation;
            nav.last_orientation_timestamp = timestamp;
            if (!gyro_ok) nav.snapshot.angular_velocity_body = nav.derived_angular_velocity;
        }
    } else {
        nav.last_orientation = null;
        nav.last_orientation_timestamp = null;
        nav.derived_angular_velocity = null;
    }
}
