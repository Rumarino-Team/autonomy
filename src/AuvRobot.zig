//! AUV control targets, PID history, thruster allocation, and DNA wire documents.
const std = @import("std");
const Auv = @import("Auv.zig");
const MissionConfig = @import("MissionConfig.zig");
const MissionIO = @import("MissionIO.zig");
const math = @import("math.zig");
const bc = @import("BodyControl.zig");
const reflection = @import("simulator/ReflectionWriter.zig");
const AuvRobot = @This();

pub const Observations = Auv.Frame;

pub const ActuatorCommand = struct {
    values: [MissionConfig.thruster_count]f32 = @splat(0),
    len: u8 = 0,
};

state: State = .{},

pub const State = struct {
    seen_objects: [Auv.Frame.max_objects]Auv.Object = undefined,
    seen_objects_len: u8 = 0,
    target_position: math.Vector3f = @splat(0),
    target_orientation: math.Quaternionf = bc.identity,
    use_position: bool = false,
    use_orientation: bool = false,
    position_axes: Auv.AxisOk = .{ .x = true, .y = true, .z = true },
    desired: math.Vector2f = .{ 0, 0 },
    teleop_translation: ?math.Vector3f = null,
    pid_sum_err: math.Vector6f = @splat(0),
    pid_prev_err: math.Vector6f = @splat(0),
    tilt_correction: math.Vector3f = .{ 0, 0, 0 },
    force_error: math.Vector3f = .{ 0, 0, 0 },
    translation_error: math.Vector3f = @splat(0),
    attitude_error: math.Vector3f = @splat(0),
    target_quaternion: math.Quaternionf = bc.identity,
    control_pose: math.Vector6f = @splat(0),
    command_input: math.Vector6f = @splat(0),
    dt: f64 = 0,
    has_dt: bool = false,
    pid_prev_timestamp_ns: ?u64 = null,
    seen_objects2d: [Auv.Frame.max_objects]Auv.Object2d = undefined,
    seen_objects2d_len: u8 = 0,
};

/// Control / telemetry AUTODNA documents derived from `State` and `Auv.Frame`.
pub const Wire = struct {
    const Goal = MissionIO.Goal;
    const sensor_fields = [_][]const u8{
        "objects_len",  "camera_pose",    "timestamp",         "position_ok",   "attitude_ok",
        "accel",        "gyro",           "imu_quat",          "objects2d_len", "image_width",
        "image_height", "pressure_depth", "pressure_depth_ok", "frame_error",
    };

    pub const teleop_fields = [_][]const u8{ "teleop_translation", "target_orientation" };
    pub const TeleopWire = reflection.projectRow(State, &teleop_fields);
    pub const TeleopDoc = struct { command: TeleopWire };
    pub const teleop_schema_hash: u64 = reflection.documentSchemaHash(TeleopDoc);

    pub const ThrusterDoc = struct {
        len: u8 = 0,
        values: [MissionConfig.thruster_count]f32 = @splat(0),
    };
    pub const thruster_schema_hash: u64 = reflection.documentSchemaHash(ThrusterDoc);

    pub const TelemetryDoc = struct {
        pub const SensorWire = reflection.projectRow(Auv.Frame, &sensor_fields);

        robot: State,
        sensors: SensorWire,
        odometry: MissionConfig.OdometryPid,
        thrusters: [MissionConfig.thruster_count]f32 = @splat(0),
        thrusters_len: u8 = 0,
        goals: [MissionConfig.queue_capacity_max]Goal = undefined,
        goals_len: u8 = 0,
        step: [64]u8 = @splat(0),
        step_len: u8 = 0,
        tam: [MissionConfig.thruster_count]math.Vector6f = undefined,
        tam_len: u8 = 0,
        clear_screen: bool = false,
    };

    pub const telemetry_schema_hash: u64 = reflection.documentSchemaHash(TelemetryDoc);

    pub fn emptyTelemetry() TelemetryDoc {
        var sensors = std.mem.zeroes(TelemetryDoc.SensorWire);
        sensors.camera_pose.quat[3] = 1;
        return .{
            .robot = std.mem.zeroes(State),
            .sensors = sensors,
            .odometry = std.mem.zeroes(MissionConfig.OdometryPid),
        };
    }

    pub fn packGoal(position: math.Vector3f, look_at: math.Vector3f) Goal {
        return .{ .position = position, .look_at = look_at };
    }

    pub const SnapshotBuffers = struct {
        clear_screen: bool = false,

        /// The returned document borrows scratch storage and the step and thruster slices.
        pub fn capture(
            self: *SnapshotBuffers,
            robot: *const AuvRobot,
            frame: *const Auv.Frame,
            config: *const MissionConfig.MissionConfig,
            goals: *const std.Deque(Goal),
            step: []const u8,
            thrusters: []const f32,
        ) TelemetryDoc {
            const state = &robot.state;
            var doc: TelemetryDoc = .{
                .robot = state.*,
                .sensors = undefined,
                .odometry = config.odometry,
            };

            var goals_len: usize = 0;
            var goal_it = goals.iterator();
            while (goal_it.next()) |goal| {
                if (goals_len == doc.goals.len) break;
                doc.goals[goals_len] = packGoal(goal.position, goal.look_at);
                goals_len += 1;
            }
            if (goals_len == 0 and state.use_position and frame.position_ok.x and frame.position_ok.y and frame.position_ok.z) {
                var look_at = state.target_position;
                if (state.use_orientation) {
                    const q = bc.normalizedQuat(state.target_orientation) catch bc.identity;
                    const ray = math.quaternionRotate(q, config.camera_forward);
                    look_at = frame.camera_pose.pos + ray * @as(math.Vector3f, @splat(2));
                }
                doc.goals[0] = packGoal(state.target_position, look_at);
                goals_len = 1;
            }

            const tam = doc.tam[0..config.tam_len];
            @memcpy(tam, config.tam[0..tam.len]);
            const step_len = @min(step.len, doc.step.len);
            @memcpy(doc.step[0..step_len], step[0..step_len]);
            const thrust_len = @min(thrusters.len, doc.thrusters.len);
            @memcpy(doc.thrusters[0..thrust_len], thrusters[0..thrust_len]);

            // Direct thruster control keeps the clear request latched for the run.
            self.clear_screen = config.clear_print_stuff or (config.mission_id == .thruster_control and self.clear_screen);
            doc.clear_screen = self.clear_screen;
            doc.sensors = reflection.copyProject(Auv.Frame, &sensor_fields, frame);
            doc.thrusters_len = @intCast(thrust_len);
            doc.goals_len = @intCast(goals_len);
            doc.step_len = @intCast(step_len);
            doc.tam_len = @intCast(tam.len);
            return doc;
        }
    };
};

pub fn reset(self: *AuvRobot) void {
    self.state = .{};
}

/// Keep existing slots stable so mission helpers can hold a pointer to a detection.
pub fn observe(self: *AuvRobot, frame: *const Auv.Frame) void {
    const state = &self.state;
    for (frame.objects[0..frame.objects_len]) |object| {
        for (state.seen_objects[0..state.seen_objects_len]) |*seen| {
            if (seen.id == object.id) {
                seen.* = object;
                break;
            }
        } else if (state.seen_objects_len < state.seen_objects.len) {
            state.seen_objects[state.seen_objects_len] = object;
            state.seen_objects_len += 1;
        }
    }

    for (frame.objects2d[0..frame.objects2d_len]) |box| {
        for (state.seen_objects2d[0..state.seen_objects2d_len]) |*seen| {
            if (seen.id == box.id) {
                seen.* = box;
                break;
            }
        } else if (state.seen_objects2d_len < state.seen_objects2d.len) {
            state.seen_objects2d[state.seen_objects2d_len] = box;
            state.seen_objects2d_len += 1;
        }
    }
}

pub fn update(
    self: *AuvRobot,
    frame: *const Auv.Frame,
    config: *const MissionConfig.MissionConfig,
    elapsed: ?f64,
    out: *ActuatorCommand,
) void {
    const ctx = &self.state;
    const body_error = bodyError(ctx, frame, config);
    const dt = if (ctx.pid_prev_timestamp_ns != null) elapsed else null;
    ctx.pid_prev_timestamp_ns = frame.timestamp;

    const tam_len = config.tam_len;
    const tam_rows = config.tam[0..tam_len];
    const camera_forward = config.camera_forward;
    const rotation_limit = config.rotation_limit;
    const thrust_direction = bc.ThrustDirections.fromTamLinearForces(tam_rows) catch unreachable;
    const current = frame.camera_pose.quat;
    const error_body: math.Vector3f = .{ body_error[0], body_error[1], body_error[2] };
    const rot: math.Vector3f = .{ body_error[3], body_error[4], body_error[5] };
    const delta = quaternionFromRotationVector(rot);
    const current_q = bc.normalizedQuat(current) catch bc.identity;
    var target_q = bc.normalizedQuat(math.quaternionMul(current_q, delta)) catch current_q;
    const odom = config.odometry;
    var desired_tilt: math.Vector3f = .{ 0, 0, 0 };
    var force_error: math.Vector3f = .{ 0, 0, 0 };
    const moments = bc.ThrustDirections.fromTamMoments(tam_rows) catch unreachable;
    const translation_command: math.Vector3f = blk: {
        force_error = error_body;
        const available = thrust_direction.alongThrust(error_body);
        var force_body: math.Vector3f = .{ 0, 0, 0 };
        inline for (0..3) |i| {
            force_body[i] = odom.kp[i] * error_body[i];
            if (dt) |t| {
                ctx.pid_sum_err[i] += available[i] * t;
                const rate = (error_body[i] - ctx.pid_prev_err[i]) / t;
                force_body[i] += odom.ki[i] * ctx.pid_sum_err[i];
                force_body[i] += odom.kd[i] * rate;
            }
            ctx.pid_prev_err[i] = error_body[i];
        }
        const coverage = bc.coverForce(thrust_direction, moments, force_body, rotation_limit, camera_forward);
        if (coverage.tilt) |tilt| desired_tilt = tilt.axis * @as(math.Vector3f, @splat(tilt.angle));
        break :blk coverage.force;
    };
    const tilt_step = if (dt) |t| 0.8 * t else 0;
    ctx.tilt_correction = slew3(ctx.tilt_correction, desired_tilt, tilt_step);
    const tilt_len = math.length3f(ctx.tilt_correction);
    if (tilt_len > 1e-4) {
        const tilt_q = bc.forwardRotation(ctx.tilt_correction, tilt_len) catch bc.identity;
        target_q = bc.normalizedQuat(math.quaternionMul(target_q, tilt_q)) catch target_q;
    }
    var attitude_error = bc.attitudeError(current_q, target_q) catch math.Vector3f{ 0, 0, 0 };
    const attitude_axis = [3]bool{ frame.attitude_ok.x, frame.attitude_ok.y, frame.attitude_ok.z };
    inline for (0..3) |i| {
        if (!attitude_axis[i]) attitude_error[i] = 0;
    }
    const translation_body: math.Vector3f = blk: {
        var force = thrust_direction.alongThrust(translation_command);
        if (math.length3f(attitude_error) > odom.rotation_gate) {
            force[0] = 0;
            force[1] = 0;
        }
        break :blk force;
    };
    ctx.force_error = force_error;

    var rotation: math.Vector3f = .{ 0, 0, 0 };
    inline for (0..3) |i| {
        const lane = i + 3;
        if (attitude_axis[i]) {
            rotation[i] = odom.kp[lane] * attitude_error[i];
            if (dt) |t| {
                const rate = (attitude_error[i] - ctx.pid_prev_err[lane]) / t;
                ctx.pid_sum_err[lane] += attitude_error[i] * t;
                rotation[i] += odom.ki[lane] * ctx.pid_sum_err[lane];
                rotation[i] += odom.kd[lane] * rate;
            }
            ctx.pid_prev_err[lane] = attitude_error[i];
        } else {
            ctx.pid_sum_err[lane] = 0;
            ctx.pid_prev_err[lane] = 0;
        }
    }
    const input: math.Vector6f = .{
        translation_body[0], translation_body[1], translation_body[2],
        rotation[0],         rotation[1],         rotation[2],
    };
    var thruster_math: [MissionConfig.thruster_count]f64 = undefined;
    _ = math.tamToThrusters(
        tam_rows,
        input,
        config.thrustor_saturate,
        5.0,
        thruster_math[0..tam_len],
    );
    for (thruster_math[0..tam_len], out.values[0..tam_len]) |value, *channel| channel.* = @floatCast(value);
    ctx.has_dt = dt != null;
    ctx.dt = dt orelse 0;
    ctx.control_pose = math.poseTo6f(frame.camera_pose);
    ctx.translation_error = error_body;
    ctx.target_quaternion = target_q;
    ctx.attitude_error = attitude_error;
    ctx.command_input = input;
    out.len = @intCast(tam_len);
}

fn resetOdometryPid(self: *State) void {
    self.pid_sum_err = @splat(0);
    self.pid_prev_err = @splat(0);
    self.tilt_correction = .{ 0, 0, 0 };
}

fn lookAtHeading(heading: f64) math.Vector3f {
    return .{ -@sin(heading), @cos(heading), 0 };
}

fn slew3(current: math.Vector3f, desired: math.Vector3f, max_step: f64) math.Vector3f {
    if (max_step <= 0) return current;
    const delta = desired - current;
    const len = math.length3f(delta);
    if (len <= max_step or len <= 1e-6) return desired;
    return current + delta / @as(math.Vector3f, @splat(len / max_step));
}

fn bodyTranslationError(frame: *const Auv.Frame, target: math.Vector3f, use_axis: Auv.AxisOk) math.Vector3f {
    var world = bc.positionError(frame.camera_pose.pos, target) catch return .{ 0, 0, 0 };
    const ok = frame.position_ok;
    if (!ok.x or !use_axis.x) world[0] = 0;
    if (!ok.y or !use_axis.y) world[1] = 0;
    if (!ok.z or !use_axis.z) world[2] = 0;
    const body_q = bc.normalizedQuat(frame.camera_pose.quat) catch return .{ 0, 0, 0 };
    return math.quaternionRotate(math.quaternionConjugate(body_q), world);
}

pub fn attitudeToward(frame: *const Auv.Frame, orientation: math.Quaternionf) ?math.Vector3f {
    const current = bc.normalizedQuat(frame.camera_pose.quat) catch return null;
    const desired = bc.normalizedQuat(orientation) catch return null;
    return bc.attitudeError(current, desired) catch null;
}

fn poseError(ctx: *const State, frame: *const Auv.Frame) math.Vector6f {
    const translation = if (ctx.use_position)
        bodyTranslationError(frame, ctx.target_position, ctx.position_axes)
    else
        math.Vector3f{ 0, 0, 0 };
    const angular = if (ctx.use_orientation)
        attitudeToward(frame, ctx.target_orientation) orelse math.Vector3f{ 0, 0, 0 }
    else
        math.Vector3f{ 0, 0, 0 };
    return .{ translation[0], translation[1], translation[2], angular[0], angular[1], angular[2] };
}

fn poseAvailable(ctx: *const State, frame: *const Auv.Frame) [6]bool {
    const position = frame.position_ok;
    const attitude = frame.attitude_ok;
    return .{
        ctx.use_position and ctx.position_axes.x and position.x,
        ctx.use_position and ctx.position_axes.y and position.y,
        ctx.use_position and ctx.position_axes.z and position.z,
        ctx.use_orientation and attitude.x,
        ctx.use_orientation and attitude.y,
        ctx.use_orientation and attitude.z,
    };
}

fn visualBodyError(ctx: *const State, frame: *const Auv.Frame, config: *const MissionConfig.MissionConfig) ?math.Vector6f {
    if (frame.objects2d_len == 0) return null;
    const object = frame.objects2d[0];
    const metrics = math.boxMetrics(
        frame.image_width,
        frame.image_height,
        .{ object.top_left.x, object.top_left.y },
        .{ object.bottom_right.x, object.bottom_right.y },
    );
    const ibvs = config.ibvs;
    const image_error = [_]f64{
        math.withinDeadband(ctx.desired[0] - metrics.cx, ibvs.center_deadband),
        math.withinDeadband(ctx.desired[1] - metrics.cy, ibvs.center_deadband),
        ibvs.target_height - metrics.height,
    };
    var interaction_rows: [math.interaction_output_len][]const f64 = undefined;
    const ibvs_cols = @min(ibvs.interaction_cols, MissionConfig.max_ibvs_error_cols);
    for (&interaction_rows, config.ibvs.interaction) |*row, interaction_row| {
        row.* = interaction_row[0..ibvs_cols];
    }
    const mapped = math.mapInteraction(interaction_rows[0..], &image_error) catch {
        return null;
    };
    const current = bc.normalizedQuat(frame.camera_pose.quat) catch return null;
    const yaw = math.quaternionToEuler(current)[2] + mapped[5];
    const look_q = bc.lookWithinLimits(
        config.camera_forward,
        lookAtHeading(yaw),
        bc.identity,
        config.rotation_limit,
    ) catch return null;
    var angular = bc.attitudeError(current, look_q) catch return null;
    angular[0] += mapped[3];
    angular[1] += mapped[4];
    return .{ mapped[0], mapped[1], mapped[2], angular[0], angular[1], angular[2] };
}

fn anyAxis(available: [6]bool) bool {
    for (available) |ok| if (ok) return true;
    return false;
}

fn bodyError(ctx: *State, frame: *const Auv.Frame, config: *const MissionConfig.MissionConfig) math.Vector6f {
    const visual = visualBodyError(ctx, frame, config);
    var available = poseAvailable(ctx, frame);
    var position_error = poseError(ctx, frame);
    if (ctx.teleop_translation) |body| {
        position_error[0] = body[0];
        position_error[1] = body[1];
        position_error[2] = body[2];
        available[0] = true;
        available[1] = true;
        available[2] = true;
    }
    if (visual == null and !anyAxis(available)) {
        resetOdometryPid(ctx);
        ctx.pid_prev_timestamp_ns = null;
    }
    return math.greedy6(position_error, visual, available);
}

fn quaternionFromRotationVector(rot: math.Vector3f) math.Quaternionf {
    const angle = math.length3f(rot);
    if (angle <= 1e-6) return bc.identity;
    return bc.forwardRotation(rot, angle) catch bc.identity;
}