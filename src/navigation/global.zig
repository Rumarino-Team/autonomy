const std = @import("std");
const Auv = @import("../Auv.zig");
const math = @import("../math.zig");
const nav_helpers = @import("nav_helpers.zig");
const ConfigLoader = @import("../ConfigLoader.zig");
const policy_mod = @import("../policy.zig");

pub const yaw_gate_default: f32 = std.math.pi / 8.0;

pub const GlobalControllerState = struct {
    ErrorConstant: math.Vector6f,
    DerivativeConstant: math.Vector6f,
    IntegralConstant: math.Vector6f,
    sumError: math.Vector6f = @splat(0),
    prevError: math.Vector6f = @splat(0),
};

fn pidWrench(controller: *GlobalControllerState, err: math.Vector6f, dt: ?f32) math.Vector6f {
    const vel_err: math.Vector6f = if (dt) |dt_val| blk: {
        controller.sumError += err * @as(math.Vector6f, @splat(dt_val));
        break :blk (err - controller.prevError) / @as(math.Vector6f, @splat(dt_val));
    } else @as(math.Vector6f, @splat(0));

    const wrench = controller.ErrorConstant * err + controller.IntegralConstant * controller.sumError + controller.DerivativeConstant * vel_err;

    controller.prevError = err;

    return wrench;
}

/// SLAM frame, tracked 3D objects, goal, and global PID memory.
pub const GlobalState = struct {
    frame: Auv.Frame = undefined,
    seen_objects: [Auv.Frame.max_objects]Auv.Object = undefined,
    seen_objects_len: u8 = 0,
    goal: math.Vector6f = @splat(0),
    prev_timestamp_ns: ?u64 = null,
    controller: GlobalControllerState = undefined,
    policy_runner: policy_mod.Runner = .{},
};

pub fn initController(state: *GlobalState, config: ConfigLoader.Config) void {
    state.controller = .{
        .ErrorConstant = config.kp,
        .IntegralConstant = config.ki,
        .DerivativeConstant = config.kd,
    };
}

pub fn syncControllerGains(state: *GlobalState, config: ConfigLoader.Config) void {
    state.controller.ErrorConstant = config.kp;
    state.controller.IntegralConstant = config.ki;
    state.controller.DerivativeConstant = config.kd;
}

pub const Actuation = struct {
    auv: *Auv,
    tam: []const math.Vector6f,
    close_enough: f32,
    thrustor_saturate: f32,
    thrustor_output_scale: f32,
    /// Replaces the PID when set.
    policy: ?*policy_mod.Policy = null,
    log: bool = false,
};

pub fn updateSeenObjects(state: *GlobalState) void {
    for (state.frame.objects[0..state.frame.objects_len]) |frame_object| {
        const maybe_seen_object = for (state.seen_objects[0..state.seen_objects_len]) |*seen_object| {
            if (frame_object.id == seen_object.id) break seen_object;
        } else null;

        if (maybe_seen_object) |seen_object| {
            seen_object.* = frame_object;
        } else {
            state.seen_objects[state.seen_objects_len] = frame_object;
            state.seen_objects_len += 1;
        }
    }
}

pub fn globalControllerUpdate(state: *GlobalState, act: Actuation) void {
    const pose = state.frame.camera_pose;
    const timestamp_ns = state.frame.timestamp;

    if (act.policy) |policy| {
        policyStep(state, act, policy);
        return;
    }

    const dt: ?f32 = if (state.prev_timestamp_ns) |previous| blk: {
        if (timestamp_ns > previous) {
            break :blk @as(f32, @floatFromInt(timestamp_ns - previous)) * 1e-9;
        }
        std.log.warn(
            "odometry stamp not increasing (prev={} ns, now={} ns); skipping I/D",
            .{ previous, timestamp_ns },
        );
        break :blk null;
    } else null;

    state.prev_timestamp_ns = timestamp_ns;

    const goal = state.goal;
    const current_pose = math.poseTo6f(pose);
    var pose_err = goal - current_pose;

    const rot = math.normalize4f(pose.quat);
    const forward = math.quaternionRotate(rot, .{ 0.0, 1.0, 0.0 });
    // Nose is body +Y. Euler yaw is the heading of body +X, 90° off this axis.
    const current_yaw = std.math.atan2(forward[1], forward[0]);

    const dir = math.Vector3f{ pose_err[0], pose_err[1], 0.0 };
    const xy_distance = math.length3f(dir);
    const target_yaw = if (xy_distance > act.close_enough)
        std.math.atan2(dir[1], dir[0])
    else
        goal[5];
    const yaw_error = math.wrapAngle(target_yaw - current_yaw);
    pose_err[5] = yaw_error;

    const wrench = pidWrench(&state.controller, pose_err, dt);
    const input = nav_helpers.wrenchToThrusterInput(wrench, pose.quat, yaw_error, yaw_gate_default);

    var thruster_buffer: [nav_helpers.max_thrusters]f32 = undefined;
    const thruster_values = nav_helpers.tamToThrusters(
        act.tam,
        input,
        act.thrustor_saturate,
        act.thrustor_output_scale,
        &thruster_buffer,
    );

    if (act.log) {
        std.debug.print("\x1b[2J\x1b[H", .{});
        std.log.debug("\tclose_enough = {}", .{act.close_enough});
        math.debug6f("\tkp", state.controller.ErrorConstant);
        math.debug6f("\tki", state.controller.IntegralConstant);
        math.debug6f("\tkd", state.controller.DerivativeConstant);
        std.log.debug("dt = {d:6.2} ms", .{(dt orelse 0) * 1000});
        math.debug6f("pose", current_pose);
        math.debug6f("goal", goal);
        std.log.debug("xy_distance = {}", .{xy_distance});
        math.debug6f("\twrench  ", wrench);
        math.debug3f("\tforward ", forward);
        math.debug3f("\tdir2d   ", dir);
        std.log.debug(
            "\tcurrent_yaw={d:3.2} target_yaw={d:3.2} yaw_error={d:3.2}",
            .{ current_yaw, target_yaw, yaw_error },
        );
        math.debug6f("\tinput   ", input);
        std.log.debug("\tthruster_values = {any}", .{thruster_values});
    }
    act.auv.setThrustorValues(thruster_values.ptr, @intCast(thruster_values.len));
}

fn policyStep(state: *GlobalState, act: Actuation, policy: *policy_mod.Policy) void {
    const runner = &state.policy_runner;
    if (policy.step(runner, state.frame.camera_pose, state.frame.timestamp, state.goal)) |action| {
        const values = nav_helpers.tamToThrusters(
            act.tam,
            action,
            act.thrustor_saturate,
            act.thrustor_output_scale,
            &runner.thrusters,
        );
        runner.thrusters_len = values.len;
    }
    if (act.log) {
        std.debug.print("\x1b[2J\x1b[H", .{});
        math.debug6f("pose", math.poseTo6f(state.frame.camera_pose));
        math.debug6f("goal", state.goal);
        math.debug6f("\tpolicy action", runner.prev_action);
        std.log.debug("\tthruster_values = {any}", .{runner.thrusters[0..runner.thrusters_len]});
    }
    act.auv.setThrustorValues(&runner.thrusters, @intCast(runner.thrusters_len));
}

/// First object at or after `start` whose class is in `clss`.
pub fn findObject(state: *const GlobalState, clss: []const Auv.ObjectCls, start: usize) ?*const Auv.Object {
    for (state.seen_objects[start..state.seen_objects_len]) |*object| {
        for (clss) |cls| {
            if (object.cls == cls) return object;
        }
    }
    return null;
}

pub fn setGoalPosition(state: *GlobalState, goal_pos: math.Vector3f) void {
    state.goal[0] = goal_pos[0];
    state.goal[1] = goal_pos[1];
    state.goal[2] = goal_pos[2];
}

pub fn cameraWithin(state: *const GlobalState, goal_pos: math.Vector3f, threshold: f32) bool {
    const delta = goal_pos - state.frame.camera_pose.pos;
    const dist = @sqrt(@reduce(.Add, delta * delta));
    return dist <= threshold;
}

pub const MissionError = error{
    TrackingLost,
};
