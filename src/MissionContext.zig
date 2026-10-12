const std = @import("std");
const Io = std.Io;
const MissionArgs = @import("MissionArgs.zig");

const Auv = @import("Auv.zig");
const AuvLoader = @import("AuvLoader.zig");
const ConfigLoader = @import("ConfigLoader.zig");
const FileWatcher = @import("FileWatcher.zig");
const Motion = @import("Motion.zig");
const Navigation = @import("Navigation.zig");
const MotionExecutor = @import("MotionExecutor.zig");
const MotionController = @import("MotionController.zig");
const ThrusterAllocator = @import("ThrusterAllocator.zig");

pub const Facing = Motion.Facing;
pub const MoveOptions = Motion.MoveOptions;
pub const PathOptions = Motion.PathOptions;
pub const RecoveryOptions = Motion.RecoveryOptions;
pub const SearchOptions = Motion.SearchOptions;
pub const VisualOptions = Motion.VisualOptions;
pub const BodyOptions = Motion.BodyOptions;

const MissionContext = @This();
const print_stuff = true;
const clear_print_stuff = true;

io: Io,

frame: Auv.Frame,

seen_objects: [Auv.Frame.max_objects]Auv.Object,
seen_objects_len: u8,

goal_dist_threshold: f32,
thrustor_saturate: f32,

log_counter: u32,
log_freq_div: u16,

goal: math.Vector6f,
motion: ?Motion.State = null,
navigation: Navigation = .{},
controller: MotionController = .{},
allocator: ?ThrusterAllocator = null,
backend_geometry: ?Auv.ThrusterGeometry = null,
allocation_result: ?ThrusterAllocator.Result = null,
controller_dt: f32 = 0,
execution_dt: f32 = 0,
last_execution_timestamp: ?Io.Clock.Timestamp = null,
last_valid_pose: ?math.Vector6f = null,
frame_previous_timestamp: ?u64 = null,

auv: Auv,
auv_loader: AuvLoader,
auv_watcher: FileWatcher,

config: ConfigLoader.Config,
config_loader: ConfigLoader,
config_watcher: FileWatcher,

pub fn init(arena: std.mem.Allocator, io: Io, args: MissionArgs) !MissionContext {
    var config_loader: ConfigLoader = try .init(arena, args.live_config_path);
    const config = config_loader.load(io) catch |err| {
        std.log.err("failed to load config: {s}", .{config_loader.config_path});
        return err;
    };
    if (print_stuff) {
        std.log.debug("succesfully loaded config: {s}", .{config_loader.config_path});
        std.log.debug("{any}", .{config});
    }

    var auv_loader: AuvLoader = try .init(arena, args.auv_dynlib_path);
    const auv = auv_loader.load(io) catch |err| {
        const dl_err = std.mem.span(std.c.dlerror()) orelse "NO_DL_ERROR";
        std.log.err("dlerror: `{s}`", .{dl_err});
        std.log.err("failed to load auv: {s}", .{auv_loader.auv_path});
        return err;
    };
    auv.init();
    errdefer auv.deinit();
    var backend_geometry: ?Auv.ThrusterGeometry = null;
    if (auv.getThrusterGeometry) |get_geometry| {
        var geometry: Auv.ThrusterGeometry = undefined;
        if (get_geometry(&geometry)) backend_geometry = geometry;
    }
    config_loader.backend_geometry = backend_geometry;
    const allocator = if (config.allocation) |options| try ThrusterAllocator.init(options, backend_geometry) else null;
    if (print_stuff) {
        std.log.debug("succesfully loaded auv: {s}", .{auv_loader.auv_path});
        std.log.debug("{any}", .{auv});
    }

    return .{
        .frame = std.mem.zeroes(Auv.Frame),

        .seen_objects = undefined,
        .seen_objects_len = 0,

        .goal_dist_threshold = args.goal_dist_threshold,
        .thrustor_saturate = 5,

        .log_counter = 0,
        .log_freq_div = 120,

        .goal = @splat(0),

        .frame_previous_timestamp = null,

        .auv = auv,
        .auv_loader = auv_loader,
        .auv_watcher = try .init(arena, args.auv_dynlib_path),

        .config = config,
        .allocator = allocator,
        .backend_geometry = backend_geometry,
        .config_loader = config_loader,
        .config_watcher = try .init(arena, args.live_config_path),

        .io = io,
    };
}

/// Invalidates `seen_objects`
pub fn reset(ctx: *MissionContext) void {
    ctx.seen_objects_len = 0;
    ctx.motion = null;
    ctx.navigation = .{};
    ctx.controller.reset();
    if (ctx.allocator) |*allocator| allocator.reset();
    ctx.allocation_result = null;
    ctx.controller_dt = 0;
    ctx.execution_dt = 0;
    ctx.last_execution_timestamp = null;
    ctx.last_valid_pose = null;
    ctx.frame_previous_timestamp = null;
}

pub fn yieldUntilNextFrameAndUpdate(ctx: *MissionContext) !void {
    ctx.auv.yieldUntilNextFrame(&ctx.frame);
    const start = Io.Clock.Timestamp.now(ctx.io, .awake);

    ctx.execution_dt = if (ctx.last_execution_timestamp) |previous|
        @max(0, @as(f32, @floatFromInt(previous.durationTo(start).raw.toNanoseconds())) * 1e-9)
    else
        0;
    ctx.last_execution_timestamp = start;
    ControllerStep(ctx);

    // Historical world detections serve mission discovery, never visual feedback.
    if (ctx.navigation.snapshot.measured_position and ctx.navigation.snapshot.observations_valid) {
        for (ctx.frame.objects[0..@min(ctx.frame.objects_len, Auv.Frame.max_objects)]) |frame_object| {
            if (!Navigation.validPoint(frame_object.bbox.pose.pos)) continue;
            var object = frame_object;
            object.bbox.pose.pos = ctx.navigation.toMission(object.bbox.pose.pos);
            object.bbox.pose.quat = math.quaternionMul(ctx.navigation.tracking_to_mission.quat, object.bbox.pose.quat);
            const existing = for (ctx.seen_objects[0..ctx.seen_objects_len]) |*seen| {
                if (seen.id == object.id) break seen;
            } else null;
            if (existing) |seen| {
                seen.* = object;
            } else if (ctx.seen_objects_len < Auv.Frame.max_objects) {
                ctx.seen_objects[ctx.seen_objects_len] = object;
                ctx.seen_objects_len += 1;
            }
        }
    }

    if (ctx.config_watcher.changed() catch |err| blk: {
        std.log.err("{s}", .{@errorName(err)});
        break :blk false;
    }) {
        if (ctx.config_loader.load(ctx.io)) |config| {
            const allocator = if (config.allocation) |options| try ThrusterAllocator.init(options, ctx.backend_geometry) else null;
            if (print_stuff) {
                std.log.debug("succesfully reloaded config: {s}/{s}", .{ ctx.auv_watcher.dir, ctx.auv_watcher.name });
                std.log.debug("{any}", .{config});
            }
            ctx.config = config;
            ctx.allocator = allocator;
            ctx.allocation_result = null;
            ctx.controller.reset();
        } else |err| {
            std.log.err("failed to reload config: {s}/{s}", .{ ctx.auv_watcher.dir, ctx.auv_watcher.name });
            std.log.err("{s}", .{@errorName(err)});
            std.log.info("kept previous config", .{});
        }
    }

    if (ctx.auv_watcher.changed() catch |err| blk: {
        std.log.err("{s}", .{@errorName(err)});
        break :blk false;
    }) {
        ctx.auv.deinit();
        if (ctx.auv_loader.load(ctx.io)) |auv| {
            if (print_stuff) {
                std.log.debug("succesfully reloaded auv: {s}/{s}", .{ ctx.auv_watcher.dir, ctx.auv_watcher.name });
                std.log.debug("{any}", .{auv});
            }
            ctx.auv = auv;
            ctx.auv.init();
            try ctx.refreshAllocationGeometry();
            if (print_stuff) {
                std.log.debug("restarted auv loop", .{});
            }
            return error.RestartMission;
        } else |err| {
            std.log.err("failed to reload auv: {s}/{s}", .{ ctx.auv_watcher.dir, ctx.auv_watcher.name });
            std.log.err("{s}", .{@errorName(err)});
            std.log.info("kept previous auv", .{});
        }
        ctx.auv.init();
        try ctx.refreshAllocationGeometry();
    }

    const elapsed = start.untilNow(ctx.io);
    if (print_stuff and ctx.log_counter % ctx.log_freq_div == 0) {
        std.log.debug("done in {} us\n", .{elapsed.raw.toMicroseconds()});
    }
    ctx.log_counter +%= 1;
}

const math = @import("math.zig");
fn setThrustersFromTamInput(ctx: *MissionContext, input: math.Vector6f, max_effort: f32) void {
    if (ctx.allocator) |*allocator| {
        if (!ctx.controller.feedback_available) {
            allocator.reset();
            ctx.allocation_result = null;
            var zero: [math.max_thrusters]f32 = @splat(0);
            ctx.auv.setThrustorValues(&zero, @intCast(allocator.len));
            return;
        }
        const result = allocator.allocate(input, max_effort, ctx.controller_dt);
        ctx.allocation_result = result;
        ctx.controller.allocationFeedback(input, result.achieved, ctx.config, ctx.controller_dt);
        ctx.auv.setThrustorValues(&result.commands, @intCast(result.len));
        return;
    }
    var thruster_buffer: [math.max_thrusters]f32 = undefined;
    const mixed = math.tamMul(ctx.config.tam, input * ctx.config.control.legacy_wrench_scale, &thruster_buffer);
    var peak: f32 = 0;
    for (mixed) |command| peak = @max(peak, @abs(command));
    const limit = ctx.thrustor_saturate * max_effort;
    const scale = if (peak > limit and peak > 0) limit / peak else 1;
    // Legacy configs have no force model. Uniform command scaling preserves
    // the requested wrench proportions and provides limiter feedback, rather
    // than independently clipping away the requested rotation.
    const thruster_values = math.tamToThrusters(
        ctx.config.tam,
        input * ctx.config.control.legacy_wrench_scale * @as(math.Vector6f, @splat(scale)),
        limit,
        5.0,
        &thruster_buffer,
    );
    ctx.controller.allocationFeedback(input, input * @as(math.Vector6f, @splat(scale)), ctx.config, ctx.controller_dt);
    ctx.auv.setThrustorValues(thruster_values.ptr, @intCast(thruster_values.len));
}

fn refreshAllocationGeometry(ctx: *MissionContext) !void {
    ctx.backend_geometry = null;
    if (ctx.auv.getThrusterGeometry) |get_geometry| {
        var geometry: Auv.ThrusterGeometry = undefined;
        if (get_geometry(&geometry)) ctx.backend_geometry = geometry;
    }
    ctx.config_loader.backend_geometry = ctx.backend_geometry;
    ctx.allocator = if (ctx.config.allocation) |options| try ThrusterAllocator.init(options, ctx.backend_geometry) else null;
    ctx.allocation_result = null;
    ctx.controller.reset();
}

fn ControllerStep(ctx: *MissionContext) void {
    const timestamp = ctx.frame.timestamp;
    ctx.controller_dt = if (ctx.frame_previous_timestamp) |previous|
        if (timestamp > previous) @as(f32, @floatFromInt(timestamp - previous)) * 1e-9 else 0
    else
        0;
    if (ctx.frame_previous_timestamp == null or timestamp > ctx.frame_previous_timestamp.?) {
        ctx.frame_previous_timestamp = timestamp;
    } else ctx.controller.reset();
    const elapsed = @max(ctx.controller_dt, ctx.execution_dt);
    ctx.navigation.update(&ctx.frame, ctx.config.navigation, elapsed);
    const snapshot = ctx.navigation.snapshot;
    if (snapshot.measured_position and snapshot.orientation != null) {
        ctx.last_valid_pose = math.poseTo6f(.{ .pos = snapshot.position.?, .quat = snapshot.orientation.? });
    }
    const target: MotionExecutor.Target = if (ctx.motion) |*motion|
        MotionExecutor.update(motion, &ctx.navigation, &ctx.frame, &ctx.goal, ctx.goal_dist_threshold, elapsed)
    else
        .{ .stop = true };
    const input = ctx.controller.calculate(&ctx.navigation, target, ctx.config, ctx.controller_dt);
    setThrustersFromTamInput(ctx, input, target.max_effort);
    if (print_stuff and ctx.log_counter % ctx.log_freq_div == 0) {
        if (clear_print_stuff) std.debug.print("\x1b[2J\x1b[H", .{});
        std.log.debug("motion mode={s}, measured_position={}, estimated_position={}, orientation={}, observations={}", .{
            @tagName(target.mode),        snapshot.measured_position,  snapshot.estimated_position,
            snapshot.orientation != null, snapshot.observations_valid,
        });
        math.debug6f("goal", ctx.goal);
        math.debug6f("input", input);
        if (ctx.allocation_result) |result| math.debug6f("allocation residual", result.residual);
        if (ctx.motion) |*motion| {
            std.log.debug("command={s}, facing={s}, recovering={}, recovery_seconds={d:.2}, completed={}", .{
                @tagName(motion.command), @tagName(motion.options.facing), motion.recovering, motion.recovery_seconds, motion.completed,
            });
            if (motion.failure) |failure| std.log.debug("failure={s}", .{@tagName(failure)});
        }
    }
}

/// Compatibility entry point: visual approach is an explicit Motion command.
pub fn trackObject2d(ctx: *MissionContext, id: u32) !void {
    try ctx.startVisualTarget(.{
        .object_id = id,
        .target_height = ctx.config.non_odometry.target_height,
        .center_tolerance = ctx.config.non_odometry.center_deadband,
    }, .{
        .facing = .{ .look_at_object = id },
    });
}

fn yieldUntilObjectWithCls(ctx: *MissionContext, clss: []const Auv.ObjectCls, start: usize) !*const Auv.Object {
    var seen = start;

    while (true) {
        for (ctx.seen_objects[seen..ctx.seen_objects_len]) |*reacted_object| {
            for (clss) |cls| {
                if (reacted_object.cls == cls) {
                    return reacted_object;
                }
            }
            seen += 1;
        }

        try ctx.checkMotionFailure();
        if (ctx.motion) |*motion| {
            if (motion.command == .search and motion.completed) return error.SearchTimeout;
        }
        try ctx.yieldUntilNextFrameAndUpdate();
    }
}

/// yield until getting first object with any of the `cls` in `clss`
pub fn yieldUntilFirstObjectWithAnyCls(ctx: *MissionContext, clss: []const Auv.ObjectCls) !*const Auv.Object {
    return try yieldUntilObjectWithCls(ctx, clss, 0);
}

/// yield until getting first object wit `cls`
pub fn yieldUntilFirstObjectWithCls(ctx: *MissionContext, cls: Auv.ObjectCls) !*const Auv.Object {
    return try yieldUntilFirstObjectWithAnyCls(ctx, &.{cls});
}

/// yield until getting next object with any of the `cls` in `clss` (ignoring any seen before)
pub fn yieldUntilNewObjectWithAnyCls(ctx: *MissionContext, clss: []const Auv.ObjectCls) !*const Auv.Object {
    return try yieldUntilObjectWithCls(ctx, clss, ctx.seen_objects_len);
}

/// yield until getting next object wit `cls` (ignoring any seen before)
pub fn yieldUntilNewObjectWithCls(ctx: *MissionContext, cls: Auv.ObjectCls) !*const Auv.Object {
    return try yieldUntilNewObjectWithAnyCls(ctx, &.{cls});
}

/// Six-axis teleop uses fixed yaw and simultaneous movement. Repeated packets
/// preserve the heading ramp, but issuing a pose command replaces a route.
pub fn setPoseGoal(ctx: *MissionContext, goal: math.Vector6f) void {
    inline for (0..6) |i| if (!std.math.isFinite(goal[i])) return;
    const destination: math.Vector3f = .{ goal[0], goal[1], goal[2] };
    const options: MoveOptions = .{ .facing = .{ .fixed_yaw = goal[5] }, .translation = .simultaneous };
    if (ctx.motion != null and ctx.motion.?.command == .pose and ctx.motion.?.options.facing == .fixed_yaw) {
        const motion = &ctx.motion.?;
        motion.destination = destination;
        motion.options.facing = options.facing;
        motion.desired_yaw = math.wrapAngle(goal[5]);
        // Repeated packets cannot extend an outage. Once measured tracking
        // returns, a new teleop packet can explicitly retry the pose command.
        if (motion.failure == .tracking_lost and ctx.navigation.snapshot.measured_position) ctx.resumeMotion() catch {};
    } else {
        const previous_yaw = if (ctx.motion) |*motion| motion.commanded_yaw else ctx.navigation.snapshot.yaw() orelse goal[5];
        ctx.beginMotion(destination, options);
        ctx.motion.?.commanded_yaw = previous_yaw;
    }
    ctx.goal = goal;
}

fn resetControllerHistory(ctx: *MissionContext) void {
    ctx.controller.reset();
    // A new motion keeps the current motor values for slew continuity. Only
    // stop/cancellation failures reset the allocator immediately to zero.
}

fn beginMotion(ctx: *MissionContext, destination: math.Vector3f, options: MoveOptions) void {
    const yaw = ctx.navigation.snapshot.yaw() orelse ctx.goal[5];
    ctx.motion = Motion.State.init(destination, yaw, options);
    ctx.motion.?.pressure_goal = ctx.navigation.snapshot.pressure_depth;
    if (options.facing == .look_at_object and ctx.navigation.snapshot.measured_position and ctx.navigation.snapshot.observations_valid) {
        for (ctx.frame.objects[0..@min(ctx.frame.objects_len, Auv.Frame.max_objects)]) |object| {
            if (object.id == options.facing.look_at_object and Navigation.validPoint(object.bbox.pose.pos)) {
                ctx.motion.?.last_object_position = ctx.navigation.toMission(object.bbox.pose.pos);
                break;
            }
        }
    }
    ctx.resetControllerHistory();
}

pub fn stopThrusters(ctx: *MissionContext) void {
    ctx.controller.reset();
    setThrustersFromTamInput(ctx, @splat(0), 1);
}

/// Explicit cancellation replaces the plan with a hold of measured axes.
pub fn holdPosition(ctx: *MissionContext) void {
    var pose = ctx.last_valid_pose orelse ctx.goal;
    if (ctx.navigation.snapshot.position) |position| {
        if (ctx.navigation.snapshot.measured_position) {
            pose[0] = position[0];
            pose[1] = position[1];
            pose[2] = position[2];
        }
    }
    if (ctx.navigation.snapshot.orientation) |q| {
        const angles = math.quaternionToEuler(q);
        pose[3] = angles[0];
        pose[4] = angles[1];
        pose[5] = angles[2];
    }
    if (ctx.navigation.snapshot.pressure_depth) |depth| {
        if (ctx.navigation.pressure_to_world_offset) |offset| pose[2] = depth + offset;
    }
    ctx.goal = pose;
    ctx.beginMotion(.{ pose[0], pose[1], pose[2] }, .{ .facing = .hold_current });
    ctx.motion.?.command = .hold;
    ctx.stopThrusters();
}

pub fn reanchorLocalization(ctx: *MissionContext, tracking_to_mission: math.Pose, epoch: u32) !void {
    try ctx.navigation.reanchor(tracking_to_mission, epoch);
    ctx.frame_previous_timestamp = null;
    ctx.seen_objects_len = 0;
    ctx.controller.reset();
}

/// Retry the retained command after a failure, with a new recovery/deadline
/// budget. The route geometry and saved progress remain intact.
pub fn resumeMotion(ctx: *MissionContext) !void {
    const motion = if (ctx.motion) |*motion| motion else return error.NoActiveMotion;
    motion.failure = null;
    motion.elapsed = 0;
    motion.recovery_seconds = 0;
    motion.tracking_confirm_seconds = 0;
    motion.missing_feedback_seconds = 0;
    motion.settling = .{};
    motion.completed = false;
    motion.resume_scale = 0;
    ctx.controller.reset();
}

fn checkMotionFailure(ctx: *MissionContext) !void {
    if (ctx.motion) |*motion| {
        if (motion.failure) |failure| switch (failure) {
            .object_lost => return error.ObjectLost,
            .tracking_lost => return error.TrackingLost,
            .move_timeout => return error.MoveTimeout,
            .search_timeout => return error.SearchTimeout,
            .localization_changed => return error.LocalizationChanged,
            .rejoin_too_far => return error.RejoinTooFar,
            .sensor_unavailable => return error.SensorUnavailable,
        };
    }
}

pub fn yieldUntilMotionComplete(ctx: *MissionContext) !void {
    if (ctx.motion == null) return error.NoActiveMotion;
    errdefer |err| if (err != error.RestartMission) ctx.stopThrusters();
    while (true) {
        try ctx.yieldUntilNextFrameAndUpdate();
        try ctx.checkMotionFailure();
        if (ctx.motion.?.completed) return;
    }
}

pub fn yieldUntilReachGoal(ctx: *MissionContext, goal_pos: math.Vector3f, options: MoveOptions) !void {
    try ctx.yieldUntilFollowPath(&.{goal_pos}, .{ .move = options, .corner_radius = 0 });
}

pub fn startFollowPath(ctx: *MissionContext, waypoints: []const math.Vector3f, options: PathOptions) !void {
    try options.validate();
    if (!ctx.navigation.snapshot.measured_position) return error.TrackingUnavailable;
    const path = try Motion.Path.init(ctx.navigation.snapshot.position.?, waypoints, options.corner_radius);
    ctx.beginMotion(path.end, options.move);
    ctx.motion.?.command = .{ .route = .{ .path = path, .reference_speed = options.reference_speed, .max_tracking_error = options.max_tracking_error } };
    const initial = path.sample(0);
    ctx.goal[0] = initial.position[0];
    ctx.goal[1] = initial.position[1];
    ctx.goal[2] = initial.position[2];
    ctx.motion.?.path_direction = initial.direction;
}

pub fn yieldUntilFollowPath(ctx: *MissionContext, waypoints: []const math.Vector3f, options: PathOptions) !void {
    errdefer |err| if (err != error.RestartMission) ctx.stopThrusters();
    try ctx.startFollowPath(waypoints, options);
    try ctx.yieldUntilMotionComplete();
}

pub fn startSearch(ctx: *MissionContext, options: SearchOptions) !void {
    try options.validate();
    if (ctx.navigation.snapshot.orientation == null) return error.SensorUnavailable;
    ctx.beginMotion(.{ ctx.goal[0], ctx.goal[1], ctx.goal[2] }, .{ .facing = .hold_current, .require_yaw = false });
    ctx.motion.?.command = .{ .search = options };
}

pub fn yieldUntilSearch(ctx: *MissionContext, options: SearchOptions) !void {
    try ctx.startSearch(options);
    try ctx.yieldUntilMotionComplete();
}

pub fn startVisualTarget(ctx: *MissionContext, options: VisualOptions, move: MoveOptions) !void {
    try options.validate();
    try move.validate();
    if (move.facing != .look_at_object or move.facing.look_at_object != options.object_id) return error.InvalidMoveOptions;
    ctx.beginMotion(.{ ctx.goal[0], ctx.goal[1], ctx.goal[2] }, move);
    ctx.motion.?.command = .{ .visual = options };
}

pub fn yieldUntilVisualTarget(ctx: *MissionContext, options: VisualOptions, move: MoveOptions) !void {
    try ctx.startVisualTarget(options, move);
    try ctx.yieldUntilMotionComplete();
}

pub fn startBodyMove(ctx: *MissionContext, options: BodyOptions, move: MoveOptions) !void {
    try options.validate();
    try move.validate();
    if (move.facing != .hold_current and move.facing != .fixed_yaw) return error.InvalidMoveOptions;
    if (ctx.navigation.snapshot.orientation == null or
        (move.facing == .fixed_yaw and !ctx.navigation.snapshot.heading_aligned)) return error.SensorUnavailable;
    ctx.beginMotion(.{ ctx.goal[0], ctx.goal[1], ctx.goal[2] }, move);
    ctx.motion.?.command = .{ .body = options };
}

pub fn yieldUntilBodyMove(ctx: *MissionContext, options: BodyOptions, move: MoveOptions) !void {
    try ctx.startBodyMove(options, move);
    try ctx.yieldUntilMotionComplete();
}

const ControllerTestBackend = struct {
    var outputs: [math.max_thrusters]f32 = @splat(0);
    var simulation_active = false;
    var simulation_position: math.Vector3f = @splat(0);
    var simulation_velocity: math.Vector3f = @splat(0);
    var simulation_yaw: f32 = 0;
    var simulation_yaw_velocity: f32 = 0;
    var simulation_timestamp: u64 = 1;
    var observe_object = true;
    var tracking_available = true;

    fn init() callconv(.c) void {}
    fn deinit() callconv(.c) void {}
    fn yieldFrame(frame: *Auv.Frame) callconv(.c) void {
        if (!simulation_active) return;
        const dt: f32 = 0.05;
        const half_yaw = simulation_yaw * 0.5;
        const rotation: math.Quaternionf = .{ 0, 0, @sin(half_yaw), @cos(half_yaw) };
        const force = math.quaternionRotate(rotation, .{ outputs[0] * 5, outputs[1] * 5, outputs[2] * 5 });
        // Small deterministic plant with inertia and drag. It exercises the
        // real controller, route loop and frame API without a graphics backend.
        simulation_velocity += (force - simulation_velocity) * @as(math.Vector3f, @splat(dt));
        simulation_position += simulation_velocity * @as(math.Vector3f, @splat(dt));
        simulation_yaw_velocity += (outputs[5] * 5 - simulation_yaw_velocity) * dt;
        simulation_yaw = math.wrapAngle(simulation_yaw + simulation_yaw_velocity * dt);
        simulation_timestamp += 50_000_000;
        frame.* = std.mem.zeroes(Auv.Frame);
        frame.camera_pose = .{
            .pos = simulation_position,
            .quat = .{ 0, 0, @sin(simulation_yaw * 0.5), @cos(simulation_yaw * 0.5) },
        };
        frame.timestamp = simulation_timestamp;
        frame.tracking_ok = tracking_available;
        frame.position_timestamp = if (tracking_available) frame.timestamp else 0;
        frame.imu_quat = frame.camera_pose.quat;
        frame.imu_ok = true;
        frame.imu_timestamp = frame.timestamp;
        frame.gyro_ok = true;
        frame.gyro = .{ 0, 0, simulation_yaw_velocity };
        frame.pressure_timestamp = frame.timestamp;
        frame.observations_timestamp = frame.timestamp;
        if (!tracking_available) frame.camera_pose = .{ .pos = @splat(0), .quat = .{ 0, 0, 0, 1 } };
        frame.pressure_depth_ok = true;
        frame.pressure_depth = simulation_position[2];
        if (observe_object) {
            frame.objects_len = 1;
            frame.objects[0] = .{
                .id = 7,
                .cls = .cube,
                .bbox = .{ .pose = .{ .pos = .{ 0, 0, 1 }, .quat = .{ 0, 0, 0, 1 } }, .size = .{ 2, 2, 2 } },
            };
        }
    }
    fn setThrusters(values: [*c]const f32, len: u8) callconv(.c) void {
        outputs = @splat(0);
        for (0..len) |i| outputs[i] = values[i];
    }

    var tam: [6]math.Vector6f = .{
        .{ 1, 0, 0, 0, 0, 0 }, .{ 0, 1, 0, 0, 0, 0 },
        .{ 0, 0, 1, 0, 0, 0 }, .{ 0, 0, 0, 1, 0, 0 },
        .{ 0, 0, 0, 0, 1, 0 }, .{ 0, 0, 0, 0, 0, 1 },
    };

    fn context() MissionContext {
        var frame = std.mem.zeroes(Auv.Frame);
        frame.camera_pose.quat = .{ 0, 0, 0, 1 };
        frame.tracking_ok = true;
        frame.timestamp = 50_000_001;
        frame.position_timestamp = frame.timestamp;
        frame.observations_timestamp = frame.timestamp;
        frame.imu_quat = frame.camera_pose.quat;
        frame.imu_ok = true;
        frame.gyro_ok = true;
        frame.imu_timestamp = frame.timestamp;
        return .{
            .io = undefined,
            .frame = frame,
            .navigation = .{ .last_measured_position = @splat(0), .last_position_timestamp = 1 },
            .seen_objects = undefined,
            .seen_objects_len = 0,
            .goal_dist_threshold = 0.2,
            .thrustor_saturate = 5,
            .log_counter = 1,
            .log_freq_div = 120,
            .goal = @splat(0),
            .frame_previous_timestamp = 1,
            .auv = .{ .init = ControllerTestBackend.init, .deinit = deinit, .yieldUntilNextFrame = yieldFrame, .setThrustorValues = setThrusters },
            .auv_loader = undefined,
            .auv_watcher = undefined,
            .config = .{
                .tam = &tam,
                .odometry = .{ .kp = @splat(1), .ki = @splat(0), .kd = @splat(0) },
                .non_odometry = .{ .kp = @splat(1), .ki = @splat(0), .kd = @splat(0), .center_deadband = 0.05, .target_height = 0.9 },
                .control = .{ .legacy_wrench_scale = @splat(1) },
            },
            .config_loader = undefined,
            .config_watcher = undefined,
        };
    }

    fn simulationContext(position: math.Vector3f, yaw: f32) !MissionContext {
        simulation_active = true;
        simulation_position = position;
        simulation_velocity = @splat(0);
        simulation_yaw = yaw;
        simulation_yaw_velocity = 0;
        simulation_timestamp = 1;
        observe_object = true;
        tracking_available = true;
        outputs = @splat(0);
        var ctx = context();
        ctx.io = std.testing.io;
        ctx.log_freq_div = std.math.maxInt(u16);
        ctx.frame.camera_pose = .{ .pos = position, .quat = .{ 0, 0, @sin(yaw * 0.5), @cos(yaw * 0.5) } };
        ctx.frame.timestamp = simulation_timestamp;
        ctx.frame.position_timestamp = simulation_timestamp;
        ctx.frame.observations_timestamp = simulation_timestamp;
        ctx.frame.imu_timestamp = simulation_timestamp;
        ctx.navigation.update(&ctx.frame, ctx.config.navigation, 0);
        ctx.goal = math.poseTo6f(ctx.frame.camera_pose);
        ctx.frame_previous_timestamp = null;
        ctx.config.odometry.kp = .{ 3, 3, 3, 3, 3, 4 };
        ctx.config.odometry.kd = @splat(1);
        ctx.config_watcher = try FileWatcher.init(std.testing.allocator, ".motion-test-config");
        errdefer {
            _ = std.os.linux.close(ctx.config_watcher.fd);
            std.testing.allocator.free(ctx.config_watcher.buf);
        }
        ctx.auv_watcher = try FileWatcher.init(std.testing.allocator, ".motion-test-backend");
        return ctx;
    }

    fn finishSimulation(ctx: *MissionContext) void {
        simulation_active = false;
        _ = std.os.linux.close(ctx.config_watcher.fd);
        _ = std.os.linux.close(ctx.auv_watcher.fd);
        std.testing.allocator.free(ctx.config_watcher.buf);
        std.testing.allocator.free(ctx.auv_watcher.buf);
    }
};

test "controller strafes while facing an object and gates only align_first moves" {
    var ctx = ControllerTestBackend.context();
    ctx.goal[0] = 5;
    ctx.motion = Motion.State.init(.{ 5, 0, 0 }, 0, .{
        .facing = .{ .look_at_object = 7 },
        .translation = .align_first,
    });
    ctx.frame.objects_len = 1;
    ctx.frame.objects[0] = .{
        .id = 7,
        .cls = .cube,
        .bbox = .{ .pose = .{ .pos = .{ 3, 0, 0 }, .quat = .{ 0, 0, 0, 1 } }, .size = .{ 1, 1, 1 } },
    };
    ControllerStep(&ctx);
    try std.testing.expectEqual(@as(f32, 0), ControllerTestBackend.outputs[0]);
    try std.testing.expect(ControllerTestBackend.outputs[5] < 0);
    ctx.motion.?.options.translation = .simultaneous;
    ctx.frame.timestamp += 50_000_000;
    ControllerStep(&ctx);
    try std.testing.expect(ControllerTestBackend.outputs[0] > 0);
    try std.testing.expect(ControllerTestBackend.outputs[5] < 0);
}

test "pose yaw and captured heading agree without a ninety degree turn" {
    var ctx = ControllerTestBackend.context();
    ControllerStep(&ctx);
    try std.testing.expectEqual(@as(f32, 0), ControllerTestBackend.outputs[5]);
    ctx.beginMotion(.{ 5, 0, 0 }, .{ .facing = .hold_current });
    ctx.goal[0] = 5;
    ctx.frame.timestamp += 50_000_000;
    ControllerStep(&ctx);
    try std.testing.expectEqual(@as(f32, 0), ControllerTestBackend.outputs[5]);
    try std.testing.expect(ControllerTestBackend.outputs[0] > 0);
    ctx.setPoseGoal(.{ 5, 0, 0, 0, 0, 0.3 });
    try std.testing.expectEqual(@as(f32, 0.3), ctx.motion.?.options.facing.fixed_yaw);
    try std.testing.expectEqual(Motion.Translation.simultaneous, ctx.motion.?.options.translation);
    try std.testing.expectEqual(@as(f32, 0.3), ctx.goal[5]);
}

test "odometry loss pauses motion instead of issuing visual approach or search" {
    var ctx = ControllerTestBackend.context();
    ctx.goal[0] = 5;
    ctx.motion = Motion.State.init(.{ 5, 0, 0 }, 0, .{ .facing = .face_goal, .observation_timeout = 0.04 });
    ctx.frame.tracking_ok = false;
    ControllerStep(&ctx);
    try std.testing.expectEqual(Motion.Failure.tracking_lost, ctx.motion.?.failure.?);
    try std.testing.expectEqual(@as(f32, 0), ControllerTestBackend.outputs[0]);
    try std.testing.expectEqual(@as(f32, 0), ControllerTestBackend.outputs[1]);
    try std.testing.expectEqual(@as(f32, 0), ControllerTestBackend.outputs[5]);
    ctx.holdPosition();
    try std.testing.expect(ctx.motion != null);
    for (ControllerTestBackend.outputs) |value| try std.testing.expectEqual(@as(f32, 0), value);
}

test "closed loop orbit, XYZ heading hold and a stationary yaw move complete" {
    var ctx = try ControllerTestBackend.simulationContext(.{ 5, 0, 1 }, 0);
    defer ControllerTestBackend.finishSimulation(&ctx);
    try ctx.yieldUntilFollowPath(&.{ .{ 3, 3, 1 }, .{ 3, -3, 1 }, .{ -3, -3, 1 }, .{ -3, 3, 1 }, .{ 3, 3, 1 }, .{ 5, 0, 1 } }, .{
        .move = .{ .facing = .{ .look_at_object = 7 }, .position_tolerance = 0.15, .timeout = 180 },
    });
    try std.testing.expect(math.length3f(ctx.frame.camera_pose.pos - @as(math.Vector3f, .{ 5, 0, 1 })) <= 0.15);
    const final_yaw = math.poseTo6f(ctx.frame.camera_pose)[5];
    const object_yaw = Motion.yawForDirection(@as(math.Vector3f, .{ 0, 0, 1 }) - ctx.frame.camera_pose.pos, 0);
    try std.testing.expect(@abs(math.wrapAngle(final_yaw - object_yaw)) <= std.math.pi / 36.0);

    try ctx.yieldUntilReachGoal(.{ 5, 2, 2 }, .{
        .facing = .hold_current,
        .position_tolerance = 0.15,
        .timeout = 40,
    });
    try std.testing.expect(math.length3f(ctx.frame.camera_pose.pos - @as(math.Vector3f, .{ 5, 2, 2 })) <= 0.15);
    try std.testing.expect(@abs(math.wrapAngle(math.poseTo6f(ctx.frame.camera_pose)[5] - final_yaw)) <= std.math.pi / 36.0);

    // Even with zero dwell time and no position change, require_yaw must wait
    // for the requested final yaw rather than returning on the initial pose.
    try ctx.yieldUntilReachGoal(ctx.frame.camera_pose.pos, .{
        .facing = .{ .fixed_yaw = 2.8 },
        .position_tolerance = 0.15,
        .settle_time = 0,
        .timeout = 30,
    });
    try std.testing.expect(@abs(math.wrapAngle(math.poseTo6f(ctx.frame.camera_pose)[5] - 2.8)) <= std.math.pi / 36.0);
}

test "route returns observation errors and zeros outputs on object or tracking loss" {
    var ctx = try ControllerTestBackend.simulationContext(.{ 5, 0, 1 }, 0);
    defer ControllerTestBackend.finishSimulation(&ctx);
    ControllerTestBackend.observe_object = false;
    try std.testing.expectError(error.ObjectLost, ctx.yieldUntilReachGoal(.{ 5, 2, 1 }, .{
        .facing = .{ .look_at_object = 7 },
        .observation_timeout = 0.2,
        .timeout = 3,
    }));
    for (ControllerTestBackend.outputs) |value| try std.testing.expectEqual(@as(f32, 0), value);
    try std.testing.expect(ctx.motion.?.command == .route);
    try std.testing.expect(ctx.motion.?.options.facing == .look_at_object);
    ControllerTestBackend.tracking_available = false;
    try std.testing.expectError(error.TrackingLost, ctx.yieldUntilReachGoal(.{ 5, 2, 1 }, .{
        .facing = .hold_current,
        .observation_timeout = 0.2,
        .timeout = 3,
    }));
    for (ControllerTestBackend.outputs) |value| try std.testing.expectEqual(@as(f32, 0), value);
}

test "timeout and invalid options zero outputs while retaining the route" {
    var ctx = try ControllerTestBackend.simulationContext(.{ 5, 0, 1 }, 0);
    defer ControllerTestBackend.finishSimulation(&ctx);
    try std.testing.expectError(error.MoveTimeout, ctx.yieldUntilReachGoal(.{ 5, 10, 1 }, .{
        .facing = .hold_current,
        .timeout = 0.2,
    }));
    try std.testing.expect(ctx.motion.?.command == .route);
    try std.testing.expectEqual(Motion.Failure.move_timeout, ctx.motion.?.failure.?);
    for (ControllerTestBackend.outputs) |value| try std.testing.expectEqual(@as(f32, 0), value);
    try std.testing.expectError(error.InvalidMoveOptions, ctx.yieldUntilReachGoal(.{ 5, 10, 1 }, .{
        .facing = .hold_current,
        .yaw_rate_limit = 0,
    }));
    for (ControllerTestBackend.outputs) |value| try std.testing.expectEqual(@as(f32, 0), value);
}
