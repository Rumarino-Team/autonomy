const std = @import("std");
const math = @import("math.zig");
const Motion = @import("Motion.zig");
const Navigation = @import("Navigation.zig");
const Context = @import("MissionContext.zig");
const Auv = @import("Auv.zig");
const Protocol = @import("TeleopProtocol.zig");
const net = std.Io.net;
const Server = @This();
const heap = std.heap.page_allocator;
const lease_seconds = 10;
const Cache = struct { id: u64, hash: u64, response: []u8 };
const Rotation = struct { initial: math.Quaternionf, axis: Protocol.Axis, angle: f32, rate: f32, progress: f32 = 0, settling: Motion.Settling = .{} };
const Follow = struct { id: u32, standoff: f32, depth: ?f32, duration: ?f32, seconds: f32 = 0, missing: f32 = 0 };
const Activity = union(enum) { motion, rotation: Rotation, follow: Follow };

peer: ?net.IpAddress = null,
session: [32]u8 = @splat(0),
nonce: [128]u8 = undefined,
nonce_len: usize = 0,
last_seen: i96 = 0,
highest_id: u64 = 0,
cache: [32]?Cache = @splat(null),
cache_next: usize = 0,
catalog: std.json.Value,
activity: ?Activity = null,
operation_id: ?u64 = null,
operation_session: [32]u8 = @splat(0),
operation_state: []const u8 = "idle",
operation_error: ?[]const u8 = null,
plan_arena: ?std.heap.ArenaAllocator = null,
steps: []const Protocol.Step = &.{},
step_index: usize = 0,
recapture_hold: bool = false,

fn now(ctx: *Context) i96 {
    return std.Io.Clock.Timestamp.now(ctx.io, .awake).raw.toNanoseconds();
}
fn alive(server: *const Server, ctx: *Context) bool {
    return server.peer != null and now(ctx) - server.last_seen < lease_seconds * std.time.ns_per_s;
}
fn clearCache(server: *Server) void {
    for (&server.cache) |*entry| {
        if (entry.*) |item| heap.free(item.response);
        entry.* = null;
    }
    server.highest_id = 0;
    server.cache_next = 0;
}
fn clearPlan(server: *Server) void {
    if (server.plan_arena) |*arena| arena.deinit();
    server.plan_arena = null;
    server.steps = &.{};
    server.step_index = 0;
}
fn deinit(server: *Server) void {
    server.clearCache();
    server.clearPlan();
}

fn objectMatches(selector: Protocol.Selector, id: u32, class: Auv.ObjectCls) bool {
    return (selector.id == null or selector.id.? == id) and (selector.class == null or selector.class.? == class);
}
fn selectObject(ctx: *Context, selector: Protocol.Selector) !Auv.Object {
    if (selector.id == null and selector.class == null) return error.MissingObjectSelector;
    if (!ctx.navigation.snapshot.measured_position or !ctx.navigation.snapshot.observations_valid) return error.ObservationsUnavailable;
    var closest: ?Auv.Object = null;
    var distance = std.math.inf(f32);
    for (ctx.frame.objects[0..@min(ctx.frame.objects_len, Auv.Frame.max_objects)]) |raw| {
        if (!objectMatches(selector, raw.id, raw.cls) or !Navigation.validPoint(raw.bbox.pose.pos)) continue;
        var object = raw;
        object.bbox.pose.pos = ctx.navigation.toMission(raw.bbox.pose.pos);
        object.bbox.pose.quat = math.normalize4f(math.quaternionMul(ctx.navigation.tracking_to_mission.quat, raw.bbox.pose.quat));
        const d = math.length3f(object.bbox.pose.pos - ctx.navigation.snapshot.position.?);
        if (d < distance) {
            closest = object;
            distance = d;
        }
    }
    return closest orelse error.ObjectNotFound;
}
fn selectImageObject(ctx: *Context, selector: Protocol.Selector) !u32 {
    if (selectObject(ctx, selector)) |object| return object.id else |_| {}
    if (!ctx.navigation.snapshot.observations_valid) return error.ObservationsUnavailable;
    if (selector.id == null and selector.class == null) return error.MissingObjectSelector;
    var selected: ?u32 = null;
    for (ctx.frame.objects2d[0..@min(ctx.frame.objects2d_len, Auv.Frame.max_objects)]) |object| {
        if (!objectMatches(selector, object.id, object.cls)) continue;
        if (selected != null) return error.AmbiguousObject;
        selected = object.id;
    }
    return selected orelse error.ObjectNotFound;
}
fn position(ctx: *Context) !math.Vector3f {
    if (!ctx.navigation.snapshot.measured_position) return error.TrackingUnavailable;
    return ctx.navigation.snapshot.position orelse error.TrackingUnavailable;
}
fn positive(value: f32) bool {
    return std.math.isFinite(value) and value > 0;
}
fn approachGoal(ctx: *Context, object: Auv.Object, standoff: f32, depth: ?f32) !math.Vector3f {
    if (!positive(standoff)) return error.InvalidStandoff;
    if (depth) |z| if (!std.math.isFinite(z) or z < 0) return error.InvalidDepth;
    const current = try position(ctx);
    const delta = math.xy(current - object.bbox.pose.pos);
    if (math.length2f(delta) < 1e-4) return error.UndefinedApproachDirection;
    const xy = math.xy(object.bbox.pose.pos) + math.normalize2f(delta) * @as(math.Vector2f, @splat(standoff));
    return .{ xy[0], xy[1], depth orelse current[2] };
}
fn facingSpecified(raw: std.json.Value, field: []const u8) bool {
    if (raw != .object) return false;
    const move = raw.object.get(field) orelse return false;
    return move == .object and move.object.contains("facing");
}
fn startPosition(ctx: *Context, goal: math.Vector3f, move: Motion.MoveOptions) !void {
    try ctx.startFollowPath(&.{goal}, .{ .move = move, .corner_radius = 0 });
}
fn commandValue(allocator: std.mem.Allocator, step: Protocol.Step) !std.json.Value {
    var object: std.json.ObjectMap = .{};
    try object.put(allocator, step.command, step.args);
    return .{ .object = object };
}

/// Start operations without waiting for their completion: UDP remains responsive.
fn dispatch(server: *Server, ctx: *Context, command: Protocol.Command, raw: std.json.Value, allocator: std.mem.Allocator) !bool {
    switch (command) {
        .move_to => |args| {
            try startPosition(ctx, args.position, args.move);
            server.activity = .motion;
        },
        .move_relative => |args| {
            if (!Navigation.validPoint(args.offset)) return error.InvalidOffset;
            const offset = if (args.frame == .body) math.quaternionRotate(ctx.navigation.snapshot.orientation orelse return error.SensorUnavailable, args.offset) else args.offset;
            try startPosition(ctx, try position(ctx) + offset, args.move);
            server.activity = .motion;
        },
        .follow_path => |args| {
            try ctx.startFollowPath(args.waypoints, args.options);
            server.activity = .motion;
        },
        .move_to_object => |args| {
            const object = try selectObject(ctx, args.object);
            var move = args.move;
            if (!facingSpecified(raw, "move")) move.facing = .{ .look_at_object = object.id };
            try startPosition(ctx, try approachGoal(ctx, object, args.standoff, args.depth), move);
            server.activity = .motion;
        },
        .follow_object => |args| {
            if (args.duration) |duration| if (!positive(duration)) return error.InvalidDuration;
            const object = try selectObject(ctx, args.object);
            var move = args.move;
            if (!facingSpecified(raw, "move")) move.facing = .{ .look_at_object = object.id };
            try startPosition(ctx, try approachGoal(ctx, object, args.standoff, args.depth), move);
            ctx.motion.?.command = .pose;
            server.activity = .{ .follow = .{ .id = object.id, .standoff = args.standoff, .depth = args.depth, .duration = args.duration } };
        },
        .orbit_object => |args| {
            if (!positive(args.clearance)) return error.InvalidClearance;
            const object = try selectObject(ctx, args.object);
            const waypoints = @import("missions/prequalify.zig").orbitWaypointsWithClearance(&object, try position(ctx), args.clearance);
            var options = args.options;
            const raw_options = if (raw == .object) raw.object.get("options") else null;
            if (raw_options == null or !facingSpecified(raw_options.?, "move")) options.move.facing = .{ .look_at_object = object.id };
            try ctx.startFollowPath(&waypoints, options);
            server.activity = .motion;
        },
        .pass_gate => |args| {
            if (!positive(args.approach_distance) or !positive(args.exit_distance)) return error.InvalidGateDistances;
            const object = try selectObject(ctx, args.object);
            if (object.cls != .gate) return error.NotAGate;
            const delta = math.xy(object.bbox.pose.pos - try position(ctx));
            if (math.length2f(delta) < 1e-4) return error.UndefinedApproachDirection;
            const direction = math.normalize2f(delta);
            const before = math.xy(object.bbox.pose.pos) - direction * @as(math.Vector2f, @splat(args.approach_distance));
            const after = math.xy(object.bbox.pose.pos) + direction * @as(math.Vector2f, @splat(args.exit_distance));
            var options = args.options;
            options.move.facing = .{ .fixed_yaw = Motion.yawForDirection(.{ direction[0], direction[1], 0 }, 0) };
            try ctx.startFollowPath(&.{ .{ before[0], before[1], object.bbox.pose.pos[2] }, .{ after[0], after[1], object.bbox.pose.pos[2] } }, options);
            server.activity = .motion;
        },
        .look_at_object => |args| {
            var move = args.move;
            move.facing = .{ .look_at_object = (try selectObject(ctx, args.object)).id };
            try startPosition(ctx, try position(ctx), move);
            server.activity = .motion;
        },
        .look_at_point => |args| {
            var move = args.move;
            move.facing = .{ .look_at_point = args.point };
            try startPosition(ctx, try position(ctx), move);
            server.activity = .motion;
        },
        .set_yaw => |args| {
            var move = args.move;
            move.facing = .{ .fixed_yaw = args.yaw };
            try startPosition(ctx, try position(ctx), move);
            server.activity = .motion;
        },
        .set_attitude => |args| {
            var move = args.move;
            move.facing = .hold_current;
            move.attitude = args.orientation;
            try startPosition(ctx, try position(ctx), move);
            server.activity = .motion;
        },
        .rotate => |args| {
            if (!std.math.isFinite(args.angle) or @abs(args.angle) > 32 * std.math.pi or !positive(args.rate)) return error.InvalidRotation;
            const initial = ctx.navigation.snapshot.orientation orelse return error.SensorUnavailable;
            var move = args.move;
            move.facing = .hold_current;
            move.attitude = initial;
            try startPosition(ctx, try position(ctx), move);
            ctx.motion.?.command = .pose;
            server.activity = .{ .rotation = .{ .initial = initial, .axis = args.axis, .angle = args.angle, .rate = args.rate } };
        },
        .search => |args| {
            try ctx.startSearch(args);
            server.activity = .motion;
        },
        .visual_target => |args| {
            const id = try selectImageObject(ctx, args.object orelse .{ .id = args.options.object_id });
            var options = args.options;
            options.object_id = id;
            var move = args.move;
            move.facing = .{ .look_at_object = id };
            try ctx.startVisualTarget(options, move);
            server.activity = .motion;
        },
        .body_move => |args| {
            try ctx.startBodyMove(args.options, args.move);
            server.activity = .motion;
        },
        .hold, .cancel => {
            ctx.holdPosition();
            return false;
        },
        .stop => {
            ctx.motion = null;
            ctx.stopThrusters();
            return false;
        },
        .pause => {
            if (ctx.motion) |*motion| motion.operator_paused = true else return error.NoActiveMotion;
            ctx.stopThrusters();
            return false;
        },
        .@"resume" => {
            try ctx.resumeMotion();
            if (server.operation_id != null) {
                server.operation_state = "running";
                server.operation_error = null;
            }
            return false;
        },
        .status, .objects => return false,
        .configure_motion => |args| {
            const motion = if (ctx.motion) |*motion| motion else return error.NoActiveMotion;
            const options = try Protocol.patch(Motion.MoveOptions, allocator, motion.options, args.options);
            if (server.activity != null and server.activity.? == .rotation and std.mem.eql(u8, server.operation_state, "running")) {
                if (options.attitude == null or options.facing != .hold_current or options.attitude_policy != .fixed or
                    math.length3f(math.quaternionError(motion.options.attitude.?, options.attitude.?)) > 1e-5) return error.RotationOwnsAttitude;
            }
            if (motion.command == .visual or motion.command == .body) {
                try options.validate();
                if (options.attitude_policy != .fixed) return error.UnsupportedAttitudePolicy;
                if (motion.command == .visual and (options.facing != .look_at_object or options.facing.look_at_object != motion.command.visual.object_id)) return error.InvalidMoveOptions;
                if (motion.command == .body and options.facing != .hold_current and options.facing != .fixed_yaw) return error.InvalidMoveOptions;
            } else try ctx.validateWorldMove(options);
            if (options.facing == .hold_current and motion.options.facing != .hold_current) motion.captured_yaw = ctx.navigation.snapshot.yaw() orelse return error.SensorUnavailable;
            motion.options = options;
            ctx.controller.reset();
            return false;
        },
        .configure_navigation => |args| {
            const options = try Protocol.patch(Navigation.Options, allocator, ctx.config.navigation, args.options);
            try options.validate();
            ctx.holdPosition();
            try ctx.reanchorLocalization(ctx.navigation.tracking_to_mission, ctx.frame.localization_epoch);
            ctx.config.navigation = options;
            server.recapture_hold = true;
            return false;
        },
        .reanchor => |args| {
            try ctx.reanchorLocalization(args.transform, args.epoch);
            return false;
        },
        .to_mission => |args| {
            if (!Navigation.validPoint(args.position)) return error.InvalidPosition;
            return false;
        },
        .plan => return error.NestedPlan,
    }
    return true;
}

fn query(command: Protocol.Command) bool {
    return switch (command) {
        .status, .objects, .to_mission, .pause, .@"resume", .configure_motion, .reanchor => true,
        else => false,
    };
}
fn planStepAllowed(command: Protocol.Command) bool {
    return switch (command) {
        .move_to, .move_relative, .follow_path, .move_to_object, .orbit_object, .pass_gate, .look_at_object, .look_at_point, .set_yaw, .set_attitude, .rotate, .search, .visual_target, .body_move, .follow_object => true,
        else => false,
    };
}

fn status(server: *Server, ctx: *Context, allocator: std.mem.Allocator) !std.json.Value {
    var objects: std.ArrayList(struct { id: u32, class: Auv.ObjectCls, position: math.Vector3f, size: math.Vector3f }) = .empty;
    if (ctx.navigation.snapshot.observations_valid and ctx.navigation.snapshot.measured_position) {
        for (ctx.frame.objects[0..@min(ctx.frame.objects_len, Auv.Frame.max_objects)]) |object| {
            if (!Navigation.validPoint(object.bbox.pose.pos)) continue;
            try objects.append(allocator, .{ .id = object.id, .class = object.cls, .position = ctx.navigation.toMission(object.bbox.pose.pos), .size = object.bbox.size });
        }
    }
    const motion = if (ctx.motion) |*m| try Protocol.jsonValue(allocator, .{ .command = @tagName(m.command), .mode = m.mode, .paused = m.operator_paused, .recovering = m.recovering, .completed = m.completed, .failure = m.failure, .elapsed = m.elapsed, .goal = ctx.goal, .options = m.options, .route_progress = if (m.command == .route) m.command.route.distance else @as(?f32, null), .route_length = if (m.command == .route) m.command.route.path.total_length else @as(?f32, null) }) else @as(std.json.Value, .null);
    return Protocol.jsonValue(allocator, .{
        .operation_id = server.operation_id,
        .operation_session = if (server.operation_id != null) @as(?[]const u8, &server.operation_session) else null,
        .state = server.operation_state,
        .@"error" = server.operation_error,
        .plan_step = server.step_index,
        .plan_steps = server.steps.len,
        .motion = motion,
        .navigation = ctx.navigation.snapshot,
        .navigation_options = ctx.config.navigation,
        .tracking_to_mission = ctx.navigation.tracking_to_mission,
        .localization_epoch = ctx.navigation.epoch,
        .objects = objects.items,
        .image_objects = if (ctx.navigation.snapshot.observations_valid) ctx.frame.objects2d[0..@min(ctx.frame.objects2d_len, Auv.Frame.max_objects)] else &.{},
        .rotation_progress = if (server.activity != null and server.activity.? == .rotation) @as(?f32, server.activity.?.rotation.progress) else null,
        .adaptive_pitch = ctx.controller.adaptive_pitch,
        .pitch_adapting = ctx.controller.pitch_adapting,
        .pitch_limited = ctx.controller.pitch_limited,
        .allocation_residual = if (ctx.allocation_result) |result| @as(?math.Vector6f, result.residual) else null,
    });
}

fn response(server: *Server, allocator: std.mem.Allocator, kind: []const u8, id: u64, failure: ?[]const u8, data: std.json.Value) ![]u8 {
    const bytes = try std.json.Stringify.valueAlloc(allocator, .{ .version = Protocol.version, .kind = kind, .id = id, .session = if (server.peer != null) @as(?[]const u8, &server.session) else null, .ok = failure == null, .@"error" = failure, .data = data }, .{});
    if (bytes.len > Protocol.max_datagram) return error.ResponseTooLarge;
    return bytes;
}
fn send(server: *Server, ctx: *Context, socket: *const net.Socket, peer: net.IpAddress, kind: []const u8, id: u64, failure: ?[]const u8, data: std.json.Value, allocator: std.mem.Allocator) ![]u8 {
    const bytes = try server.response(allocator, kind, id, failure, data);
    socket.send(ctx.io, &peer, bytes) catch |err| std.log.warn("teleop response send failed: {s}", .{@errorName(err)});
    return bytes;
}
fn event(server: *Server, ctx: *Context, socket: *const net.Socket) !void {
    if (!server.alive(ctx) or server.operation_id == null) return;
    var arena: std.heap.ArenaAllocator = .init(heap);
    defer arena.deinit();
    _ = try server.send(ctx, socket, server.peer.?, "event", server.operation_id.?, null, try server.status(ctx, arena.allocator()), arena.allocator());
}

fn acceptedCommand(server: *Server, ctx: *Context, command: Protocol.Command, raw: std.json.Value, allocator: std.mem.Allocator) !bool {
    if (command != .plan) return server.dispatch(ctx, command, raw, allocator);
    const steps = command.plan.steps;
    if (steps.len == 0 or steps.len > Protocol.max_plan_steps) return error.InvalidPlanLength;
    var arena: std.heap.ArenaAllocator = .init(heap);
    errdefer arena.deinit();
    const owned = try Protocol.jsonValue(arena.allocator(), steps);
    const copied = try std.json.parseFromValueLeaky([]const Protocol.Step, arena.allocator(), owned, .{});
    for (copied, 0..) |step, i| {
        const parsed = try Protocol.parseCommand(arena.allocator(), try commandValue(arena.allocator(), step));
        if (!planStepAllowed(parsed)) return error.InvalidPlanCommand;
        if (parsed == .follow_object and parsed.follow_object.duration == null and i + 1 < copied.len) return error.UnboundedFollowInPlan;
    }
    const first = try Protocol.parseCommand(arena.allocator(), try commandValue(arena.allocator(), copied[0]));
    _ = try server.dispatch(ctx, first, copied[0].args, arena.allocator());
    server.clearPlan();
    server.plan_arena = arena;
    server.steps = copied;
    server.step_index = 0;
    return true;
}

fn packet(server: *Server, ctx: *Context, socket: *const net.Socket, incoming: net.IncomingMessage) !void {
    var arena: std.heap.ArenaAllocator = .init(heap);
    defer arena.deinit();
    const allocator = arena.allocator();
    if (incoming.flags.trunc) {
        _ = try server.send(ctx, socket, incoming.from, "reply", 0, "DatagramTooLarge", .null, allocator);
        return;
    }
    const request = std.json.parseFromSliceLeaky(Protocol.Packet, allocator, incoming.data, .{}) catch |err| {
        _ = try server.send(ctx, socket, incoming.from, "reply", 0, @errorName(err), .null, allocator);
        return;
    };
    if (request.version != Protocol.version) {
        _ = try server.send(ctx, socket, incoming.from, "reply", request.id, "UnsupportedVersion", .null, allocator);
        return;
    }
    if (request.kind == .hello) {
        const nonce = request.nonce orelse "";
        if (nonce.len == 0 or nonce.len > server.nonce.len or request.id != 0) {
            _ = try server.send(ctx, socket, incoming.from, "welcome", request.id, "InvalidHandshake", .null, allocator);
            return;
        }
        if (server.alive(ctx) and !server.peer.?.eql(&incoming.from)) {
            _ = try server.send(ctx, socket, incoming.from, "welcome", 0, "SessionBusy", .null, allocator);
            return;
        }
        if (!server.alive(ctx) or server.nonce_len != nonce.len or !std.mem.eql(u8, server.nonce[0..server.nonce_len], nonce)) {
            server.clearCache();
            var random: [16]u8 = undefined;
            try ctx.io.randomSecure(&random);
            const hex = std.fmt.bytesToHex(random, .lower);
            server.session = hex;
            @memcpy(server.nonce[0..nonce.len], nonce);
            server.nonce_len = nonce.len;
        }
        server.peer = incoming.from;
        server.last_seen = now(ctx);
        var initial_status = try server.status(ctx, allocator);
        // Keep capability discovery in one bounded datagram even at the full
        // object capacity. Full observations are available through status.
        _ = initial_status.object.swapRemove("objects");
        _ = initial_status.object.swapRemove("image_objects");
        _ = try server.send(ctx, socket, incoming.from, "welcome", 0, null, try Protocol.jsonValue(allocator, .{ .nonce = nonce, .lease_seconds = lease_seconds, .max_datagram = Protocol.max_datagram, .commands = server.catalog.object.get("commands").?, .definitions = server.catalog.object.get("definitions").?, .status = initial_status }), allocator);
        return;
    }
    if (!server.alive(ctx) or !server.peer.?.eql(&incoming.from) or request.session == null or !std.mem.eql(u8, request.session.?, &server.session)) {
        _ = try server.send(ctx, socket, incoming.from, "reply", request.id, "HandshakeRequired", .null, allocator);
        return;
    }
    server.last_seen = now(ctx);
    const hash = std.hash.Wyhash.hash(0, incoming.data);
    for (server.cache) |entry| if (entry) |cached| {
        if (cached.id == request.id) {
            if (cached.hash == hash) socket.send(ctx.io, &incoming.from, cached.response) catch {} else {
                _ = try server.send(ctx, socket, incoming.from, "reply", request.id, "RequestIdConflict", .null, allocator);
            }
            return;
        }
    };
    if (request.id == 0 or request.id <= server.highest_id) {
        _ = try server.send(ctx, socket, incoming.from, "reply", request.id, "StaleRequest", .null, allocator);
        return;
    }
    // Reserve before dispatch, so even a serialization/allocation failure can
    // never turn a retransmission into a second physical command.
    server.highest_id = request.id;
    var failure: ?[]const u8 = null;
    var data: std.json.Value = .null;
    if (request.kind == .request) blk: {
        if (request.command) |source| {
            const command = Protocol.parseCommand(allocator, source) catch |err| {
                failure = @errorName(err);
                break :blk;
            };
            // A well-formed Command is a one-key tagged object.
            var iterator = source.object.iterator();
            const raw = iterator.next().?.value_ptr.*;
            const previous_id = server.operation_id;
            const previous_state = server.operation_state;
            const previous_session = server.operation_session;
            const is_query = query(command);
            const asynchronous = server.acceptedCommand(ctx, command, raw, allocator) catch |err| {
                failure = @errorName(err);
                break :blk;
            };
            if (!is_query) {
                if (previous_id != null and std.mem.eql(u8, previous_state, "running")) {
                    const next_activity = server.activity;
                    server.operation_id = previous_id;
                    server.operation_session = previous_session;
                    server.operation_state = "cancelled";
                    try server.event(ctx, socket);
                    server.activity = next_activity;
                }
                if (command != .plan) server.clearPlan();
                server.operation_id = request.id;
                server.operation_session = server.session;
                server.operation_state = if (asynchronous) "running" else "completed";
                server.operation_error = null;
                if (!asynchronous) server.activity = null;
            }
            if (command == .to_mission) data = try Protocol.jsonValue(allocator, .{ .position = ctx.navigation.toMission(command.to_mission.position) }) else data = try server.status(ctx, allocator);
        } else failure = "MissingCommand";
    } else if (request.kind == .heartbeat) data = try server.status(ctx, allocator);
    const bytes = try server.send(ctx, socket, incoming.from, "reply", request.id, failure, data, allocator);
    if (server.cache[server.cache_next]) |old| heap.free(old.response);
    server.cache[server.cache_next] = .{ .id = request.id, .hash = hash, .response = try heap.dupe(u8, bytes) };
    server.cache_next = (server.cache_next + 1) % server.cache.len;
    server.highest_id = request.id;
    if (request.kind == .bye) server.peer = null;
}

fn update(server: *Server, ctx: *Context, socket: *const net.Socket) !void {
    if (server.recapture_hold and ctx.navigation.snapshot.measured_position and ctx.navigation.snapshot.orientation != null) {
        ctx.holdPosition();
        server.recapture_hold = false;
    }
    if (server.activity == null or !std.mem.eql(u8, server.operation_state, "running")) return;
    if (ctx.motion == null) {
        server.operation_state = "cancelled";
        try server.event(ctx, socket);
        return;
    }
    if (ctx.motion.?.operator_paused) return;
    if (ctx.motion.?.failure) |failure| {
        server.operation_state = "failed";
        server.operation_error = @tagName(failure);
        ctx.stopThrusters();
        try server.event(ctx, socket);
        return;
    }
    const dt = std.math.clamp(ctx.controller_dt, 0, 0.1);
    var complete = false;
    switch (server.activity.?) {
        .motion => complete = ctx.motion.?.completed,
        .follow => |*follow| {
            follow.seconds += dt;
            if (follow.duration != null and follow.seconds >= follow.duration.?) {
                ctx.holdPosition();
                complete = true;
            } else if (selectObject(ctx, .{ .id = follow.id })) |object| {
                follow.missing = 0;
                const goal = approachGoal(ctx, object, follow.standoff, follow.depth) catch {
                    return;
                };
                ctx.motion.?.destination = goal;
                inline for (0..3) |i| ctx.goal[i] = goal[i];
            } else |_| {
                follow.missing += dt;
                if (follow.missing >= ctx.motion.?.options.observation_timeout and !ctx.motion.?.recovering) ctx.motion.?.failure = .object_lost;
            }
        },
        .rotation => |*rotation| {
            const actual = ctx.navigation.snapshot.orientation orelse return;
            const rate = ctx.navigation.snapshot.angular_velocity_body orelse return;
            if (ctx.motion.?.recovering) return;
            if (math.length3f(math.quaternionError(actual, ctx.motion.?.options.attitude.?)) <= 0.2) {
                const remaining = @abs(rotation.angle - rotation.progress);
                const increment = rotation.rate * dt;
                if (remaining <= increment) rotation.progress = rotation.angle else rotation.progress += std.math.sign(rotation.angle) * increment;
            }
            const half = rotation.progress / 2;
            const turn: math.Quaternionf = switch (rotation.axis) {
                .roll => .{ @sin(half), 0, 0, @cos(half) },
                .pitch => .{ 0, @sin(half), 0, @cos(half) },
                .yaw => .{ 0, 0, @sin(half), @cos(half) },
            };
            const desired = math.normalize4f(math.quaternionMul(rotation.initial, turn));
            ctx.motion.?.options.attitude = desired;
            const aligned = math.length3f(math.quaternionError(actual, desired)) <= ctx.motion.?.options.attitude_tolerance;
            complete = rotation.settling.update(rotation.progress == rotation.angle and aligned, math.length3f(rate) <= ctx.motion.?.options.angular_rate_tolerance, dt, ctx.motion.?.options.settle_time);
            if (complete) ctx.holdPosition();
        },
    }
    if (!complete) return;
    if (server.step_index + 1 < server.steps.len) {
        server.step_index += 1;
        const step = server.steps[server.step_index];
        const allocator = server.plan_arena.?.allocator();
        const command = try Protocol.parseCommand(allocator, try commandValue(allocator, step));
        _ = server.dispatch(ctx, command, step.args, allocator) catch |err| {
            server.operation_state = "failed";
            server.operation_error = @errorName(err);
            ctx.motion = null;
            ctx.stopThrusters();
            try server.event(ctx, socket);
            return;
        };
        try server.event(ctx, socket);
    } else {
        server.operation_state = "completed";
        try server.event(ctx, socket);
    }
}

pub fn run(ctx: *Context) !void {
    var arena: std.heap.ArenaAllocator = .init(heap);
    defer arena.deinit();
    var server: Server = .{ .catalog = try Protocol.catalog(arena.allocator()) };
    defer server.deinit();
    const address: net.IpAddress = .{ .ip4 = net.Ip4Address.loopback(ctx.config.teleop_port) };
    const socket = try address.bind(ctx.io, .{ .mode = .dgram, .protocol = .udp });
    defer socket.close(ctx.io);
    std.log.info("teleop command protocol v{d} listening on 127.0.0.1:{d}", .{ Protocol.version, ctx.config.teleop_port });
    var buffer: [Protocol.max_datagram]u8 = undefined;
    const timeout: std.Io.Timeout = .{ .duration = .{ .clock = .awake, .raw = .fromNanoseconds(0) } };
    while (true) {
        for (0..8) |_| {
            const incoming = socket.receiveTimeout(ctx.io, &buffer, timeout) catch |err| switch (err) {
                error.Timeout => break,
                else => return err,
            };
            server.packet(ctx, &socket, incoming) catch |err| std.log.warn("teleop packet failed: {s}", .{@errorName(err)});
        }
        try ctx.yieldUntilNextFrameAndUpdate();
        try server.update(ctx, &socket);
    }
}
