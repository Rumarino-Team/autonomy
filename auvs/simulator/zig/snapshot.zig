//! AUVDATA1 scene import. Import memory is private and dies after model buffers are copied.
const std = @import("std");
const c = @import("sim_c");
const S = @import("support.zig");
const Binary = @import("binary_data");
const Value = Binary.Value;

fn need(value: *const Value, name: []const u8) !*const Value {
    return value.field(name) orelse S.boom(c.SIM_INVALID, "missing snapshot field");
}
fn number(value: *const Value) !f64 {
    const n: f64 = switch (value.*) {
        .signed => |n| @floatFromInt(n),
        .unsigned => |n| @floatFromInt(n),
        .float => |n| n,
        else => return S.boom(c.SIM_INVALID, "expected number"),
    };
    try S.require(std.math.isFinite(n), "non-finite snapshot number", c.SIM_INVALID);
    return n;
}
fn integer(value: *const Value, max: u32, message: []const u8) !u32 {
    const n = try number(value);
    try S.require(n >= 0 and n <= @as(f64, @floatFromInt(max)) and n == @floor(n), message, c.SIM_INVALID);
    return @intFromFloat(n);
}
fn flag(value: *const Value) !u32 {
    if (value.* == .boolean) return @intFromBool(value.boolean);
    return @intFromBool(try number(value) != 0);
}
fn array(value: *const Value) ![]Value {
    try S.require(value.* == .sequence, "expected snapshot array", c.SIM_INVALID);
    return value.sequence;
}
fn vector(value: *const Value) !c.MathVector3f {
    const items = try array(value);
    try S.require(items.len == 3, "expected a 3-vector", c.SIM_INVALID);
    return S.v(try number(&items[0]), try number(&items[1]), try number(&items[2]));
}
fn pose(value: *const Value) !c.MathPose {
    const orientation = try array(try need(value, "orientation"));
    try S.require(orientation.len == 4, "expected an xyzw quaternion", c.SIM_INVALID);
    return .{
        .pos = try vector(try need(value, "position")),
        .quat = S.q(try number(&orientation[0]), try number(&orientation[1]), try number(&orientation[2]), try number(&orientation[3])),
    };
}
fn shape(allocator: std.mem.Allocator, value: *const Value) !c.Collider {
    var collider: c.Collider = undefined;
    c.collider_default(&collider);
    collider.data.kind = try integer(try need(value, "kind"), 3, "unknown shape");
    collider.data.pose = try pose(try need(value, "pose"));
    collider.data.collisions = try flag(try need(value, "collisions"));
    if (value.field("friction")) |f| collider.data.friction = try number(f);
    if (value.field("restitution")) |f| collider.data.restitution = try number(f);
    if (collider.data.kind != c.SIM_MESH) {
        collider.data.size = try vector(try need(value, "size"));
        return collider;
    }
    const points = try array(try need(value, "vertices"));
    const faces = try array(try need(value, "triangles"));
    const positions = try allocator.alloc(c.MathVector3f, points.len);
    const triangles = try allocator.alloc(c.SimTriangle, faces.len);
    for (points, positions) |*point, *out| out.* = try vector(point);
    for (faces, triangles) |*face, *out| {
        const indices = try array(face);
        try S.require(indices.len == 3, "triangle needs 3 indices", c.SIM_INVALID);
        out.* = .{
            .a = try integer(&indices[0], std.math.maxInt(u32), "invalid triangle index"),
            .b = try integer(&indices[1], std.math.maxInt(u32), "invalid triangle index"),
            .c = try integer(&indices[2], std.math.maxInt(u32), "invalid triangle index"),
        };
        try S.require(out.a < positions.len and out.b < positions.len and out.c < positions.len, "triangle index out of range", c.SIM_INVALID);
    }
    collider.geometry = .{
        .positions = positions.ptr,
        .position_count = @intCast(positions.len),
        .triangles = triangles.ptr,
        .triangle_count = @intCast(triangles.len),
    };
    return collider;
}
const StoredBody = struct { body: c.UnderwaterBody, colliders: []c.Collider = &.{}, vehicle: bool };
fn body(allocator: std.mem.Allocator, value: *const Value, robot_name: []const u8) !StoredBody {
    var out: StoredBody = .{ .body = undefined, .vehicle = false };
    c.underwater_body_default(&out.body);
    out.body.id = try integer(try need(value, "id"), std.math.maxInt(u32) - 1, "invalid body id");
    const name = try need(value, "name");
    try S.require(name.* == .bytes, "invalid body name", c.SIM_INVALID);
    try S.require(name.bytes.len > 0 and name.bytes.len < out.body.name.len, "invalid body name", c.SIM_INVALID);
    try S.text(&out.body.name, name.bytes);
    out.body.cls = try integer(try need(value, "class"), 255, "class must fit uint8");
    out.body.motion_type = try integer(try need(value, "motion"), 2, "invalid motion");
    out.body.tracked = try flag(try need(value, "tracked"));
    out.body.fuzz = try flag(try need(value, "fuzz"));
    out.body.initial_pose = try pose(try need(value, "pose"));
    const profile = value.field("profile");
    out.vehicle = std.mem.eql(u8, name.bytes, robot_name) or
        (profile != null and profile.?.* == .bytes and std.mem.eql(u8, profile.?.bytes, robot_name));
    if (!out.vehicle) {
        try S.require(out.body.motion_type != c.SIM_DYNAMIC, "only the vehicle may be dynamic", c.SIM_INVALID);
        const shapes = try array(try need(value, "shapes"));
        try S.require(shapes.len > 0, "scenery body has no geometry", c.SIM_INVALID);
        out.colliders = try allocator.alloc(c.Collider, shapes.len);
        for (shapes, out.colliders) |*item, *collider| collider.* = try shape(allocator, item);
    }
    return out;
}
fn addCount(a: u32, b: u32) !u32 {
    return std.math.add(u32, a, b) catch S.boom(c.SIM_INVALID, "snapshot capacity overflow");
}
fn load(simulation: ?*c.Simulation, path: ?[*:0]const u8, robot: ?*const c.Robot) !void {
    try S.require(simulation != null and path != null and robot != null, "missing snapshot arguments", c.SIM_INVALID);
    var memory = std.heap.ArenaAllocator.init(std.heap.c_allocator);
    defer memory.deinit();
    const allocator = memory.allocator();
    var io_impl = std.Io.Threaded.init(std.heap.c_allocator, .{});
    defer io_impl.deinit();
    const bytes = std.Io.Dir.cwd().readFileAlloc(io_impl.io(), std.mem.span(path.?), allocator, .limited(Binary.max_bytes + 29)) catch |err| {
        if (err == error.OutOfMemory) return err;
        return S.boom(c.SIM_BACKEND_ERROR, @errorName(err));
    };
    const document = Binary.decodeDynamic(allocator, bytes, .scene, 1) catch |err| {
        if (err == error.OutOfMemory) return err;
        return S.boom(c.SIM_BACKEND_ERROR, @errorName(err));
    };
    try S.require(try number(try need(&document, "version")) == 1, "unsupported snapshot version", c.SIM_INVALID);
    const values = try array(try need(&document, "bodies"));
    try S.require(values.len > 0 and values.len <= 100000, "invalid snapshot body count", c.SIM_INVALID);
    const stored = try allocator.alloc(StoredBody, values.len);
    var start: ?c.MathPose = null;
    var scenery: u32 = 0;
    var colliders: u32 = 0;
    var vertices: u32 = 0;
    var triangles: u32 = 0;
    for (values, stored) |*value, *item| {
        item.* = try body(allocator, value, std.mem.sliceTo(&robot.?.name, 0));
        if (item.vehicle) {
            try S.require(start == null, "snapshot has two vehicles", c.SIM_INVALID);
            start = item.body.initial_pose;
            continue;
        }
        try S.require(item.body.id != 1, "body id 1 is reserved for the vehicle", c.SIM_INVALID);
        scenery += 1;
        colliders = try addCount(colliders, @intCast(item.colliders.len));
        for (item.colliders) |collider| {
            vertices = try addCount(vertices, collider.geometry.position_count);
            triangles = try addCount(triangles, collider.geometry.triangle_count);
        }
    }
    var config: c.SimulationConfig = undefined;
    var message: [512]u8 = @splat(0);
    try S.check(c.simulation_robot_config(robot, &config, &message, message.len), S.span(&message));
    config.object_capacity = @max(config.object_capacity, scenery + 2);
    config.collider_capacity = try addCount(config.collider_capacity, colliders);
    config.vertex_capacity = try addCount(config.vertex_capacity, vertices);
    config.triangle_capacity = try addCount(config.triangle_capacity, triangles);
    try S.check(c.simulation_init(simulation, &config, &message, message.len), S.span(&message));
    errdefer c.simulation_destroy(simulation);
    try S.check(c.simulation_add_robot(simulation, robot, 1, if (start) |*p| p else null, &message, message.len), S.span(&message));
    for (stored) |*item| {
        if (item.vehicle) continue;
        var object: ?*c.ObjectBlock = null;
        try S.check(c.simulation_add_object(simulation, &item.body, item.colliders.ptr, @intCast(item.colliders.len), 0, &object, &message, message.len), S.span(&message));
    }
    try S.check(c.simulation_finalize(simulation, &message, message.len), S.span(&message));
}
pub export fn simulation_scene_load(simulation: ?*c.Simulation, path: ?[*:0]const u8, robot: ?*const c.Robot, err: ?[*]u8, capacity: u32) callconv(.c) c.SimResult {
    return S.guard(err, capacity, load(simulation, path, robot));
}
