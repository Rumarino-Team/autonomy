const std = @import("std");
const c = @import("sim_c");
const model = @import("model.zig");

const gate = 0;
const marker = 1;
const opening = 2;
const wall = 3;
const floor = 4;
const bin = 5;
const board = 6;
const buoy = 7;

fn fail(error_out: ?[*]u8, capacity: u32, text: []const u8) void {
    const buffer = error_out orelse return;
    if (capacity == 0) return;
    const len = @min(text.len, capacity - 1);
    @memcpy(buffer[0..len], text[0..len]);
    buffer[len] = 0;
}

fn box(
    s: *c.Simulation,
    id: u32,
    name: []const u8,
    cls: u32,
    tracked: u32,
    collisions: u32,
    x: f32,
    y: f32,
    z: f32,
    hx: f32,
    hy: f32,
    hz: f32,
    error_out: ?[*]u8,
    capacity: u32,
) c.SimResult {
    var body: c.UnderwaterBody = undefined;
    model.underwater_body_default(&body);
    body.id = id;
    body.cls = cls;
    body.tracked = tracked;
    body.fuzz = if (id == 3) 1 else 0;
    body.initial_pose.pos = .{ .buf = .{ x, y, z } };
    @memset(&body.name, 0);
    @memcpy(body.name[0..name.len], name);
    var collider: c.Collider = undefined;
    model.collider_default(&collider);
    collider.data.size = .{ .buf = .{ hx, hy, hz } };
    collider.data.collisions = collisions;
    var object: ?*c.ObjectBlock = null;
    return model.simulation_add_object(s, &body, @ptrCast(&collider), 1, 0, &object, error_out, capacity);
}

export fn simulation_scene_create(
    s: ?*c.Simulation,
    scene: ?[*:0]const u8,
    robot: ?*const c.Robot,
    error_out: ?[*]u8,
    capacity: u32,
) callconv(.c) c.SimResult {
    const name = std.mem.span(scene orelse "");
    if (s == null or scene == null or (!std.mem.eql(u8, name, "robot_only") and !std.mem.eql(u8, name, "robosub_course"))) {
        fail(error_out, capacity, "unknown scene (robot_only or robosub_course)");
        return c.SIM_INVALID;
    }
    var config: c.SimulationConfig = undefined;
    var result = model.simulation_robot_config(robot, &config, error_out, capacity);
    if (result != c.SIM_OK) return result;
    result = model.simulation_init(s, &config, error_out, capacity);
    if (result != c.SIM_OK) return result;
    var start = std.mem.zeroes(c.MathPose);
    start.pos = .{ .buf = .{ 0, 2, 2 } };
    start.quat = .{ .buf = .{ 0, 0, 1, 0 } };
    result = model.simulation_add_robot(s, robot, 1, if (std.mem.eql(u8, name, "robosub_course")) &start else null, error_out, capacity);
    if (result != c.SIM_OK) {
        model.simulation_destroy(s);
        return result;
    }
    if (std.mem.eql(u8, name, "robosub_course")) {
        const boxes = [_]struct { id: u32, label: []const u8, cls: u32, tracked: u32, collisions: u32, x: f32, y: f32, z: f32, hx: f32, hy: f32, hz: f32 }{
            .{ .id = 2, .label = "gate opening", .cls = opening, .tracked = 1, .collisions = 0, .x = 1.0465, .y = 8, .z = 2.3935, .hx = 1.4915, .hy = 0.05, .hz = 0.8865 },
            .{ .id = 3, .label = "marker", .cls = marker, .tracked = 1, .collisions = 1, .x = 0.9985, .y = 20.1525, .z = 1.1675, .hx = 0.2175, .hy = 0.5335, .hz = 0.1895 },
            .{ .id = 10, .label = "gate left post", .cls = gate, .tracked = 1, .collisions = 1, .x = 2.573, .y = 8, .z = 2.220, .hx = 0.035, .hy = 0.040, .hz = 0.783 },
            .{ .id = 11, .label = "gate right post", .cls = gate, .tracked = 1, .collisions = 1, .x = -0.480, .y = 8, .z = 2.220, .hx = 0.035, .hy = 0.040, .hz = 0.783 },
            .{ .id = 12, .label = "gate top bar", .cls = gate, .tracked = 1, .collisions = 1, .x = 1.0465, .y = 8, .z = 1.472, .hx = 1.5615, .hy = 0.040, .hz = 0.035 },
            .{ .id = 100, .label = "Pool west wall", .cls = wall, .tracked = 1, .collisions = 1, .x = 6.45, .y = 10, .z = 1.64, .hx = 0.15, .hy = 14.3, .hz = 1.64 },
            .{ .id = 101, .label = "Pool east wall", .cls = wall, .tracked = 1, .collisions = 1, .x = -6.45, .y = 10, .z = 1.64, .hx = 0.15, .hy = 14.3, .hz = 1.64 },
            .{ .id = 102, .label = "Pool south wall", .cls = wall, .tracked = 1, .collisions = 1, .x = 0, .y = -4.45, .z = 1.64, .hx = 6.6, .hy = 0.15, .hz = 1.64 },
            .{ .id = 103, .label = "Pool north wall", .cls = wall, .tracked = 1, .collisions = 1, .x = 0, .y = 24.45, .z = 1.64, .hx = 6.6, .hy = 0.15, .hz = 1.64 },
            .{ .id = 104, .label = "Pool floor", .cls = floor, .tracked = 1, .collisions = 1, .x = 0, .y = 10, .z = 3.38, .hx = 6.6, .hy = 14.6, .hz = 0.1 },
            .{ .id = 110, .label = "Bin", .cls = bin, .tracked = 1, .collisions = 1, .x = 2.677, .y = 10.0755, .z = 2.931, .hx = 0.305, .hy = 0.4575, .hz = 0.079 },
            .{ .id = 111, .label = "Bin_Copy", .cls = bin, .tracked = 1, .collisions = 1, .x = 1.677, .y = 10.5755, .z = 2.931, .hx = 0.305, .hy = 0.4575, .hz = 0.079 },
            .{ .id = 112, .label = "Board", .cls = board, .tracked = 1, .collisions = 1, .x = 1.505, .y = 12, .z = 1, .hx = 0.005, .hy = 0.3, .hz = 0.3 },
            .{ .id = 113, .label = "GateSphere", .cls = buoy, .tracked = 1, .collisions = 1, .x = -3.264, .y = 18.936, .z = 1.673, .hx = 0.35, .hy = 0.35, .hz = 0.35 },
        };
        for (boxes) |item| {
            result = box(s.?, item.id, item.label, item.cls, item.tracked, item.collisions, item.x, item.y, item.z, item.hx, item.hy, item.hz, error_out, capacity);
            if (result != c.SIM_OK) {
                model.simulation_destroy(s);
                return result;
            }
        }
    }
    result = model.simulation_finalize(s, error_out, capacity);
    if (result == c.SIM_OK) return result;
    model.simulation_destroy(s);
    return result;
}
