const std = @import("std");
const c = @import("sim_c");
const S = @import("support.zig");
const data = @import("robot_data.zig");

extern fn simulation_config_default(out: *c.SimulationConfig) callconv(.c) void;
extern "c" fn strcmp(a: [*:0]const u8, b: [*:0]const u8) c_int;

const templates = [_]*const c.Robot{ &data.hydrus, &data.proteus, &data.bluerov, &data.girona500 };

pub export fn sim_robot_count() callconv(.c) u32 {
    return templates.len;
}

pub export fn sim_robot_at(index: u32) callconv(.c) ?*const c.Robot {
    return if (index < templates.len) templates[index] else null;
}

pub export fn sim_robot_find(name: ?[*:0]const u8) callconv(.c) ?*const c.Robot {
    const text = name orelse return null;
    for (templates) |robot| {
        if (strcmp(text, @ptrCast(&robot.name)) == 0) return robot;
    }
    return null;
}

pub export fn simulation_robot_config(
    robot: ?*const c.Robot,
    out: ?*c.SimulationConfig,
    e: ?[*]u8,
    n: u32,
) callconv(.c) c.SimResult {
    return S.guard(e, n, struct {
        fn run(definition: ?*const c.Robot, dest: ?*c.SimulationConfig) !void {
            var known = false;
            if (definition) |value| {
                for (templates) |item| if (item == value) {
                    known = true;
                };
            }
            try S.require(dest != null and known, "unknown robot template", c.SIM_INVALID);
            var config: c.SimulationConfig = undefined;
            simulation_config_default(&config);
            const source = definition.?;
            var positions: u32 = 0;
            var triangles: u32 = 0;
            var hull_positions: u32 = 0;
            var i: u32 = 0;
            while (i < source.object.collider_count) : (i += 1) {
                positions += source.colliders[i].geometry.position_count;
                triangles += source.colliders[i].geometry.triangle_count;
            }
            i = 0;
            while (i < source.hull_count) : (i += 1) hull_positions += source.hulls[i].position_count;
            if (config.vertex_capacity < positions) config.vertex_capacity = positions;
            if (config.triangle_capacity < triangles) config.triangle_capacity = triangles;
            if (config.hull_vertex_capacity < hull_positions) config.hull_vertex_capacity = hull_positions;
            dest.?.* = config;
        }
    }.run(robot, out));
}
