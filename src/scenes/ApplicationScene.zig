//! Reloadable scene construction. Output is a stable host/worker-owned C root;
//! every retained allocation is made by native core functions, not this DLL.
const std = @import("std");
pub fn construct(api: anytype, output: anytype, scene_name: [:0]const u8, robot_name: [:0]const u8, physics: anytype, detail: []u8) !void {
    const T = @TypeOf(output.*);
    const allocator = std.heap.page_allocator;
    const source = try allocator.create(T);
    source.* = std.mem.zeroes(T);
    defer {
        api.destroy(source);
        allocator.destroy(source);
    }
    const robot = api.robot(robot_name) orelse return error.UnknownRobot;
    const from_file = std.mem.endsWith(u8, scene_name, ".bin") or std.mem.endsWith(u8, scene_name, ".blend");
    const create = if (from_file) api.load_scene orelse return error.MissingSceneLoader else api.create;
    if (create(source, scene_name, robot, detail.ptr, @intCast(detail.len)) != 0) return error.InvalidScene;
    if (api.clone(source, output, detail.ptr, @intCast(detail.len)) != 0) return error.SceneClone;
    errdefer api.destroy(output);
    output.settings.dt_numerator = physics.dt_numerator;
    output.settings.dt_denominator = physics.dt_denominator;
    output.settings.hydro_period = physics.hydro_period;
    if (api.finalize(output, detail.ptr, @intCast(detail.len)) != 0) return error.InvalidScene;
}
