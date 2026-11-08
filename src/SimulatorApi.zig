//! Shared native function tables only; application builders/fuzz logic live outside the host.
const std = @import("std");
const Auv = @import("Auv.zig");
pub const c = @import("sim_c");
pub const Api = struct {
    library: @import("DynLib.zig").DynLib,
    core_library: ?@import("DynLib.zig").DynLib = null,
    backend: c.PhysicsBackend = std.mem.zeroes(c.PhysicsBackend),
    auv: Auv,
    robot: *const fn ([*c]const u8) callconv(.c) ?*const c.Robot,
    load_scene: ?*const fn ([*c]c.Simulation, [*c]const u8, [*c]const c.Robot, [*c]u8, u32) callconv(.c) c.SimResult = null,
    create: *const fn ([*c]c.Simulation, [*c]const u8, [*c]const c.Robot, [*c]u8, u32) callconv(.c) c.SimResult,
    clone: *const fn ([*c]const c.Simulation, [*c]c.Simulation, [*c]u8, u32) callconv(.c) c.SimResult,
    destroy: *const fn ([*c]c.Simulation) callconv(.c) void,
    finalize: *const fn ([*c]c.Simulation, [*c]u8, u32) callconv(.c) c.SimResult,
    draft: *const fn ([*c]c.Simulation, u32, [*c][*c]c.ObjectBlock, [*c]u8, u32) callconv(.c) c.SimResult,
    fault: *const fn ([*c]c.Simulation, [*c]const c.FaultBlock, [*c]u8, u32) callconv(.c) c.SimResult,
    open_backend: *const fn ([*c]c.PhysicsBackend, [*c]const u8, [*c]u8, u32) callconv(.c) c.SimResult,
    close_backend: *const fn ([*c]c.PhysicsBackend) callconv(.c) void,
    init_scene: *const fn ([*c]const c.Simulation, [*c]const c.AuvSimulatorOptions, [*c]u8, u32) callconv(.c) c.SimResult,
    configure_blender: ?*const fn (u32, u16) callconv(.c) void = null,
    result: *const fn ([*c]c.AuvSimulatorResult, u32, [*c]u8, u32) callconv(.c) c.SimResult,

    pub fn load(path: []const u8) !Api {
        var lib = try @import("DynLib.zig").DynLib.open(path);
        errdefer lib.close();
        var core: ?@import("DynLib.zig").DynLib = null;
        if (@import("builtin").os.tag == .windows) {
            const allocator = std.heap.page_allocator;
            const directory = std.fs.path.dirname(path) orelse ".";
            const first = try std.fs.path.join(allocator, &.{ directory, "libsimulator_core.dll" });
            defer allocator.free(first);
            core = @import("DynLib.zig").DynLib.open(first) catch blk: {
                const second = try std.fs.path.join(allocator, &.{ directory, "simulator_core.dll" });
                defer allocator.free(second);
                break :blk try @import("DynLib.zig").DynLib.open(second);
            };
        }
        errdefer if (core) |*c_lib| c_lib.close();
        const records = if (core) |*c_lib| c_lib else &lib;
        return .{
            .core_library = core,
            .load_scene = lib.lookup(*const fn ([*c]c.Simulation, [*c]const u8, [*c]const c.Robot, [*c]u8, u32) callconv(.c) c.SimResult, "auv_simulator_load_scene"),
            .configure_blender = lib.lookup(*const fn (u32, u16) callconv(.c) void, "auv_simulator_configure_blender"),
            .library = lib,
            .auv = .{
                .init = lib.lookup(*const Auv.InitFunc, "auv_init") orelse return error.MissingSimulatorApi,
                .deinit = lib.lookup(*const Auv.DeinitFunc, "auv_deinit") orelse return error.MissingSimulatorApi,
                .yieldUntilNextFrame = lib.lookup(*const Auv.YieldUntilNextFrameFunc, "auv_yield_until_next_frame") orelse return error.MissingSimulatorApi,
                .setThrustorValues = lib.lookup(*const Auv.SetThrustorValuesFunc, "auv_set_thrustor_values") orelse return error.MissingSimulatorApi,
                .simulationStatus = lib.lookup(*const Auv.SimulationStatusFunc, "auv_simulation_status") orelse return error.MissingSimulatorApi,
            },
            .robot = records.lookup(*const fn ([*c]const u8) callconv(.c) ?*const c.Robot, "sim_robot_find") orelse return error.MissingSimulatorApi,
            .create = try symbol(records, @FieldType(Api, "create"), "simulation_scene_create"),
            .clone = try symbol(records, @FieldType(Api, "clone"), "simulation_clone"),
            .destroy = try symbol(records, @FieldType(Api, "destroy"), "simulation_destroy"),
            .finalize = try symbol(records, @FieldType(Api, "finalize"), "simulation_finalize"),
            .draft = try symbol(records, @FieldType(Api, "draft"), "simulation_draft_object"),
            .fault = try symbol(records, @FieldType(Api, "fault"), "simulation_add_fault"),
            .open_backend = try symbol(records, @FieldType(Api, "open_backend"), "physics_backend_open"),
            .close_backend = try symbol(records, @FieldType(Api, "close_backend"), "physics_backend_close"),
            .init_scene = try symbol(&lib, @FieldType(Api, "init_scene"), "auv_simulator_init_scene"),
            .result = try symbol(&lib, @FieldType(Api, "result"), "auv_simulator_read_result"),
        };
    }
    fn symbol(lib: *@import("DynLib.zig").DynLib, comptime T: type, comptime name: [:0]const u8) !T {
        return lib.lookup(T, name) orelse error.MissingSimulatorApi;
    }
    pub fn pinBackend(api: *Api, path: [:0]const u8) !void {
        var detail: [512]u8 = undefined;
        try check(api.open_backend(&api.backend, path, &detail, detail.len));
        if (!std.mem.eql(u8, std.mem.sliceTo(&api.backend.api.name, 0), "mujoco")) return error.FuzzRequiresMuJoCo;
    }
    pub fn deinit(api: *Api) void {
        api.close_backend(&api.backend);
        api.library.close();
        if (api.core_library) |*core| core.close();
    }
};

pub fn check(result: c.SimResult) !void {
    return switch (result) {
        c.SIM_OK => {},
        c.SIM_CAPACITY => error.SceneCapacity,
        c.SIM_NO_MEMORY => error.OutOfMemory,
        c.SIM_UNSUPPORTED => error.SimulatorUnsupported,
        c.SIM_BACKEND_ERROR => error.SimulatorBackend,
        else => error.InvalidScene,
    };
}
