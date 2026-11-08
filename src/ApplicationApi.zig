//! Stable host/application ABI. Only C-compatible values cross this boundary.
const Auv = @import("Auv.zig");
pub const AuvCallbacks = extern struct {
    init: *const Auv.InitFunc,
    yield_frame: *const Auv.YieldUntilNextFrameFunc,
    commands: *const Auv.SetThrustorValuesFunc,
    deinit: *const Auv.DeinitFunc,
    status: ?*const Auv.SimulationStatusFunc,
    pub fn from(a: Auv) AuvCallbacks {
        return .{ .init = a.init, .yield_frame = a.yieldUntilNextFrame, .commands = a.setThrustorValues, .deinit = a.deinit, .status = a.simulationStatus };
    }
    pub fn to(a: AuvCallbacks) Auv {
        return .{ .init = a.init, .yieldUntilNextFrame = a.yield_frame, .setThrustorValues = a.commands, .deinit = a.deinit, .simulationStatus = a.status };
    }
};
pub const version: u32 = 1;
pub const Launch = extern struct {
    plugin: [*:0]const u8,
    backend: [*:0]const u8,
    robot: [*:0]const u8,
    scene: [*:0]const u8,
    telemetry_address: [*:0]const u8,
    frame_ticks: u64,
    max_ticks: u64,
    dt_numerator: u32,
    dt_denominator: u32,
    hydro_period: u32,
    result_path: ?[*:0]const u8,
    blender_port: u16,
    blender_enabled: u8,
    teleop_port: u16,
    realtime: u8,
    telemetry: u8,
    fuzz: u8,
    teleop_enabled: u8,
};
pub const Services = extern struct {
    struct_size: u32 = @sizeOf(Services),
    abi_version: u32 = version,
    argc: u32,
    argv: [*]const [*:0]const u8,
    cancelled: *const fn () callconv(.c) bool,
    telemetry: *const fn ([*]const u8, usize) callconv(.c) void,
    teleop: *const fn ([*]f32) callconv(.c) bool,
    // A host-native callback, never code from an application generation.
    auv: *const AuvCallbacks,
    plugin_path: [*:0]const u8,
    backend_path: [*:0]const u8,
    executable_path: [*:0]const u8,
    application_path: [*:0]const u8,
};
pub const Api = extern struct {
    struct_size: u32 = @sizeOf(Api),
    abi_version: u32 = version,
    create: *const fn (*const Services, *?*anyopaque, [*]u8, usize) callconv(.c) i32,
    destroy: *const fn (*anyopaque) callconv(.c) void,
    describe: *const fn (*anyopaque, *Launch) callconv(.c) void,
    build_scene: *const fn (*anyopaque, *anyopaque, [*]u8, usize) callconv(.c) i32,
    flags: *const fn (*anyopaque) callconv(.c) u32,
    update: *const fn (*anyopaque, [*]u8, usize) callconv(.c) i32,
    capture: *const fn (*anyopaque, ?[*]u8, usize, *usize, [*]u8, usize) callconv(.c) i32,
    restore: *const fn (*anyopaque, [*]const u8, usize, [*]u8, usize) callconv(.c) i32,
    activate: *const fn (*anyopaque) callconv(.c) void,
};
pub const GetApi = *const fn (u32) callconv(.c) ?*const Api;

comptime {
    const c = @import("application_c");
    for (.{ .{ Launch, c.AutonomyLaunch }, .{ Services, c.AutonomyHostServices }, .{ Api, c.AutonomyApplicationApi }, .{ AuvCallbacks, c.AutonomyAuvCallbacks } }) |types| {
        if (@sizeOf(types[0]) != @sizeOf(types[1]) or @alignOf(types[0]) != @alignOf(types[1])) @compileError("application C ABI layout mismatch");
        const info = @typeInfo(types[0]).@"struct";
        if (@hasField(@TypeOf(info), "fields")) {
            for (info.fields) |field| if (@offsetOf(types[0], field.name) != @offsetOf(types[1], field.name)) @compileError("application C ABI field mismatch");
        } else {
            for (info.field_names) |name| if (@offsetOf(types[0], name) != @offsetOf(types[1], name)) @compileError("application C ABI field mismatch");
        }
    }
}
