const std = @import("std");

const runtime = @import("runtime.zig").App;
const missions = @import("MissionContext.zig");
const MissionIO = @import("MissionIO.zig");

const simulator_enabled = @import("build_options").simulator;

test {
    _ = missions;
    _ = @import("AuvRobot.zig");
    _ = @import("ReflectionWriter.zig");
    _ = MissionIO;
    _ = runtime;
}

pub fn main(init: std.process.Init) !void {
    const process_allocator = init.arena.allocator();
    const args = try init.minimal.args.toSlice(process_allocator);
    var windowed = simulator_enabled;
    for (args[1..]) |arg| {
        if (std.mem.eql(u8, arg, "--headless") and simulator_enabled) {
            windowed = false;
        } else {
            std.log.err("unknown argument: {s}", .{arg});
            return error.UnknownArgument;
        }
    }
    try runtime.initApp(process_allocator, init.io, windowed, true);
    defer runtime.deinitApp(process_allocator);

    var mission_arena = std.heap.ArenaAllocator.init(process_allocator);
    defer mission_arena.deinit();
    while (true) {
        defer _ = mission_arena.reset(.retain_capacity);
        runMission(mission_arena.allocator()) catch |err| switch (@as(anyerror, err)) {
            error.WindowClosed => return,
            error.MissionRestart => {},
            error.MissionStopped => runtime.telemetry.result = .stopped,
            else => {
                runtime.telemetry.result = .failed;
                runtime.telemetry.report(err);
            },
        };
    }
}

fn runMission(allocator: std.mem.Allocator) !void {
    const telemetry_ptr = runtime.telemetry;
    std.Io.random(runtime.io, std.mem.asBytes(&telemetry_ptr.run_id));
    telemetry_ptr.run_id |= 1;
    telemetry_ptr.state = .starting;
    telemetry_ptr.result = .none;
    telemetry_ptr.last_error = .{};
    telemetry_ptr.clearCommands();
    defer {
        telemetry_ptr.state = .idle;
        telemetry_ptr.clearCommands();
        runtime.deinitMission();
    }
    if (!runtime.mission_active) try runtime.initApp(allocator, runtime.io, runtime.visualizer != null, false);

    missions.init();
    try missions.prime();
    telemetry_ptr.state = .running;
    try missions.run(runtime.config.mission_id);
    telemetry_ptr.result = .completed;
}