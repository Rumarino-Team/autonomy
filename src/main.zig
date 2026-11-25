const std = @import("std");
const AuvApplication = @import("application/Application.zig").AuvApplication;
const MissionContext = @import("MissionContext.zig");
const missions = @import("missions.zig");
const simulator_enabled = @import("build_options").simulator;

test {
    _ = @import("application/Application.zig");
    _ = @import("AuvRobot.zig");
    _ = @import("ReflectionWriter.zig");
    _ = @import("MissionIO.zig");
    _ = MissionContext;
    _ = missions;
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
    var app: AuvApplication = undefined;
    try app.initProcess(process_allocator, init.io, windowed);
    defer app.deinitProcess();
    const telemetry = app.telemetry;

    var mission_arena = std.heap.ArenaAllocator.init(process_allocator);
    defer mission_arena.deinit();
    while (true) {
        defer _ = mission_arena.reset(.retain_capacity);
        runMission(mission_arena.allocator(), &app) catch |err| switch (@as(anyerror, err)) {
            error.WindowClosed => return,
            error.MissionRestart => {},
            error.MissionStopped => telemetry.result = .stopped,
            else => {
                telemetry.result = .failed;
                telemetry.report(err);
            },
        };
    }
}

fn runMission(allocator: std.mem.Allocator, app: *AuvApplication) !void {
    const telemetry = app.telemetry;
    std.Io.random(app.io, std.mem.asBytes(&telemetry.run_id));
    telemetry.run_id |= 1;
    telemetry.state = .starting;
    telemetry.result = .none;
    telemetry.last_error = .{};
    telemetry.clearCommands();
    defer {
        telemetry.state = .idle;
        telemetry.clearCommands();
    }
    try app.initMission(allocator);
    defer app.deinitMission();

    var mission: MissionContext = undefined;
    mission.init(app);
    try mission.prime();
    telemetry.state = .running;
    switch (app.config.mission_id) {
        .prequalify_coco => try missions.cocoMission(&mission),
        .prequalify_robosub => try missions.robosubMission(&mission),
        .teleop => try missions.teleopMission(&mission),
        .mission_control => try missions.missionControl(&mission),
        .thruster_control => try missions.thrusterControl(&mission),
    }
    telemetry.result = .completed;
}
