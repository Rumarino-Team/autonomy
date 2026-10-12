const std = @import("std");
const math = @import("math.zig");
const MissionContext = @import("MissionContext.zig");
const MissionArgs = @import("MissionArgs.zig");

pub fn main(init: std.process.Init) !void {
    const args_slice = try init.minimal.args.toSlice(init.arena.allocator());
    const args: MissionArgs = try .init(args_slice);

    std.log.info("args = {f}", .{args});

    var ctx: MissionContext = try .init(init.arena.allocator(), init.io, args);

    mission_loop: while (true) {
        ctx.yieldUntilNextFrameAndUpdate() catch |err| switch (err) {
            error.RestartMission => continue,
            else => return err,
        };
        var warmup_seconds: f32 = 0;
        while (ctx.navigation.snapshot.orientation == null) {
            warmup_seconds += @max(ctx.controller_dt, ctx.execution_dt);
            if (warmup_seconds >= 2) {
                ctx.stopThrusters();
                return error.SensorUnavailable;
            }
            ctx.yieldUntilNextFrameAndUpdate() catch |err| switch (err) {
                error.RestartMission => continue :mission_loop,
                else => return err,
            };
        }
        ctx.holdPosition();

        const mission_result = switch (args.mission_id) {
            .prequalify => @import("missions/prequalify.zig").mission(&ctx),
            .teleop => @import("missions/prequalify.zig").teleopMission(&ctx),
            .teleop_command => @import("missions/teleop.zig").teleopCommandMission(&ctx),
        };

        mission_result catch |err| switch (err) {
            error.RestartMission => {
                std.log.info("restarting mission context", .{});
                ctx.reset();
                std.log.info("restarting mission", .{});
                continue;
            },
            else => {
                ctx.stopThrusters();
                return err;
            },
        };

        std.log.info("succesfully completed mission", .{});
        break;
    }
}

test {
    // The test build does not execute main, so explicitly discover the motion
    // and controller modules' regression tests.
    std.testing.refAllDecls(@import("Motion.zig"));
    std.testing.refAllDecls(@import("MotionController.zig"));
    std.testing.refAllDecls(@import("ThrusterAllocator.zig"));
    std.testing.refAllDecls(MissionContext);
    std.testing.refAllDecls(@import("missions/prequalify.zig"));
    std.testing.refAllDecls(math);
}
