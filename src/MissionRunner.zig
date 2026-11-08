//! Frame-driven mission state. No mission stack survives a DLL update.
const std = @import("std");
const math = @import("math.zig");
const MissionContext = @import("MissionContext.zig");
const Args = @import("MissionArgs.zig");
const Auv = @import("Auv.zig");
const missions = @import("missions/prequalify.zig");
pub const Stage = enum { initial, find_first, pass_first, find_second, circle_second, teleop, complete };
pub const State = struct {
    stage: Stage = .initial,
    target_id: ?u32 = null,
    pending: enum { initial, find, queue, teleop } = .initial,
    translation: math.Vector3f = @splat(0),
    orientation: math.Quaternionf = .{ 0, 0, 0, 1 },
    pub fn name(s: State, mission: Args.MissionId) []const u8 {
        const def = missions.definition(mission);
        return switch (s.stage) {
            .initial => "initial",
            .complete => def.circle_second,
            .teleop => "teleop",
            .find_first => def.find_first,
            .pass_first => def.pass_first,
            .find_second => def.find_second,
            .circle_second => def.circle_second,
        };
    }
};
fn find(ctx: *MissionContext, cls: u8) ?*const Auv.Object {
    for (ctx.seen_objects[0..ctx.seen_objects_len]) |*o| if (o.cls == cls) return o;
    return null;
}
// Advance all instantaneous transitions until the next physics frame is needed.
fn prepare(ctx: *MissionContext, state: *State, args: Args) !void {
    while (true) {
        ctx.step = state.name(args.mission_id);
        switch (state.stage) {
            .initial => {
                state.pending = .initial;
                return;
            },
            .complete => return,
            .teleop => {
                if (ctx.teleop) |poll| {
                    var values: [7]f32 = undefined;
                    if (poll(&values)) {
                        const t: math.Vector3f = values[0..3].*;
                        const q: math.Quaternionf = values[3..7].*;
                        const n = @reduce(.Add, q * q);
                        if (std.math.isFinite(t[0]) and std.math.isFinite(t[1]) and std.math.isFinite(t[2]) and std.math.isFinite(n) and n > 1e-12) {
                            state.translation = t;
                            state.orientation = math.normalize4f(q);
                        }
                    }
                }
                ctx.use_position = false;
                ctx.use_orientation = true;
                ctx.target_orientation = state.orientation;
                ctx.translation = if (math.length3f(state.translation) <= 1e-4) null else state.translation;
                state.pending = .teleop;
                return;
            },
            .find_first, .find_second => {
                ctx.use_position = true;
                ctx.use_orientation = true;
                ctx.translation = null;
                const def = missions.definition(args.mission_id);
                const cls = if (state.stage == .find_first) def.first_class else def.second_class;
                const object = find(ctx, cls) orelse {
                    state.pending = .find;
                    return;
                };
                state.target_id = object.id;
                if (state.stage == .find_first) {
                    try ctx.preparePass(object);
                    state.stage = .pass_first;
                } else {
                    try ctx.prepareCircle(object);
                    state.stage = .circle_second;
                }
            },
            .pass_first, .circle_second => {
                if (try ctx.prepareGoal()) {
                    state.pending = .queue;
                    return;
                }
                state.stage = if (state.stage == .pass_first) .find_second else .complete;
            },
        }
    }
}
pub fn step(ctx: *MissionContext, state: *State, args: Args) !bool {
    if (state.stage == .complete) return true;
    try prepare(ctx, state, args);
    if (state.stage == .complete) return true;
    try ctx.yieldUntilNextFrameAndUpdate();
    switch (state.pending) {
        .initial => {
            ctx.target_position = math.vec3(ctx.frame.camera_pose.pos);
            state.stage = if (args.mission_id == .teleop) .teleop else .find_first;
        },
        .queue => ctx.finishGoal(),
        else => {},
    }
    try prepare(ctx, state, args);
    return state.stage == .complete;
}
pub fn execute(ctx: *MissionContext, args: Args) !void {
    var state: State = .{};
    while (!try step(ctx, &state, args)) {}
    if (!ctx.batch) std.log.info("successfully completed mission", .{});
}

test "waiting mission stages and queued maneuvers have explicit resumable state" {
    const Fake = struct {
        fn init() callconv(.c) void {}
        fn commands(_: [*c]const f32, _: u8) callconv(.c) void {}
        fn frame(out: *Auv.Frame) callconv(.c) void {
            out.* = std.mem.zeroes(Auv.Frame);
            out.camera_pose.quat.buf = .{ 0, 0, 0, 1 };
            out.imu_quat.buf = .{ 0, 0, 0, 1 };
            out.timestamp = 16666666;
        }
    };
    var arena = std.heap.ArenaAllocator.init(std.testing.allocator);
    defer arena.deinit();
    var io = std.Io.Threaded.init_single_threaded;
    defer io.deinit();
    const args = Args.defaults();
    var ctx = try MissionContext.initBatch(arena.allocator(), io.io(), args, .{ .init = Fake.init, .deinit = Fake.init, .yieldUntilNextFrame = Fake.frame, .setThrustorValues = Fake.commands }, @import("ConfigLoader.zig").current());
    var state: State = .{};
    try std.testing.expect(!try step(&ctx, &state, args));
    try std.testing.expectEqual(Stage.find_first, state.stage);
    const Binary = @import("BinaryData.zig");
    const saved = try ctx.save(arena.allocator());
    const bytes = try Binary.pack(arena.allocator(), .state, 1, .{ .mission = saved, .runner = state });
    const T = struct { mission: MissionContext.Saved, runner: State };
    const restored = try Binary.decode(arena.allocator(), T, bytes, .state, 1);
    try ctx.restore(restored.mission);
    state = restored.runner;
    try std.testing.expect(!try step(&ctx, &state, args));
    try std.testing.expectEqual(Stage.find_first, state.stage);
    state.stage = .find_second;
    try std.testing.expect(!try step(&ctx, &state, args));
    try std.testing.expectEqual(Stage.find_second, state.stage);
    // Every maneuver uses explicit queued goals, independent of the function stack.
    var object = std.mem.zeroes(Auv.Object);
    object.id = 2;
    object.bbox.pose.quat.buf = .{ 0, 0, 0, 1 };
    object.bbox.size.buf = .{ 2, 2, 2 };
    object.bbox.pose.pos.buf = .{ 0, 4, 2 };
    try ctx.preparePass(&object);
    try std.testing.expectEqual(@as(usize, 2), ctx.queue.len);
    try ctx.prepareMove(&object);
    try std.testing.expectEqual(@as(usize, 1), ctx.queue.len);
    try ctx.prepareCircle(&object);
    try std.testing.expectEqual(@as(usize, 5), ctx.queue.len);
    try ctx.prepareLook(&object);
}
