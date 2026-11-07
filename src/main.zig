const std = @import("std");
// Zig 0.16's default alternate signal stack is a TLS byte array (alignment 1).
// Clang ASan requires aligned signal-stack bounds during C++ exception handling.
// Instrumented-plugin tests leave signal-stack ownership to the ASan runtime.
const external_asan = @import("runtime_options").external_asan;
pub const std_options: std.Options = .{
    .signal_stack_size = if (external_asan) null else 1 << 18,
    .enable_segfault_handler = if (external_asan) false else std.debug.default_enable_segfault_handler,
};
const MissionContext = @import("MissionContext.zig");
const MissionArgs = @import("MissionArgs.zig");

test {
    _ = @import("Pid.zig");
    _ = @import("SceneContext.zig");
    std.testing.refAllDecls(@import("BodyControl.zig"));
    std.testing.refAllDecls(@import("MissionContext.zig"));
    std.testing.refAllDecls(@import("ConfigLoader.zig"));
}

pub fn main(init: std.process.Init) !void {
    const args_slice = try init.minimal.args.toSlice(init.arena.allocator());
    const args: MissionArgs = try .init(args_slice);

    std.log.info("args = {f}", .{args});

    MissionContext.installSignalHandlers();
    if (args.fuzz) {
        try @import("FuzzRunner.zig").run(init.io, args);
        return;
    }
    var ctx: MissionContext = try .init(init.arena.allocator(), init.io, args);
    defer ctx.deinit();
    try @import("MissionRunner.zig").execute(&ctx, args);
}
