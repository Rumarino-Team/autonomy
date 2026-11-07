//! Native integration fixture: the second OS worker deliberately fails to start.
const std = @import("std");
const external_asan = @import("runtime_options").external_asan;
pub const std_options: std.Options = .{
    .signal_stack_size = if (external_asan) null else 1 << 18,
    .enable_segfault_handler = if (external_asan) false else std.debug.default_enable_segfault_handler,
};
const Runner = @import("FuzzRunner.zig");
const Launcher = struct {
    var calls: usize = 0;
    pub fn spawn(shared: anytype) std.Thread.SpawnError!std.Thread {
        calls += 1;
        if (calls == 2) return error.ThreadQuotaExceeded;
        return Runner.spawnWorker(shared);
    }
};
pub fn main(init: std.process.Init) !void {
    const args = try @import("MissionArgs.zig").init(try init.minimal.args.toSlice(init.arena.allocator()));
    @import("MissionContext.zig").installSignalHandlers();
    Runner.runWithLauncher(init.io, args, Launcher) catch |err| {
        if (err == error.ThreadQuotaExceeded) return;
        return err;
    };
    return error.ExpectedStartupFailure;
}
