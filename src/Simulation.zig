//! Import the independently versioned physics ABI without changing Auv.Frame.
const std = @import("std");
pub const c = @cImport({
    @cInclude("sim_backend.h");
});

test "portable physics ABI has canonical scalar and quaternion layout" {
    try std.testing.expectEqual(@as(usize, 24), @sizeOf(c.SimVec3));
    try std.testing.expectEqual(@as(usize, 32), @sizeOf(c.SimQuat));
    try std.testing.expectEqual(@as(usize, 24), @offsetOf(c.SimQuat, "w"));
    try std.testing.expectEqual(@as(u32, 1), c.SIM_BACKEND_ABI_VERSION);
}
