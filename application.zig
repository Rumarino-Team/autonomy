//! Shared-library root: application and configuration are in the same Zig module.
pub const std_options: @import("std").Options = .{
    .signal_stack_size = null,
    .enable_segfault_handler = false,
};
comptime {
    _ = @import("src/Application.zig");
}
test {
    _ = @import("src/Application.zig");
}
