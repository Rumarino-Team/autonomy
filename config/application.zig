//! Edit this file or its imports, then use ordinary `zig build`.
//! A running development host rebuilds these sources automatically.
const Types = @import("../src/Configuration.zig");
pub const config: Types.Application = .{
    .controller = @import("hydrus.zig").controller,
    .reload = .{ .watch = true, .preserve_state = true },
    // FuzzProfile defaults retain the user-selected 1.05/1.05/1.15 rad rotations.
};
