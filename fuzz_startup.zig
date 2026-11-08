pub const std_options = @import("src/FuzzStartupTest.zig").std_options;
pub fn main(init: @import("std").process.Init) !void {
    try @import("src/FuzzStartupTest.zig").main(init);
}
