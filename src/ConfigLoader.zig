//! Compatibility name for the controller type. Configuration is compiled, never parsed.
pub const Config = @import("Configuration.zig").Controller;
pub const OdometryPid = @import("Configuration.zig").OdometryPid;
pub const Ibvs = @import("Configuration.zig").Ibvs;
pub const validate = @import("Configuration.zig").validate;
pub fn current() Config {
    return @import("../config/application.zig").config.controller;
}
