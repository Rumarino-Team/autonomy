const std = @import("std");
const rl = @import("raylib");
const IO = @import("../MissionIO.zig");
const Wire = @import("../AuvRobot.zig").Wire;
const Config = IO.CompiledConfig;
const Self = @This();

status: ?IO.RuntimeStatus = null,
client_id: u64 = 0,
pending_config: ?IO.LiveConfigPacket = null,
last_status: f64 = -1,
direct_values: [Config.thruster_count]f32 = @splat(0),
direct_command: ?Wire.ThrusterDoc = null,
direct_run_id: u64 = 0,

pub fn synchronized(self: *const Self) bool {
    return self.status != null and rl.getTime() - self.last_status < 2;
}

pub fn queueConfig(self: *Self, packet: IO.LiveConfigPacket) void {
    if (self.synchronized() and self.pending_config == null) self.pending_config = packet;
}

pub fn acceptStatus(self: *Self, status: IO.RuntimeStatus, packet: *IO.LiveConfigPacket) void {
    self.status = status;
    self.last_status = rl.getTime();
    if (status.run_id != self.direct_run_id or status.state != .running or
        !status.active or status.active_config.mission_id != .thruster_control)
    {
        self.direct_command = null;
        if (status.run_id != self.direct_run_id) {
            self.direct_run_id = status.run_id;
            self.direct_values = @splat(0);
        }
    }
    packet.* = .{ .revision = status.revision, .config = status.desired };
    if (self.pending_config) |pending| {
        if (status.config_reply_revision == pending.revision) self.pending_config = null;
    }
}

pub fn zeroThrusters(self: *Self) void {
    self.direct_values = @splat(0);
}

pub fn publishThrusters(self: *Self, status: IO.RuntimeStatus) void {
    if (status.state != .running or !status.active or status.active_config.mission_id != .thruster_control) return;
    const count = status.active_config.tam_len;
    if (count > self.direct_values.len) return;
    for (self.direct_values[0..count]) |value| {
        if (!std.math.isFinite(value) or value < -1 or value > 1) return;
    }
    var command: Wire.ThrusterDoc = .{ .len = count };
    @memcpy(command.values[0..count], self.direct_values[0..count]);
    self.direct_command = command;
}
