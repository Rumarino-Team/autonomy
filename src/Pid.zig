const std = @import("std");

sum: f32 = 0,
previous: ?f32 = null,

const Pid = @This();

/// `measured_rate` is the derivative of the measured state, in the error frame.
/// Null selects error differencing for controllers that do not have velocity.
pub fn step(self: *Pid, err: f32, measured_rate: ?f32, dt: ?f32, kp: f32, ki: f32, kd: f32, integral_limit: f32, output_limit: f32) f32 {
    const t = if (dt) |v| (if (v > 0 and std.math.isFinite(v)) v else 0) else 0;
    const rate = if (measured_rate) |v| -v else if (self.previous != null and t > 0) (err - self.previous.?) / t else 0;
    self.previous = err;
    const pd = kp * err + kd * rate;
    // Reconfigured limits apply to retained history immediately after reload.
    self.sum = if (ki != 0) std.math.clamp(self.sum, -integral_limit / @abs(ki), integral_limit / @abs(ki)) else 0;
    const next_sum = if (ki != 0) std.math.clamp(self.sum + err * t, -integral_limit / @abs(ki), integral_limit / @abs(ki)) else 0;
    const candidate = pd + ki * next_sum;
    // Stop integrating further into saturation; permit integration out of it.
    if (@abs(candidate) <= output_limit or candidate * (ki * err) < 0) self.sum = next_sum;
    return std.math.clamp(pd + ki * self.sum, -output_limit, output_limit);
}

test "measurement damping has no waypoint derivative kick" {
    var pid: Pid = .{};
    _ = pid.step(0, 0, 0.01, 2, 0, 10, 0, 100);
    try std.testing.expectEqual(@as(f32, 20), pid.step(10, 0, 0.01, 2, 0, 10, 0, 100));
    try std.testing.expectEqual(@as(f32, 10), pid.step(10, 1, 0.01, 2, 0, 10, 0, 100));
}

test "integrator bounds and saturation recovery" {
    var pid: Pid = .{};
    for (0..1000) |_| _ = pid.step(100, 0, 0.01, 2, 1, 0, 3, 10);
    try std.testing.expectEqual(@as(f32, 0), pid.sum);
    for (0..1000) |_| _ = pid.step(1, 0, 0.01, 2, 1, 0, 3, 10);
    try std.testing.expectApproxEqAbs(@as(f32, 3), pid.sum, 1e-5);
    try std.testing.expect(pid.step(-1, 0, 0.1, 2, 1, 0, 3, 10) < 1);
    try std.testing.expect(std.math.isFinite(pid.step(0, null, 0, 2, 1, 1, 3, 10)));
}

test "reloaded integral limit clamps retained history" {
    var pid: Pid = .{ .sum = 3 };
    _ = pid.step(100, 0, 0.01, 2, 1, 0, 0.1, 1);
    try std.testing.expectApproxEqAbs(@as(f32, 0.1), pid.sum, 1e-6);
}
