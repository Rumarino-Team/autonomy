const std = @import("std");
const math = @import("math.zig");
const Controller = @This();

pub const ControllerState = struct {
    ErrorConstant: math.Vector6f,
    DerivativeConstant: math.Vector6f,
    IntegralConstant: math.Vector6f,
    sumError: math.Vector6f = @splat(0),
    prevError: math.Vector6f = @splat(0),
};

pub fn pidStep(ctx: *ControllerState, err: math.Vector6f, dt: ?f32) math.Vector6f {
    const vel_err: math.Vector6f = if (dt) |dt_val| blk: {
        ctx.sumError += err * @as(math.Vector6f, @splat(dt_val));
        break :blk (err - ctx.prevError) / @as(math.Vector6f, @splat(dt_val));
    } else @as(math.Vector6f, @splat(0));

    const wrench = ctx.ErrorConstant * err + ctx.IntegralConstant * ctx.sumError + ctx.DerivativeConstant * vel_err;

    ctx.prevError = err;

    return wrench;
}

/// Maps a 6-DOF PID wrench into the 6-DOF command vector expected by `math.tamMul`.
///
/// Pose `orientation` (unit quaternion `rot`) rotates body vectors into world:
///   f_world = R(rot) · f_body     (3×1 = 3×3 · 3×1)
///
/// PID linear output `wrench[0..3]` is treated as force in **world** axes (from
/// world-frame position error). Thruster mixing uses **body** X/Y after policy:
///   f_body = R(rot)⁻¹ · f_world = R(conjugate(rot)) · f_world
///
/// Angular part of `wrench` is not re-rotated here; roll gets a sign fix for TAM.
/// `yaw_error` is err[5] (heading error); if |yaw_error| > gate, body X/Y force is
/// zeroed so the vehicle turns before surging.
///
/// Returns TAM input: [Fx_body, Fy_body, Fz, Mx, My, Mz].
pub fn wrenchToThrusterInput(
    wrench: math.Vector6f,
    orientation: math.Quaternionf,
    yaw_error: f32,
    close_enough_yaw_gate: f32,
) math.Vector6f {
    const rot = math.normalize4f(orientation);
    const world_force: math.Vector3f = .{ wrench[0], wrench[1], wrench[2] };
    const rotated = math.quaternionRotate(math.quaternionConjugate(rot), world_force);
    var body_force = rotated;
    if (@abs(yaw_error) > close_enough_yaw_gate) {
        body_force[0] = 0.0;
        body_force[1] = 0.0;
    } else {
        body_force[0] = @max(body_force[0], 0.0); // This clamping force the auv to always move forward
        body_force[1] = @max(body_force[1], 0.0);
    }
    return .{
        body_force[0],
        body_force[1],
        wrench[2],
        -wrench[3],
        wrench[4],
        wrench[5],
    };
}

pub const max_thrusters = 8;
/// Linear map from 6-DOF command to per-thruster values: t_i = dot(tam[i], input).
/// Clamps to ±`thrustor_saturate`, then divides by 5 (backend scale in MissionContext).
/// `out` must be at least `tam.len` elements; returns `out[0..tam.len]`.
pub fn tamToThrusters(
    tam: []const math.Vector6f,
    input: math.Vector6f,
    thrustor_saturate: f32,
    output_scale: f32,
    out: []f32,
) []f32 {
    std.debug.assert(out.len >= tam.len);
    std.debug.assert(output_scale > 0);
    const thruster_values = math.tamMul(tam, input, out);
    for (thruster_values) |*value| {
        value.* = std.math.clamp(value.*, -thrustor_saturate, thrustor_saturate);
        value.* /= output_scale;
    }
    return thruster_values;
}
