const std = @import("std");
const Auv = @import("Auv.zig");

pub inline fn dot3f(a: Auv.Vector3f, b: Auv.Vector3f) f32 {
    return @reduce(.Add, a * b);
}

pub inline fn length3f(v: Auv.Vector3f) f32 {
    return @sqrt(dot3f(v, v));
}

pub inline fn normalize3f(v: Auv.Vector3f) Auv.Vector3f {
    const len = length3f(v);
    return v / @as(Auv.Vector3f, @splat(len));
}

pub inline fn cross3f(a: Auv.Vector3f, b: Auv.Vector3f) Auv.Vector3f {
    const a_yzx = @shuffle(f32, a, undefined, Auv.Vector3f{ 1, 2, 0 });
    const a_zxy = @shuffle(f32, a, undefined, Auv.Vector3f{ 2, 0, 1 });
    const b_yzx = @shuffle(f32, b, undefined, Auv.Vector3f{ 1, 2, 0 });
    const b_zxy = @shuffle(f32, b, undefined, Auv.Vector3f{ 2, 0, 1 });

    return a_yzx * b_zxy - a_zxy * b_yzx;
}

pub inline fn normalize4f(v: Auv.Quaternionf) Auv.Quaternionf {
    const len = @sqrt(@reduce(.Add, v * v));
    return v / @as(Auv.Quaternionf, @splat(len));
}

pub inline fn quaternionConjugate(q: Auv.Quaternionf) Auv.Quaternionf {
    return Auv.Quaternionf{ -1, -1, -1, 1 } * q;
}

pub inline fn quaternionMul(a: Auv.Quaternionf, b: Auv.Quaternionf) Auv.Quaternionf {
    const av: Auv.Vector3f = a[0..3].*;
    const bv: Auv.Vector3f = b[0..3].*;

    const xyz =
        a[3] * bv +
        b[3] * av +
        cross3f(av, bv);

    const w = a[3] * b[3] - dot3f(av, bv);

    return .{ xyz[0], xyz[1], xyz[2], w };
}

pub inline fn quaternionRotate(q: Auv.Quaternionf, v: Auv.Vector3f) Auv.Vector3f {
    const qv: Auv.Vector3f = .{ q[0], q[1], q[2] };
    const t = @as(Auv.Vector3f, @splat(2.0)) * cross3f(qv, v);

    return v + @as(Auv.Vector3f, @splat(q[3])) * t + cross3f(qv, t);
}

pub inline fn quaternionToEuler(q: Auv.Quaternionf) Auv.Vector3f {
    const x, const y, const z, const w = q;

    const sinr_cosp: f32 = 2.0 * (w * x + y * z);
    const cosr_cosp: f32 = 1.0 - 2.0 * (x * x + y * y);
    const roll: f32 = std.math.atan2(sinr_cosp, cosr_cosp);

    const sinp: f32 = 2.0 * (w * y - z * x);
    const pitch: f32 = if (@abs(sinp) >= 1.0)
        if (sinp < 0.0) -std.math.pi / 2.0 else std.math.pi / 2.0
    else
        std.math.asin(sinp);

    const siny_cosp: f32 = 2.0 * (w * z + x * y);
    const cosy_cosp: f32 = 1.0 - 2.0 * (y * y + z * z);
    const yaw: f32 = std.math.atan2(siny_cosp, cosy_cosp);

    return .{ roll, pitch, yaw };
}

pub inline fn wrapAngle(angle: f32) f32 {
    return @mod(angle + std.math.pi, 2.0 * std.math.pi) - std.math.pi;
}

pub fn rotationBetween(forward: Auv.Vector3f, dir: Auv.Vector3f) Auv.Quaternionf {
    const dot = dot3f(forward, dir);

    if (dot < -1.0 + 1e-6) {
        // Opposite vectors: choose an arbitrary perpendicular axis.
        const axis = if (@abs(forward[0]) < @abs(forward[1]))
            normalize3f(cross3f(forward, .{ 1, 0, 0 }))
        else
            normalize3f(cross3f(forward, .{ 0, 1, 0 }));

        // 180 degree rotation: sin(pi/2) = 1, cos(pi/2) = 0.
        return .{ axis[0], axis[1], axis[2], 0.0 };
    }

    const q: Auv.Quaternionf = .{
        cross3f(forward, dir)[0],
        cross3f(forward, dir)[1],
        cross3f(forward, dir)[2],
        1.0 + dot,
    };

    return normalize4f(q);
}

// pub inline fn poseTo6f(pose: Auv.Pose) Auv.Vector6f {
//     const x, const y, const z = pose.pos;
//     const roll, const pitch, const yaw = quaternionToEuler(pose.quat);
//
//     return .{ x, y, z, roll, pitch, yaw };
// }

pub inline fn poseTo6f(pose: Auv.Pose) Auv.Vector6f {
    const x, const y, const z = pose.pos;
    const rot = normalize4f(pose.quat);
    const roll, const pitch, const yaw = quaternionToEuler(rot);

    return .{ x, y, z, roll, pitch, yaw };
}

pub fn tamMul(tam: []const Auv.Vector6f, v: Auv.Vector6f, out: []f32) []f32 {
    const result = out[0..tam.len];

    for (tam, result) |row, *r| {
        r.* = @reduce(.Add, row * v);
    }

    return result;
}

pub fn debug3f(comptime s: []const u8, v: Auv.Vector3f) void {
    std.log.debug(s ++ " {{ {d:5.2} {d:5.2} {d:5.2} }}", .{
        v[0], v[1], v[2],
    });
}

pub fn debug6f(comptime s: []const u8, v: Auv.Vector6f) void {
    std.log.debug(s ++ " {{ {d:5.2} {d:5.2} {d:5.2} {d:5.2} {d:5.2} {d:5.2} }}", .{
        v[0], v[1], v[2], v[3], v[4], v[5],
    });
}
