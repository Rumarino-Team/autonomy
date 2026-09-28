//! Low-level control policy trained in rl/ (PPO on the MuJoCo Hydrus physics).
//! Replaces the global-navigation PID: pose goal in, 6-DOF TAM command out.
//! The observation must match rl/pose_env.py build_observation().
const std = @import("std");
const Io = std.Io;
const math = @import("math.zig");

pub const obs_size = 20;
pub const act_size = 6;

const magic = "HYPL";
const version: u32 = 1;

const Activation = enum(u32) { identity = 0, tanh = 1 };

const Layer = struct {
    in: usize,
    out: usize,
    activation: Activation,
    weight: []f32,
    bias: []f32,
};

pub const LoadError = error{ BadMagic, BadVersion, BadShape, Truncated };

/// MLP plus the observation normalization from training, read from rl/export_policy.py output.
pub const Policy = struct {
    policy_hz: f32,
    max_goal_dist: f32,
    close_enough: f32,
    clip_obs: f32,
    obs_mean: [obs_size]f32,
    obs_inv_std: [obs_size]f32,
    layers: []Layer,
    scratch: [2][]f32,

    pub fn load(gpa: std.mem.Allocator, io: Io, path: []const u8) !Policy {
        const bytes = try Io.Dir.cwd().readFileAlloc(io, path, gpa, .limited(64 << 20));
        defer gpa.free(bytes);
        var r: Reader = .{ .bytes = bytes };

        if (!std.mem.eql(u8, try r.take(4), magic)) return LoadError.BadMagic;
        if (try r.int() != version) return LoadError.BadVersion;
        const n_obs = try r.int();
        const n_act = try r.int();
        const n_layers = try r.int();
        if (n_obs != obs_size or n_act != act_size or n_layers == 0) return LoadError.BadShape;

        var policy: Policy = undefined;
        policy.policy_hz = try r.float();
        policy.max_goal_dist = try r.float();
        policy.close_enough = try r.float();
        policy.clip_obs = try r.float();
        for (&policy.obs_mean) |*v| v.* = try r.float();
        for (&policy.obs_inv_std) |*v| v.* = try r.float();

        policy.layers = try gpa.alloc(Layer, n_layers);
        var loaded: usize = 0;
        errdefer {
            for (policy.layers[0..loaded]) |layer| {
                gpa.free(layer.weight);
                gpa.free(layer.bias);
            }
            gpa.free(policy.layers);
        }
        var width: usize = obs_size;
        var prev_out: usize = obs_size;
        for (policy.layers) |*layer| {
            layer.in = try r.int();
            layer.out = try r.int();
            layer.activation = std.enums.fromInt(Activation, try r.int()) orelse return LoadError.BadShape;
            if (layer.in != prev_out) return LoadError.BadShape;
            layer.weight = try gpa.alloc(f32, layer.in * layer.out);
            errdefer gpa.free(layer.weight);
            layer.bias = try gpa.alloc(f32, layer.out);
            errdefer gpa.free(layer.bias);
            for (layer.weight) |*v| v.* = try r.float();
            for (layer.bias) |*v| v.* = try r.float();
            loaded += 1;
            prev_out = layer.out;
            width = @max(width, layer.out);
        }
        if (prev_out != act_size or r.pos != bytes.len) return LoadError.BadShape;

        policy.scratch[0] = try gpa.alloc(f32, width);
        errdefer gpa.free(policy.scratch[0]);
        policy.scratch[1] = try gpa.alloc(f32, width);
        return policy;
    }

    pub fn deinit(policy: *Policy, gpa: std.mem.Allocator) void {
        for (policy.layers) |layer| {
            gpa.free(layer.weight);
            gpa.free(layer.bias);
        }
        gpa.free(policy.layers);
        gpa.free(policy.scratch[0]);
        gpa.free(policy.scratch[1]);
    }

    /// Deterministic action in [-1, 1], used directly as the TAM input.
    pub fn forward(policy: *Policy, obs: [obs_size]f32) [act_size]f32 {
        var x: []f32 = policy.scratch[0][0..obs_size];
        for (x, obs, policy.obs_mean, policy.obs_inv_std) |*dst, o, mean, inv_std| {
            dst.* = std.math.clamp((o - mean) * inv_std, -policy.clip_obs, policy.clip_obs);
        }
        var which: usize = 0;
        for (policy.layers) |layer| {
            const y = policy.scratch[1 - which][0..layer.out];
            for (y, 0..) |*dst, row| {
                const w = layer.weight[row * layer.in ..][0..layer.in];
                var sum: f32 = layer.bias[row];
                for (w, x) |wi, xi| sum += wi * xi;
                dst.* = switch (layer.activation) {
                    .identity => sum,
                    .tanh => std.math.tanh(sum),
                };
            }
            x = y;
            which = 1 - which;
        }
        var action: [act_size]f32 = undefined;
        for (&action, x) |*dst, v| dst.* = std.math.clamp(v, -1.0, 1.0);
        return action;
    }

    /// Returns a new action when a policy period has elapsed since the last one, else null
    /// (keep applying the previous command). Frames arrive at the physics rate, faster than
    /// the policy rate it was trained at.
    pub fn step(
        policy: *Policy,
        runner: *Runner,
        pose: math.Pose,
        timestamp_ns: u64,
        goal: math.Vector6f,
    ) ?[act_size]f32 {
        const period = 1.0 / policy.policy_hz;
        var dt: f32 = period;
        if (runner.last_ns) |last| {
            if (timestamp_ns < last) {
                runner.* = .{};
            } else {
                dt = @as(f32, @floatFromInt(timestamp_ns - last)) * 1e-9;
                if (dt < 0.95 * period) return null;
            }
        }
        const first = runner.last_ns == null;
        const prev_pos = if (first) pose.pos else runner.prev_pos;
        const prev_quat = if (first) pose.quat else runner.prev_quat;
        const obs = observation(policy.*, pose, prev_pos, prev_quat, dt, goal, runner.prev_action);
        const action = policy.forward(obs);
        runner.prev_pos = pose.pos;
        runner.prev_quat = pose.quat;
        runner.prev_action = action;
        runner.last_ns = timestamp_ns;
        return action;
    }
};

/// Per-mission policy memory: last pose and action at the policy rate.
pub const Runner = struct {
    last_ns: ?u64 = null,
    prev_pos: math.Vector3f = @splat(0),
    prev_quat: math.Quaternionf = .{ 0, 0, 0, 1 },
    prev_action: [act_size]f32 = @splat(0),
    thrusters: [8]f32 = @splat(0),
    thrusters_len: usize = 0,
};

/// Body-frame goal error (world xy length and depth clipped separately), world down axis, sin/cos heading error, linear and
/// angular velocity from pose differences, previous action.
pub fn observation(
    policy: Policy,
    pose: math.Pose,
    prev_pos: math.Vector3f,
    prev_quat: math.Quaternionf,
    dt: f32,
    goal: math.Vector6f,
    prev_action: [act_size]f32,
) [obs_size]f32 {
    const q = math.normalize4f(pose.quat);
    const inv = math.quaternionConjugate(q);
    var world_err = math.Vector3f{ goal[0], goal[1], goal[2] } - pose.pos;
    const xy_len = @sqrt(world_err[0] * world_err[0] + world_err[1] * world_err[1]);
    if (xy_len > policy.max_goal_dist) {
        world_err[0] *= policy.max_goal_dist / xy_len;
        world_err[1] *= policy.max_goal_dist / xy_len;
    }
    world_err[2] = std.math.clamp(world_err[2], -policy.max_goal_dist, policy.max_goal_dist);
    const err = math.quaternionRotate(inv, world_err);
    const down = math.quaternionRotate(inv, .{ 0, 0, 1 });

    // Same heading target as global.zig pidStep.
    const forward = math.quaternionRotate(q, .{ 0, 1, 0 });
    const current_yaw = std.math.atan2(forward[1], forward[0]);
    const dx = goal[0] - pose.pos[0];
    const dy = goal[1] - pose.pos[1];
    const target_yaw = if (@sqrt(dx * dx + dy * dy) > policy.close_enough) std.math.atan2(dy, dx) else goal[5];
    const yaw_error = math.wrapAngle(target_yaw - current_yaw);

    const vel = math.quaternionRotate(inv, pose.pos - prev_pos) / @as(math.Vector3f, @splat(dt));
    const rel = math.quaternionMul(math.quaternionConjugate(math.normalize4f(prev_quat)), q);
    const sign: f32 = if (rel[3] < 0) -1.0 else 1.0;
    const ang = math.Vector3f{ rel[0], rel[1], rel[2] } * @as(math.Vector3f, @splat(2.0 * sign / dt));

    return .{
        err[0],          err[1],          err[2],
        down[0],         down[1],         down[2],
        @sin(yaw_error), @cos(yaw_error), vel[0],
        vel[1],          vel[2],          ang[0],
        ang[1],          ang[2],          prev_action[0],
        prev_action[1],  prev_action[2],  prev_action[3],
        prev_action[4],  prev_action[5],
    };
}

const Reader = struct {
    bytes: []const u8,
    pos: usize = 0,

    fn take(r: *Reader, n: usize) LoadError![]const u8 {
        if (r.pos + n > r.bytes.len) return LoadError.Truncated;
        defer r.pos += n;
        return r.bytes[r.pos..][0..n];
    }

    fn int(r: *Reader) LoadError!u32 {
        return std.mem.readInt(u32, (try r.take(4))[0..4], .little);
    }

    fn float(r: *Reader) LoadError!f32 {
        return @bitCast(try r.int());
    }
};
