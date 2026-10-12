const std = @import("std");
const math = @import("math.zig");
const Auv = @import("Auv.zig");
const Allocator = @This();

pub const Thruster = struct {
    /// Position in body coordinates, before subtracting center_of_mass.
    position: math.Vector3f = @splat(0),
    /// Direction of positive physical thrust, independent of motor polarity.
    direction: math.Vector3f = .{ 0, 1, 0 },
    forward_force: f32,
    reverse_force: f32,
    command_sign: f32 = 1,
    /// |force| = full_force * |normalized motor command|^exponent.
    exponent: f32 = 2,
    /// Propeller reaction moment / signed thrust, metres.
    torque_per_force: f32 = 0,
    /// Normalized motor command change per second; zero disables slew limiting.
    command_rate: f32 = 0,
};

pub const Options = struct {
    use_backend_geometry: bool = false,
    center_of_mass: math.Vector3f = @splat(0),
    thrusters: []const Thruster = &.{},
    /// Relative residual weights for N and Nm; higher values prioritize an axis.
    weights: math.Vector6f = .{ 1, 1, 1, 4, 4, 3 },
    regularization: f32 = 0.00001,
    iterations: u16 = 80,
    require_full_rank: bool = true,

    pub fn validate(options: Options) !void {
        if (options.thrusters.len == 0 or options.thrusters.len > math.max_thrusters or
            !finite3(options.center_of_mass) or !positive(options.regularization) or
            options.iterations == 0 or options.iterations > 512) return error.InvalidAllocationOptions;
        inline for (0..6) |axis| if (!positive(options.weights[axis])) return error.InvalidAllocationOptions;
        for (options.thrusters) |thruster| {
            if (!finite3(thruster.position) or !finite3(thruster.direction) or
                math.length3f(thruster.direction) < 0.0001 or !positive(thruster.forward_force) or
                !positive(thruster.reverse_force) or !positive(thruster.exponent) or
                @abs(thruster.command_sign) != 1 or !std.math.isFinite(thruster.torque_per_force) or
                !std.math.isFinite(thruster.command_rate) or thruster.command_rate < 0)
                return error.InvalidThrusterModel;
        }
    }
};

pub const Result = struct {
    commands: [math.max_thrusters]f32 = @splat(0),
    len: usize,
    requested: math.Vector6f,
    achieved: math.Vector6f,
    residual: math.Vector6f,
    saturated: bool,
};

columns: [math.max_thrusters]math.Vector6f = undefined,
models: [math.max_thrusters]Thruster = undefined,
len: usize,
options: Options,
previous_commands: [math.max_thrusters]f32 = @splat(0),

pub fn init(options: Options, geometry: ?Auv.ThrusterGeometry) !Allocator {
    try options.validate();
    var allocator: Allocator = .{ .len = options.thrusters.len, .options = options };
    const backend = if (options.use_backend_geometry) geometry orelse return error.MissingThrusterGeometry else null;
    if (backend) |mounts| {
        if (mounts.len != allocator.len or !finite3(mounts.center_of_mass)) return error.InvalidThrusterGeometry;
    }
    for (options.thrusters, 0..) |model, i| {
        var thruster = model;
        const center = if (backend) |mounts| blk: {
            thruster.position = mounts.mounts[i].position;
            thruster.direction = mounts.mounts[i].direction;
            break :blk mounts.center_of_mass;
        } else options.center_of_mass;
        if (!finite3(thruster.position) or !finite3(thruster.direction) or math.length3f(thruster.direction) < 0.0001)
            return error.InvalidThrusterGeometry;
        const direction = math.normalize3f(thruster.direction);
        const moment = math.cross3f(thruster.position - center, direction) + direction * @as(math.Vector3f, @splat(thruster.torque_per_force));
        allocator.columns[i] = .{ direction[0], direction[1], direction[2], moment[0], moment[1], moment[2] };
        allocator.models[i] = thruster;
    }
    if (options.require_full_rank and allocator.rank() < 6) return error.UnderactuatedGeometry;
    return allocator;
}

pub fn reset(allocator: *Allocator) void {
    allocator.previous_commands = @splat(0);
}

/// Unit normal to a two-dimensional body-force plane. A full three-dimensional
/// force span has no missing direction; a line cannot be repaired by one tilt.
pub fn translationPlaneNormal(allocator: *const Allocator) ?math.Vector3f {
    var basis: [3]math.Vector3f = undefined;
    var count: usize = 0;
    for (allocator.columns[0..allocator.len]) |column| {
        var direction: math.Vector3f = .{ column[0], column[1], column[2] };
        for (basis[0..count]) |axis| direction -= axis * @as(math.Vector3f, @splat(math.dot3f(direction, axis)));
        const length = math.length3f(direction);
        if (length <= 1e-4) continue;
        basis[count] = direction / @as(math.Vector3f, @splat(length));
        count += 1;
        if (count == 3) return null;
    }
    if (count != 2) return null;
    return math.normalize3f(math.cross3f(basis[0], basis[1]));
}

/// This first adaptive attitude policy is for a missing sideways force axis
/// with independent attitude torque, as on Proteus. Other layouts stay explicit.
pub fn supportsPitchAdaptation(allocator: *const Allocator) bool {
    const normal = allocator.translationPlaneNormal() orelse return allocator.rank() == 6;
    return @abs(normal[0]) >= 0.999 and allocator.rank() >= 5;
}

/// Bounded coordinate descent solves min ||W(B f - requested)||² + lambda ||f||².
/// Limits and motor slew bounds are enforced during the solve, never by clipping
/// an unconstrained inverse afterward. Work is bounded and allocation-free.
pub fn allocate(allocator: *Allocator, requested: math.Vector6f, max_effort: f32, elapsed: f32) Result {
    var result: Result = .{ .len = allocator.len, .requested = requested, .achieved = @splat(0), .residual = requested, .saturated = false };
    inline for (0..6) |axis| if (!std.math.isFinite(requested[axis])) {
        allocator.reset();
        result.requested = @splat(0);
        result.residual = @splat(0);
        return result;
    };
    const limit = std.math.clamp(max_effort, 0, 1);
    const dt = if (elapsed > 0 and elapsed <= 0.25) elapsed else 0;
    var forces: [math.max_thrusters]f32 = @splat(0);
    var low: [math.max_thrusters]f32 = undefined;
    var high: [math.max_thrusters]f32 = undefined;
    var residual: math.Vector6f = -requested;
    for (0..allocator.len) |i| {
        const model = allocator.models[i];
        var low_command = -limit;
        var high_command = limit;
        if (model.command_rate > 0) {
            const previous = std.math.clamp(allocator.previous_commands[i], -limit, limit);
            low_command = @max(low_command, previous - model.command_rate * dt);
            high_command = @min(high_command, previous + model.command_rate * dt);
        }
        const a = commandToForce(model, low_command);
        const b = commandToForce(model, high_command);
        low[i] = @min(a, b);
        high[i] = @max(a, b);
        forces[i] = std.math.clamp(commandToForce(model, allocator.previous_commands[i]), low[i], high[i]);
        residual += allocator.columns[i] * @as(math.Vector6f, @splat(forces[i]));
    }
    const weights2 = allocator.options.weights * allocator.options.weights;
    for (0..allocator.options.iterations) |_| {
        var change: f32 = 0;
        for (0..allocator.len) |i| {
            const column = allocator.columns[i];
            const gradient = @reduce(.Add, column * weights2 * residual) + allocator.options.regularization * forces[i];
            const curvature = @reduce(.Add, column * weights2 * column) + allocator.options.regularization;
            const next = std.math.clamp(forces[i] - gradient / curvature, low[i], high[i]);
            const delta = next - forces[i];
            forces[i] = next;
            residual += column * @as(math.Vector6f, @splat(delta));
            change = @max(change, @abs(delta));
        }
        if (change < 0.00001) break;
    }
    for (0..allocator.len) |i| {
        result.commands[i] = std.math.clamp(forceToCommand(allocator.models[i], forces[i]), -limit, limit);
        const actual_force = commandToForce(allocator.models[i], result.commands[i]);
        result.achieved += allocator.columns[i] * @as(math.Vector6f, @splat(actual_force));
    }
    allocator.previous_commands = result.commands;
    result.residual = requested - result.achieved;
    result.saturated = @reduce(.Add, result.residual * result.residual) > 0.0001;
    return result;
}

fn commandToForce(model: Thruster, command: f32) f32 {
    const signed = command * model.command_sign;
    const maximum = if (signed >= 0) model.forward_force else model.reverse_force;
    return std.math.sign(signed) * maximum * std.math.pow(f32, @abs(signed), model.exponent);
}

fn forceToCommand(model: Thruster, force: f32) f32 {
    const maximum = if (force >= 0) model.forward_force else model.reverse_force;
    return model.command_sign * std.math.sign(force) * std.math.pow(f32, @abs(force) / maximum, 1 / model.exponent);
}

fn rank(allocator: *const Allocator) usize {
    var matrix = std.mem.zeroes([6][math.max_thrusters]f32);
    inline for (0..6) |row| {
        for (0..allocator.len) |col| matrix[row][col] = allocator.columns[col][row];
    }
    var pivot: usize = 0;
    for (0..allocator.len) |col| {
        if (pivot == 6) break;
        var best = pivot;
        for (pivot..6) |row| if (@abs(matrix[row][col]) > @abs(matrix[best][col])) {
            best = row;
        };
        if (@abs(matrix[best][col]) < 0.00001) continue;
        std.mem.swap([math.max_thrusters]f32, &matrix[pivot], &matrix[best]);
        for (pivot + 1..6) |row| {
            const scale = matrix[row][col] / matrix[pivot][col];
            for (col..allocator.len) |k| matrix[row][k] -= scale * matrix[pivot][k];
        }
        pivot += 1;
    }
    return pivot;
}

fn finite3(v: math.Vector3f) bool {
    inline for (0..3) |i| if (!std.math.isFinite(v[i])) return false;
    return true;
}
fn positive(v: f32) bool {
    return std.math.isFinite(v) and v > 0;
}
