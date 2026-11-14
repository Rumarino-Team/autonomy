const std = @import("std");
const c = @import("sim_c");
const S = @import("support.zig");
const arena = @import("arena.zig");

fn coefficients(force: c.MathVector3f, basis: c.MathQuaternionf, cd: c.MathVector3f, scale: f64) c.MathVector3f {
    @setFloatMode(.strict);
    const d = S.rotate(S.conjugate(basis), force);
    const len = S.norm(d);
    const factor = if (len < 1.4901161193847656e-8) 0 else scale * (@abs(d.buf[0]) * cd.buf[0] + @abs(d.buf[1]) * cd.buf[1] + @abs(d.buf[2]) * cd.buf[2]) / len;
    return S.scale(force, factor);
}

fn gravity(context: *const c.SimForceContext, w: []c.SimWrench) void {
    @setFloatMode(.strict);
    for (w, 0..) |*wrench, i| {
        const body = context.model.*.object_table[i].*.body;
        if (body.motion_type == c.SIM_DYNAMIC)
            wrench.force = S.add(wrench.force, S.scale(context.model.*.settings.gravity, body.dry_mass - body.mass));
    }
}

fn hydro(context: *const c.SimForceContext, w: []c.SimWrench) !void {
    @setFloatMode(.strict);
    const model = context.model.*;
    if (model.settings.native_hydrodynamics != 0) return;
    if (context.tick % model.settings.hydro_period == 0) {
        var i: usize = 0;
        while (i < context.body_count) : (i += 1) context.fluid_cache[i] = std.mem.zeroes(c.SimWrench);
        const minz_raw = arena.arena_alloc(context.scratch, try S.bytes(w.len, @sizeOf(f64)), @alignOf(f64));
        const maxz_raw = arena.arena_alloc(context.scratch, try S.bytes(w.len, @sizeOf(f64)), @alignOf(f64));
        try S.require(minz_raw != null and maxz_raw != null, "hydrodynamic scratch exhausted", c.SIM_CAPACITY);
        const minz: [*]f64 = @ptrCast(@alignCast(minz_raw.?));
        const maxz: [*]f64 = @ptrCast(@alignCast(maxz_raw.?));
        @memset(minz[0..w.len], std.math.inf(f64));
        @memset(maxz[0..w.len], -std.math.inf(f64));
        const Wet = struct { lo: f64 = std.math.inf(f64), hi: f64 = -std.math.inf(f64) };
        var wet: [5]Wet = @splat(.{});
        var p: usize = 0;
        while (p < model.robot.hull_count) : (p += 1) {
            const h = model.robot.hulls[p];
            const transform = S.compose(context.bodies[h.object.*.id.index].pose, h.pose);
            var vertex: u32 = 0;
            while (vertex < h.position_count) : (vertex += 1) {
                const z = S.point(transform, h.positions[vertex]).buf[2];
                wet[p].lo = @min(wet[p].lo, z);
                wet[p].hi = @max(wet[p].hi, z);
            }
            const index = h.object.*.id.index;
            minz[index] = @min(minz[index], wet[p].lo);
            maxz[index] = @max(maxz[index], wet[p].hi);
        }
        for (w, 0..) |_, index| if (minz[index] > 0 and std.math.isFinite(minz[index])) {
            const body = model.object_table[index].*.body;
            S.addForce(&context.fluid_cache[index], S.scale(model.settings.gravity, -model.settings.water_density * body.volume), S.point(context.bodies[index].pose, body.buoyancy_center), context.bodies[index].center_of_mass);
        };
        p = 0;
        while (p < model.robot.hull_count) : (p += 1) {
            const h = model.robot.hulls[p];
            const index = h.object.*.id.index;
            const body = context.bodies[index];
            if (maxz[index] <= 0) continue;
            const transform = S.compose(body.pose, h.pose);
            const out = &context.fluid_cache[index];
            if (minz[index] <= 0 and wet[p].hi > 0) {
                const fraction = if (wet[p].lo > 0) 1 else wet[p].hi / (wet[p].hi - wet[p].lo);
                S.addForce(out, S.scale(model.settings.gravity, -fraction * h.volume * model.settings.water_density), S.point(transform, h.centroid), body.center_of_mass);
            }
            var pressure_linear = S.v(0, 0, 0);
            var pressure_angular = S.v(0, 0, 0);
            var skin_linear = S.v(0, 0, 0);
            var skin_angular = S.v(0, 0, 0);
            var t: u32 = 0;
            while (t < h.position_count) : (t += 3) {
                const a = S.point(transform, h.positions[t]);
                const b = S.point(transform, h.positions[t + 1]);
                const d = S.point(transform, h.positions[t + 2]);
                var normal = S.cross(S.sub(b, a), S.sub(d, a));
                const twice = S.norm(normal);
                if (twice < 1e-6) continue;
                normal = S.scale(normal, 1 / twice);
                const center = S.scale(S.add(S.add(a, b), d), 1.0 / 3.0);
                if (minz[index] <= 0 and center.buf[2] <= 0) continue;
                const arm = S.sub(center, body.center_of_mass);
                const relative = S.sub(model.settings.current, S.add(body.linear_velocity, S.cross(body.angular_velocity, arm)));
                const vn = S.dot(relative, normal);
                if (vn < -1e-12) {
                    const force = S.scale(relative, S.norm(relative) * -vn * 0.5 * twice);
                    pressure_linear = S.add(pressure_linear, force);
                    pressure_angular = S.add(pressure_angular, S.cross(arm, force));
                }
                const tangent = S.sub(relative, S.scale(normal, vn));
                if (S.dot(tangent, tangent) > 1e-9) {
                    const force = S.scale(tangent, 0.5 * twice);
                    skin_linear = S.add(skin_linear, force);
                    skin_angular = S.add(skin_angular, S.cross(arm, force));
                }
            }
            const rho = model.settings.water_density;
            const frame = S.multiply(body.pose.quat, h.coefficient_frame);
            out.force = S.add(S.add(out.force, coefficients(pressure_linear, frame, h.drag, 0.5 * rho)), coefficients(skin_linear, frame, h.skin, rho));
            out.torque = S.add(S.add(out.torque, coefficients(pressure_angular, frame, h.drag, 0.5 * rho)), coefficients(skin_angular, frame, h.skin, rho));
        }
    }
    for (w, 0..) |*wrench, index| {
        wrench.force = S.add(wrench.force, context.fluid_cache[index].force);
        wrench.torque = S.add(wrench.torque, context.fluid_cache[index].torque);
    }
}

fn thrusters(context: *const c.SimForceContext, w: []c.SimWrench) void {
    @setFloatMode(.strict);
    const model = context.model.*;
    var i: usize = 0;
    while (i < model.robot.thruster_count) : (i += 1) {
        const t = model.robot.thrusters[i];
        const rotor = &context.rotors[i];
        const body = context.bodies[t.object.*.id.index];
        const spec = t.spec;
        const max = spec.max_rpm / 60 * 2 * std.math.pi;
        const command = if (i < context.command_count) context.commands[i] else 0;
        const target = spec.command_sign * std.math.clamp(command, -1, 1) * max;
        const error_omega = target - rotor.omega;
        const damping = if (rotor.omega > 0) @abs(rotor.torque) else -@abs(rotor.torque);
        rotor.omega = std.math.clamp((spec.inertia * rotor.omega + context.dt * (spec.kp * target + spec.ki * rotor.integral - damping)) / (spec.inertia + context.dt * spec.kp), -2 * max, 2 * max);
        rotor.integral = std.math.clamp(rotor.integral + error_omega * context.dt, -5, 5);
        const p = S.point(body.pose, t.position);
        const axis = S.rotate(body.pose.quat, t.direction);
        rotor.thrust = 0;
        if (p.buf[2] <= 0) {
            rotor.torque = 0;
            continue;
        }
        const inflow = S.dot(axis, S.sub(S.add(body.linear_velocity, S.cross(body.angular_velocity, S.sub(p, body.center_of_mass))), model.settings.current));
        const n = rotor.omega / (2 * std.math.pi);
        const d = spec.diameter;
        const rho = model.settings.water_density;
        rotor.thrust = rho * d * d * d * @abs(n) * (d * spec.thrust * n - spec.thrust * inflow);
        rotor.torque = -rho * d * d * d * d * @abs(n) * (d * spec.torque * n - spec.torque * inflow);
        const wrench = &w[t.object.*.id.index];
        S.addForce(wrench, S.scale(axis, rotor.thrust), p, body.center_of_mass);
        wrench.torque = S.add(wrench.torque, S.scale(axis, rotor.torque));
    }
}

fn evaluateGravity(context: ?*const c.SimForceContext, _: ?*anyopaque, w: ?[*]c.SimWrench, n: u32, e: ?[*]u8, cap: u32) callconv(.c) c.SimResult {
    return S.guard(e, cap, struct {
        fn run(ctx: ?*const c.SimForceContext, wrenches: ?[*]c.SimWrench, count: u32) !void {
            gravity(ctx.?, wrenches.?[0..count]);
        }
    }.run(context, w, n));
}

fn evaluateHydro(context: ?*const c.SimForceContext, _: ?*anyopaque, w: ?[*]c.SimWrench, n: u32, e: ?[*]u8, cap: u32) callconv(.c) c.SimResult {
    return S.guard(e, cap, struct {
        fn run(ctx: ?*const c.SimForceContext, wrenches: ?[*]c.SimWrench, count: u32) !void {
            try hydro(ctx.?, wrenches.?[0..count]);
        }
    }.run(context, w, n));
}

fn evaluateThrusters(context: ?*const c.SimForceContext, _: ?*anyopaque, w: ?[*]c.SimWrench, n: u32, e: ?[*]u8, cap: u32) callconv(.c) c.SimResult {
    return S.guard(e, cap, struct {
        fn run(ctx: ?*const c.SimForceContext, wrenches: ?[*]c.SimWrench, count: u32) !void {
            thrusters(ctx.?, wrenches.?[0..count]);
        }
    }.run(context, w, n));
}

fn named(comptime name: []const u8, comptime contribution: []const u8, eval: @FieldType(c.ForceModule, "evaluate")) c.ForceModule {
    var module = std.mem.zeroes(c.ForceModule);
    @memcpy(module.name[0..name.len], name);
    @memcpy(module.contribution[0..contribution.len], contribution);
    module.evaluate = eval;
    return module;
}

var builtins = [_]c.ForceModule{
    named("gravity_compensation", "gravity", evaluateGravity),
    named("hydrodynamics", "hydrodynamics", evaluateHydro),
    named("thrusters", "actuation", evaluateThrusters),
};

pub fn builtinModules() []const c.ForceModule {
    return &builtins;
}

export fn sim_add_force(w: ?*c.SimWrench, f: c.MathVector3f, p: c.MathVector3f, com: c.MathVector3f) callconv(.c) void {
    S.addForce(w orelse return, f, p, com);
}
