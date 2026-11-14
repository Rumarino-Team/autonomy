const std = @import("std");
const c = @import("sim_c");
const S = @import("support.zig");
const arena = @import("arena.zig");
const model = @import("model.zig");
const forces = @import("forces.zig");
const sensors = @import("sensors.zig");
const state = @import("state.zig");

fn bind(buffer: anytype, alloc: c.SimAllocator) void {
    buffer.alloc = alloc;
}

fn read(r: *state.Runtime) !void {
    r.bodies.resize(r.model.object_count) catch |err| return err;
    var count: u32 = 0;
    try S.check(r.backend.read_state.?(r.world, r.bodies.ptr, @intCast(r.bodies.n), &count), "read states failed");
    try S.require(count == r.bodies.n, "wrong state count", c.SIM_INVALID);
    var i: u32 = 0;
    while (i < count) : (i += 1) {
        const body = r.bodies.ptr.?[i];
        try S.require(body.id == r.model.object_table[i].*.body.id and S.finite3(body.linear_velocity) and S.finite3(body.angular_velocity) and S.finite3(body.center_of_mass), "invalid backend state", c.SIM_INVALID);
        try S.validPose(body.pose);
    }
    const sized = r.backend.read_contacts.?(r.world, null, 0, &count);
    try S.require(sized == c.SIM_OK or sized == c.SIM_BUFFER_TOO_SMALL, "contact sizing failed", c.SIM_INVALID);
    try r.contacts.resize(count);
    try S.check(r.backend.read_contacts.?(r.world, r.contacts.ptr, count, &count), "read contacts failed");
    try S.require(count == r.contacts.n, "wrong contact count", c.SIM_INVALID);
    for (r.contacts.items()) |contact| {
        try S.require(S.finite3(contact.position) and S.finite3(contact.normal) and std.math.isFinite(contact.normal_force), "invalid backend contact", c.SIM_INVALID);
    }
}

fn reset(r: *state.Runtime, states: ?[*]const c.SimBodyState, count: u32) !void {
    try S.require((states != null and count == r.model.object_count) or (states == null and count == 0), "reset requires all states", c.SIM_INVALID);
    var message: [512]u8 = @splat(0);
    try S.check(r.backend.reset.?(r.world, states, count, &message, message.len), std.mem.sliceTo(&message, 0));
    r.tick = 0;
    r.rotors = std.mem.zeroes([8]c.RotorState);
    r.commands = @splat(0);
    r.input = @splat(0);
    @memset(r.fluid.items(), std.mem.zeroes(c.SimWrench));
    @memset(r.wrenches.items(), std.mem.zeroes(c.SimWrench));
    arena.arena_reset(&r.scratch);
    for (r.modules.items()) |module| if (module.api.reset) |reset_fn| {
        message = @splat(0);
        try S.check(reset_fn(module.state, &message, message.len), std.mem.sliceTo(&message, 0));
    };
    try read(r);
    var i: usize = 0;
    while (i < r.bodies.n) : (i += 1) {
        r.previous_linear.ptr.?[i] = r.bodies.ptr.?[i].linear_velocity;
        r.previous_angular.ptr.?[i] = r.bodies.ptr.?[i].angular_velocity;
    }
    sensors.sample(r);
}

fn destroyRuntime(r: *state.Runtime) void {
    if (r.world) |world| if (r.backend.world_destroy) |destroy| destroy(world);
    var i = r.modules.n;
    while (i > 0) {
        i -= 1;
        const module = r.modules.ptr.?[i];
        if (module.created != 0) if (module.api.destroy) |destroy| destroy(module.state);
    }
    r.scratch_memory.deinit();
    r.modules.deinit();
    r.detections.deinit();
    r.samples.deinit();
    r.previous_angular.deinit();
    r.previous_linear.deinit();
    r.fluid.deinit();
    r.wrenches.deinit();
    r.contacts.deinit();
    r.bodies.deinit();
    std.heap.c_allocator.destroy(r);
}

fn moduleName(module: c.ForceModule) []const u8 {
    return std.mem.sliceTo(&module.name, 0);
}

fn contribution(module: c.ForceModule) []const u8 {
    return std.mem.sliceTo(&module.contribution, 0);
}

export fn simulation_runtime_create(
    m: ?*const c.Simulation,
    b: ?*const c.PhysicsBackend,
    registry: ?*const c.ForceRegistry,
    out: ?*?*state.Runtime,
    e: ?[*]u8,
    n: u32,
) callconv(.c) c.SimResult {
    if (out) |slot| slot.* = null;
    return S.guard(e, n, struct {
        fn run(model_in: ?*const c.Simulation, backend: ?*const c.PhysicsBackend, forces_in: ?*const c.ForceRegistry, slot: ?*?*state.Runtime) !void {
            try S.require(slot != null and backend != null and backend.?.library != null, "missing backend", c.SIM_INVALID);
            const api = backend.?.api;
            try S.require(api.struct_size == @sizeOf(c.PhysicsBackendApi) and api.abi_version == c.PHYSICS_BACKEND_ABI_VERSION and api.world_create != null and api.world_destroy != null and api.reset != null and api.step != null and api.read_state != null and api.read_contacts != null and api.state_hash != null, "backend ABI v5 required", c.SIM_UNSUPPORTED);
            var message: [256]u8 = @splat(0);
            try S.check(simulation_validate(model_in, &message, message.len), std.mem.sliceTo(&message, 0));
            try S.require(model_in.?.frozen != 0, "backend requires finalized model", c.SIM_INVALID);
            const scene = model_in.?;
            try S.require(api.integrators & (@as(u32, 1) << @intCast(scene.settings.integrator)) != 0, "unsupported integrator", c.SIM_UNSUPPORTED);
            try S.require(scene.settings.native_hydrodynamics == 0 or api.capabilities & c.SIM_CAP_NATIVE_HYDRO != 0, "native hydrodynamics unsupported", c.SIM_UNSUPPORTED);
            const alloc = model.modelConfig(scene).allocator;
            const r = std.heap.c_allocator.create(state.Runtime) catch return error.OutOfMemory;
            r.* = .{ .model = scene, .backend = api };
            inline for (.{ &r.bodies, &r.contacts, &r.wrenches, &r.fluid, &r.previous_linear, &r.previous_angular, &r.samples, &r.detections, &r.modules, &r.scratch_memory }) |buffer| bind(buffer, alloc);
            errdefer destroyRuntime(r);
            try r.bodies.resize(scene.object_count);
            try r.wrenches.resize(scene.object_count);
            try r.fluid.resize(scene.object_count);
            try r.previous_linear.resize(scene.object_count);
            try r.previous_angular.resize(scene.object_count);
            try r.samples.resize(scene.sensor_count);
            try r.detections.resize(try S.bytes(scene.sensor_count, scene.object_count));
            try r.scratch_memory.resize(model.modelConfig(scene).scratch_bytes);
            arena.arena_init(&r.scratch, r.scratch_memory.ptr, r.scratch_memory.n);
            const builtin = forces.builtinModules();
            try S.require(forces_in == null or (forces_in.?.count <= 4096 and (forces_in.?.count == 0 or forces_in.?.entries != null)), "invalid force registry", c.SIM_INVALID);
            if (forces_in) |reg| {
                var i: u32 = 0;
                while (i < reg.count) : (i += 1) {
                    const entry = reg.entries[i];
                    try S.require(std.mem.indexOfScalar(u8, &entry.name, 0) != null and entry.name[0] != 0 and std.mem.indexOfScalar(u8, &entry.contribution, 0) != null and entry.evaluate != null and (entry.create != null) == (entry.destroy != null), "invalid callback descriptor", c.SIM_INVALID);
                    var j: u32 = 0;
                    while (j < i) : (j += 1) try S.require(!std.mem.eql(u8, moduleName(entry), moduleName(reg.entries[j])), "duplicate registered force", c.SIM_INVALID);
                    for (builtin) |item| try S.require(!std.mem.eql(u8, moduleName(entry), moduleName(item)), "builtin force cannot be replaced", c.SIM_INVALID);
                }
            }
            const find = struct {
                fn lookup(builtin_modules: []const c.ForceModule, registered: ?*const c.ForceRegistry, name: []const u8) ?*const c.ForceModule {
                    for (builtin_modules) |*item| if (std.mem.eql(u8, name, moduleName(item.*))) return item;
                    if (registered) |found| {
                        var j: u32 = 0;
                        while (j < found.count) : (j += 1) if (std.mem.eql(u8, name, moduleName(found.entries[j]))) return &found.entries[j];
                    }
                    return null;
                }
            }.lookup;
            const add = struct {
                fn append(runtime: *state.Runtime, builtin_modules: []const c.ForceModule, reg: ?*const c.ForceRegistry, name: []const u8) !void {
                    const found = find(builtin_modules, reg, name) orelse return S.boom(c.SIM_INVALID, "unknown force module");
                    if (runtime.model.settings.native_hydrodynamics != 0 and std.mem.eql(u8, name, "hydrodynamics")) return;
                    for (runtime.modules.items()) |item| {
                        try S.require(!std.mem.eql(u8, moduleName(item.api), name), "duplicate force selection", c.SIM_INVALID);
                        try S.require(contribution(found.*).len == 0 or !std.mem.eql(u8, contribution(item.api), contribution(found.*)), "duplicate physical contribution", c.SIM_INVALID);
                    }
                    try S.require(runtime.model.settings.native_hydrodynamics == 0 or !std.mem.eql(u8, contribution(found.*), "hydrodynamics"), "duplicate native hydrodynamics provider", c.SIM_INVALID);
                    try runtime.modules.resize(runtime.modules.n + 1);
                    var entry = &runtime.modules.ptr.?[runtime.modules.n - 1];
                    entry.* = .{ .api = found.*, .state = null, .created = 0 };
                    if (found.create) |create| {
                        entry.created = 1;
                        var detail: [256]u8 = @splat(0);
                        try S.check(create(runtime.model, found.user, &entry.state, &detail, detail.len), std.mem.sliceTo(&detail, 0));
                    }
                }
            }.append;
            if (scene.module_count != 0) {
                var i: u32 = 0;
                while (i < scene.module_count) : (i += 1) try add(r, builtin, forces_in, std.mem.sliceTo(&scene.module_table[i].*.name, 0));
            } else for (builtin) |item| try add(r, builtin, forces_in, moduleName(item));
            var has_gravity = false;
            for (r.modules.items()) |item| has_gravity = has_gravity or std.mem.eql(u8, contribution(item.api), "gravity");
            try S.require(has_gravity, "gravity compensation provider is required", c.SIM_INVALID);
            var detail: [512]u8 = @splat(0);
            try S.check(api.world_create.?(scene, @ptrCast(&r.world), &detail, detail.len), std.mem.sliceTo(&detail, 0));
            try reset(r, null, 0);
            slot.?.* = r;
        }
    }.run(m, b, registry, out));
}

extern fn simulation_validate(s: ?*const c.Simulation, e: ?[*]u8, n: u32) callconv(.c) c.SimResult;

export fn simulation_runtime_destroy(r: ?*state.Runtime) callconv(.c) void {
    if (r) |runtime| destroyRuntime(runtime);
}

export fn simulation_runtime_reset(r: ?*state.Runtime, states: ?[*]const c.SimBodyState, count: u32, e: ?[*]u8, n: u32) callconv(.c) c.SimResult {
    return S.guard(e, n, struct {
        fn run(runtime: ?*state.Runtime, values: ?[*]const c.SimBodyState, nstates: u32) !void {
            try S.require(runtime != null, "null runtime", c.SIM_INVALID);
            try reset(runtime.?, values, nstates);
        }
    }.run(r, states, count));
}

export fn simulation_runtime_step(r: ?*state.Runtime, input: ?[*]const f64, count: u32, e: ?[*]u8, n: u32) callconv(.c) c.SimResult {
    return S.guard(e, n, struct {
        fn run(runtime_in: ?*state.Runtime, commands: ?[*]const f64, ncommands: u32) !void {
            const runtime = runtime_in orelse return S.boom(c.SIM_INVALID, "invalid command count");
            try S.require(ncommands == runtime.model.robot.thruster_count and (ncommands == 0 or commands != null), "invalid command count", c.SIM_INVALID);
            try S.require(runtime.tick < std.math.maxInt(u64), "tick overflow", c.SIM_INVALID);
            var i: u32 = 0;
            while (i < ncommands) : (i += 1) try S.require(std.math.isFinite(commands.?[i]) and commands.?[i] >= -1 and commands.?[i] <= 1, "command outside [-1,1]", c.SIM_INVALID);
            i = 0;
            while (i < ncommands) : (i += 1) {
                runtime.commands[i] = commands.?[i];
                runtime.input[i] = commands.?[i];
            }
            i = 0;
            while (i < runtime.model.fault_count) : (i += 1) {
                const f = runtime.model.fault_table[i].*;
                if (runtime.tick >= f.start and runtime.tick < f.end and (f.kind == c.SIM_FAULT_DEAD or f.kind == c.SIM_FAULT_WEAK))
                    runtime.commands[f.thruster] *= if (f.kind == c.SIM_FAULT_DEAD) 0 else f.magnitude;
            }
            i = 0;
            while (i < runtime.model.object_count) : (i += 1) {
                runtime.wrenches.ptr.?[i] = .{ .id = runtime.model.object_table[i].*.body.id, .force = S.v(0, 0, 0), .torque = S.v(0, 0, 0) };
                runtime.previous_linear.ptr.?[i] = runtime.bodies.ptr.?[i].linear_velocity;
                runtime.previous_angular.ptr.?[i] = runtime.bodies.ptr.?[i].angular_velocity;
            }
            arena.arena_reset(&runtime.scratch);
            defer arena.arena_reset(&runtime.scratch);
            const ctx = c.SimForceContext{
                .model = runtime.model,
                .bodies = runtime.bodies.ptr,
                .body_count = runtime.model.object_count,
                .commands = &runtime.commands,
                .command_count = ncommands,
                .tick = runtime.tick,
                .dt = @as(f64, @floatFromInt(runtime.model.settings.dt_numerator)) / @as(f64, @floatFromInt(runtime.model.settings.dt_denominator)),
                .rotors = &runtime.rotors,
                .fluid_cache = runtime.fluid.ptr,
                .scratch = &runtime.scratch,
            };
            var message: [512]u8 = @splat(0);
            for (runtime.modules.items()) |module| {
                message = @splat(0);
                try S.check(module.api.evaluate.?( &ctx, module.state, runtime.wrenches.ptr, runtime.model.object_count, &message, message.len), std.mem.sliceTo(&message, 0));
            }
            for (runtime.wrenches.items()) |wrench| try S.require(S.finite3(wrench.force) and S.finite3(wrench.torque), "non-finite force output", c.SIM_INVALID);
            message = @splat(0);
            try S.check(runtime.backend.step.?(runtime.world, runtime.wrenches.ptr, runtime.model.object_count, &message, message.len), std.mem.sliceTo(&message, 0));
            runtime.tick += 1;
            try read(runtime);
            for (runtime.bodies.items()) |body| {
                try S.require(S.norm(body.pose.pos) <= runtime.model.settings.bounds.position and S.norm(body.linear_velocity) <= runtime.model.settings.bounds.speed and S.norm(body.angular_velocity) <= runtime.model.settings.bounds.angular_speed, "simulation bound exceeded", c.SIM_INVALID);
            }
            sensors.sample(runtime);
        }
    }.run(r, input, count));
}

export fn simulation_runtime_states(r: ?*const state.Runtime, p: ?*?*const c.SimBodyState, n: ?*u32) callconv(.c) c.SimResult {
    if (r == null or p == null or n == null) return c.SIM_INVALID;
    p.?.* = @ptrCast(r.?.bodies.ptr);
    n.?.* = @intCast(r.?.bodies.n);
    return c.SIM_OK;
}

export fn simulation_runtime_contacts(r: ?*const state.Runtime, p: ?*?*const c.SimContact, n: ?*u32) callconv(.c) c.SimResult {
    if (r == null or p == null or n == null) return c.SIM_INVALID;
    p.?.* = @ptrCast(r.?.contacts.ptr);
    n.?.* = @intCast(r.?.contacts.n);
    return c.SIM_OK;
}

export fn simulation_runtime_rotors(r: ?*const state.Runtime, p: ?*?*const c.RotorState, n: ?*u32) callconv(.c) c.SimResult {
    if (r == null or p == null or n == null) return c.SIM_INVALID;
    p.?.* = @ptrCast(&r.?.rotors);
    n.?.* = r.?.model.robot.thruster_count;
    return c.SIM_OK;
}

export fn simulation_runtime_wrenches(r: ?*const state.Runtime, p: ?*?*const c.SimWrench, n: ?*u32) callconv(.c) c.SimResult {
    if (r == null or p == null or n == null) return c.SIM_INVALID;
    p.?.* = @ptrCast(r.?.wrenches.ptr);
    n.?.* = @intCast(r.?.wrenches.n);
    return c.SIM_OK;
}

export fn simulation_runtime_tick(r: ?*const state.Runtime) callconv(.c) u64 {
    return if (r) |runtime| runtime.tick else 0;
}

export fn simulation_runtime_hash(r: ?*const state.Runtime) callconv(.c) u64 {
    const runtime = r orelse return 0;
    var h = S.Hash{};
    h.scalar(u64, runtime.tick);
    h.scalar(u64, runtime.backend.state_hash.?(runtime.world));
    for (runtime.bodies.constItems()) |body| h.state(body);
    var i: u32 = 0;
    while (i < runtime.model.robot.thruster_count) : (i += 1) {
        const rotor = runtime.rotors[i];
        h.scalar(f64, rotor.omega);
        h.scalar(f64, rotor.integral);
        h.scalar(f64, rotor.torque);
        h.scalar(f64, rotor.thrust);
        h.scalar(f64, runtime.commands[i]);
    }
    for (runtime.fluid.constItems()) |wrench| {
        h.vec(wrench.force);
        h.vec(wrench.torque);
    }
    for (runtime.previous_linear.constItems()) |value| h.vec(value);
    for (runtime.previous_angular.constItems()) |value| h.vec(value);
    sensors.hashSensors(runtime, &h);
    return h.value;
}

export fn simulation_runtime_observe(r: ?*state.Runtime, out: ?*c.Observation, e: ?[*]u8, n: u32) callconv(.c) c.SimResult {
    return S.guard(e, n, struct {
        fn run(runtime: ?*state.Runtime, observation: ?*c.Observation) !void {
            try S.require(runtime != null and observation != null, "null observation arguments", c.SIM_INVALID);
            observation.?.* = runtime.?.observation;
        }
    }.run(r, out));
}

export fn simulation_runtime_sensor_samples(r: ?*const state.Runtime, out: ?*?*const c.SensorSample, count: ?*u32) callconv(.c) c.SimResult {
    if (r == null or out == null or count == null) return c.SIM_INVALID;
    out.?.* = @ptrCast(r.?.samples.ptr);
    count.?.* = @intCast(r.?.samples.n);
    return c.SIM_OK;
}
