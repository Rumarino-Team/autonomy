//! MuJoCo adapter behind sim_backend.h. The model is borrowed; engine and buffers are world-owned.
const std = @import("std");
const c = @import("sim_c");
const mj = @import("mujoco_c");
const S = @import("support.zig");

const BodyBinding = struct { data: c_int = 0, body: ?*const c.ObjectBlock = null };
const ShapeBinding = struct { data: c_int = 0, shape: ?*const c.Collider = null };
fn BindingPool(comptime T: type) type {
    return struct {
        records: S.Buffer(T) = .{},
        pool: c.PoolAllocator = std.mem.zeroes(c.PoolAllocator),
        fn init(self: *@This(), allocator: c.SimAllocator, count: usize) !void {
            try S.require(count <= std.math.maxInt(u32), "binding count", c.SIM_CAPACITY);
            self.records.alloc = allocator;
            try self.records.reserve(count);
            try S.require(c.pool_init(&self.pool, @ptrCast(self.records.ptr), try S.bytes(self.records.cap, @sizeOf(T)), @sizeOf(T), @alignOf(T), @intCast(count)) != 0, "binding pool init", c.SIM_INVALID);
        }
        fn append(self: *@This(), item: T) !void {
            const ptr = c.pool_alloc(&self.pool) orelse return S.boom(c.SIM_CAPACITY, "binding capacity");
            const record: *T = @ptrCast(@alignCast(ptr));
            record.* = item;
            self.records.n = self.pool.count;
        }
        fn at(self: *const @This(), i: usize) c_int {
            return self.records.ptr.?[i].data;
        }
    };
}
const World = struct {
    source: *const c.Simulation,
    allocator: c.SimAllocator,
    model: ?*mj.mjModel = null,
    data: ?*mj.mjData = null,
    bodies: BindingPool(BodyBinding) = .{},
    geoms: BindingPool(ShapeBinding) = .{},
    states: S.Buffer(c.SimBodyState) = .{},
    contacts: S.Buffer(c.SimContact) = .{},
    seen: S.Buffer(u8) = .{},
    hash_state: S.Buffer(mj.mjtNum) = .{},

    fn destroy(w: *World) void {
        if (w.data) |data| mj.mj_deleteData(data);
        if (w.model) |model| mj.mj_deleteModel(model);
        w.hash_state.deinit();
        w.seen.deinit();
        w.contacts.deinit();
        w.states.deinit();
        w.geoms.records.deinit();
        w.bodies.records.deinit();
        const allocator = w.allocator;
        allocator.deallocate.?(allocator.user, w, @sizeOf(World), @alignOf(World));
    }
    fn externalId(w: *const World, engine_id: c_int) u32 {
        for (0..w.source.object_count) |i| if (w.bodies.at(i) == engine_id) return w.source.object_table[i].*.body.id;
        return std.math.maxInt(u32);
    }
    fn objectIndex(w: *const World, id: u32) !usize {
        for (0..w.source.object_count) |i| if (w.source.object_table[i].*.body.id == id) return i;
        return S.boom(c.SIM_INVALID, "unknown object ID");
    }
    fn refresh(w: *World) !void {
        const m = w.model.?;
        const d = w.data.?;
        mj.mj_forward(m, d);
        try w.states.resize(w.source.object_count);
        try w.contacts.resize(0);
        for (w.states.items(), 0..) |*state, i| {
            const id: usize = @intCast(w.bodies.at(i));
            const p = d.xpos + 3 * id;
            const q = d.xquat + 4 * id;
            const com = d.xipos + 3 * id;
            var velocity: [6]f64 = undefined;
            mj.mj_objectVelocity(m, d, mj.mjOBJ_BODY, @intCast(id), &velocity, 0);
            state.* = .{
                .id = w.source.object_table[i].*.body.id,
                .pose = .{ .pos = S.v(p[0], p[1], p[2]), .quat = S.q(q[1], q[2], q[3], q[0]) },
                .center_of_mass = S.v(com[0], com[1], com[2]),
                .linear_velocity = S.v(velocity[3], velocity[4], velocity[5]),
                .angular_velocity = S.v(velocity[0], velocity[1], velocity[2]),
            };
        }
        for (0..@intCast(d.ncon)) |i| {
            const contact = &d.contact[i];
            const a = m.geom_bodyid[@intCast(contact.geom[0])];
            const b = m.geom_bodyid[@intCast(contact.geom[1])];
            if (a == b) continue;
            var force: [6]f64 = @splat(0);
            mj.mj_contactForce(m, d, @intCast(i), &force);
            const index = w.contacts.n;
            try w.contacts.resize(index + 1);
            w.contacts.ptr.?[index] = .{
                .a = w.externalId(a),
                .b = w.externalId(b),
                .position = S.v(contact.pos[0], contact.pos[1], contact.pos[2]),
                .normal = S.v(contact.frame[0], contact.frame[1], contact.frame[2]),
                .normal_force = force[0],
            };
        }
    }
    fn reset(w: *World, input: [*c]const c.SimBodyState, count: u32) !void {
        try S.require((input == null and count == 0) or (input != null and count == w.source.object_count), "reset needs all body states", c.SIM_INVALID);
        // Validate every entry before resetting or mutating engine state.
        for (0..count) |i| {
            const b = &w.source.object_table[i].*.body;
            const state = &input[i];
            try S.require(state.id == b.id and S.finite3(state.linear_velocity) and S.finite3(state.angular_velocity), "invalid reset state", c.SIM_INVALID);
            try S.validPose(state.pose);
            if (b.motion_type == c.SIM_STATIC) {
                for (state.pose.pos.buf, b.initial_pose.pos.buf) |a, z| try S.require(a == z, "static body cannot move on reset", c.SIM_INVALID);
                for (state.pose.quat.buf, b.initial_pose.quat.buf) |a, z| try S.require(a == z, "static body cannot rotate on reset", c.SIM_INVALID);
            }
        }
        const m = w.model.?;
        const d = w.data.?;
        mj.mj_resetData(m, d);
        for (0..w.source.object_count) |i| {
            const b = &w.source.object_table[i].*.body;
            const state: c.SimBodyState = if (input != null) input[i] else .{
                .id = b.id,
                .pose = b.initial_pose,
                .center_of_mass = std.mem.zeroes(c.MathVector3f),
                .linear_velocity = b.initial_velocity,
                .angular_velocity = b.initial_angular_velocity,
            };
            const id: usize = @intCast(w.bodies.at(i));
            if (b.motion_type == c.SIM_DYNAMIC) {
                const joint: usize = @intCast(m.body_jntadr[id]);
                const qa: usize = @intCast(m.jnt_qposadr[joint]);
                const va: usize = @intCast(m.jnt_dofadr[joint]);
                transform(d.qpos + qa, d.qpos + qa + 3, state.pose);
                // Freejoint translation is at the body origin; ABI velocity is at COM.
                const offset = S.rotate(state.pose.quat, b.center_of_mass.pos);
                xyz(d.qvel + va, S.sub(state.linear_velocity, S.cross(state.angular_velocity, offset)));
                xyz(d.qvel + va + 3, S.rotate(S.conjugate(state.pose.quat), state.angular_velocity));
            } else if (b.motion_type == c.SIM_KINEMATIC) {
                const k: usize = @intCast(m.body_mocapid[id]);
                transform(d.mocap_pos + 3 * k, d.mocap_quat + 4 * k, state.pose);
            }
        }
        try w.refresh();
    }
    fn step(w: *World, input: [*c]const c.SimWrench, count: u32) !void {
        try S.require(count <= w.source.object_count and (count == 0 or input != null), "invalid wrench count", c.SIM_INVALID);
        @memset(w.seen.items(), 0);
        // Invalid requests must leave engine state and applied forces untouched.
        for (0..count) |i| {
            const force = &input[i];
            try S.require(S.finite3(force.force) and S.finite3(force.torque), "invalid wrench values", c.SIM_INVALID);
            const index = if (w.source.object_table[i].*.body.id == force.id) i else try w.objectIndex(force.id);
            try S.require(w.seen.ptr.?[index] == 0, "duplicate wrench ID", c.SIM_INVALID);
            w.seen.ptr.?[index] = 1;
        }
        const m = w.model.?;
        const d = w.data.?;
        @memset(d.xfrc_applied[0..@intCast(6 * m.nbody)], 0);
        for (0..count) |i| {
            const force = &input[i];
            const index = if (w.source.object_table[i].*.body.id == force.id) i else try w.objectIndex(force.id);
            const out = d.xfrc_applied + 6 * @as(usize, @intCast(w.bodies.at(index)));
            xyz(out, force.force);
            xyz(out + 3, force.torque);
        }
        mj.mj_step(m, d);
        try w.refresh();
    }
};
fn namePrint(buffer: []u8, comptime format: []const u8, args: anytype) ![:0]u8 {
    return std.fmt.bufPrintSentinel(buffer, format, args, 0);
}
fn xyz(out: [*c]f64, vector: c.MathVector3f) void {
    for (0..3) |i| out[i] = vector.buf[i];
}
fn quaternion(out: [*c]f64, q: c.MathQuaternionf) void {
    // Shared xyzw quaternions and MuJoCo wxyz, both in NED coordinates.
    out[0] = q.buf[3];
    for (0..3) |i| out[i + 1] = q.buf[i];
}
fn transform(pos: [*c]f64, quat: [*c]f64, p: c.MathPose) void {
    xyz(pos, p.pos);
    quaternion(quat, p.quat);
}
fn create(scene: [*c]const c.Simulation, out: [*c]?*c.PhysicsWorld) !void {
    var message: [512]u8 = @splat(0);
    try S.check(c.simulation_validate(scene, &message, message.len), S.span(&message));
    try S.require(scene.*.frozen != 0, "backend requires finalized model", c.SIM_INVALID);
    try S.require(scene.*.settings.native_hydrodynamics == 0, "MuJoCo does not expose native hydrodynamics", c.SIM_INVALID);
    var allocator: c.SimAllocator = undefined;
    try S.check(c.simulation_get_allocator(scene, &allocator), "invalid model allocator");
    const w: *World = @ptrCast(@alignCast(try S.allocate(allocator, @sizeOf(World), @alignOf(World))));
    w.* = .{ .source = @ptrCast(scene), .allocator = allocator };
    errdefer w.destroy();
    w.states.alloc = allocator;
    w.contacts.alloc = allocator;
    w.seen.alloc = allocator;
    w.hash_state.alloc = allocator;
    var geom_count: usize = 0;
    for (0..scene.*.object_count) |i| geom_count = std.math.add(usize, geom_count, scene.*.object_table[i].*.collider_count) catch return S.boom(c.SIM_CAPACITY, "binding count");
    try w.bodies.init(allocator, scene.*.object_count);
    try w.geoms.init(allocator, geom_count);
    const spec = mj.mj_makeSpec();
    try S.require(spec != null, "cannot allocate MuJoCo spec", c.SIM_NO_MEMORY);
    defer mj.mj_deleteSpec(spec);
    spec.*.compiler.degree = 0;
    spec.*.option.timestep = @as(f64, @floatFromInt(scene.*.settings.dt_numerator)) / @as(f64, @floatFromInt(scene.*.settings.dt_denominator));
    xyz(&spec.*.option.gravity, scene.*.settings.gravity);
    spec.*.option.integrator = mj.mjINT_IMPLICITFAST;
    spec.*.option.enableflags &= ~@as(c_int, mj.mjENBL_SLEEP);
    const root = mj.mjs_findBody(spec, "world");
    try S.require(root != null, "missing MuJoCo root body", c.SIM_BACKEND_ERROR);
    var name_buffer: [80]u8 = undefined;
    for (0..scene.*.object_count) |i| {
        const object = scene.*.object_table[i];
        const b = &object.*.body;
        const body = mj.mjs_addBody(root, null);
        try S.require(body != null, "cannot allocate MuJoCo body", c.SIM_NO_MEMORY);
        _ = mj.mjs_setName(body.*.element, (try namePrint(&name_buffer, "b{d}", .{b.id})).ptr);
        transform(&body.*.pos, &body.*.quat, b.initial_pose);
        body.*.mocap = @intFromBool(b.motion_type == c.SIM_KINEMATIC);
        if (b.motion_type == c.SIM_DYNAMIC) {
            try S.require(mj.mjs_addFreeJoint(body) != null, "cannot allocate MuJoCo joint", c.SIM_NO_MEMORY);
            body.*.explicitinertial = 1;
            transform(&body.*.ipos, &body.*.iquat, b.center_of_mass);
            body.*.mass = b.mass;
            // Compile with isotropic inertia, then restore calibrated moments below.
            const placeholder = @max(b.inertia.buf[0], b.inertia.buf[1], b.inertia.buf[2]);
            xyz(&body.*.inertia, S.v(placeholder, placeholder, placeholder));
        }
        for (0..object.*.collider_count) |k| {
            const collider = &object.*.colliders[k];
            const g = &collider.data;
            try S.require(g.restitution == 0, "MuJoCo adapter currently requires zero restitution", c.SIM_INVALID);
            const geom = mj.mjs_addGeom(body, null);
            try S.require(geom != null, "cannot allocate MuJoCo geom", c.SIM_NO_MEMORY);
            _ = mj.mjs_setName(geom.*.element, (try namePrint(&name_buffer, "g{d}_{d}", .{ i, k })).ptr);
            transform(&geom.*.pos, &geom.*.quat, g.pose);
            geom.*.density = 0;
            geom.*.contype = @intCast(g.collisions);
            geom.*.conaffinity = @intCast(g.collisions);
            xyz(&geom.*.friction, S.v(g.friction, 0, 0));
            if (g.kind == c.SIM_MESH) {
                const mesh = mj.mjs_addMesh(spec, null);
                try S.require(mesh != null, "cannot allocate MuJoCo mesh", c.SIM_NO_MEMORY);
                const name = try namePrint(&name_buffer, "m{d}_{d}", .{ i, k });
                _ = mj.mjs_setName(mesh.*.element, name.ptr);
                mj.mjs_setString(geom.*.meshname, name.ptr);
                var positions: S.Buffer(f32) = .{ .alloc = allocator };
                defer positions.deinit();
                var triangles: S.Buffer(c_int) = .{ .alloc = allocator };
                defer triangles.deinit();
                try positions.resize(try S.bytes(collider.geometry.position_count, 3));
                try triangles.resize(try S.bytes(collider.geometry.triangle_count, 3));
                try S.require(positions.n <= std.math.maxInt(c_int) and triangles.n <= std.math.maxInt(c_int), "MuJoCo mesh exceeds index limits", c.SIM_CAPACITY);
                for (0..collider.geometry.position_count) |v| for (0..3) |axis| {
                    positions.ptr.?[3 * v + axis] = collider.geometry.positions[v].buf[axis];
                };
                for (0..collider.geometry.triangle_count) |t| {
                    const f = collider.geometry.triangles[t];
                    for ([_]u32{ f.a, f.b, f.c }, 0..) |index, axis| {
                        try S.require(index <= std.math.maxInt(c_int), "MuJoCo mesh index overflow", c.SIM_CAPACITY);
                        triangles.ptr.?[3 * t + axis] = @intCast(index);
                    }
                }
                // These C setters copy into MuJoCo-owned asset buffers.
                mj.mjs_setFloat(mesh.*.uservert, positions.ptr, @intCast(positions.n));
                mj.mjs_setInt(mesh.*.userface, triangles.ptr, @intCast(triangles.n));
                geom.*.type = mj.mjGEOM_MESH;
            } else {
                geom.*.type = if (g.kind == c.SIM_BOX) mj.mjGEOM_BOX else if (g.kind == c.SIM_SPHERE) mj.mjGEOM_SPHERE else mj.mjGEOM_CYLINDER;
                if (g.kind == c.SIM_BOX) xyz(&geom.*.size, g.size) else if (g.kind == c.SIM_SPHERE) geom.*.size[0] = g.size.buf[0] else xyz(&geom.*.size, S.v(g.size.buf[0], g.size.buf[2], 0));
            }
        }
    }
    w.model = mj.mj_compile(spec, null);
    if (w.model == null) return S.boom(c.SIM_BACKEND_ERROR, std.mem.span(mj.mjs_getError(spec)));
    const m = w.model.?;
    for (0..scene.*.object_count) |i| {
        const object = scene.*.object_table[i];
        const b = &object.*.body;
        const id = mj.mj_name2id(m, mj.mjOBJ_BODY, (try namePrint(&name_buffer, "b{d}", .{b.id})).ptr);
        try S.require(id >= 0, "compiled MuJoCo body missing", c.SIM_BACKEND_ERROR);
        try w.bodies.append(.{ .data = id, .body = object });
        if (b.motion_type == c.SIM_DYNAMIC) xyz(m.body_inertia + 3 * @as(usize, @intCast(id)), b.inertia);
        for (0..object.*.collider_count) |k| {
            const geom_id = mj.mj_name2id(m, mj.mjOBJ_GEOM, (try namePrint(&name_buffer, "g{d}_{d}", .{ i, k })).ptr);
            try S.require(geom_id >= 0, "compiled MuJoCo geom missing", c.SIM_BACKEND_ERROR);
            try w.geoms.append(.{ .data = geom_id, .shape = &object.*.colliders[k] });
        }
    }
    try w.seen.resize(scene.*.object_count);
    try w.states.reserve(scene.*.object_count);
    try w.hash_state.resize(@intCast(mj.mj_stateSize(m, mj.mjSTATE_INTEGRATION)));
    m.opt.integrator = switch (scene.*.settings.integrator) {
        c.SIM_INTEGRATOR_EULER => mj.mjINT_EULER,
        c.SIM_INTEGRATOR_IMPLICIT => mj.mjINT_IMPLICIT,
        c.SIM_INTEGRATOR_RK4 => mj.mjINT_RK4,
        else => mj.mjINT_IMPLICITFAST,
    };
    const constants = mj.mj_makeData(m);
    try S.require(constants != null, "cannot allocate MuJoCo constants", c.SIM_NO_MEMORY);
    mj.mj_setConst(m, constants);
    mj.mj_deleteData(constants);
    w.data = mj.mj_makeData(m);
    try S.require(w.data != null, "cannot allocate MuJoCo world", c.SIM_NO_MEMORY);
    try w.reset(null, 0);
    out.* = @ptrCast(w);
}
fn worldCreate(scene: [*c]const c.Simulation, out: [*c]?*c.PhysicsWorld, err: [*c]u8, n: u32) callconv(.c) c.SimResult {
    if (out == null) return c.SIM_INVALID;
    out.* = null;
    if (scene == null) return c.SIM_INVALID;
    return S.guard(err, n, create(scene, out));
}
fn worldDestroy(p: ?*c.PhysicsWorld) callconv(.c) void {
    const w: *World = @ptrCast(@alignCast(p orelse return));
    w.destroy();
}
fn reset(p: ?*c.PhysicsWorld, states: [*c]const c.SimBodyState, count: u32, err: [*c]u8, n: u32) callconv(.c) c.SimResult {
    const w: *World = @ptrCast(@alignCast(p orelse return c.SIM_INVALID));
    return S.guard(err, n, w.reset(states, count));
}
fn step(p: ?*c.PhysicsWorld, input: [*c]const c.SimWrench, count: u32, err: [*c]u8, n: u32) callconv(.c) c.SimResult {
    const w: *World = @ptrCast(@alignCast(p orelse return c.SIM_INVALID));
    return S.guard(err, n, w.step(input, count));
}
fn copyOut(comptime T: type, buffer: *const S.Buffer(T), out: [*c]T, capacity: u32, count: [*c]u32) c.SimResult {
    if (count == null) return c.SIM_INVALID;
    count.* = @intCast(buffer.n);
    if (capacity < buffer.n) return c.SIM_BUFFER_TOO_SMALL;
    if (buffer.n > 0 and out == null) return c.SIM_INVALID;
    if (buffer.n > 0) @memcpy(out[0..buffer.n], buffer.constItems());
    return c.SIM_OK;
}
fn readState(p: ?*const c.PhysicsWorld, out: [*c]c.SimBodyState, capacity: u32, count: [*c]u32) callconv(.c) c.SimResult {
    const w: *const World = @ptrCast(@alignCast(p orelse return c.SIM_INVALID));
    return copyOut(c.SimBodyState, &w.states, out, capacity, count);
}
fn readContacts(p: ?*const c.PhysicsWorld, out: [*c]c.SimContact, capacity: u32, count: [*c]u32) callconv(.c) c.SimResult {
    const w: *const World = @ptrCast(@alignCast(p orelse return c.SIM_INVALID));
    return copyOut(c.SimContact, &w.contacts, out, capacity, count);
}
fn hash(p: ?*const c.PhysicsWorld) callconv(.c) u64 {
    const w: *const World = @ptrCast(@alignCast(p orelse return 0));
    mj.mj_getState(w.model, w.data, w.hash_state.ptr, mj.mjSTATE_INTEGRATION);
    var h: S.Hash = .{};
    for (w.hash_state.constItems()) |n| h.scalar(f64, n);
    return h.value;
}
pub export fn sim_backend_get_api(version: u32, size: u32, out: [*c]c.PhysicsBackendApi) callconv(.c) c.SimResult {
    if (out == null or version != c.PHYSICS_BACKEND_ABI_VERSION or size != @sizeOf(c.PhysicsBackendApi)) return c.SIM_UNSUPPORTED;
    out.* = std.mem.zeroes(c.PhysicsBackendApi);
    out.*.struct_size = @sizeOf(c.PhysicsBackendApi);
    out.*.abi_version = c.PHYSICS_BACKEND_ABI_VERSION;
    out.*.capabilities = c.SIM_CAP_THREADS;
    out.*.integrators = (@as(u32, 1) << (c.SIM_INTEGRATOR_RK4 + 1)) - 1;
    S.text(&out.*.name, "mujoco") catch return c.SIM_BACKEND_ERROR;
    S.text(&out.*.version, std.mem.span(mj.mj_versionString())) catch return c.SIM_BACKEND_ERROR;
    out.*.world_create = worldCreate;
    out.*.world_destroy = worldDestroy;
    out.*.reset = reset;
    out.*.step = step;
    out.*.read_state = readState;
    out.*.read_contacts = readContacts;
    out.*.state_hash = hash;
    return c.SIM_OK;
}
