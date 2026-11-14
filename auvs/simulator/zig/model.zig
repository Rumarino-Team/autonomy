const std = @import("std");
const c = @import("sim_c");
const S = @import("support.zig");
const arena = @import("arena.zig");

const Storage = struct {
    config: c.SimulationConfig,
    objects: S.Buffer(c.ObjectBlock) = .{},
    faults: S.Buffer(c.FaultBlock) = .{},
    modules: S.Buffer(c.ModuleBlock) = .{},
    vertices: S.Buffer(c.MathVector3f) = .{},
    hulls: S.Buffer(c.MathVector3f) = .{},
    triangles: S.Buffer(c.SimTriangle) = .{},
    colliders: S.Buffer(c.Collider) = .{},
    sensors: S.Buffer(c.SensorRec) = .{},
    object_table: S.Buffer(?*const c.ObjectBlock) = .{},
    fault_table: S.Buffer(?*const c.FaultBlock) = .{},
    module_table: S.Buffer(?*const c.ModuleBlock) = .{},
    geometry: S.Buffer(c.MeshData) = .{},
    hashes: S.Buffer(u64) = .{},

    fn deinit(self: *Storage) void {
        self.hashes.deinit();
        self.geometry.deinit();
        self.module_table.deinit();
        self.fault_table.deinit();
        self.object_table.deinit();
        self.sensors.deinit();
        self.colliders.deinit();
        self.triangles.deinit();
        self.hulls.deinit();
        self.vertices.deinit();
        self.modules.deinit();
        self.faults.deinit();
        self.objects.deinit();
    }
};

pub fn modelConfig(s: *const c.Simulation) c.SimulationConfig {
    return store(s).config;
}

fn store(s: *const c.Simulation) *Storage {
    return @ptrCast(@alignCast(s.storage.?));
}

fn bind(buffer: anytype, alloc: c.SimAllocator) void {
    buffer.alloc = alloc;
}

pub export fn simulation_config_default(out: ?*c.SimulationConfig) callconv(.c) void {
    const config = out orelse return;
    config.* = std.mem.zeroes(c.SimulationConfig);
    config.object_capacity = 32;
    config.fault_capacity = 16;
    config.module_capacity = 16;
    config.collider_capacity = 160;
    config.vertex_capacity = 100;
    config.triangle_capacity = 100;
    config.hull_vertex_capacity = 100;
    config.sensor_capacity = 16;
    config.scratch_bytes = 65536;
}

pub export fn simulation_settings_default(out: ?*c.SimulationSettings) callconv(.c) void {
    const settings = out orelse return;
    settings.* = std.mem.zeroes(c.SimulationSettings);
    settings.dt_numerator = 1;
    settings.dt_denominator = 360;
    settings.integrator = c.SIM_INTEGRATOR_DEFAULT;
    settings.hydro_period = 7;
    settings.gravity = S.v(0, 0, 9.81);
    settings.water_density = 1025;
    settings.bounds = .{ .position = 10000, .speed = 1000, .angular_speed = 1000 };
}

pub export fn underwater_body_default(out: ?*c.UnderwaterBody) callconv(.c) void {
    const body = out orelse return;
    body.* = std.mem.zeroes(c.UnderwaterBody);
    body.inertia = S.v(1, 1, 1);
    body.center_of_mass.quat = S.q(0, 0, 0, 1);
    body.initial_pose.quat = S.q(0, 0, 0, 1);
}

pub export fn collider_default(out: ?*c.Collider) callconv(.c) void {
    const collider = out orelse return;
    collider.* = std.mem.zeroes(c.Collider);
    collider.data.collisions = 1;
    collider.data.pose.quat = S.q(0, 0, 0, 1);
    collider.data.size = S.v(1, 1, 1);
    collider.data.friction = 0.5;
}

pub export fn rotor_spec_default(out: ?*c.RotorSpec) callconv(.c) void {
    const spec = out orelse return;
    spec.* = .{
        .max_rpm = 1000,
        .diameter = 0.18,
        .thrust = 0.48,
        .torque = 0.05,
        .inertia = 0.00146468,
        .kp = 1,
        .ki = 10,
        .command_sign = -1,
    };
}

pub export fn sim_list_append(l: ?*c.ListBase, n: ?*c.IntrusiveDataNode) callconv(.c) c.SimResult {
    const list = l orelse return c.SIM_INVALID;
    const node = n orelse return c.SIM_INVALID;
    if (node.list != null or node.link.next != null or node.link.prev != null) return c.SIM_INVALID;
    if ((list.first != null) != (list.last != null)) return c.SIM_INVALID;
    node.list = list;
    node.link.prev = list.last;
    if (list.last != null) list.last.*.next = &node.link else list.first = &node.link;
    list.last = &node.link;
    return c.SIM_OK;
}

pub export fn sim_list_unlink(l: ?*c.ListBase, n: ?*c.IntrusiveDataNode) callconv(.c) c.SimResult {
    const list = l orelse return c.SIM_INVALID;
    const node = n orelse return c.SIM_INVALID;
    if (node.list != list) return c.SIM_INVALID;
    if (node.link.prev != null) node.link.prev.*.next = node.link.next else list.first = node.link.next;
    if (node.link.next != null) node.link.next.*.prev = node.link.prev else list.last = node.link.prev;
    node.link = std.mem.zeroes(c.Link);
    node.list = null;
    return c.SIM_OK;
}

pub export fn sim_node_cast(
    n: ?*const c.IntrusiveDataNode,
    k: c.DataKind,
    o: ?*const anyopaque,
    out: ?*?*const anyopaque,
) callconv(.c) c.SimResult {
    if (out) |slot| slot.* = null;
    if (out == null or n == null or n.?.kind != k or n.?.owner != o) return c.SIM_INVALID;
    out.?.* = n;
    return c.SIM_OK;
}

fn reserveOne(buffer: anytype, count: usize, total: *usize) !void {
    const Child = @typeInfo(@typeInfo(@TypeOf(buffer.ptr)).optional.child).pointer.child;
    const size = try S.bytes(count, @sizeOf(Child));
    try S.require(size <= S.max_scene_bytes - total.*, "model capacity exceeds limit", c.SIM_CAPACITY);
    total.* += size;
    try buffer.reserve(count);
}

pub export fn simulation_init(
    s: ?*c.Simulation,
    input: ?*const c.SimulationConfig,
    e: ?[*]u8,
    n: u32,
) callconv(.c) c.SimResult {
    const root = s orelse return S.guard(e, n, struct {
        fn run() !void {
            return S.boom(c.SIM_INVALID, "root must be zero initialized");
        }
    }.run());
    return S.guard(e, n, struct {
        fn run(sim: *c.Simulation, config_in: ?*const c.SimulationConfig) !void {
            try S.require(sim.initialized == 0, "root must be zero initialized", c.SIM_INVALID);
            var config = std.mem.zeroes(c.SimulationConfig);
            simulation_config_default(&config);
            if (config_in) |value| config = value.*;
            config.allocator = try S.resolveAllocator(config.allocator);
            try S.require(
                config.object_capacity != 0 and config.object_capacity <= 4096 and
                    config.collider_capacity <= 4096 * 1024 and config.module_capacity <= 4096 and
                    config.fault_capacity <= 1000000 and config.sensor_capacity <= 64 and
                    config.scratch_bytes <= S.max_scene_bytes,
                "invalid model capacities",
                c.SIM_INVALID,
            );
            sim.* = std.mem.zeroes(c.Simulation);
            sim.struct_size = @sizeOf(c.Simulation);
            sim.abi_version = c.SIM_MODEL_ABI_VERSION;
            simulation_settings_default(&sim.settings);
            const raw = config.allocator.allocate.?(config.allocator.user, @sizeOf(Storage), @alignOf(Storage)) orelse return error.OutOfMemory;
            if (@intFromPtr(raw) % @alignOf(Storage) != 0) {
                config.allocator.deallocate.?(config.allocator.user, raw, @sizeOf(Storage), @alignOf(Storage));
                return S.boom(c.SIM_INVALID, "misaligned storage");
            }
            const storage: *Storage = @ptrCast(@alignCast(raw));
            storage.* = .{ .config = config };
            inline for (.{
                &storage.objects,   &storage.faults,      &storage.modules,     &storage.vertices,
                &storage.hulls,     &storage.triangles,   &storage.colliders,   &storage.sensors,
                &storage.object_table, &storage.fault_table, &storage.module_table, &storage.geometry,
                &storage.hashes,
            }) |buffer| bind(buffer, config.allocator);
            sim.storage = @ptrCast(storage);
            sim.initialized = S.initialized;
            errdefer simulation_destroy(sim);
            var total: usize = 0;
            try reserveOne(&storage.objects, config.object_capacity, &total);
            try reserveOne(&storage.faults, config.fault_capacity, &total);
            try reserveOne(&storage.modules, config.module_capacity, &total);
            try reserveOne(&storage.vertices, config.vertex_capacity, &total);
            try reserveOne(&storage.triangles, config.triangle_capacity, &total);
            try reserveOne(&storage.hulls, config.hull_vertex_capacity, &total);
            try reserveOne(&storage.colliders, config.collider_capacity, &total);
            try reserveOne(&storage.sensors, config.sensor_capacity, &total);
            try reserveOne(&storage.object_table, config.object_capacity, &total);
            try reserveOne(&storage.fault_table, config.fault_capacity, &total);
            try reserveOne(&storage.module_table, config.module_capacity, &total);
            try reserveOne(&storage.geometry, config.collider_capacity, &total);
            try reserveOne(&storage.hashes, config.collider_capacity, &total);
            try S.check(if (arena.pool_init(&sim.object_pool, storage.objects.ptr, storage.objects.cap * @sizeOf(c.ObjectBlock), @sizeOf(c.ObjectBlock), @alignOf(c.ObjectBlock), config.object_capacity) != 0) c.SIM_OK else c.SIM_INVALID, "object pool init");
            try S.check(if (arena.pool_init(&sim.fault_pool, storage.faults.ptr, storage.faults.cap * @sizeOf(c.FaultBlock), @sizeOf(c.FaultBlock), @alignOf(c.FaultBlock), config.fault_capacity) != 0) c.SIM_OK else c.SIM_INVALID, "fault pool init");
            try S.check(if (arena.pool_init(&sim.module_pool, storage.modules.ptr, storage.modules.cap * @sizeOf(c.ModuleBlock), @sizeOf(c.ModuleBlock), @alignOf(c.ModuleBlock), config.module_capacity) != 0) c.SIM_OK else c.SIM_INVALID, "module pool init");
            arena.arena_init(&sim.vertices, storage.vertices.ptr, storage.vertices.cap * @sizeOf(c.MathVector3f));
            arena.arena_init(&sim.triangles, storage.triangles.ptr, storage.triangles.cap * @sizeOf(c.SimTriangle));
            arena.arena_init(&sim.hull_vertices, storage.hulls.ptr, storage.hulls.cap * @sizeOf(c.MathVector3f));
            sim.object_table = @ptrCast(storage.object_table.ptr);
            sim.fault_table = @ptrCast(storage.fault_table.ptr);
            sim.module_table = @ptrCast(storage.module_table.ptr);
            sim.geometry = @ptrCast(storage.geometry.ptr);
            sim.sensors = @ptrCast(storage.sensors.ptr);
        }
    }.run(root, input));
}

pub export fn simulation_destroy(s: ?*c.Simulation) callconv(.c) void {
    const sim = s orelse return;
    if (sim.storage) |raw| {
        const storage: *Storage = @ptrCast(@alignCast(raw));
        const alloc = storage.config.allocator;
        storage.deinit();
        alloc.deallocate.?(alloc.user, raw, @sizeOf(Storage), @alignOf(Storage));
    }
    sim.* = std.mem.zeroes(c.Simulation);
}

pub export fn simulation_get_allocator(s: ?*const c.Simulation, out: ?*c.SimAllocator) callconv(.c) c.SimResult {
    if (s == null or out == null or s.?.initialized != S.initialized or s.?.storage == null) return c.SIM_INVALID;
    out.?.* = store(s.?).config.allocator;
    return c.SIM_OK;
}

fn draft(s: ?*c.Simulation) error{Failed}!*c.Simulation {
    const sim = s orelse return S.boom(c.SIM_INVALID, "uninitialized simulation");
    try S.require(sim.initialized == S.initialized and sim.storage != null, "uninitialized simulation", c.SIM_INVALID);
    try S.require(sim.frozen == 0, "simulation is frozen", c.SIM_INVALID);
    return sim;
}

fn linkNode(
    n: *c.IntrusiveDataNode,
    kind: u32,
    uid: u32,
    s: *c.Simulation,
    list: *c.ListBase,
    index: u32,
    pooled: bool,
) !void {
    n.* = std.mem.zeroes(c.IntrusiveDataNode);
    n.kind = kind;
    n.uid = uid;
    n.owner = s;
    n.index = index;
    n.pooled = if (pooled) 1 else 0;
    try S.check(sim_list_append(list, n), "invalid membership");
}

fn geometryHash(m: c.MeshData) u64 {
    var h = S.Hash{};
    h.scalar(u32, m.position_count);
    h.scalar(u32, m.triangle_count);
    var i: u32 = 0;
    while (i < m.position_count) : (i += 1) h.vec(m.positions[i]);
    i = 0;
    while (i < m.triangle_count) : (i += 1) {
        const t = m.triangles[i];
        h.scalar(u32, t.a);
        h.scalar(u32, t.b);
        h.scalar(u32, t.c);
    }
    return h.value;
}

fn meshEqual(a: c.MeshData, b: c.MeshData) bool {
    if (a.position_count != b.position_count or a.triangle_count != b.triangle_count) return false;
    var i: u32 = 0;
    while (i < a.position_count) : (i += 1) {
        var k: usize = 0;
        while (k < 3) : (k += 1) if (a.positions[i].buf[k] != b.positions[i].buf[k]) return false;
    }
    i = 0;
    while (i < a.triangle_count) : (i += 1) {
        const x = a.triangles[i];
        const y = b.triangles[i];
        if (x.a != y.a or x.b != y.b or x.c != y.c) return false;
    }
    return true;
}

fn intern(s: *c.Simulation, m: c.MeshData) !c.MeshData {
    const storage = store(s);
    const hash = geometryHash(m);
    var i: u32 = 0;
    while (i < s.geometry_count) : (i += 1) {
        if (storage.hashes.ptr.?[i] == hash and meshEqual(storage.geometry.ptr.?[i], m)) return storage.geometry.ptr.?[i];
    }
    try S.require(s.geometry_count < storage.geometry.cap, "geometry table exhausted", c.SIM_CAPACITY);
    const positions = arena.arena_alloc(&s.vertices, try S.bytes(m.position_count, @sizeOf(c.MathVector3f)), @alignOf(c.MathVector3f));
    const triangles = arena.arena_alloc(&s.triangles, try S.bytes(m.triangle_count, @sizeOf(c.SimTriangle)), @alignOf(c.SimTriangle));
    try S.require(positions != null and triangles != null, "geometry arena exhausted", c.SIM_CAPACITY);
    const p: [*]c.MathVector3f = @ptrCast(@alignCast(positions.?));
    const t: [*]c.SimTriangle = @ptrCast(@alignCast(triangles.?));
    i = 0;
    while (i < m.position_count) : (i += 1) {
        const a = m.positions[i];
        p[i] = S.v(S.canonical(a.buf[0]), S.canonical(a.buf[1]), S.canonical(a.buf[2]));
    }
    @memcpy(t[0..m.triangle_count], m.triangles[0..m.triangle_count]);
    const result = c.MeshData{ .positions = p, .position_count = m.position_count, .triangles = t, .triangle_count = m.triangle_count };
    storage.geometry.ptr.?[s.geometry_count] = result;
    storage.hashes.ptr.?[s.geometry_count] = hash;
    s.geometry_count += 1;
    return result;
}

fn shapeValid(collider: c.Collider) !void {
    const g = collider.data;
    try S.require(g.kind <= c.SIM_MESH and g.collisions <= 1, "invalid collider kind/flags", c.SIM_INVALID);
    try S.validPose(g.pose);
    try S.require(S.finite3(g.size) and S.finite3(g.drag) and S.finite3(g.skin) and std.math.isFinite(g.friction) and g.friction >= 0 and std.math.isFinite(g.restitution) and g.restitution >= 0 and g.restitution <= 1, "invalid collider values", c.SIM_INVALID);
    if (g.kind == c.SIM_MESH) {
        const m = collider.geometry;
        try S.require(m.positions != null and m.triangles != null and m.position_count >= 4 and m.position_count <= 1000000 and m.triangle_count != 0 and m.triangle_count <= 2000000, "invalid mesh arrays", c.SIM_INVALID);
        var i: u32 = 0;
        while (i < m.position_count) : (i += 1) try S.require(S.finite3(m.positions[i]), "invalid mesh position", c.SIM_INVALID);
        i = 0;
        while (i < m.triangle_count) : (i += 1) {
            const t = m.triangles[i];
            try S.require(t.a < m.position_count and t.b < m.position_count and t.c < m.position_count, "invalid triangle index", c.SIM_INVALID);
        }
    } else {
        try S.require(collider.geometry.position_count == 0 and collider.geometry.triangle_count == 0 and collider.geometry.positions == null and collider.geometry.triangles == null, "primitive has mesh data", c.SIM_INVALID);
        try S.require(g.size.buf[0] > 0 and (g.kind != c.SIM_BOX or (g.size.buf[1] > 0 and g.size.buf[2] > 0)) and (g.kind != c.SIM_CYLINDER or g.size.buf[2] > 0), "invalid primitive size", c.SIM_INVALID);
    }
}

fn bodyValid(b: c.UnderwaterBody) !void {
    try S.require(b.id != std.math.maxInt(u32) and b.motion_type <= c.SIM_KINEMATIC and std.mem.indexOfScalar(u8, &b.name, 0) != null, "invalid body ID/type/name", c.SIM_INVALID);
    try S.validPose(b.initial_pose);
    try S.validPose(b.center_of_mass);
    try S.require(S.finite3(b.initial_velocity) and S.finite3(b.initial_angular_velocity) and S.finite3(b.inertia) and S.finite3(b.buoyancy_center) and std.math.isFinite(b.mass) and b.mass >= 0 and std.math.isFinite(b.dry_mass) and b.dry_mass >= 0 and std.math.isFinite(b.volume) and b.volume >= 0, "invalid body values", c.SIM_INVALID);
    if (b.motion_type == c.SIM_DYNAMIC)
        try S.require(b.mass > 0 and b.inertia.buf[0] > 0 and b.inertia.buf[1] > 0 and b.inertia.buf[2] > 0, "dynamic mass/inertia must be positive", c.SIM_INVALID);
}

pub fn objectMember(s: *const c.Simulation, p: ?*const c.ObjectBlock) bool {
    const object = p orelse return false;
    var i: u32 = 0;
    while (i < s.object_count) : (i += 1) if (s.object_table[i] == object) return true;
    return false;
}

fn sensorMember(s: *const c.Simulation, p: ?*const c.SensorRec) bool {
    const sensor = p orelse return false;
    var i: u32 = 0;
    while (i < s.sensor_count) : (i += 1) if (sensor == s.sensors + i) return true;
    return false;
}

fn sensorValid(s: *const c.Simulation, sensor: c.SensorRec) !void {
    try S.require(objectMember(s, sensor.object), "foreign sensor owner", c.SIM_INVALID);
    try S.require(sensor.kind <= c.SIM_SENSOR_CAMERA and sensor.enabled <= 1 and sensor.period_ticks != 0, "invalid sensor kind, enabled flag, or sampling period", c.SIM_INVALID);
    try S.validPose(sensor.pose);
    if (sensor.kind == c.SIM_SENSOR_POSE)
        try S.require(sensor.settings.tracking.position_axes <= 7 and sensor.settings.tracking.attitude_axes <= 7, "invalid tracking axis mask", c.SIM_INVALID);
    if (sensor.kind == c.SIM_SENSOR_CAMERA) {
        const camera = sensor.settings.camera;
        try S.require(camera.width != 0 and camera.width <= 65536 and camera.height != 0 and camera.height <= 65536 and std.math.isFinite(camera.horizontal_fov) and camera.horizontal_fov > 0 and camera.horizontal_fov < 180 and std.math.isFinite(camera.near_range) and std.math.isFinite(camera.far_range) and camera.near_range > 0 and camera.far_range > camera.near_range, "invalid camera settings", c.SIM_INVALID);
    }
}

fn routingValid(s: *const c.Simulation, r: c.SensorRouting) !void {
    const refs = [_]? *const c.SensorRec{ r.tracking, r.imu, r.pressure, r.camera };
    for (refs, 0..) |sensor, i| if (sensor) |item| {
        try S.require(sensorMember(s, item) and item.kind == i, "invalid sensor routing", c.SIM_INVALID);
    };
    var owner: ?*const c.ObjectBlock = null;
    for (refs) |sensor| if (sensor) |item| {
        try S.require(owner == null or owner == item.object, "routed sensors must share one body", c.SIM_INVALID);
        owner = item.object;
    };
}

fn faultKind(kind: u32) u32 {
    if (kind == c.SIM_FAULT_TRACKING) return c.SIM_SENSOR_POSE;
    if (kind == c.SIM_FAULT_DEPTH) return c.SIM_SENSOR_PRESSURE;
    if (kind == c.SIM_FAULT_CAMERA) return c.SIM_SENSOR_CAMERA;
    return c.SIM_SENSOR_IMU;
}

fn faultValid(s: *const c.Simulation, f: c.FaultBlock) !void {
    try S.require(f.kind <= c.SIM_FAULT_CAMERA and f.start < f.end and std.math.isFinite(f.magnitude), "invalid fault", c.SIM_INVALID);
    if (f.kind == c.SIM_FAULT_DEAD or f.kind == c.SIM_FAULT_WEAK) {
        try S.require(f.sensor == null and f.thruster < s.robot.thruster_count and f.magnitude >= 0 and f.magnitude <= 1, "invalid actuator fault", c.SIM_INVALID);
    } else {
        try S.require(sensorMember(s, f.sensor), "foreign sensor fault target", c.SIM_INVALID);
        try S.require(f.sensor.*.kind == faultKind(f.kind), "sensor fault kind mismatch", c.SIM_INVALID);
    }
}

fn written(buf: []u8) []const u8 {
    return std.mem.sliceTo(buf, 0);
}

pub export fn simulation_add_object(
    s: ?*c.Simulation,
    b: ?*const c.UnderwaterBody,
    colliders: ?[*]const c.Collider,
    count: u32,
    fixed: u32,
    out: ?*?*c.ObjectBlock,
    e: ?[*]u8,
    n: u32,
) callconv(.c) c.SimResult {
    if (out) |slot| slot.* = null;
    return S.guard(e, n, struct {
        fn run(
            sim_in: ?*c.Simulation,
            body: ?*const c.UnderwaterBody,
            shapes: ?[*]const c.Collider,
            nshapes: u32,
            embedded: u32,
            slot: ?*?*c.ObjectBlock,
        ) !void {
            const sim = try draft(sim_in);
            try S.require(body != null and shapes != null and slot != null and nshapes != 0 and nshapes <= 1024 and embedded <= 1, "invalid object arguments", c.SIM_INVALID);
            try bodyValid(body.?.*);
            var i: u32 = 0;
            while (i < sim.object_count) : (i += 1)
                try S.require(sim.object_table[i].*.body.id != body.?.id, "duplicate body ID", c.SIM_INVALID);
            const storage = store(sim);
            try S.require(sim.object_count < storage.config.object_capacity, "object table exhausted", c.SIM_CAPACITY);
            try S.require(embedded == 0 or (sim.robot.registered == 0 and nshapes <= 5), "invalid fixed robot", c.SIM_INVALID);
            try S.require(embedded != 0 or nshapes <= storage.colliders.cap - storage.colliders.n, "collider arena exhausted", c.SIM_CAPACITY);
            i = 0;
            while (i < nshapes) : (i += 1) {
                try S.require(shapes.?[i].object == null or shapes.?[i].object.*.id.owner == @as(*const anyopaque, @ptrCast(sim)), "foreign collider owner", c.SIM_INVALID);
                try shapeValid(shapes.?[i]);
            }
            const vertices = sim.vertices.offset;
            const triangles = sim.triangles.offset;
            const geometry = sim.geometry_count;
            var meshes: [1024]c.MeshData = std.mem.zeroes([1024]c.MeshData);
            {
                errdefer {
                    sim.vertices.offset = vertices;
                    sim.triangles.offset = triangles;
                    sim.geometry_count = geometry;
                }
                i = 0;
                while (i < nshapes) : (i += 1) if (shapes.?[i].data.kind == c.SIM_MESH) {
                    meshes[i] = try intern(sim, shapes.?[i].geometry);
                };
            }
            const object: *c.ObjectBlock = if (embedded != 0)
                &sim.robot.object
            else
                @ptrCast(@alignCast(arena.pool_alloc(&sim.object_pool) orelse return S.boom(c.SIM_CAPACITY, "object pool exhausted")));
            const dest: [*]c.Collider = if (embedded != 0)
                &sim.robot.colliders
            else
                storage.colliders.ptr.? + storage.colliders.n;
            if (embedded == 0) storage.colliders.n += nshapes;
            object.* = std.mem.zeroes(c.ObjectBlock);
            object.body = body.?.*;
            object.colliders = dest;
            object.collider_count = nshapes;
            i = 0;
            while (i < nshapes) : (i += 1) dest[i] = .{ .data = shapes.?[i].data, .geometry = meshes[i], .object = object };
            try linkNode(&object.id, c.SIM_DATA_OBJECT, body.?.id, sim, &sim.objects, sim.object_count, embedded == 0);
            storage.object_table.ptr.?[sim.object_count] = object;
            sim.object_count += 1;
            if (embedded != 0) sim.robot.registered = 1;
            slot.?.* = object;
        }
    }.run(s, b, colliders, count, fixed, out));
}

pub export fn simulation_add_thruster(s: ?*c.Simulation, t: ?*const c.Thruster, e: ?[*]u8, n: u32) callconv(.c) c.SimResult {
    return S.guard(e, n, struct {
        fn run(sim_in: ?*c.Simulation, thruster: ?*const c.Thruster) !void {
            const sim = try draft(sim_in);
            try S.require(thruster != null and objectMember(sim, thruster.?.object), "foreign thruster object", c.SIM_INVALID);
            try S.require(sim.robot.thruster_count < 8, "thruster capacity", c.SIM_CAPACITY);
            try S.require(S.finite3(thruster.?.position) and S.finite3(thruster.?.direction) and @abs(S.norm(thruster.?.direction) - 1) < 1e-4, "invalid thruster direction", c.SIM_INVALID);
            const spec = thruster.?.spec;
            try S.require(std.math.isFinite(spec.max_rpm) and spec.max_rpm > 0 and std.math.isFinite(spec.diameter) and spec.diameter > 0 and std.math.isFinite(spec.inertia) and spec.inertia > 0 and std.math.isFinite(spec.thrust) and std.math.isFinite(spec.torque) and std.math.isFinite(spec.kp) and spec.kp >= 0 and std.math.isFinite(spec.ki) and spec.ki >= 0 and (spec.command_sign == 1 or spec.command_sign == -1), "invalid rotor specification", c.SIM_INVALID);
            sim.robot.thrusters[sim.robot.thruster_count] = thruster.?.*;
            sim.robot.thruster_count += 1;
        }
    }.run(s, t));
}

pub export fn simulation_add_hull(s: ?*c.Simulation, h: ?*const c.HullRec, e: ?[*]u8, n: u32) callconv(.c) c.SimResult {
    return S.guard(e, n, struct {
        fn run(sim_in: ?*c.Simulation, hull: ?*const c.HullRec) !void {
            const sim = try draft(sim_in);
            try S.require(hull != null and objectMember(sim, hull.?.object), "foreign hull object", c.SIM_INVALID);
            try S.require(sim.robot.hull_count < 5, "hull capacity", c.SIM_CAPACITY);
            try S.require(hull.?.positions != null and hull.?.position_count != 0 and hull.?.position_count % 3 == 0, "invalid hull triangle soup", c.SIM_INVALID);
            try S.validPose(hull.?.pose);
            try S.validPose(.{ .pos = S.v(0, 0, 0), .quat = hull.?.coefficient_frame });
            try S.require(S.finite3(hull.?.drag) and S.finite3(hull.?.skin) and S.finite3(hull.?.centroid) and std.math.isFinite(hull.?.volume) and hull.?.volume >= 0, "invalid hull values", c.SIM_INVALID);
            var i: u32 = 0;
            while (i < hull.?.position_count) : (i += 1) try S.require(S.finite3(hull.?.positions[i]), "invalid hull vertex", c.SIM_INVALID);
            const p = arena.arena_alloc(&sim.hull_vertices, try S.bytes(hull.?.position_count, @sizeOf(c.MathVector3f)), @alignOf(c.MathVector3f));
            try S.require(p != null, "hull arena exhausted", c.SIM_CAPACITY);
            const dest: [*]c.MathVector3f = @ptrCast(@alignCast(p.?));
            @memcpy(dest[0..hull.?.position_count], hull.?.positions[0..hull.?.position_count]);
            sim.robot.hulls[sim.robot.hull_count] = hull.?.*;
            sim.robot.hulls[sim.robot.hull_count].positions = dest;
            sim.robot.hull_count += 1;
        }
    }.run(s, h));
}

pub extern fn simulation_robot_config(robot: ?*const c.Robot, out: ?*c.SimulationConfig, e: ?[*]u8, n: u32) callconv(.c) c.SimResult;

pub export fn simulation_add_robot(
    s: ?*c.Simulation,
    definition: ?*const c.Robot,
    body_id: u32,
    pose: ?*const c.MathPose,
    e: ?[*]u8,
    n: u32,
) callconv(.c) c.SimResult {
    return S.guard(e, n, struct {
        fn run(sim_in: ?*c.Simulation, robot: ?*const c.Robot, id: u32, start: ?*const c.MathPose) !void {
            const sim = try draft(sim_in);
            var needed = std.mem.zeroes(c.SimulationConfig);
            var message: [256]u8 = @splat(0);
            try S.check(simulation_robot_config(robot, &needed, &message, message.len), written(&message));
            try S.require(sim.robot.registered == 0 and sim.robot.thruster_count == 0 and sim.robot.hull_count == 0, "robot hardware already configured", c.SIM_INVALID);
            const r = robot.?;
            try S.require(r.sensor_count <= store(sim).sensors.cap - sim.sensor_count, "sensor capacity", c.SIM_CAPACITY);
            var hull_bytes: usize = 0;
            var i: u32 = 0;
            while (i < r.hull_count) : (i += 1) hull_bytes += try S.bytes(r.hulls[i].position_count, @sizeOf(c.MathVector3f));
            try S.require(hull_bytes <= sim.hull_vertices.capacity - sim.hull_vertices.offset, "robot hull arena exhausted", c.SIM_CAPACITY);
            var body = r.object.body;
            body.id = id;
            if (start) |value| body.initial_pose = value.*;
            const previous = sim.robot;
            const vertices = sim.vertices.offset;
            const triangles = sim.triangles.offset;
            const hulls = sim.hull_vertices.offset;
            const geometry = sim.geometry_count;
            const objects = sim.object_count;
            const sensor_count = sim.sensor_count;
            const previous_routing = sim.routing;
            errdefer {
                if (sim.object_count != objects) {
                    _ = sim_list_unlink(&sim.objects, &sim.robot.object.id);
                    store(sim).object_table.ptr.?[objects] = null;
                }
                sim.robot = previous;
                sim.sensor_count = sensor_count;
                sim.routing = previous_routing;
                sim.object_count = objects;
                sim.geometry_count = geometry;
                sim.vertices.offset = vertices;
                sim.triangles.offset = triangles;
                sim.hull_vertices.offset = hulls;
            }
            var object: ?*c.ObjectBlock = null;
            try S.check(simulation_add_object(sim, &body, r.colliders[0..].ptr, r.object.collider_count, 1, &object, &message, message.len), written(&message));
            i = 0;
            while (i < r.thruster_count) : (i += 1) {
                var thruster = r.thrusters[i];
                thruster.object = object;
                try S.check(simulation_add_thruster(sim, &thruster, &message, message.len), written(&message));
            }
            i = 0;
            while (i < r.hull_count) : (i += 1) {
                var hull = r.hulls[i];
                hull.object = object;
                try S.check(simulation_add_hull(sim, &hull, &message, message.len), written(&message));
            }
            i = 0;
            while (i < r.sensor_count) : (i += 1) {
                var sensor = r.sensors[i];
                sensor.object = object;
                var inserted: ?*const c.SensorRec = null;
                try S.check(simulation_add_sensor(sim, &sensor, &inserted, &message, message.len), written(&message));
            }
            sim.robot.sensors = sim.sensors + sensor_count;
            sim.robot.sensor_count = r.sensor_count;
            var routing = std.mem.zeroes(c.SensorRouting);
            i = 0;
            while (i < r.sensor_count) : (i += 1) {
                const sensor = &sim.robot.sensors[i];
                switch (sensor.kind) {
                    c.SIM_SENSOR_POSE => routing.tracking = sensor,
                    c.SIM_SENSOR_IMU => routing.imu = sensor,
                    c.SIM_SENSOR_PRESSURE => routing.pressure = sensor,
                    c.SIM_SENSOR_CAMERA => routing.camera = sensor,
                    else => {},
                }
            }
            try S.check(simulation_set_sensor_routing(sim, &routing, &message, message.len), written(&message));
            sim.robot.name = r.name;
            sim.robot.asset_file = r.asset_file;
            sim.robot.asset_hash = r.asset_hash;
        }
    }.run(s, definition, body_id, pose));
}

pub export fn sensor_default(sensor: ?*c.SensorRec, kind: c.SensorKind) callconv(.c) void {
    const out = sensor orelse return;
    out.* = std.mem.zeroes(c.SensorRec);
    out.kind = kind;
    out.pose.quat = S.q(0, 0, 0, 1);
    out.enabled = 1;
    out.period_ticks = if (kind == c.SIM_SENSOR_IMU) 1 else if (kind == c.SIM_SENSOR_CAMERA) 12 else 6;
    if (kind == c.SIM_SENSOR_POSE) out.settings.tracking = .{ .position_axes = 7, .attitude_axes = 7 };
    if (kind == c.SIM_SENSOR_CAMERA) out.settings.camera = .{ .width = 800, .height = 600, .horizontal_fov = 60, .near_range = 0.05, .far_range = 50 };
}

pub export fn simulation_add_sensor(
    s: ?*c.Simulation,
    sensor: ?*const c.SensorRec,
    out: ?*?*const c.SensorRec,
    e: ?[*]u8,
    n: u32,
) callconv(.c) c.SimResult {
    if (out) |slot| slot.* = null;
    return S.guard(e, n, struct {
        fn run(sim_in: ?*c.Simulation, value: ?*const c.SensorRec, slot: ?*?*const c.SensorRec) !void {
            const sim = try draft(sim_in);
            try S.require(value != null and slot != null, "invalid sensor arguments", c.SIM_INVALID);
            try sensorValid(sim, value.?.*);
            var i: u32 = 0;
            while (i < sim.sensor_count) : (i += 1) try S.require(sim.sensors[i].id != value.?.id, "duplicate sensor ID", c.SIM_INVALID);
            try S.require(sim.sensor_count < store(sim).sensors.cap, "sensor capacity", c.SIM_CAPACITY);
            const dest = &store(sim).sensors.ptr.?[sim.sensor_count];
            sim.sensor_count += 1;
            dest.* = value.?.*;
            slot.?.* = dest;
        }
    }.run(s, sensor, out));
}

pub export fn simulation_set_sensor_routing(s: ?*c.Simulation, routing: ?*const c.SensorRouting, e: ?[*]u8, n: u32) callconv(.c) c.SimResult {
    return S.guard(e, n, struct {
        fn run(sim_in: ?*c.Simulation, value: ?*const c.SensorRouting) !void {
            const sim = try draft(sim_in);
            try S.require(value != null, "null routing", c.SIM_INVALID);
            try routingValid(sim, value.?.*);
            sim.routing = value.?.*;
        }
    }.run(s, routing));
}

pub export fn simulation_add_fault(s: ?*c.Simulation, f: ?*const c.FaultBlock, e: ?[*]u8, n: u32) callconv(.c) c.SimResult {
    return S.guard(e, n, struct {
        fn run(sim_in: ?*c.Simulation, fault: ?*const c.FaultBlock) !void {
            const sim = try draft(sim_in);
            try S.require(fault != null, "null fault", c.SIM_INVALID);
            try faultValid(sim, fault.?.*);
            const p: *c.FaultBlock = @ptrCast(@alignCast(arena.pool_alloc(&sim.fault_pool) orelse return S.boom(c.SIM_CAPACITY, "fault capacity")));
            p.kind = fault.?.kind;
            p.thruster = fault.?.thruster;
            p.sensor = fault.?.sensor;
            p.start = fault.?.start;
            p.end = fault.?.end;
            p.magnitude = fault.?.magnitude;
            try linkNode(&p.id, c.SIM_DATA_FAULT, sim.fault_count, sim, &sim.faults, sim.fault_count, true);
            store(sim).fault_table.ptr.?[sim.fault_count] = p;
            sim.fault_count += 1;
        }
    }.run(s, f));
}

pub export fn simulation_add_module(s: ?*c.Simulation, name: ?[*:0]const u8, e: ?[*]u8, n: u32) callconv(.c) c.SimResult {
    return S.guard(e, n, struct {
        fn run(sim_in: ?*c.Simulation, module_name: ?[*:0]const u8) !void {
            const sim = try draft(sim_in);
            const text = module_name orelse return S.boom(c.SIM_INVALID, "invalid module name");
            const bytes = std.mem.span(text);
            try S.require(bytes.len != 0 and bytes.len < 64, "invalid module name", c.SIM_INVALID);
            var i: u32 = 0;
            while (i < sim.module_count) : (i += 1)
                try S.require(!std.mem.eql(u8, bytes, std.mem.sliceTo(&sim.module_table[i].*.name, 0)), "duplicate module", c.SIM_INVALID);
            const p: *c.ModuleBlock = @ptrCast(@alignCast(arena.pool_alloc(&sim.module_pool) orelse return S.boom(c.SIM_CAPACITY, "module capacity")));
            try S.text(&p.name, bytes);
            try linkNode(&p.id, c.SIM_DATA_MODULE, sim.module_count, sim, &sim.modules, sim.module_count, true);
            store(sim).module_table.ptr.?[sim.module_count] = p;
            sim.module_count += 1;
        }
    }.run(s, name));
}

fn listValid(s: *const c.Simulation, list: *const c.ListBase, table: anytype, count: u32, kind: u32) !void {
    var p = list.first;
    var prev: ?*c.Link = null;
    var i: u32 = 0;
    while (i < count) : (i += 1) {
        const item = table[i];
        try S.require(p != null and item != null and p == &item.*.id.link, "list/table mismatch", c.SIM_INVALID);
        const id = item.*.id;
        try S.require(id.owner == @as(*const anyopaque, @ptrCast(s)) and id.kind == kind and id.list == @as(*c.ListBase, @ptrCast(@constCast(list))) and id.index == i and p.*.prev == prev, "invalid intrusive owner/type/index", c.SIM_INVALID);
        prev = p;
        p = p.*.next;
    }
    try S.require(p == null and prev == list.last, "list count mismatch", c.SIM_INVALID);
}

pub export fn simulation_validate(s: ?*const c.Simulation, e: ?[*]u8, n: u32) callconv(.c) c.SimResult {
    return S.guard(e, n, struct {
        fn run(sim_in: ?*const c.Simulation) !void {
            const sim = sim_in orelse return S.boom(c.SIM_INVALID, "invalid model version/root");
            try S.require(sim.struct_size == @sizeOf(c.Simulation) and sim.abi_version == c.SIM_MODEL_ABI_VERSION and sim.initialized == S.initialized and sim.storage != null, "invalid model version/root", c.SIM_INVALID);
            const storage = store(sim);
            try S.require(@intFromPtr(sim.object_table) == @intFromPtr(storage.object_table.ptr) and @intFromPtr(sim.fault_table) == @intFromPtr(storage.fault_table.ptr) and @intFromPtr(sim.module_table) == @intFromPtr(storage.module_table.ptr) and @intFromPtr(sim.geometry) == @intFromPtr(storage.geometry.ptr), "foreign model tables", c.SIM_INVALID);
            try S.require(@intFromPtr(sim.vertices.data) == @intFromPtr(storage.vertices.ptr) and sim.vertices.offset <= sim.vertices.capacity and @intFromPtr(sim.triangles.data) == @intFromPtr(storage.triangles.ptr) and sim.triangles.offset <= sim.triangles.capacity, "invalid geometry arenas", c.SIM_INVALID);
            try S.require(sim.object_count != 0 and sim.object_count <= storage.config.object_capacity and sim.fault_count <= storage.config.fault_capacity and sim.module_count <= storage.config.module_capacity and sim.geometry_count <= storage.config.collider_capacity and sim.robot.thruster_count <= 8 and sim.robot.hull_count <= 5 and sim.sensor_count <= storage.config.sensor_capacity, "invalid model counts", c.SIM_INVALID);
            const d = sim.settings;
            try S.require(d.dt_numerator != 0 and d.dt_denominator != 0 and d.integrator <= c.SIM_INTEGRATOR_RK4 and d.hydro_period != 0 and d.native_hydrodynamics <= 1 and S.finite3(d.gravity) and S.finite3(d.current) and std.math.isFinite(d.water_density) and d.water_density > 0 and std.math.isFinite(d.bounds.position) and d.bounds.position > 0 and std.math.isFinite(d.bounds.speed) and d.bounds.speed > 0 and std.math.isFinite(d.bounds.angular_speed) and d.bounds.angular_speed > 0, "invalid settings", c.SIM_INVALID);
            try listValid(sim, &sim.objects, sim.object_table, sim.object_count, c.SIM_DATA_OBJECT);
            try listValid(sim, &sim.faults, sim.fault_table, sim.fault_count, c.SIM_DATA_FAULT);
            try listValid(sim, &sim.modules, sim.module_table, sim.module_count, c.SIM_DATA_MODULE);
            var i: u32 = 0;
            while (i < sim.object_count) : (i += 1) {
                const o: *const c.ObjectBlock = @ptrCast(sim.object_table[i]);
                try bodyValid(o.body);
                try S.require(o.id.uid == o.body.id and o.id.pooled <= 1, "invalid object identity", c.SIM_INVALID);
                try S.require(o.colliders != null and o.collider_count != 0 and o.collider_count <= 1024, "invalid collider array", c.SIM_INVALID);
                var j: u32 = 0;
                while (j < i) : (j += 1) try S.require(sim.object_table[j].*.body.id != o.body.id, "duplicate body ID", c.SIM_INVALID);
                var k: u32 = 0;
                while (k < o.collider_count) : (k += 1) {
                    const g = o.colliders[k];
                    try S.require(g.object == o, "foreign collider", c.SIM_INVALID);
                    try shapeValid(g);
                    if (g.data.kind == c.SIM_MESH) {
                        var found = false;
                        var m: u32 = 0;
                        while (m < sim.geometry_count) : (m += 1) {
                            const registered = sim.geometry[m];
                            found = found or (g.geometry.positions == registered.positions and g.geometry.triangles == registered.triangles and g.geometry.position_count == registered.position_count and g.geometry.triangle_count == registered.triangle_count);
                        }
                        try S.require(found, "unregistered geometry", c.SIM_INVALID);
                    }
                }
            }
            try S.require(sim.robot.registered == 0 or (objectMember(sim, &sim.robot.object) and @intFromPtr(sim.robot.object.colliders) == @intFromPtr(&sim.robot.colliders) and sim.robot.object.collider_count <= 5), "invalid embedded robot", c.SIM_INVALID);
            i = 0;
            while (i < sim.robot.thruster_count) : (i += 1) {
                const t = sim.robot.thrusters[i];
                const spec = t.spec;
                try S.require(objectMember(sim, t.object) and S.finite3(t.position) and S.finite3(t.direction) and @abs(S.norm(t.direction) - 1) < 1e-4, "invalid thruster", c.SIM_INVALID);
                try S.require(std.math.isFinite(spec.max_rpm) and spec.max_rpm > 0 and std.math.isFinite(spec.diameter) and spec.diameter > 0 and std.math.isFinite(spec.inertia) and spec.inertia > 0 and std.math.isFinite(spec.thrust) and std.math.isFinite(spec.torque) and std.math.isFinite(spec.kp) and spec.kp >= 0 and std.math.isFinite(spec.ki) and spec.ki >= 0 and (spec.command_sign == 1 or spec.command_sign == -1), "invalid rotor", c.SIM_INVALID);
            }
            i = 0;
            while (i < sim.robot.hull_count) : (i += 1) {
                const h = sim.robot.hulls[i];
                try S.require(objectMember(sim, h.object) and h.positions != null and h.position_count != 0 and h.position_count % 3 == 0, "invalid hull", c.SIM_INVALID);
                const begin = @intFromPtr(sim.hull_vertices.data);
                const address = @intFromPtr(h.positions);
                try S.require(address >= begin and address - begin <= sim.hull_vertices.offset and (try S.bytes(h.position_count, @sizeOf(c.MathVector3f))) <= sim.hull_vertices.offset - (address - begin), "foreign hull geometry", c.SIM_INVALID);
                try S.validPose(h.pose);
                try S.validPose(.{ .pos = S.v(0, 0, 0), .quat = h.coefficient_frame });
                try S.require(S.finite3(h.drag) and S.finite3(h.skin) and S.finite3(h.centroid) and std.math.isFinite(h.volume) and h.volume >= 0, "invalid hull values", c.SIM_INVALID);
                var j: u32 = 0;
                while (j < h.position_count) : (j += 1) try S.require(S.finite3(h.positions[j]), "invalid hull vertex", c.SIM_INVALID);
            }
            try S.require(@intFromPtr(sim.sensors) == @intFromPtr(storage.sensors.ptr), "foreign sensor table", c.SIM_INVALID);
            i = 0;
            while (i < sim.sensor_count) : (i += 1) {
                try sensorValid(sim, sim.sensors[i]);
                var j: u32 = 0;
                while (j < i) : (j += 1) try S.require(sim.sensors[i].id != sim.sensors[j].id, "duplicate sensor ID", c.SIM_INVALID);
            }
            if (sim.robot.sensor_count != 0) {
                try S.require(sim.robot.sensor_count <= sim.sensor_count, "invalid robot sensors", c.SIM_INVALID);
                var found = false;
                i = 0;
                while (i + sim.robot.sensor_count <= sim.sensor_count) : (i += 1) if (sim.robot.sensors == sim.sensors + i) {
                    found = true;
                };
                try S.require(found, "foreign robot sensor view", c.SIM_INVALID);
            }
            try routingValid(sim, sim.routing);
            i = 0;
            while (i < sim.fault_count) : (i += 1) try faultValid(sim, sim.fault_table[i].*);
            i = 0;
            while (i < sim.module_count) : (i += 1) {
                const name = &sim.module_table[i].*.name;
                try S.require(name[0] != 0 and std.mem.indexOfScalar(u8, name, 0) != null, "invalid force module name", c.SIM_INVALID);
                var j: u32 = 0;
                while (j < i) : (j += 1) try S.require(!std.mem.eql(u8, std.mem.sliceTo(name, 0), std.mem.sliceTo(&sim.module_table[j].*.name, 0)), "duplicate module selection", c.SIM_INVALID);
            }
        }
    }.run(s));
}

pub export fn simulation_draft_object(s: ?*c.Simulation, body_id: u32, out: ?*?*c.ObjectBlock, e: ?[*]u8, n: u32) callconv(.c) c.SimResult {
    if (out) |slot| slot.* = null;
    return S.guard(e, n, struct {
        fn run(sim_in: ?*c.Simulation, id: u32, slot: ?*?*c.ObjectBlock) !void {
            try S.require(slot != null, "missing mutable object output", c.SIM_INVALID);
            const sim = try draft(sim_in);
            var i: u32 = 0;
            while (i < sim.object_count) : (i += 1) if (sim.object_table[i].*.body.id == id) {
                const object: *c.ObjectBlock = @constCast(@as(*const c.ObjectBlock, @ptrCast(sim.object_table[i])));
                try S.require(object.id.owner == @as(*const anyopaque, @ptrCast(sim)) and object.id.kind == c.SIM_DATA_OBJECT, "foreign object", c.SIM_INVALID);
                slot.?.* = object;
                return;
            };
            return S.boom(c.SIM_INVALID, "unknown body ID");
        }
    }.run(s, body_id, out));
}

pub export fn simulation_finalize(s: ?*c.Simulation, e: ?[*]u8, n: u32) callconv(.c) c.SimResult {
    return S.guard(e, n, struct {
        fn run(sim_in: ?*c.Simulation) !void {
            const sim = try draft(sim_in);
            var message: [256]u8 = @splat(0);
            try S.check(simulation_validate(sim, &message, message.len), written(&message));
            sim.frozen = 1;
        }
    }.run(s));
}

pub export fn simulation_clone(source: ?*const c.Simulation, out: ?*c.Simulation, e: ?[*]u8, n: u32) callconv(.c) c.SimResult {
    return S.guard(e, n, struct {
        fn run(src_in: ?*const c.Simulation, dest_in: ?*c.Simulation) !void {
            const dest = dest_in orelse return S.boom(c.SIM_INVALID, "clone destination must be zero initialized");
            try S.require(dest.initialized == 0, "clone destination must be zero initialized", c.SIM_INVALID);
            var message: [256]u8 = @splat(0);
            try S.check(simulation_validate(src_in, &message, message.len), written(&message));
            const src = src_in.?;
            try S.check(simulation_init(dest, &store(src).config, &message, message.len), written(&message));
            errdefer simulation_destroy(dest);
            dest.settings = src.settings;
            var i: u32 = 0;
            while (i < src.object_count) : (i += 1) {
                const o: *const c.ObjectBlock = @ptrCast(src.object_table[i]);
                var shapes = std.mem.zeroes([1024]c.Collider);
                var j: u32 = 0;
                while (j < o.collider_count) : (j += 1) {
                    shapes[j] = o.colliders[j];
                    shapes[j].object = null;
                }
                var obj: ?*c.ObjectBlock = null;
                try S.check(simulation_add_object(dest, &o.body, &shapes, o.collider_count, if (o == &src.robot.object) 1 else 0, &obj, &message, message.len), written(&message));
            }
            i = 0;
            while (i < src.robot.thruster_count) : (i += 1) {
                var thruster = src.robot.thrusters[i];
                thruster.object = dest.object_table[thruster.object.*.id.index];
                try S.check(simulation_add_thruster(dest, &thruster, &message, message.len), written(&message));
            }
            i = 0;
            while (i < src.robot.hull_count) : (i += 1) {
                var hull = src.robot.hulls[i];
                hull.object = dest.object_table[hull.object.*.id.index];
                try S.check(simulation_add_hull(dest, &hull, &message, message.len), written(&message));
            }
            i = 0;
            while (i < src.sensor_count) : (i += 1) {
                var sensor = src.sensors[i];
                sensor.object = dest.object_table[sensor.object.*.id.index];
                var inserted: ?*const c.SensorRec = null;
                try S.check(simulation_add_sensor(dest, &sensor, &inserted, &message, message.len), written(&message));
            }
            const fixSensor = struct {
                fn find(from: *const c.Simulation, to: *c.Simulation, sensor: ?*const c.SensorRec) !?*const c.SensorRec {
                    const value = sensor orelse return null;
                    var k: u32 = 0;
                    while (k < from.sensor_count) : (k += 1) if (value == from.sensors + k) return to.sensors + k;
                    return S.boom(c.SIM_INVALID, "foreign sensor reference");
                }
            }.find;
            if (src.robot.sensor_count != 0) {
                dest.robot.sensors = try fixSensor(src, dest, src.robot.sensors);
                dest.robot.sensor_count = src.robot.sensor_count;
            }
            const routes = c.SensorRouting{
                .tracking = try fixSensor(src, dest, src.routing.tracking),
                .imu = try fixSensor(src, dest, src.routing.imu),
                .pressure = try fixSensor(src, dest, src.routing.pressure),
                .camera = try fixSensor(src, dest, src.routing.camera),
            };
            try S.check(simulation_set_sensor_routing(dest, &routes, &message, message.len), written(&message));
            i = 0;
            while (i < src.fault_count) : (i += 1) {
                var fault = src.fault_table[i].*;
                fault.sensor = try fixSensor(src, dest, fault.sensor);
                try S.check(simulation_add_fault(dest, &fault, &message, message.len), written(&message));
            }
            i = 0;
            while (i < src.module_count) : (i += 1) {
                try S.check(simulation_add_module(dest, @ptrCast(&src.module_table[i].*.name), &message, message.len), written(&message));
            }
            dest.robot.name = src.robot.name;
            dest.robot.asset_file = src.robot.asset_file;
            dest.robot.asset_hash = src.robot.asset_hash;
        }
    }.run(source, out));
}
