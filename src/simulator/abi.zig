//! Catalog DNA rows (mostly f32 geometry) and runtime simulator value types.
const std = @import("std");

pub const ColumnSpan = extern struct {
    count: u32 = 0,
    offset: u32 = 0,
};

pub fn spanBytes(backing: []const u8, payload: usize, span: ColumnSpan) []const u8 {
    if (span.count == 0) return &.{};
    const start = payload + @as(usize, span.offset);
    const end = start + @as(usize, span.count);
    if (end > backing.len) return &.{};
    return backing[start..end];
}

pub fn spanAs(comptime T: type, backing: []const u8, payload: usize, span: ColumnSpan) []const T {
    if (span.count == 0) return &.{};
    const byte_len = @as(usize, span.count) * @sizeOf(T);
    const raw = spanBytes(backing, payload, .{ .count = @intCast(byte_len), .offset = span.offset });
    if (raw.len != byte_len) return &.{};
    return @alignCast(std.mem.bytesAsSlice(T, raw));
}

pub const Vec3f = [3]f32;
pub const Quatf = [4]f32;

pub const nil: u32 = std.math.maxInt(u32);

/// Subslice of `RobotCatalog.refs` / `SceneCatalog.refs` for one adjacency list.
pub const LinkHead = struct {
    start: u32 = 0,
    count: u32 = 0,
};

pub fn headSlice(refs: []const u32, head: LinkHead) []const u32 {
    if (head.count == 0) return &.{};
    if (@as(u64, head.start) + head.count > refs.len) return &.{};
    return refs[head.start..][0..head.count];
}

pub fn childAt(comptime Row: type, rows: []const Row, refs: []const u32, head: LinkHead, nth: usize) *const Row {
    const indices = headSlice(refs, head);
    if (nth >= indices.len) unreachable;
    return &rows[indices[nth]];
}

pub fn sceneBodyIndex(refs: []const u32, head: LinkHead) u32 {
    const parents = headSlice(refs, head);
    if (parents.len == 0) return 0;
    return parents[0];
}

pub const Entity = struct {
    name_offset: u32,
    name_len: u32,
    asset_offset: u32,
    asset_len: u32,
    asset_hash: [32]u8,
    id: u32,
    cls: u32,
    motion: u32,
    tracked: u8,
    fuzz: u8,
    pad0: u16,
    mass: f64,
    inertia: [3]f32,
    pad1: u32,
    com_pos: [3]f32,
    com_quat: [4]f32,
    initial_pos: [3]f32,
    initial_quat: [4]f32,
    initial_velocity: [3]f32,
    initial_angular_velocity: [3]f32,
    dry_mass: f64,
    volume: f64,
    buoyancy: [3]f32,
    collider_children: LinkHead = .{},
    thruster_children: LinkHead = .{},
    hull_children: LinkHead = .{},
    sensor_children: LinkHead = .{},
};

pub const Collider = struct {
    entity_parents: LinkHead = .{},
    shape: u32,
    collisions: u8,
    pad0: [3]u8,
    pos: [3]f32,
    quat: [4]f32,
    size: [3]f32,
    drag: [3]f32,
    skin: [3]f32,
    pad1: u32,
    friction: f64,
    restitution: f64,
    vertex_start: u32,
    vertex_count: u32,
    triangle_start: u32,
    triangle_count: u32,
};

pub const Thruster = struct {
    entity_parents: LinkHead = .{},
    scene_bodies: LinkHead = .{},
    position: Vec3f,
    direction: Vec3f,
    max_rpm: f64,
    diameter: f64,
    thrust: f64,
    torque: f64,
    inertia: f64,
    kp: f64,
    ki: f64,
    command_sign: f64,
};

pub const Hull = struct {
    entity_parents: LinkHead = .{},
    scene_bodies: LinkHead = .{},
    pos: [3]f32,
    quat: [4]f32,
    drag: [3]f32,
    skin: [3]f32,
    centroid: [3]f32,
    coefficient_frame: [4]f32,
    volume: f64,
    vertex_start: u32,
    vertex_count: u32,
};

pub const Sensor = struct {
    entity_parents: LinkHead = .{},
    scene_bodies: LinkHead = .{},
    id: u32,
    enabled: u8,
    pad0: [3]u8,
    period_ticks: u32,
    pos: [3]f32,
    quat: [4]f32,
    kind: u32,
    position_axes: u8,
    attitude_axes: u8,
    pad1: u16,
    width: u32,
    height: u32,
    horizontal_fov: f64,
    near_range: f64,
    far_range: f64,
};

pub const Name = struct {
    name_offset: u32,
    name_len: u32,
    entity_start: u32,
    entity_count: u32,
};

/// Declared under this name so existing AUTODNA blobs keep the schema type string.
const RobotCatalogDoc = struct {
    strings: ColumnSpan = .{},
    refs: ColumnSpan = .{},
    entities: ColumnSpan = .{},
    colliders: ColumnSpan = .{},
    thrusters: ColumnSpan = .{},
    hulls: ColumnSpan = .{},
    sensors: ColumnSpan = .{},
    positions: ColumnSpan = .{},
    indices: ColumnSpan = .{},
    names: ColumnSpan = .{},
};

const SceneCatalogDoc = struct {
    strings: ColumnSpan = .{},
    refs: ColumnSpan = .{},
    entities: ColumnSpan = .{},
    colliders: ColumnSpan = .{},
    positions: ColumnSpan = .{},
    indices: ColumnSpan = .{},
    names: ColumnSpan = .{},
};

/// Element type of each column. Field names match the wire `ColumnSpan`s.
const RobotColumns = struct {
    strings: u8,
    refs: u32,
    entities: Entity,
    colliders: Collider,
    thrusters: Thruster,
    hulls: Hull,
    sensors: Sensor,
    positions: f32,
    indices: u32,
    names: Name,
};

const SceneColumns = struct {
    strings: u8,
    refs: u32,
    entities: Entity,
    colliders: Collider,
    positions: f32,
    indices: u32,
    names: Name,
};

fn CatalogView(comptime Doc: type, comptime Columns: type) type {
    comptime {
        for (@typeInfo(Doc).@"struct".field_names) |name| {
            if (!@hasField(Columns, name)) @compileError("catalog column missing type: " ++ name);
        }
        for (@typeInfo(Columns).@"struct".field_names) |name| {
            if (!@hasField(Doc, name)) @compileError("catalog column has no span: " ++ name);
        }
    }
    return struct {
        /// Span row written in the AUTODNA root. Alias keeps the on-disk type name.
        pub const Wire = Doc;

        backing: []const u8 = &.{},
        /// Byte offset of the catalog root row.
        payload: usize = 0,
        wire: Doc = .{},

        pub fn column(self: *const @This(), comptime name: []const u8) []const @FieldType(Columns, name) {
            return spanAs(@FieldType(Columns, name), self.backing, self.payload, @field(self.wire, name));
        }
    };
}

/// Zero-copy view of a robot catalog AUTODNA blob (`backing` must outlive the view).
pub const RobotCatalog = CatalogView(RobotCatalogDoc, RobotColumns);

/// Zero-copy view of a scene catalog AUTODNA blob.
pub const SceneCatalog = CatalogView(SceneCatalogDoc, SceneColumns);

pub const thruster_slots: usize = 8;
pub const max_snapshot_contacts: usize = 1024;

pub const BodyState = struct {
    id: u32,
    pose_pos: Vec3f,
    pose_quat: Quatf,
    com: Vec3f,
    linear_vel: Vec3f,
    angular_vel: Vec3f,
};

pub const Wrench = struct {
    id: u32,
    force: Vec3f,
    torque: Vec3f,
};

pub const RotorState = struct {
    omega: f64,
    integral: f64,
    torque: f64,
    thrust: f64,
};

pub const Contact = struct {
    a: u32,
    b: u32,
    position: Vec3f,
    normal: Vec3f,
    pad: u32,
    normal_force: f64,
};

pub const SessionMeta = struct {
    tick: u64,
    thruster_len: u8,
    robot_name_len: u8,
    scene_name_len: u8,
    pad0: [5]u8,
    commands: [thruster_slots]f64,
};

/// Snapshot columns. `meta` is one `SessionMeta` row. Byte columns store their length in `count`.
pub const SessionStateCatalog = struct {
    strings: ColumnSpan = .{},
    meta: ColumnSpan = .{},
    robot_catalog: ColumnSpan = .{},
    scene_catalog: ColumnSpan = .{},
    body_states: ColumnSpan = .{},
    wrenches: ColumnSpan = .{},
    fluid: ColumnSpan = .{},
    previous_linear: ColumnSpan = .{},
    previous_angular: ColumnSpan = .{},
    rotors: ColumnSpan = .{},
    contacts: ColumnSpan = .{},
};

// --- Runtime / physics value types (f64) ---

pub const Vec3 = [3]f64;
pub const Quat = [4]f64;
pub const Pose = struct {
    pos: Vec3 = @splat(0),
    quat: Quat = .{ 0, 0, 0, 1 },
};

pub const BodyIndex = enum(usize) { robot = 0, _ };
pub const SensorIndex = enum(usize) { _ };
pub const max_hulls = 5;

pub const thruster_count = 8;
pub const ThrusterCommands = [thruster_count]f64;
pub const ThrusterCommandInput = [thruster_count]f32;

pub const Triangle = struct { a: u32, b: u32, c: u32 };

/// Catalog row/column helpers for name strings and mesh subslices.
pub const rna = struct {
    pub fn string(strings: []const u8, entity: Entity) []const u8 {
        if (entity.name_len == 0) return &.{};
        const start: usize = entity.name_offset;
        const end = start + @as(usize, entity.name_len);
        if (end > strings.len) return &.{};
        return strings[start..end];
    }

    /// `start` / `count` are vertex indices (3 f32 each) into the flat `positions` column.
    pub fn vec3s(positions: []const f32, start: u32, count: u32) []const [3]f32 {
        if (count == 0) return &.{};
        const base = @as(usize, start) * 3;
        const end = base + @as(usize, count) * 3;
        if (end > positions.len) return &.{};
        const slice = positions[base..end];
        return @alignCast(std.mem.bytesAsSlice([3]f32, std.mem.sliceAsBytes(slice)));
    }

    pub fn triangles(indices: []const u32, start: u32, count: u32) []const Triangle {
        if (count == 0) return &.{};
        const base = @as(usize, start) * 3;
        const end = base + @as(usize, count) * 3;
        if (end > indices.len) return &.{};
        const triples = indices[base..end];
        return @alignCast(std.mem.bytesAsSlice(Triangle, std.mem.sliceAsBytes(triples)));
    }
};
pub const Mesh = struct {
    positions: []const Vec3f,
    triangles: []const Triangle,
};
pub const Shape = union(enum) { box, sphere, cylinder, mesh: Mesh };
pub const ColliderInput = struct {
    shape: Shape = .box,
    collisions: bool = true,
    pose: Pose = .{},
    size: Vec3 = @splat(1),
    drag: Vec3 = @splat(0),
    skin: Vec3 = @splat(0),
    friction: f64 = 0.5,
    restitution: f64 = 0,
};

pub const Motion = enum { static, dynamic, kinematic };

pub const SensorKind = enum(u32) { pose, imu, pressure, camera };
pub const AxisMask = u3;
pub const Tracking = struct { position_axes: AxisMask, attitude_axes: AxisMask };
pub const Camera = struct {
    width: u32,
    height: u32,
    horizontal_fov: f64,
    near_range: f64,
    far_range: f64,
};
pub const SensorInput = struct {
    id: u32,
    body: BodyIndex = .robot,
    pose: Pose,
    enabled: bool,
    period_ticks: u32,
    settings: union(SensorKind) { pose: Tracking, imu, pressure, camera: Camera },

    pub fn kind(self: SensorInput) SensorKind {
        return self.settings;
    }
};
pub const Routing = std.EnumArray(SensorKind, ?SensorIndex);

pub const Fault = struct {
    start: u64,
    end: u64,
    effect: union(enum) {
        dead: usize,
        weak: struct { thruster: usize, scale: f64 },
        disable_sensor: SensorIndex,
        gyro_bias: struct { sensor: SensorIndex, bias: f64 },
        depth_bias: struct { sensor: SensorIndex, bias: f64 },
    },

    pub fn active(self: Fault, tick: u64) bool {
        return tick >= self.start and tick < self.end;
    }
};

pub const Integrator = enum { default, euler, implicit, implicit_fast, rk4 };
pub const Bounds = struct {
    position: f64 = 10000,
    speed: f64 = 1000,
    angular_speed: f64 = 1000,
};
pub const Settings = struct {
    dt_numerator: u32 = 1,
    dt_denominator: u32 = 360,
    integrator: Integrator = .default,
    hydro_period: u32 = 7,
    gravity: Vec3 = .{ 0, 0, 9.81 },
    current: Vec3 = @splat(0),
    water_density: f64 = 1025,
    bounds: Bounds = .{},

    pub fn timestep(self: Settings) f64 {
        return @as(f64, @floatFromInt(self.dt_numerator)) / @as(f64, @floatFromInt(self.dt_denominator));
    }
};
pub const Force = enum { gravity, hydrodynamics, thrusters };

pub const Detection = struct {
    id: u32,
    cls: u32,
    pose: Pose,
    size: Vec3,
    x0: u32 = 0,
    y0: u32 = 0,
    x1: u32 = 0,
    y1: u32 = 0,
};
pub const Imu = struct { orientation: Quat, gyro: Vec3, accel: Vec3 };
pub const SensorSample = struct {
    tick: u64 = 0,
    value: ?union(SensorKind) {
        pose: struct { pose: Pose, axes: Tracking },
        imu: Imu,
        pressure: f64,
        camera: struct { pose: Pose, objects: []const Detection },
    } = null,
};

pub const Observation = struct {
    body: Pose = .{},
    position_axes: AxisMask = 0,
    attitude_axes: AxisMask = 0,
    image_width: u32 = 0,
    image_height: u32 = 0,
    camera: Pose = .{},
    imu: Quat = .{ 0, 0, 0, 1 },
    gyro: Vec3 = @splat(0),
    accel: Vec3 = @splat(0),
    depth: f64 = 0,
    depth_ok: bool = false,
    objects: []const Detection = &.{},
};