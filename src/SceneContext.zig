//! Thread-owned C scene storage. Initialize in place: Simulation contains intrusive
//! pointers into its own root and must never be copied after construction.
const std = @import("std");
const math = @import("math.zig");
const Auv = @import("Auv.zig");
const DynLib = @import("DynLib.zig").DynLib;
pub const c = @import("sim_c");
const SceneContext = @This();

pub const Api = struct {
    library: DynLib,
    backend: c.PhysicsBackend = std.mem.zeroes(c.PhysicsBackend),
    auv: Auv,
    robot: *const fn ([*c]const u8) callconv(.c) ?*const c.Robot,
    create: *const fn ([*c]c.Simulation, [*c]const u8, [*c]const c.Robot, [*c]u8, u32) callconv(.c) c.SimResult,
    clone: *const fn ([*c]const c.Simulation, [*c]c.Simulation, [*c]u8, u32) callconv(.c) c.SimResult,
    destroy: *const fn ([*c]c.Simulation) callconv(.c) void,
    finalize: *const fn ([*c]c.Simulation, [*c]u8, u32) callconv(.c) c.SimResult,
    draft: *const fn ([*c]c.Simulation, u32, [*c][*c]c.ObjectBlock, [*c]u8, u32) callconv(.c) c.SimResult,
    fault: *const fn ([*c]c.Simulation, [*c]const c.FaultBlock, [*c]u8, u32) callconv(.c) c.SimResult,
    open_backend: *const fn ([*c]c.PhysicsBackend, [*c]const u8, [*c]u8, u32) callconv(.c) c.SimResult,
    close_backend: *const fn ([*c]c.PhysicsBackend) callconv(.c) void,
    init_scene: *const fn ([*c]const c.Simulation, [*c]const c.AuvSimulatorOptions, [*c]u8, u32) callconv(.c) c.SimResult,
    result: *const fn ([*c]c.AuvSimulatorResult, u32, [*c]u8, u32) callconv(.c) c.SimResult,

    pub fn load(path: []const u8) !Api {
        var lib = try DynLib.open(path);
        errdefer lib.close();
        return .{
            .library = lib,
            .auv = .{
                .init = lib.lookup(*const Auv.InitFunc, "auv_init") orelse return error.MissingSimulatorApi,
                .deinit = lib.lookup(*const Auv.DeinitFunc, "auv_deinit") orelse return error.MissingSimulatorApi,
                .yieldUntilNextFrame = lib.lookup(*const Auv.YieldUntilNextFrameFunc, "auv_yield_until_next_frame") orelse return error.MissingSimulatorApi,
                .setThrustorValues = lib.lookup(*const Auv.SetThrustorValuesFunc, "auv_set_thrustor_values") orelse return error.MissingSimulatorApi,
                .simulationStatus = lib.lookup(*const Auv.SimulationStatusFunc, "auv_simulation_status") orelse return error.MissingSimulatorApi,
            },
            .robot = lib.lookup(*const fn ([*c]const u8) callconv(.c) ?*const c.Robot, "sim_robot_find") orelse return error.MissingSimulatorApi,
            .create = try symbol(&lib, @FieldType(Api, "create"), "simulation_scene_create"),
            .clone = try symbol(&lib, @FieldType(Api, "clone"), "simulation_clone"),
            .destroy = try symbol(&lib, @FieldType(Api, "destroy"), "simulation_destroy"),
            .finalize = try symbol(&lib, @FieldType(Api, "finalize"), "simulation_finalize"),
            .draft = try symbol(&lib, @FieldType(Api, "draft"), "simulation_draft_object"),
            .fault = try symbol(&lib, @FieldType(Api, "fault"), "simulation_add_fault"),
            .open_backend = try symbol(&lib, @FieldType(Api, "open_backend"), "physics_backend_open"),
            .close_backend = try symbol(&lib, @FieldType(Api, "close_backend"), "physics_backend_close"),
            .init_scene = try symbol(&lib, @FieldType(Api, "init_scene"), "auv_simulator_init_scene"),
            .result = try symbol(&lib, @FieldType(Api, "result"), "auv_simulator_read_result"),
        };
    }
    fn symbol(lib: *DynLib, comptime T: type, comptime name: [:0]const u8) !T {
        return lib.lookup(T, name) orelse error.MissingSimulatorApi;
    }
    pub fn pinBackend(api: *Api, path: [:0]const u8) !void {
        var detail: [512]u8 = undefined;
        try check(api.open_backend(&api.backend, path, &detail, detail.len));
        if (!std.mem.eql(u8, std.mem.sliceTo(&api.backend.api.name, 0), "mujoco")) return error.FuzzRequiresMuJoCo;
    }
    pub fn deinit(api: *Api) void {
        api.close_backend(&api.backend);
        api.library.close();
    }
};

pub const Range = struct {
    min: f64,
    max: f64,
    pub fn sample(r: Range, random: std.Random) f64 {
        return r.min + (r.max - r.min) * random.float(f64);
    }
};
pub const PlacementGroup = struct { anchor: u32, members: []const u32 };
const gate_members = [_]u32{ 2, 10, 11, 12 };
const default_groups = [_]PlacementGroup{.{ .anchor = 2, .members = &gate_members }};
pub const FuzzProfile = struct {
    pub const version: u32 = 1;
    position: [3]f64 = .{ 0.2, 0.2, 0.1 },
    rotation: [3]f64 = .{ 1.05, 1.05, 1.15 },
    velocity: f64 = 0.2,
    angular_velocity: f64 = 0.2,
    mass: Range = .{ .min = 0.97, .max = 1.03 },
    volume: Range = .{ .min = 0.97, .max = 1.03 },
    drag: Range = .{ .min = 0.7, .max = 1.4 },
    thrust: Range = .{ .min = 0.8, .max = 1.2 },
    rotor_inertia: Range = .{ .min = 0.7, .max = 1.5 },
    current_speed: Range = .{ .min = 0, .max = 0.15 },
    actuator_fault: bool = true,
    sensor_fault: bool = true,
    placements: bool = true,
    groups: []const PlacementGroup = &default_groups,
    pub fn validate(profile: FuzzProfile) !void {
        for (profile.position ++ profile.rotation ++ [_]f64{ profile.velocity, profile.angular_velocity }) |v|
            if (!std.math.isFinite(v) or v < 0) return error.InvalidFuzzProfile;
        for ([_]Range{ profile.mass, profile.volume, profile.drag, profile.thrust, profile.rotor_inertia, profile.current_speed }, 0..) |range, i| {
            if (!std.math.isFinite(range.min) or !std.math.isFinite(range.max) or range.max < range.min or range.min < 0)
                return error.InvalidFuzzProfile;
            if ((i == 0 or i == 1 or i == 3 or i == 4) and range.min == 0) return error.InvalidFuzzProfile;
        }
        for (profile.groups, 0..) |group, i| {
            var anchors: usize = 0;
            for (group.members, 0..) |id, j| {
                if (id == group.anchor) anchors += 1;
                for (group.members[0..j]) |prior| if (prior == id) {
                    return error.InvalidFuzzProfile;
                };
                for (profile.groups[0..i]) |prior| for (prior.members) |member| if (member == id) {
                    return error.InvalidFuzzProfile;
                };
            }
            if (anchors != 1) return error.InvalidFuzzProfile;
        }
    }
    pub fn nominal() FuzzProfile {
        return .{
            .position = .{ 0, 0, 0 },
            .rotation = .{ 0, 0, 0 },
            .velocity = 0,
            .angular_velocity = 0,
            .mass = .{ .min = 1, .max = 1 },
            .volume = .{ .min = 1, .max = 1 },
            .drag = .{ .min = 1, .max = 1 },
            .thrust = .{ .min = 1, .max = 1 },
            .rotor_inertia = .{ .min = 1, .max = 1 },
            .current_speed = .{ .min = 0, .max = 0 },
            .actuator_fault = false,
            .sensor_fault = false,
            .placements = false,
        };
    }
};

baseline: c.Simulation = std.mem.zeroes(c.Simulation),
scene: c.Simulation = std.mem.zeroes(c.Simulation),
api: *const Api,
active: bool = false,
detail: [512]u8 = @splat(0),

pub fn init(self: *SceneContext, api: *const Api, scene_name: [:0]const u8, robot_name: [:0]const u8) !void {
    self.* = .{ .api = api };
    const robot = api.robot(robot_name) orelse return error.UnknownRobot;
    try check(api.create(&self.baseline, scene_name, robot, &self.detail, self.detail.len));
}
pub fn endCase(self: *SceneContext) void {
    if (self.active) self.api.auv.deinit();
    self.active = false;
    self.api.destroy(&self.scene);
}
pub fn deinit(self: *SceneContext) void {
    self.endCase();
    self.api.destroy(&self.baseline);
}
pub fn prepare(self: *SceneContext, random: std.Random, profile: FuzzProfile) !void {
    self.endCase();
    @memset(&self.detail, 0);
    try check(self.api.clone(&self.baseline, &self.scene, &self.detail, self.detail.len));
    try self.randomize(random, profile);
    try check(self.api.finalize(&self.scene, &self.detail, self.detail.len));
}
pub fn start(self: *SceneContext, backend_path: [:0]const u8, max_ticks: u64, frame_ticks: u64) !void {
    const options: c.AuvSimulatorOptions = .{
        .struct_size = @sizeOf(c.AuvSimulatorOptions),
        .version = c.AUV_SIMULATOR_EXTENSION_VERSION,
        .backend_path = backend_path,
        .max_ticks = max_ticks,
        .frame_ticks = frame_ticks,
        .realtime = 0,
        .print_summary = 0,
    };
    try check(self.api.init_scene(&self.scene, &options, &self.detail, self.detail.len));
    self.active = true;
}
pub fn readResult(self: *SceneContext) !c.AuvSimulatorResult {
    var result: c.AuvSimulatorResult = undefined;
    try check(self.api.result(&result, @sizeOf(c.AuvSimulatorResult), &self.detail, self.detail.len));
    return result;
}
pub fn check(result: c.SimResult) !void {
    return switch (result) {
        c.SIM_OK => {},
        c.SIM_CAPACITY => error.SceneCapacity,
        c.SIM_NO_MEMORY => error.OutOfMemory,
        c.SIM_UNSUPPORTED => error.SimulatorUnsupported,
        c.SIM_BACKEND_ERROR => error.SimulatorBackend,
        else => error.InvalidScene,
    };
}
fn object(self: *SceneContext, id: u32) !*c.ObjectBlock {
    var out: [*c]c.ObjectBlock = null;
    try check(self.api.draft(&self.scene, id, &out, &self.detail, self.detail.len));
    return @ptrCast(out);
}
fn vec(r: std.Random, limits: [3]f64) math.Vector3f {
    var out: [3]f32 = undefined;
    for (&out, limits) |*value, limit| value.* = @floatCast((r.float(f64) * 2 - 1) * limit);
    return out;
}
fn cv(v: math.Vector3f) c.MathVector3f {
    return .{ .buf = .{ v[0], v[1], v[2] } };
}
fn zv(v: c.MathVector3f) math.Vector3f {
    return v.buf;
}
fn orientation(p: *c.MathPose, delta: math.Quaternionf) void {
    const q = math.normalize4f(math.quaternionMul(p.quat.buf, delta));
    p.quat.buf = .{ q[0], q[1], q[2], q[3] };
}
fn scale(v: *c.MathVector3f, amount: f64) void {
    for (&v.buf) |*component| component.* *= @floatCast(amount);
}
fn groupMember(profile: FuzzProfile, id: u32) bool {
    for (profile.groups) |group| for (group.members) |member| {
        if (member == id) return true;
    };
    return false;
}
// Fixed draw order: dynamic objects; placement groups; marked standalone scenery;
// current; active thrusters; actuator fault; sensor fault. Increment version if changed.
pub fn randomize(self: *SceneContext, random: std.Random, profile: FuzzProfile) !void {
    const s = &self.scene;
    if (s.frozen != 0) return error.FrozenScene;
    try profile.validate();
    for (s.object_table[0..s.object_count]) |ptr| {
        if (ptr.*.body.motion_type != c.SIM_DYNAMIC) continue;
        const o = try self.object(ptr.*.body.id);
        o.body.initial_pose.pos = cv(zv(o.body.initial_pose.pos) + vec(random, profile.position));
        orientation(&o.body.initial_pose, math.eulerToQuaternion(vec(random, profile.rotation)));
        o.body.initial_velocity = cv(zv(o.body.initial_velocity) + vec(random, @splat(profile.velocity)));
        o.body.initial_angular_velocity = cv(zv(o.body.initial_angular_velocity) + vec(random, @splat(profile.angular_velocity)));
        const mass = profile.mass.sample(random);
        const volume = profile.volume.sample(random);
        const drag = profile.drag.sample(random);
        o.body.mass *= mass;
        o.body.dry_mass *= mass;
        scale(&o.body.inertia, mass);
        o.body.volume *= volume;
        for (s.robot.hulls[0..s.robot.hull_count]) |*h| if (h.object == o) {
            h.volume *= volume;
            scale(&h.drag, drag);
            scale(&h.skin, drag);
        };
    }
    if (profile.placements) {
        for (profile.groups) |group| {
            var exists = false;
            for (s.object_table[0..s.object_count]) |o| {
                if (o.*.body.id == group.anchor) exists = true;
            }
            if (!exists) continue;
            const anchor = (try self.object(group.anchor)).body.initial_pose.pos;
            const delta = vec(random, profile.position);
            const rotation = math.eulerToQuaternion(vec(random, profile.rotation));
            for (group.members) |id| {
                const o = try self.object(id);
                if (o.body.motion_type == c.SIM_DYNAMIC) return error.InvalidPlacementGroup;
                o.body.initial_pose.pos = cv(zv(anchor) + delta + math.quaternionRotate(rotation, zv(o.body.initial_pose.pos) - zv(anchor)));
                // Group rotation is world-space, around the original anchor.
                const q = math.normalize4f(math.quaternionMul(rotation, o.body.initial_pose.quat.buf));
                o.body.initial_pose.quat.buf = .{ q[0], q[1], q[2], q[3] };
            }
        }
        for (s.object_table[0..s.object_count]) |ptr| {
            if (ptr.*.body.motion_type == c.SIM_DYNAMIC or ptr.*.body.fuzz == 0 or groupMember(profile, ptr.*.body.id)) continue;
            const o = try self.object(ptr.*.body.id);
            o.body.initial_pose.pos = cv(zv(o.body.initial_pose.pos) + vec(random, profile.position));
            orientation(&o.body.initial_pose, math.eulerToQuaternion(vec(random, profile.rotation)));
        }
    }
    const speed = profile.current_speed.sample(random);
    const angle = (random.float(f64) * 2 - 1) * std.math.pi;
    s.settings.current = .{ .buf = .{ @floatCast(speed * @cos(angle)), @floatCast(speed * @sin(angle)), 0 } };
    for (s.robot.thrusters[0..s.robot.thruster_count]) |*t| {
        const thrust = profile.thrust.sample(random);
        t.spec.thrust *= thrust;
        t.spec.torque *= thrust;
        t.spec.inertia *= profile.rotor_inertia.sample(random);
    }
    var uid: u32 = 1;
    for (s.fault_table[0..s.fault_count]) |f| uid = @max(uid, std.math.add(u32, f.*.id.uid, 1) catch return error.FaultIdOverflow);
    if (profile.actuator_fault and s.robot.thruster_count > 0) {
        var f = std.mem.zeroes(c.FaultBlock);
        f.id.uid = uid;
        uid = std.math.add(u32, uid, 1) catch return error.FaultIdOverflow;
        f.thruster = random.intRangeLessThan(u32, 0, s.robot.thruster_count);
        f.start = random.intRangeAtMost(u64, 36, 360);
        f.end = f.start + 180;
        f.kind = if (random.float(f64) < 0.25) c.SIM_FAULT_DEAD else c.SIM_FAULT_WEAK;
        f.magnitude = (Range{ .min = 0.3, .max = 0.7 }).sample(random);
        try check(self.api.fault(s, &f, &self.detail, self.detail.len));
    }
    if (profile.sensor_fault) {
        const Target = struct { kind: u32, sensor: [*c]const c.SensorRec };
        var targets: [5]Target = undefined;
        var count: usize = 0;
        const candidates = [_]Target{
            .{ .kind = c.SIM_FAULT_TRACKING, .sensor = s.routing.tracking },
            .{ .kind = c.SIM_FAULT_IMU, .sensor = s.routing.imu },
            .{ .kind = c.SIM_FAULT_CAMERA, .sensor = s.routing.camera },
            .{ .kind = c.SIM_FAULT_GYRO, .sensor = s.routing.imu },
            .{ .kind = c.SIM_FAULT_DEPTH, .sensor = s.routing.pressure },
        };
        for (candidates) |t| if (t.sensor != null) {
            targets[count] = t;
            count += 1;
        };
        if (count > 0) {
            const target = targets[random.intRangeLessThan(usize, 0, count)];
            var f = std.mem.zeroes(c.FaultBlock);
            f.id.uid = uid;
            f.kind = target.kind;
            f.sensor = target.sensor;
            f.start = 360;
            f.end = 720;
            const bias = (Range{ .min = -0.05, .max = 0.05 }).sample(random);
            f.magnitude = if (f.kind == c.SIM_FAULT_GYRO or f.kind == c.SIM_FAULT_DEPTH) bias else 0;
            try check(self.api.fault(s, &f, &self.detail, self.detail.len));
        }
    }
}
pub fn splitmix64(x: u64) u64 {
    var z = x +% 0x9e3779b97f4a7c15;
    z = (z ^ (z >> 30)) *% 0xbf58476d1ce4e5b9;
    z = (z ^ (z >> 27)) *% 0x94d049bb133111eb;
    return z ^ (z >> 31);
}
pub fn caseSeed(master: u64, index: u64) u64 {
    return splitmix64(master +% index *% 0x9e3779b97f4a7c15);
}
test "versioned case seeds and explicit RNG" {
    try std.testing.expectEqual(@as(u64, 0x910a2dec89025cc1), splitmix64(1));
    try std.testing.expectEqual(splitmix64(1), caseSeed(1, 0));
    try std.testing.expect(caseSeed(1, 0) != caseSeed(1, 1));
    var a = std.Random.Xoshiro256.init(caseSeed(1, 0));
    var b = std.Random.Xoshiro256.init(caseSeed(1, 0));
    for (0..100) |_| try std.testing.expectEqual(a.next(), b.next());
}

pub const BodySnapshot = struct { body: c.UnderwaterBody };
pub const HullSnapshot = struct { object_id: u32, pose: c.MathPose, drag: [3]f32, skin: [3]f32, volume: f64 };
pub const ThrusterSnapshot = struct { object_id: u32, position: [3]f32, direction: [3]f32, spec: c.RotorSpec };
pub const FaultSnapshot = struct { id: u32, kind: u32, thruster: u32, sensor_id: ?u32, start: u64, end: u64, magnitude: f64 };
pub const SensorSnapshot = struct { id: u32, kind: u32, object_id: u32, pose: c.MathPose, enabled: u32, period: u32, camera: ?c.CameraSettings };
pub const Snapshot = struct {
    current: [3]f32,
    objects: []BodySnapshot,
    hulls: []HullSnapshot,
    thrusters: []ThrusterSnapshot,
    faults: []FaultSnapshot,
    sensors: []SensorSnapshot,
};
// Serialize named numeric fields, never intrusive links, pointers, or SIMD padding.
pub fn snapshot(self: *const SceneContext, allocator: std.mem.Allocator) !Snapshot {
    const s = &self.scene;
    const objects = try allocator.alloc(BodySnapshot, s.object_count);
    for (objects, s.object_table[0..s.object_count]) |*out, o| out.* = .{ .body = o.*.body };
    const hulls = try allocator.alloc(HullSnapshot, s.robot.hull_count);
    for (hulls, s.robot.hulls[0..s.robot.hull_count]) |*out, h| out.* = .{
        .object_id = h.object.*.body.id,
        .pose = h.pose,
        .drag = h.drag.buf,
        .skin = h.skin.buf,
        .volume = h.volume,
    };
    const thrusters = try allocator.alloc(ThrusterSnapshot, s.robot.thruster_count);
    for (thrusters, s.robot.thrusters[0..s.robot.thruster_count]) |*out, t| out.* = .{
        .object_id = t.object.*.body.id,
        .position = t.position.buf,
        .direction = t.direction.buf,
        .spec = t.spec,
    };
    const faults = try allocator.alloc(FaultSnapshot, s.fault_count);
    for (faults, s.fault_table[0..s.fault_count]) |*out, f| out.* = .{
        .id = f.*.id.uid,
        .kind = f.*.kind,
        .thruster = f.*.thruster,
        .sensor_id = if (f.*.sensor != null) f.*.sensor.*.id else null,
        .start = f.*.start,
        .end = f.*.end,
        .magnitude = f.*.magnitude,
    };
    const sensors = try allocator.alloc(SensorSnapshot, s.sensor_count);
    for (sensors, s.sensors[0..s.sensor_count]) |*out, sensor| out.* = .{
        .id = sensor.id,
        .kind = sensor.kind,
        .object_id = sensor.object.*.body.id,
        .pose = sensor.pose,
        .enabled = sensor.enabled,
        .period = sensor.period_ticks,
        .camera = if (sensor.kind == c.SIM_SENSOR_CAMERA) sensor.settings.camera else null,
    };
    return .{ .current = s.settings.current.buf, .objects = objects, .hulls = hulls, .thrusters = thrusters, .faults = faults, .sensors = sensors };
}

test "profile validation and xoshiro output are pinned" {
    try (FuzzProfile{}).validate();
    try FuzzProfile.nominal().validate();
    var invalid = FuzzProfile{};
    invalid.mass.min = 0;
    try std.testing.expectError(error.InvalidFuzzProfile, invalid.validate());
    invalid = FuzzProfile{};
    invalid.current_speed.max = std.math.inf(f64);
    try std.testing.expectError(error.InvalidFuzzProfile, invalid.validate());
    var random = std.Random.Xoshiro256.init(1);
    try std.testing.expectEqual(@as(u64, 0xcfc5d07f6f03c29b), random.next());
}
