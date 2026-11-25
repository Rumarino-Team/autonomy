//! In-memory robot and scene catalogs. Entity rows list children via `LinkHead` runs in `refs`.
const std = @import("std");
const abi = @import("abi.zig");
const reflection = @import("ReflectionWriter.zig");

pub const rna = reflection.rna;
pub const LinkHead = abi.LinkHead;

pub const nil = abi.nil;
pub const Entity = abi.Entity;
pub const Collider = abi.Collider;
pub const Thruster = abi.Thruster;
pub const Hull = abi.Hull;
pub const Sensor = abi.Sensor;
pub const Name = abi.Name;
pub const RobotCatalog = abi.RobotCatalog;
pub const SceneCatalog = abi.SceneCatalog;

/// Geometry columns for one entity. Robot and scene catalogs both carry these slices.
pub const BodySlot = struct {
    strings: []const u8,
    refs: []const u32,
    entities: []const Entity,
    colliders: []const Collider,
    positions: []const f32,
    indices: []const u32,
    slot: u32,
};

/// `@embedFile` is byte-aligned. Catalog blobs are kept in an aligned image for row access.
pub fn alignedEmbed(comptime file: anytype) []const u8 {
    const Storage = struct {
        var ready = false;
        var bytes: [file.len]u8 align(@alignOf(u64)) = undefined;
    };
    if (!Storage.ready) {
        @memcpy(Storage.bytes[0..], file);
        Storage.ready = true;
    }
    return &Storage.bytes;
}

/// `@embedFile` paths per mission profile (kept in sync with `MissionConfig.scene_catalog_embed`).
pub const scene_catalog_by_profile = struct {
    pub const stonefish_sim = "scenes.bin";
    pub const mujoco_sim = "scenes.bin";
    pub const proteus_hwd = "scenes.bin";
};

/// Scene AUTODNA bytes embedded at build time for `profile` (`mission_profile` enum tag).
pub fn sceneCatalogBytes(profile: anytype) []const u8 {
    return switch (profile) {
        .stonefish_sim => alignedEmbed(@embedFile(scene_catalog_by_profile.stonefish_sim)),
        .mujoco_sim => alignedEmbed(@embedFile(scene_catalog_by_profile.mujoco_sim)),
        .proteus_hwd => alignedEmbed(@embedFile(scene_catalog_by_profile.proteus_hwd)),
    };
}

pub fn bodySlot(catalog: anytype, slot: u32) BodySlot {
    return .{
        .strings = catalog.column("strings"),
        .refs = catalog.column("refs"),
        .entities = catalog.column("entities"),
        .colliders = catalog.column("colliders"),
        .positions = catalog.column("positions"),
        .indices = catalog.column("indices"),
        .slot = slot,
    };
}

pub fn readCatalog(comptime Catalog: type, bytes: []const u8) !Catalog {
    const doc = try reflection.readColumnDocument(Catalog.Wire, bytes);
    return .{
        .backing = bytes,
        .payload = doc.payload,
        .wire = doc.row,
    };
}

pub fn entity(slot: BodySlot) *const Entity {
    return &slot.entities[slot.slot];
}

pub fn entityInitialPose(slot: BodySlot, spawn_override: ?abi.Pose) abi.Pose {
    if (spawn_override) |pose| return pose;
    const row = entity(slot);
    return poseOf(row.initial_pos, row.initial_quat);
}

pub fn entityCenterOfMass(slot: BodySlot) abi.Pose {
    const row = entity(slot);
    return poseOf(row.com_pos, row.com_quat);
}

pub fn sensorKind(record: *const Sensor) abi.SensorKind {
    return switch (record.kind) {
        0 => .pose,
        1 => .imu,
        2 => .pressure,
        3 => .camera,
        else => unreachable,
    };
}

pub fn boxMesh(hx: f64, hy: f64, hz: f64) struct { positions: [8]abi.Vec3f, triangles: [12]abi.Triangle } {
    const positions = [_]abi.Vec3f{
        .{ @floatCast(-hx), @floatCast(-hy), @floatCast(-hz) }, .{ @floatCast(hx), @floatCast(-hy), @floatCast(-hz) }, .{ @floatCast(hx), @floatCast(hy), @floatCast(-hz) }, .{ @floatCast(-hx), @floatCast(hy), @floatCast(-hz) },
        .{ @floatCast(-hx), @floatCast(-hy), @floatCast(hz) },  .{ @floatCast(hx), @floatCast(-hy), @floatCast(hz) },  .{ @floatCast(hx), @floatCast(hy), @floatCast(hz) },  .{ @floatCast(-hx), @floatCast(hy), @floatCast(hz) },
    };
    const triangles = [_]abi.Triangle{
        .{ .a = 0, .b = 1, .c = 2 }, .{ .a = 0, .b = 2, .c = 3 },
        .{ .a = 4, .b = 6, .c = 5 }, .{ .a = 4, .b = 7, .c = 6 },
        .{ .a = 0, .b = 4, .c = 5 }, .{ .a = 0, .b = 5, .c = 1 },
        .{ .a = 2, .b = 6, .c = 7 }, .{ .a = 2, .b = 7, .c = 3 },
        .{ .a = 0, .b = 3, .c = 7 }, .{ .a = 0, .b = 7, .c = 4 },
        .{ .a = 1, .b = 5, .c = 6 }, .{ .a = 1, .b = 6, .c = 2 },
    };
    return .{ .positions = positions, .triangles = triangles };
}

pub fn shapeFrom(slot: BodySlot, record: *const Collider) abi.Shape {
    return switch (record.shape) {
        0 => .box,
        1 => .sphere,
        2 => .cylinder,
        3 => .{ .mesh = .{
            .positions = rna.vec3s(slot.positions, record.vertex_start, record.vertex_count),
            .triangles = rna.triangles(slot.indices, record.triangle_start, record.triangle_count),
        } },
        else => unreachable,
    };
}

pub fn motionFrom(value: u32) abi.Motion {
    return switch (value) {
        0 => .static,
        1 => .dynamic,
        else => .kinematic,
    };
}

pub fn poseOf(pos: [3]f32, quat: [4]f32) abi.Pose {
    return .{
        .pos = .{ @floatCast(pos[0]), @floatCast(pos[1]), @floatCast(pos[2]) },
        .quat = .{ @floatCast(quat[0]), @floatCast(quat[1]), @floatCast(quat[2]), @floatCast(quat[3]) },
    };
}

pub fn validateRobot(catalog: *const RobotCatalog) !void {
    try validateGeometry(catalog);
    for (catalog.column("names")) |name| {
        if (name.entity_count != 1) return error.InvalidDatabase;
    }
    const refs = catalog.column("refs");
    const entities = catalog.column("entities");
    const positions = catalog.column("positions");
    for (catalog.column("thrusters")) |thruster| {
        const parents = abi.headSlice(refs, thruster.entity_parents);
        if (parents.len == 0 or parents[0] >= entities.len) return error.InvalidDatabase;
    }
    for (catalog.column("hulls")) |hull| {
        const parents = abi.headSlice(refs, hull.entity_parents);
        if (parents.len == 0 or parents[0] >= entities.len) return error.InvalidDatabase;
        if (hull.vertex_count == 0 or hull.vertex_count % 3 != 0) return error.InvalidDatabase;
        if (@as(u64, hull.vertex_start) + hull.vertex_count > positions.len / 3) return error.InvalidDatabase;
    }
    for (catalog.column("sensors")) |sensor| {
        const parents = abi.headSlice(refs, sensor.entity_parents);
        if (parents.len == 0 or parents[0] >= entities.len) return error.InvalidDatabase;
        if (sensor.enabled > 1 or sensor.kind > 3) return error.InvalidDatabase;
        if (sensor.position_axes > 7 or sensor.attitude_axes > 7) return error.InvalidDatabase;
    }
}

pub fn validateScene(catalog: *const SceneCatalog) !void {
    return validateGeometry(catalog);
}

fn validateGeometry(catalog: anytype) !void {
    const refs = catalog.column("refs");
    const strings = catalog.column("strings");
    const entities = catalog.column("entities");
    const positions = catalog.column("positions");
    const indices = catalog.column("indices");
    if (positions.len % 3 != 0 or indices.len % 3 != 0) return error.InvalidDatabase;
    const triangle_total: u64 = indices.len / 3;
    for (positions) |component| if (!std.math.isFinite(component)) return error.InvalidDatabase;
    for (entities) |row| {
        try bounds(strings.len, row.name_offset, row.name_len);
        try bounds(strings.len, row.asset_offset, row.asset_len);
        if (row.name_len == 0 or row.tracked > 1 or row.fuzz > 1 or row.motion > 2)
            return error.InvalidDatabase;
        try validateHead(refs, row.collider_children);
        try validateHead(refs, row.thruster_children);
        try validateHead(refs, row.hull_children);
        try validateHead(refs, row.sensor_children);
    }
    for (catalog.column("colliders")) |collider| {
        const parents = abi.headSlice(refs, collider.entity_parents);
        if (parents.len == 0 or parents[0] >= entities.len) return error.InvalidDatabase;
        if (collider.collisions > 1) return error.InvalidDatabase;
        switch (collider.shape) {
            0, 1, 2 => if (collider.vertex_count != 0 or collider.triangle_count != 0) return error.InvalidDatabase,
            3 => {
                if (collider.vertex_count < 4 or collider.triangle_count == 0) return error.InvalidDatabase;
                if (@as(u64, collider.vertex_start) + collider.vertex_count > positions.len / 3) return error.InvalidDatabase;
                if (@as(u64, collider.triangle_start) + collider.triangle_count > triangle_total) return error.InvalidDatabase;
                const start = @as(usize, collider.triangle_start) * 3;
                const end = start + @as(usize, collider.triangle_count) * 3;
                for (indices[start..end]) |index| if (index >= collider.vertex_count) return error.InvalidDatabase;
            },
            else => return error.InvalidDatabase,
        }
    }
    for (catalog.column("names")) |name| {
        try bounds(strings.len, name.name_offset, name.name_len);
        if (name.name_len == 0 or name.entity_count == 0) return error.InvalidDatabase;
        if (@as(u64, name.entity_start) + name.entity_count > entities.len) return error.InvalidDatabase;
    }
}

fn validateHead(refs: []const u32, head: LinkHead) !void {
    if (head.count == 0) return;
    if (@as(u64, head.start) + head.count > refs.len) return error.InvalidDatabase;
}

fn bounds(len: usize, offset: u32, size: u32) !void {
    if (@as(u64, offset) + size > len) return error.InvalidDatabase;
}

test "embedded scene meshes use local triangle indices" {
    const scenes = alignedEmbed(@embedFile("scenes.bin"));
    const catalog = try readCatalog(SceneCatalog, scenes);
    const colliders = catalog.column("colliders");
    const indices = catalog.column("indices");
    for (colliders) |c| {
        if (c.shape != 3 or c.triangle_count == 0) continue;
        const start = @as(usize, c.triangle_start) * 3;
        const end = start + @as(usize, c.triangle_count) * 3;
        for (indices[start..end]) |idx| try std.testing.expect(idx < c.vertex_count);
    }
}

test "embedded robot meshes use local triangle indices" {
    const robots = alignedEmbed(@embedFile("robots.bin"));
    const catalog = try readCatalog(RobotCatalog, robots);
    const colliders = catalog.column("colliders");
    const indices = catalog.column("indices");
    for (colliders) |c| {
        if (c.shape != 3 or c.triangle_count == 0) continue;
        const start = @as(usize, c.triangle_start) * 3;
        const end = start + @as(usize, c.triangle_count) * 3;
        for (indices[start..end]) |idx| try std.testing.expect(idx < c.vertex_count);
    }
}

test "embedded catalogs match current AUTODNA schema" {
    const robots = alignedEmbed(@embedFile("robots.bin"));
    const scenes = alignedEmbed(@embedFile("scenes.bin"));
    try std.testing.expect(reflection.matchesDocument(RobotCatalog.Wire, robots));
    try std.testing.expect(reflection.matchesDocument(SceneCatalog.Wire, scenes));
    _ = try readCatalog(RobotCatalog, robots);
    _ = try readCatalog(SceneCatalog, scenes);
}



