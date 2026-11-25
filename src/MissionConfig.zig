//! Compiled startup defaults and vehicle tuning presets.
const std = @import("std");
const bc = @import("BodyControl.zig");
const math = @import("math.zig");
const simulator_default = @import("build_options").simulator;

/// Thrusters / TAM rows (max). Active count per vehicle is `tam_len`.
pub const thruster_count = 8;
pub const tam_cols = 6;

pub const max_ibvs_error_cols = 8;
pub const telemetry_addr_max = 64;
pub const telemetry_log_path_max = 256;
pub const queue_capacity_max = 64;

pub const TelemetryTransport = enum(u8) {
    udp = 0,
    stdout = 2,
    disabled = 3,
};

pub fn FixedText(comptime capacity: usize) type {
    return struct {
        bytes: [capacity]u8 = std.mem.zeroes([capacity]u8),
        len: u16 = 0,

        pub fn fromLiteral(comptime text: []const u8) @This() {
            if (text.len > capacity) {
                @compileError("text exceeds FixedText capacity");
            }
            var out: @This() = .{};
            out.len = @intCast(text.len);
            std.mem.copyForwards(u8, out.bytes[0..text.len], text);
            return out;
        }

        pub fn slice(self: *const @This()) error{InvalidTextLength}![]const u8 {
            if (self.len > self.bytes.len) return error.InvalidTextLength;
            return self.bytes[0..self.len];
        }

        /// Return the stored bytes directly as a C-compatible sentinel slice.
        pub fn sentinelSlice(self: *const @This()) error{InvalidTextLength}![:0]const u8 {
            if (self.len >= self.bytes.len or self.bytes[self.len] != 0) return error.InvalidTextLength;
            const text = self.bytes[0..self.len :0];
            if (std.mem.indexOfScalar(u8, text, 0) != null) return error.InvalidTextLength;
            return text;
        }
    };
}

pub fn fixedTextFromSlice(comptime capacity: usize, text: []const u8) error{InvalidTextLength}!FixedText(capacity) {
    if (text.len > capacity or std.mem.indexOfScalar(u8, text, 0) != null) return error.InvalidTextLength;
    var out: FixedText(capacity) = .{};
    out.len = @intCast(text.len);
    @memcpy(out.bytes[0..text.len], text);
    return out;
}

pub const OdometryPid = struct {
    rotation_gate: f64,
    kp: math.Vector6f,
    ki: math.Vector6f,
    kd: math.Vector6f,
};

pub const Ibvs = struct {
    center_deadband: f64,
    target_height: f64,
    interaction_cols: u8,
    _pad: [3]u8 = .{ 0, 0, 0 },
    interaction: [math.interaction_output_len][max_ibvs_error_cols]f64,
};

pub const MissionProfileId = enum(u8) {
    stonefish_sim,
    mujoco_sim,
    proteus_hwd,

    pub fn fromName(name: []const u8) ?MissionProfileId {
        return std.meta.stringToEnum(MissionProfileId, name);
    }
};

pub const MissionId = enum(u8) { prequalify_coco, prequalify_robosub, teleop, mission_control, thruster_control };

pub const path_max = 256;
pub const name_max = 64;
pub const control_port: u16 = 8080;
pub const default_telemetry_addr: []const u8 = "127.0.0.1:5600";
pub const default_telemetry_log_path: []const u8 = "";
pub const startup_mission: MissionId = .mission_control;
pub const startup_profile: MissionProfileId = if (simulator_default) .mujoco_sim else .stonefish_sim;

/// Compiled vehicle tuning presets.
pub const MissionProfiles = struct {
    stonefish_sim: MissionConfig,
    mujoco_sim: MissionConfig,
    proteus_hwd: MissionConfig,
};

/// Teleop/telemetry, hardware serial, and simulation I/O — set in presets; rebuild to change.
pub const RuntimeConfig = struct {
    mission_profile: MissionProfileId = startup_profile,
    sensor_library: FixedText(path_max) = FixedText(path_max).fromLiteral("./zig-out/lib/libauv_proteus_hwd.so"),
    serial_port: FixedText(path_max) = .{},
    sim_robot: FixedText(name_max) = FixedText(name_max).fromLiteral("hydrus"),
    sim_scene: FixedText(name_max) = FixedText(name_max).fromLiteral("robosub_course"),
    sim_realtime: bool = false,
    teleop_port: u16 = control_port,
    telemetry_transport: TelemetryTransport = .udp,
    telemetry_addr: FixedText(telemetry_addr_max) = FixedText(telemetry_addr_max).fromLiteral(default_telemetry_addr),
    telemetry_log_path: FixedText(telemetry_log_path_max) = .{},

    pub fn operationallyEqual(a: *const RuntimeConfig, b: *const RuntimeConfig) bool {
        return a.mission_profile == b.mission_profile and textEqual(a.sensor_library, b.sensor_library) and
            textEqual(a.serial_port, b.serial_port) and
            textEqual(a.sim_robot, b.sim_robot) and textEqual(a.sim_scene, b.sim_scene) and
            a.sim_realtime == b.sim_realtime and
            a.teleop_port == b.teleop_port and a.telemetry_transport == b.telemetry_transport and
            textEqual(a.telemetry_addr, b.telemetry_addr) and textEqual(a.telemetry_log_path, b.telemetry_log_path);
    }

    pub fn telemetryAddr(self: *const RuntimeConfig) error{InvalidTextLength}![]const u8 {
        return try self.telemetry_addr.slice();
    }

    pub fn telemetryLogPath(self: *const RuntimeConfig) error{InvalidTextLength}!?[]const u8 {
        if (self.telemetry_log_path.len == 0) return null;
        return try self.telemetry_log_path.slice();
    }

    pub fn validate(self: *const RuntimeConfig) MissionConfig.ValidateError!void {
        if (self.sim_robot.len == 0 or self.sim_scene.len == 0) return error.InvalidPath;
        if (self.sensor_library.len == 0 and self.mission_profile != .mujoco_sim) return error.InvalidPath;
        if (self.serial_port.len != 0 and self.mission_profile != .proteus_hwd) return error.InvalidProfile;
        if (self.teleop_port != control_port and !@import("builtin").is_test) return error.InvalidPath;
        switch (self.telemetry_transport) {
            .udp => _ = try self.telemetryAddr(),
            .stdout, .disabled => {},
        }
        _ = try self.telemetryLogPath();
    }
};

pub const MissionConfig = struct {
    runtime: RuntimeConfig = .{},
    mission_id: MissionId = startup_mission,
    /// AUTODNA scene catalog for this profile (`@embedFile` path relative to `src/simulator/`).
    comptime scene_catalog_embed: []const u8 = "scenes.bin",

    // Control tuning copied into mutable runtime state and optionally updated
    // by a validated live-control packet.
    camera_forward: math.Vector3f,
    rotation_limit: math.Vector3f,
    tam: [thruster_count]math.Vector6f,
    tam_len: u8,
    _pad1: [7]u8 = .{ 0, 0, 0, 0, 0, 0, 0 },
    odometry: OdometryPid,
    ibvs: Ibvs,
    goal_dist_threshold: f64,
    thrustor_saturate: f64,
    log_freq_div: u16,
    queue_capacity: u16,
    print_stuff: bool,
    clear_print_stuff: bool,

    pub fn operationallyEqual(a: *const MissionConfig, b: *const MissionConfig) bool {
        return a.mission_id == b.mission_id and a.runtime.operationallyEqual(&b.runtime);
    }

    pub fn telemetryAddr(self: *const MissionConfig) error{InvalidTextLength}![]const u8 {
        return self.runtime.telemetryAddr();
    }

    pub fn telemetryLogPath(self: *const MissionConfig) error{InvalidTextLength}!?[]const u8 {
        return self.runtime.telemetryLogPath();
    }

    pub const ValidateError = error{
        InvalidTamLen,
        InvalidProfile,
        InvalidPath,
        InvalidQueueCapacity,
        InvalidInteractionCols,
        InvalidTextLength,
    };

    pub fn validate(self: *const MissionConfig) ValidateError!void {
        try self.runtime.validate();
        if (self.tam_len != compiledProfile(self.runtime.mission_profile).tam_len) return error.InvalidProfile;
        if (self.tam_len == 0 or self.tam_len > thruster_count) return error.InvalidTamLen;
        if (self.queue_capacity == 0 or self.queue_capacity > queue_capacity_max) return error.InvalidQueueCapacity;
        if (self.ibvs.interaction_cols == 0 or self.ibvs.interaction_cols > max_ibvs_error_cols) return error.InvalidInteractionCols;
    }
};

// --- Presets (edit here; rebuild the main executable to apply compile-time checks) ---

fn presetZeroTam() [thruster_count]math.Vector6f {
    return @splat(@splat(0));
}

fn presetZeroInteraction() [math.interaction_output_len][max_ibvs_error_cols]f64 {
    return @splat(@splat(0));
}

fn presetBaseline() MissionConfig {
    return .{
        .runtime = .{
            .teleop_port = control_port,
            .telemetry_transport = .udp,
        },
        .camera_forward = .{ 0, 1, 0 },
        .rotation_limit = .{ std.math.inf(f64), std.math.inf(f64), std.math.inf(f64) },
        .tam = presetZeroTam(),
        .tam_len = 0,
        .odometry = .{
            .rotation_gate = std.math.pi / 8.0,
            .kp = .{ 1, 1, 1, 1, 1, 1 },
            .ki = .{ 0, 0, 0, 0, 0, 0 },
            .kd = .{ 0, 0, 0, 0, 0, 0 },
        },
        .ibvs = .{
            .center_deadband = 0.05,
            .target_height = 0.9,
            .interaction_cols = 3,
            .interaction = presetZeroInteraction(),
        },
        .goal_dist_threshold = 1.0,
        .thrustor_saturate = 5,
        .log_freq_div = 120,
        .queue_capacity = 32,
        .print_stuff = true,
        .clear_print_stuff = true,
    };
}

fn presetStonefishSim() MissionConfig {
    var config = presetBaseline();
    config.runtime.mission_profile = .stonefish_sim;
    config.mission_id = .prequalify_robosub;
    config.runtime.sensor_library = FixedText(path_max).fromLiteral("./zig-out/lib/libauv_stonefish_sim.so");
    config.runtime.telemetry_transport = .stdout;
    config.rotation_limit = .{ 0, 0, 100 };
    config.odometry = .{
        .rotation_gate = 0.292699081688714,
        .kp = .{ 1.0, 0.7, 1.1, 3.0, 6.0, 8.0 },
        .ki = .{ 0.0, 0.0, 0.4, 1.0, 1.0, 0.0 },
        .kd = .{ 0.3, 0.3, 0.35, 3.3, 6.2, 8.0 },
    };
    config.tam = .{
        .{ -2.4142, 1.0, 0.0, 0.0, 0.0, 1.0 },
        .{ 2.4142, 1.0, 0.0, 0.0, 0.0, -1.0 },
        .{ -2.4142, -1.0, 0.0, 0.0, 0.0, -1.0 },
        .{ 2.4142, -1.0, 0.0, 0.0, 0.0, 1.0 },
        .{ 0.0, 0.0, -1.0, -1.0, 1.0, 0.0 },
        .{ 0.0, 0.0, -1.0, -1.0, -1.0, 0.0 },
        .{ 0.0, 0.0, -1.0, 1.0, 1.0, 0.0 },
        .{ 0.0, 0.0, -1.0, 1.0, -1.0, 0.0 },
    };
    config.tam_len = 8;
    config.ibvs.interaction = .{
        .{ 0, 0, 0, 0, 0, 0, 0, 0 },
        .{ 0, 0, 4, 0, 0, 0, 0, 0 },
        .{ 0, -1, 0, 0, 0, 0, 0, 0 },
        .{ -1, 0, 0, 0, 0, 0, 0, 0 },
        .{ 0, 0, 0, 0, 0, 0, 0, 0 },
        .{ 0, 0, 0, 0, 0, 0, 0, 0 },
    };
    return config;
}

fn presetMujocoSim() MissionConfig {
    var config = presetStonefishSim();
    config.runtime.mission_profile = .mujoco_sim;
    config.mission_id = .prequalify_robosub;
    config.runtime.sensor_library = .{};
    config.runtime.telemetry_transport = .udp;
    config.runtime.sim_robot = FixedText(name_max).fromLiteral("hydrus");
    config.runtime.sim_scene = FixedText(name_max).fromLiteral("robosub_course");
    return config;
}

fn presetProteusHwd() MissionConfig {
    var config = presetBaseline();
    config.runtime.mission_profile = .proteus_hwd;
    config.runtime.sim_robot = FixedText(name_max).fromLiteral("proteus");
    config.rotation_limit = .{ 0.3, 0, 100 };
    config.ibvs.target_height = 0.35;
    config.odometry = .{
        .rotation_gate = 0.392699081688714,
        .kp = .{ 1.0, 1.0, 2.2, 2.4, 2.4, 2.0 },
        .ki = .{ 0.0, 0.0, 0.05, 0.05, 0.05, 0.0 },
        .kd = .{ 0.1, 0.1, 0.4, 0.2, 0.2, 2.0 },
    };
    config.tam = .{
        .{ 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 },
        .{ 0.0, 1.0, 0.0, 0.0, 0.0, -1.0 },
        .{ 0.0, 0.0, -1.0, 1.0, 1.0, 0.0 },
        .{ 0.0, 0.0, -1.0, 1.0, -1.0, 0.0 },
        .{ 0.0, 0.0, -1.0, -1.0, 1.0, 0.0 },
        .{ 0.0, 0.0, -1.0, -1.0, -1.0, 0.0 },
        .{ 0, 0, 0, 0, 0, 0 },
        .{ 0, 0, 0, 0, 0, 0 },
    };
    config.tam_len = 6;
    config.ibvs.interaction = .{
        .{ 0, 0, 0, 0, 0, 0, 0, 0 },
        .{ 0, 0, 1, 0, 0, 0, 0, 0 },
        .{ 0, 0, 0, 0, 0, 0, 0, 0 },
        .{ 0, 0, 0, 0, 0, 0, 0, 0 },
        .{ 0, 0, 0, 0, 0, 0, 0, 0 },
        .{ -1, 0, 0, 0, 0, 0, 0, 0 },
    };
    return config;
}

pub const mission_profiles: MissionProfiles = .{
    .stonefish_sim = presetStonefishSim(),
    .mujoco_sim = presetMujocoSim(),
    .proteus_hwd = presetProteusHwd(),
};

/// Return the compiled profile used to initialize runtime configuration.
pub fn compiledProfile(profile: MissionProfileId) *const MissionConfig {
    return switch (profile) {
        .stonefish_sim => &mission_profiles.stonefish_sim,
        .mujoco_sim => &mission_profiles.mujoco_sim,
        .proteus_hwd => &mission_profiles.proteus_hwd,
    };
}

fn rowIsZero(row: math.Vector6f) bool {
    inline for (0..tam_cols) |i| if (row[i] != 0) return false;
    return true;
}

/// Same geometry the controller uses: a body force is either in the TAM span or bought by `coverForce`.
fn assertPreset(comptime name: []const u8, comptime config: MissionConfig) void {
    @setEvalBranchQuota(100_000);
    if (config.tam_len == 0 or config.tam_len > thruster_count) {
        @compileError(name ++ ": tam_len out of range");
    }
    if (config.queue_capacity == 0 or config.queue_capacity > queue_capacity_max) {
        @compileError(name ++ ": queue_capacity out of range");
    }
    if (config.ibvs.interaction_cols != 3) {
        @compileError(name ++ ": interaction_cols must be 3 (image error is cx, cy, height)");
    }
    if (config.scene_catalog_embed.len == 0 or std.mem.indexOfScalar(u8, config.scene_catalog_embed, 0) != null) {
        @compileError(name ++ ": scene_catalog_embed must be a non-empty path without NUL");
    }

    var last_nonzero: usize = 0;
    for (config.tam, 0..) |row, index| {
        inline for (0..tam_cols) |i| {
            if (!std.math.isFinite(row[i])) @compileError(name ++ ": TAM gain is not finite");
        }
        if (!rowIsZero(row)) last_nonzero = index + 1;
    }
    if (last_nonzero != config.tam_len) {
        @compileError(name ++ ": tam_len does not match the last nonzero TAM row");
    }

    inline for (.{ config.odometry.kp, config.odometry.ki, config.odometry.kd }) |gains| {
        inline for (0..6) |i| {
            if (!std.math.isFinite(gains[i])) @compileError(name ++ ": odometry gain is not finite");
        }
    }
    if (!std.math.isFinite(config.odometry.rotation_gate)) {
        @compileError(name ++ ": rotation_gate is not finite");
    }
    for (config.ibvs.interaction) |row| {
        for (row[0..config.ibvs.interaction_cols]) |gain| {
            if (!std.math.isFinite(gain)) @compileError(name ++ ": interaction gain is not finite");
        }
    }

    if (!(math.length3f(config.camera_forward) > 1e-4)) {
        @compileError(name ++ ": camera_forward has zero length");
    }

    const active = config.tam[0..config.tam_len];
    const forces = bc.ThrustDirections.fromTamLinearForces(active) catch {
        @compileError(name ++ ": TAM forces are not finite");
    };
    const moments = bc.ThrustDirections.fromTamMoments(active) catch {
        @compileError(name ++ ": TAM moments are not finite");
    };
    if (moments.rank != 3) {
        @compileError(name ++ ": TAM does not torque about roll, pitch, and yaw");
    }

    if (forces.rank == 0) @compileError(name ++ ": TAM produces no body force");

    const axes = [3]math.Vector3f{
        .{ 1, 0, 0 },
        .{ 0, 1, 0 },
        .{ 0, 0, 1 },
    };
    const axis_names = [_][]const u8{ "X", "Y", "Z" };
    const limits = config.rotation_limit;
    // `coverForce` only swings a thruster that already carries force, so a missing axis is
    // probed as `axis + thrust`, not as a pure unit command on the missing axis.
    inline for (axes, axis_names, 0..) |axis, axis_name, i| {
        const direct = forces.alongThrust(axis);
        if (@abs(direct[i]) > 1e-3) continue;
        var recovered = false;
        for (forces.basis[0..forces.rank]) |thrust| {
            const covered = bc.coverForce(forces, moments, axis + thrust, limits, config.camera_forward);
            if (@abs(covered.force[i]) > 1e-3) recovered = true;
        }
        if (!recovered) {
            @compileError(name ++ ": body " ++ axis_name ++ " force is not in the TAM and coverForce cannot recover it");
        }
    }
}

fn assertSceneCatalogEmbedMatches(comptime name: []const u8, comptime profile: MissionProfileId) void {
    const database = @import("simulator/database.zig");
    const embed = compiledProfile(profile).scene_catalog_embed;
    const table = @field(database.scene_catalog_by_profile, name);
    if (!std.mem.eql(u8, embed, table)) {
        @compileError("scene_catalog_embed for " ++ name ++ " must match database.scene_catalog_by_profile");
    }
}

comptime {
    assertSceneCatalogEmbedMatches("stonefish_sim", .stonefish_sim);
    assertSceneCatalogEmbedMatches("mujoco_sim", .mujoco_sim);
    assertSceneCatalogEmbedMatches("proteus_hwd", .proteus_hwd);
    if (mission_profiles.proteus_hwd.tam_len != 6) @compileError("Proteus firmware requires six TAM rows");
    assertPreset("stonefish_sim", mission_profiles.stonefish_sim);
    assertPreset("mujoco_sim", mission_profiles.mujoco_sim);
    assertPreset("proteus_hwd", mission_profiles.proteus_hwd);
}

pub fn textEqual(a: anytype, b: @TypeOf(a)) bool {
    return a.len == b.len and a.len <= a.bytes.len and std.mem.eql(u8, a.bytes[0..a.len], b.bytes[0..b.len]);
}
pub const startup_config: MissionConfig = blk: {
    var config = compiledProfile(startup_profile).*;
    if (startup_profile == .proteus_hwd) config.mission_id = startup_mission;
    break :blk config;
};
