//! Vehicle tuning presets (`mission_profiles`) live at the bottom of this file (replaces `auvs/*/auv.json`).
const std = @import("std");
const bc = @import("BodyControl.zig");
const math = @import("math.zig");

/// Thrusters / TAM rows (max). Active count per vehicle is `tam_len`.
pub const thruster_count = 8;
pub const tam_cols = 6;

pub const max_ibvs_error_cols = 8;
pub const telemetry_addr_max = 64;
pub const telemetry_log_path_max = 256;
pub const telemetry_shm_name_max = 64;
pub const queue_capacity_max = 64;

pub const TelemetryTransport = enum(u8) {
    udp,
    shared_memory,
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
    };
}

pub const OdometryPid = struct {
    rotation_gate: f32,
    kp: [6]f32,
    ki: [6]f32,
    kd: [6]f32,
};

pub const Ibvs = struct {
    center_deadband: f32,
    target_height: f32,
    interaction_cols: u8,
    _pad: [3]u8 = .{ 0, 0, 0 },
    interaction: [math.interaction_output_len][max_ibvs_error_cols]f32,
};

pub const MissionProfileId = enum {
    stonefish_sim,
    proteus_hwd,

    pub fn fromName(name: []const u8) ?MissionProfileId {
        return std.meta.stringToEnum(MissionProfileId, name);
    }
};

/// All vehicle presets in the plugin (`mission_profiles` symbol).
pub const MissionProfiles = struct {
    stonefish_sim: MissionConfig,
    proteus_hwd: MissionConfig,
};

pub const MissionConfig = struct {
    // Startup/control connection. These values must match between the runtime
    // and viewer built from this module.
    teleop_port: u16,
    telemetry_transport: TelemetryTransport,
    telemetry_addr: FixedText(telemetry_addr_max),
    telemetry_shm_name: FixedText(telemetry_shm_name_max),
    telemetry_log_path: FixedText(telemetry_log_path_max),

    // Control tuning copied into mutable runtime state and optionally updated
    // by a validated live-control packet.
    camera_forward: [3]f32,
    rotation_limit: [3]f32,
    tam: [thruster_count][tam_cols]f32,
    tam_len: u8,
    _pad1: [7]u8 = .{ 0, 0, 0, 0, 0, 0, 0 },
    odometry: OdometryPid,
    ibvs: Ibvs,
    goal_dist_threshold: f32,
    thrustor_saturate: f32,
    log_freq_div: u16,
    queue_capacity: u16,
    print_stuff: bool,
    clear_print_stuff: bool,

    pub fn telemetryAddr(self: *const MissionConfig) error{InvalidTextLength}![]const u8 {
        const text = try self.telemetry_addr.slice();
        if (text.len == 0) return error.InvalidTextLength;
        return text;
    }

    pub fn telemetryLogPath(self: *const MissionConfig) error{InvalidTextLength}!?[]const u8 {
        const text = try self.telemetry_log_path.slice();
        if (text.len == 0) return null;
        return text;
    }

    pub fn telemetryShmName(self: *const MissionConfig) error{InvalidTextLength}![]const u8 {
        const text = try self.telemetry_shm_name.slice();
        if (text.len == 0) return error.InvalidTextLength;
        return text;
    }

    pub const ValidateError = error{
        InvalidTamLen,
        InvalidQueueCapacity,
        InvalidInteractionCols,
        InvalidTextLength,
    };

    pub fn validate(self: *const MissionConfig) ValidateError!void {
        if (self.tam_len == 0 or self.tam_len > thruster_count) return error.InvalidTamLen;
        if (self.queue_capacity == 0 or self.queue_capacity > queue_capacity_max) return error.InvalidQueueCapacity;
        if (self.ibvs.interaction_cols == 0 or self.ibvs.interaction_cols > max_ibvs_error_cols) return error.InvalidInteractionCols;
        _ = try self.telemetryAddr();
        if (self.telemetry_transport == .shared_memory) _ = try self.telemetryShmName();
        _ = try self.telemetryLogPath();
    }
};

comptime {
    if (@offsetOf(MissionConfig, "tam") <= @offsetOf(MissionConfig, "rotation_limit")) {
        @compileError("MissionConfig field order drift: tam must follow rotation_limit");
    }
    if (@offsetOf(MissionConfig, "odometry") <= @offsetOf(MissionConfig, "tam")) {
        @compileError("MissionConfig field order drift: odometry must follow tam");
    }
}

// --- Presets (edit here; rebuild the main executable to apply compile-time checks) ---

fn presetZeroTam() [thruster_count][tam_cols]f32 {
    return std.mem.zeroes([thruster_count][tam_cols]f32);
}

fn presetZeroInteraction() [math.interaction_output_len][max_ibvs_error_cols]f32 {
    return std.mem.zeroes([math.interaction_output_len][max_ibvs_error_cols]f32);
}

fn presetBaseline() MissionConfig {
    return .{
        .teleop_port = 8080,
        .camera_forward = .{ 0, 1, 0 },
        .rotation_limit = .{ std.math.inf(f32), std.math.inf(f32), std.math.inf(f32) },
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
        .telemetry_transport = .udp,
        .telemetry_addr = FixedText(telemetry_addr_max).fromLiteral("127.0.0.1:5600"),
        .telemetry_shm_name = FixedText(telemetry_shm_name_max).fromLiteral("autonomy_telemetry"),
        .telemetry_log_path = FixedText(telemetry_log_path_max).fromLiteral(""),
    };
}

fn presetStonefishSim() MissionConfig {
    var config = presetBaseline();
    config.rotation_limit = .{ 0, 0, 100 };
    config.telemetry_transport = .shared_memory;
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

fn presetProteusHwd() MissionConfig {
    var config = presetBaseline();
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
    .proteus_hwd = presetProteusHwd(),
};

/// Return the compiled profile used to initialize runtime configuration.
pub fn compiledProfile(profile: MissionProfileId) *const MissionConfig {
    return switch (profile) {
        .stonefish_sim => &mission_profiles.stonefish_sim,
        .proteus_hwd => &mission_profiles.proteus_hwd,
    };
}

/// The current viewer/runtime integration is compiled for this profile.
pub const viewer_profile = MissionProfileId.stonefish_sim;

fn rowIsZero(row: [tam_cols]f32) bool {
    for (row) |gain| if (gain != 0) return false;
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

    var last_nonzero: usize = 0;
    for (config.tam, 0..) |row, index| {
        for (row) |gain| {
            if (!std.math.isFinite(gain)) @compileError(name ++ ": TAM gain is not finite");
        }
        if (!rowIsZero(row)) last_nonzero = index + 1;
    }
    if (last_nonzero != config.tam_len) {
        @compileError(name ++ ": tam_len does not match the last nonzero TAM row");
    }

    const pid = .{ config.odometry.kp, config.odometry.ki, config.odometry.kd };
    inline for (pid) |gains| {
        for (gains) |gain| {
            if (!std.math.isFinite(gain)) @compileError(name ++ ": odometry gain is not finite");
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

    const forward: math.Vector3f = .{
        config.camera_forward[0],
        config.camera_forward[1],
        config.camera_forward[2],
    };
    if (!(math.length3f(forward) > 1e-4)) {
        @compileError(name ++ ": camera_forward has zero length");
    }

    var tam_rows: [thruster_count]math.Vector6f = undefined;
    for (0..config.tam_len) |i| tam_rows[i] = config.tam[i];
    const active = tam_rows[0..config.tam_len];
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
    const limits: math.Vector3f = .{
        config.rotation_limit[0],
        config.rotation_limit[1],
        config.rotation_limit[2],
    };
    // `coverForce` only swings a thruster that already carries force, so a missing axis is
    // probed as `axis + thrust`, not as a pure unit command on the missing axis.
    inline for (axes, axis_names, 0..) |axis, axis_name, i| {
        const direct = forces.alongThrust(axis);
        if (@abs(direct[i]) > 1e-3) continue;
        var recovered = false;
        for (forces.basis[0..forces.rank]) |thrust| {
            const covered = bc.coverForce(forces, moments, axis + thrust, limits, forward);
            if (@abs(covered.force[i]) > 1e-3) recovered = true;
        }
        if (!recovered) {
            @compileError(name ++ ": body " ++ axis_name ++ " force is not in the TAM and coverForce cannot recover it");
        }
    }
}

comptime {
    assertPreset("stonefish_sim", mission_profiles.stonefish_sim);
    assertPreset("proteus_hwd", mission_profiles.proteus_hwd);
}
