//! Host I/O: cross-platform AUV loading, binary packets, live config, and UDP/shared-memory telemetry.
const std = @import("std");
const builtin = @import("builtin");
const Io = std.Io;
const math = @import("math.zig");
const Auv = @import("Auv.zig");
const MissionConfig = @import("MissionConfig.zig");
const SharedMemory = @import("SharedMemory.zig");

pub const Config = MissionConfig;
pub const Math = math;
pub const TelemetryTransport = MissionConfig.TelemetryTransport;

pub const DynLib = if (builtin.os.tag == .windows) WindowsDynLib else std.DynLib;

pub fn loadError() [:0]const u8 {
    if (builtin.os.tag == .windows) return "LoadLibrary failed";
    return std.mem.span(std.c.dlerror()) orelse "NO_DL_ERROR";
}

const WindowsDynLib = struct {
    handle: std.os.windows.HMODULE,

    pub fn open(path: []const u8) error{ FileNotFound, NameTooLong, InvalidWtf8 }!WindowsDynLib {
        var wide: [std.os.windows.PATH_MAX_WIDE]u16 = undefined;
        const len = std.unicode.wtf8ToWtf16Le(wide[0 .. wide.len - 1], path) catch |err| switch (err) {
            error.InvalidWtf8 => return error.InvalidWtf8,
        };
        if (len >= wide.len) return error.NameTooLong;
        wide[len] = 0;
        const handle = load(wide[0..len :0]) orelse return error.FileNotFound;
        return .{ .handle = handle };
    }

    pub fn close(self: *WindowsDynLib) void {
        _ = FreeLibrary(self.handle);
        self.* = undefined;
    }

    pub fn lookup(self: *WindowsDynLib, comptime T: type, name: [:0]const u8) ?T {
        const symbol = GetProcAddress(self.handle, name.ptr) orelse return null;
        return @as(T, @ptrCast(@alignCast(symbol)));
    }
};

extern "kernel32" fn LoadLibraryExW(
    name: [*:0]const u16,
    file: ?std.os.windows.HANDLE,
    flags: u32,
) callconv(.winapi) ?std.os.windows.HMODULE;
extern "kernel32" fn LoadLibraryW(name: [*:0]const u16) callconv(.winapi) ?std.os.windows.HMODULE;
extern "kernel32" fn GetFullPathNameW(
    name: [*:0]const u16,
    buf_len: u32,
    buf: [*]u16,
    file_part: ?*?[*:0]u16,
) callconv(.winapi) u32;
extern "kernel32" fn FreeLibrary(module: std.os.windows.HMODULE) callconv(.winapi) std.os.windows.BOOL;
extern "kernel32" fn GetProcAddress(
    module: std.os.windows.HMODULE,
    name: [*:0]const u8,
) callconv(.winapi) ?*anyopaque;

const dll_load_dir: u32 = 0x00000100;
const dll_default_dirs: u32 = 0x00001000;

fn load(path: [*:0]const u16) ?std.os.windows.HMODULE {
    var has_dir = path[0] != 0 and path[1] == ':';
    var i: usize = 0;
    while (path[i] != 0) : (i += 1) {
        if (path[i] == '\\' or path[i] == '/') has_dir = true;
    }
    if (!has_dir) return LoadLibraryW(path);

    var full: [std.os.windows.PATH_MAX_WIDE]u16 = undefined;
    const n = GetFullPathNameW(path, full.len, &full, null);
    if (n == 0 or n >= full.len)
        return LoadLibraryExW(path, null, dll_load_dir | dll_default_dirs);
    full[n] = 0;
    return LoadLibraryExW(full[0..n :0].ptr, null, dll_load_dir | dll_default_dirs);
}

pub const SerialBinary = struct {
    // Little-endian field order. Slices use a u8 length followed by their items.
    // Struct fields omit C layout padding.
    pub fn write(w: *Io.Writer, value: anytype) !void {
        const T = @TypeOf(value);
        switch (@typeInfo(T)) {
            .int => {
                var buf: [@sizeOf(T)]u8 = undefined;
                std.mem.writeInt(T, &buf, value, .little);
                try w.writeAll(&buf);
            },
            .float => try write(w, @as(u32, @bitCast(value))),
            .bool => try write(w, @as(u8, @intFromBool(value))),
            .@"enum" => |info| try write(w, @as(info.tag_type, @backingInt(value))),
            .array => |info| {
                if (info.child == u8) {
                    try w.writeAll(&value);
                } else {
                    for (value) |item| try write(w, item);
                }
            },
            .vector => |info| {
                const array: [info.len]info.child = value;
                try write(w, array);
            },
            .pointer => |info| {
                if (info.size != .slice) @compileError("only slices can be written");
                if (value.len > std.math.maxInt(u8)) return error.TooLong;
                try write(w, @as(u8, @intCast(value.len)));
                if (info.child == u8) {
                    try w.writeAll(value);
                } else {
                    for (value) |item| try write(w, item);
                }
            },
            .@"struct" => |info| inline for (info.field_names) |name| {
                try write(w, @field(value, name));
            },
            else => @compileError("unsupported serial type " ++ @typeName(T)),
        }
    }

    pub fn read(r: *Io.Reader, comptime T: type) !T {
        switch (@typeInfo(T)) {
            .int => {
                var buf: [@sizeOf(T)]u8 = undefined;
                try r.readSliceAll(&buf);
                return std.mem.readInt(T, &buf, .little);
            },
            .float => return @bitCast(try read(r, u32)),
            .bool => return try read(r, u8) != 0,
            .@"enum" => |info| {
                const raw = try read(r, info.tag_type);
                return std.enums.fromInt(T, raw) orelse return error.InvalidEnum;
            },
            .array => |info| {
                if (info.child == u8) {
                    var out: T = undefined;
                    try r.readSliceAll(&out);
                    return out;
                }
                var out: T = undefined;
                for (&out) |*item| item.* = try read(r, info.child);
                return out;
            },
            .vector => |info| {
                var array: [info.len]info.child = undefined;
                inline for (0..info.len) |i| array[i] = try read(r, info.child);
                return @bitCast(array);
            },
            .pointer => |info| {
                if (info.size != .slice) @compileError("only slices can be read");
                const len: u8 = try read(r, u8);
                if (info.child == u8) {
                    var buf: [256]u8 = undefined;
                    try r.readSliceAll(buf[0..len]);
                    return buf[0..len];
                }
                @compileError("only u8 slices can be read from pointer");
            },
            .@"struct" => |info| {
                var out: T = undefined;
                inline for (info.field_names, info.field_types) |name, field_type| {
                    @field(out, name) = try read(r, field_type);
                }
                return out;
            },
            else => @compileError("unsupported serial type " ++ @typeName(T)),
        }
    }

    pub fn readFromBuffer(comptime T: type, buf: []const u8) !struct { value: T, consumed: usize } {
        var stream: Io.Reader = .fixed(buf);
        const value = try read(&stream, T);
        return .{ .value = value, .consumed = stream.seek };
    }
};

pub const MissionDiynLoader = struct {
    auv_path: []const u8,
    dynlib: DynLib,

    pub fn init(auv_path: []const u8) MissionDiynLoader {
        return .{
            .auv_path = auv_path,
            .dynlib = undefined,
        };
    }

    pub fn load(loader: *MissionDiynLoader) !Auv {
        loader.dynlib = try DynLib.open(loader.auv_path);
        errdefer loader.dynlib.close();

        return .{
            .init_impl = loader.dynlib.lookup(
                *const Auv.InitFunc,
                "auv_init",
            ) orelse return error.MissingAuvInit,
            .yield_until_next_frame_impl = loader.dynlib.lookup(
                *const Auv.YieldUntilNextFrameFunc,
                "auv_yield_until_next_frame",
            ) orelse return error.MissingAuvYieldNextFrame,
            .set_thrustor_values_impl = loader.dynlib.lookup(
                *const Auv.SetThrustorValuesFunc,
                "auv_set_thrustor_values",
            ) orelse return error.MissingAuvSetThrustorsInput,
            .deinit_impl = loader.dynlib.lookup(
                *const Auv.DeinitFunc,
                "auv_deinit",
            ) orelse return error.MissingAuvDeinit,
        };
    }
};

pub const ReadPacket = struct {
    magic: [4]u8 = magic_value,
    wire_version: u16 = wire_version_value,
    camera_forward: [3]f32,
    rotation_limit: [3]f32,
    tam: [MissionConfig.thruster_count][MissionConfig.tam_cols]f32,
    tam_len: u8,
    odometry: MissionConfig.OdometryPid,
    ibvs: MissionConfig.Ibvs,
    goal_dist_threshold: f32,
    thrustor_saturate: f32,
    log_freq_div: u16,
    queue_capacity: u16,
    print_stuff: bool,
    clear_print_stuff: bool,
    telemetry_transport: TelemetryTransport,
    telemetry_addr: MissionConfig.FixedText(MissionConfig.telemetry_addr_max),
    telemetry_shm_name: MissionConfig.FixedText(MissionConfig.telemetry_shm_name_max),
    telemetry_log_path: MissionConfig.FixedText(MissionConfig.telemetry_log_path_max),

    const wire_version_value: u16 = 2;
    const magic_value: [4]u8 = "MCF1".*;

    const DecodeError = error{
        BufferTooShort,
        BadMagic,
        UnsupportedWireVersion,
        InvalidEnum,
        TooLong,
    };

    const ValidateError = error{
        InvalidTamLen,
        InvalidQueueCapacity,
        InvalidInteractionCols,
        InvalidTextLength,
        NonFinite,
    };

    fn decode(buf: []const u8) DecodeError!ReadPacket {
        if (buf.len < 4 + 2) return error.BufferTooShort;
        if (!std.mem.eql(u8, buf[0..4], &magic_value)) return error.BadMagic;
        const version = std.mem.readInt(u16, buf[4..6], .little);
        if (version != wire_version_value) return error.UnsupportedWireVersion;

        const parsed = SerialBinary.readFromBuffer(ReadPacket, buf) catch return error.BufferTooShort;
        if (!std.mem.eql(u8, &parsed.value.magic, &magic_value)) return error.BadMagic;
        if (parsed.value.wire_version != wire_version_value) return error.UnsupportedWireVersion;
        return parsed.value;
    }

    fn requireFinite(v: f32) ValidateError!void {
        if (!std.math.isFinite(v)) return error.NonFinite;
    }

    fn validateTam(
        tam: *const [MissionConfig.thruster_count][MissionConfig.tam_cols]f32,
        tam_len: u8,
    ) ValidateError!void {
        for (tam, 0..) |row, i| {
            if (i >= tam_len) break;
            for (row) |v| try requireFinite(v);
        }
    }

    fn validateOdometry(odometry: *const MissionConfig.OdometryPid) ValidateError!void {
        try requireFinite(odometry.rotation_gate);
        for (odometry.kp) |v| try requireFinite(v);
        for (odometry.ki) |v| try requireFinite(v);
        for (odometry.kd) |v| try requireFinite(v);
    }

    fn validateIbvs(ibvs: *const MissionConfig.Ibvs) ValidateError!void {
        if (ibvs.interaction_cols == 0 or ibvs.interaction_cols > MissionConfig.max_ibvs_error_cols)
            return error.InvalidInteractionCols;
        try requireFinite(ibvs.center_deadband);
        try requireFinite(ibvs.target_height);
        for (&ibvs.interaction) |row| {
            for (row[0..ibvs.interaction_cols]) |v| try requireFinite(v);
        }
    }

    fn validate(packet: *const ReadPacket) ValidateError!void {
        if (packet.tam_len == 0 or packet.tam_len > MissionConfig.thruster_count)
            return error.InvalidTamLen;
        if (packet.queue_capacity == 0 or packet.queue_capacity > MissionConfig.queue_capacity_max)
            return error.InvalidQueueCapacity;
        try validateTam(&packet.tam, packet.tam_len);
        try validateOdometry(&packet.odometry);
        try validateIbvs(&packet.ibvs);
        try requireFinite(packet.goal_dist_threshold);
        try requireFinite(packet.thrustor_saturate);
        _ = try packet.telemetry_addr.slice();
        if (packet.telemetry_transport == .shared_memory) {
            const shm_name = try packet.telemetry_shm_name.slice();
            if (shm_name.len == 0) return error.InvalidTextLength;
        }
        _ = try packet.telemetry_log_path.slice();
    }

    pub fn fromConfig(config: *const MissionConfig.MissionConfig) ReadPacket {
        return .{
            .magic = magic_value,
            .wire_version = wire_version_value,
            .camera_forward = config.camera_forward,
            .rotation_limit = config.rotation_limit,
            .tam = config.tam,
            .tam_len = config.tam_len,
            .odometry = config.odometry,
            .ibvs = config.ibvs,
            .goal_dist_threshold = config.goal_dist_threshold,
            .thrustor_saturate = config.thrustor_saturate,
            .log_freq_div = config.log_freq_div,
            .queue_capacity = config.queue_capacity,
            .print_stuff = config.print_stuff,
            .clear_print_stuff = config.clear_print_stuff,
            .telemetry_transport = config.telemetry_transport,
            .telemetry_addr = config.telemetry_addr,
            .telemetry_shm_name = config.telemetry_shm_name,
            .telemetry_log_path = config.telemetry_log_path,
        };
    }

    fn mergeInto(dst: *MissionConfig.MissionConfig, packet: ReadPacket) void {
        const teleop_port = dst.teleop_port;

        dst.camera_forward = packet.camera_forward;
        dst.rotation_limit = packet.rotation_limit;
        dst.tam = packet.tam;
        dst.tam_len = packet.tam_len;
        dst.odometry = packet.odometry;
        dst.ibvs = packet.ibvs;
        dst.goal_dist_threshold = packet.goal_dist_threshold;
        dst.thrustor_saturate = packet.thrustor_saturate;
        dst.log_freq_div = packet.log_freq_div;
        dst.queue_capacity = packet.queue_capacity;
        dst.print_stuff = packet.print_stuff;
        dst.clear_print_stuff = packet.clear_print_stuff;
        dst.telemetry_transport = packet.telemetry_transport;
        dst.telemetry_addr = packet.telemetry_addr;
        dst.telemetry_shm_name = packet.telemetry_shm_name;
        dst.telemetry_log_path = packet.telemetry_log_path;

        dst.teleop_port = teleop_port;
    }
};

pub const Goal = struct {
    position: math.Vector3f,
    look_at: math.Vector3f,
};

pub const WritePacket = struct {
    magic: [4]u8 = "AUV1".*,
    timestamp: u64,
    camera_pose: math.Pose,
    position_ok: Auv.AxisOk,
    attitude_ok: Auv.AxisOk,
    depth_ok: bool,
    depth: f32,
    objects: []const Auv.Object,
    boxes: []const Auv.Object2d,
    thrusters: []const f32,
    step: []const u8,
    force_error: math.Vector3f,
    goals: []const Goal,
};

/// Values needed to emit the controller's detailed diagnostics away from the
/// control thread. This deliberately owns every value; it contains no slices.
pub const ControlDebugSnapshot = struct {
    clear_screen: bool,
    cycle_us: u64,
    has_dt: bool,
    dt: f32,
    timestamp_ns: u64,
    kp: math.Vector6f,
    ki: math.Vector6f,
    kd: math.Vector6f,
    tam: [MissionConfig.thruster_count]math.Vector6f,
    tam_len: u8,
    pose: math.Vector6f,
    position_ok: Auv.AxisOk,
    attitude_ok: Auv.AxisOk,
    translation_error: math.Vector3f,
    target_orientation: math.Quaternionf,
    target_quaternion: math.Quaternionf,
    attitude_error: math.Vector3f,
    rotation_limit: math.Vector3f,
    input: math.Vector6f,
    thrusters: [MissionConfig.thruster_count]f32,
    thrusters_len: u8,
};

const IoEvent = struct {
    frame: TelemetryFrame,
    control_debug: ?ControlDebugSnapshot,
};

pub const telemetry_step_max = 64;
pub const telemetry_ring_slot_count = 32;
pub const telemetry_frame_abi_version: u32 = 1;

pub const TelemetryPose = extern struct {
    pos: [3]f32,
    quat: [4]f32,
};

pub const TelemetryObject = extern struct {
    pose: TelemetryPose,
    size: [3]f32,
    id: u32,
    cls: u8,
    _pad: [3]u8,
};

pub const TelemetryPoint2u = extern struct {
    x: u32,
    y: u32,
};

pub const TelemetryObject2d = extern struct {
    top_left: TelemetryPoint2u,
    bottom_right: TelemetryPoint2u,
    id: u32,
    cls: u8,
    _pad: [3]u8,
};

pub const TelemetryAxisOk = extern struct {
    x: bool,
    y: bool,
    z: bool,
    _pad: u8,
};

pub const TelemetryGoal = extern struct {
    position: [3]f32,
    look_at: [3]f32,
};

/// Fixed-size shared-memory representation of `WritePacket`.
/// All variable-length data is stored inline so a consumer can read it by pointer cast.
pub const TelemetryFrame = extern struct {
    magic: [4]u8,
    abi_version: u32,
    timestamp: u64,
    camera_pose: TelemetryPose,
    position_ok: TelemetryAxisOk,
    attitude_ok: TelemetryAxisOk,
    depth_ok: bool,
    _pad0: u8,
    _pad1: u16,
    depth: f32,
    objects_len: u16,
    boxes_len: u16,
    thrusters_len: u8,
    step_len: u8,
    goals_len: u16,
    objects: [Auv.Frame.max_objects]TelemetryObject,
    boxes: [Auv.Frame.max_objects]TelemetryObject2d,
    thrusters: [MissionConfig.thruster_count]f32,
    step: [telemetry_step_max]u8,
    force_error: [3]f32,
    goals: [MissionConfig.queue_capacity_max]TelemetryGoal,

    pub fn writeFromPacket(frame: *TelemetryFrame, packet: WritePacket) void {
        frame.* = std.mem.zeroes(TelemetryFrame);
        frame.magic = "AUV1".*;
        frame.abi_version = telemetry_frame_abi_version;
        frame.timestamp = packet.timestamp;
        frame.camera_pose = .{
            .pos = packet.camera_pose.pos,
            .quat = packet.camera_pose.quat,
        };
        frame.position_ok = .{
            .x = packet.position_ok.x,
            .y = packet.position_ok.y,
            .z = packet.position_ok.z,
            ._pad = 0,
        };
        frame.attitude_ok = .{
            .x = packet.attitude_ok.x,
            .y = packet.attitude_ok.y,
            .z = packet.attitude_ok.z,
            ._pad = 0,
        };
        frame.depth_ok = packet.depth_ok;
        frame.depth = packet.depth;
        frame.force_error = packet.force_error;

        const objects_len = @min(packet.objects.len, frame.objects.len);
        frame.objects_len = @intCast(objects_len);
        for (packet.objects[0..objects_len], frame.objects[0..objects_len]) |source, *dest| {
            dest.* = .{
                .pose = .{ .pos = source.bbox.pose.pos, .quat = source.bbox.pose.quat },
                .size = source.bbox.size,
                .id = source.id,
                .cls = source.cls,
                ._pad = .{ 0, 0, 0 },
            };
        }

        const boxes_len = @min(packet.boxes.len, frame.boxes.len);
        frame.boxes_len = @intCast(boxes_len);
        for (packet.boxes[0..boxes_len], frame.boxes[0..boxes_len]) |source, *dest| {
            dest.* = .{
                .top_left = .{ .x = source.top_left.x, .y = source.top_left.y },
                .bottom_right = .{ .x = source.bottom_right.x, .y = source.bottom_right.y },
                .id = source.id,
                .cls = source.cls,
                ._pad = .{ 0, 0, 0 },
            };
        }

        const thrusters_len = @min(packet.thrusters.len, frame.thrusters.len);
        frame.thrusters_len = @intCast(thrusters_len);
        @memcpy(frame.thrusters[0..thrusters_len], packet.thrusters[0..thrusters_len]);

        const step_len = @min(packet.step.len, frame.step.len);
        frame.step_len = @intCast(step_len);
        @memcpy(frame.step[0..step_len], packet.step[0..step_len]);

        const goals_len = @min(packet.goals.len, frame.goals.len);
        frame.goals_len = @intCast(goals_len);
        for (packet.goals[0..goals_len], frame.goals[0..goals_len]) |source, *dest| {
            dest.* = .{ .position = source.position, .look_at = source.look_at };
        }
    }

    pub fn fromWritePacket(packet: WritePacket) TelemetryFrame {
        var frame: TelemetryFrame = undefined;
        frame.writeFromPacket(packet);
        return frame;
    }
};

fn layoutMix(h: u64, value: usize) u64 {
    return h *% 1_000_003 +% @as(u64, @intCast(value));
}

pub const telemetry_layout_fingerprint: u64 = blk: {
    var h: u64 = telemetry_frame_abi_version;
    h = layoutMix(h, @sizeOf(TelemetryFrame));
    h = layoutMix(h, @alignOf(TelemetryFrame));
    h = layoutMix(h, @offsetOf(TelemetryFrame, "camera_pose"));
    h = layoutMix(h, @offsetOf(TelemetryFrame, "objects"));
    h = layoutMix(h, @offsetOf(TelemetryFrame, "boxes"));
    h = layoutMix(h, @offsetOf(TelemetryFrame, "goals"));
    break :blk h;
};

const RingHeader = extern struct {
    magic: [4]u8,
    abi_version: u32,
    layout_fingerprint: u64,
    owner_pid: u64,
    slot_count: u32,
    slot_size: u32,
    published_seq: u64,
};

const RingSlot = extern struct {
    seq: u64,
    frame: TelemetryFrame,
};

const RingMemory = extern struct {
    header: RingHeader,
    slots: [telemetry_ring_slot_count]RingSlot,
};

fn publishTelemetryFrame(ring: *RingMemory, frame: TelemetryFrame) void {
    const previous = @atomicLoad(u64, &ring.header.published_seq, .monotonic);
    const seq = previous +% 1;
    const slot = &ring.slots[seq % telemetry_ring_slot_count];
    @atomicStore(u64, &slot.seq, seq *% 2 -% 1, .release);
    slot.frame = frame;
    @atomicStore(u64, &slot.seq, seq *% 2, .release);
    @atomicStore(u64, &ring.header.published_seq, seq, .release);
}

fn ownerPid() u64 {
    if (builtin.os.tag == .windows) return 0;
    return @intCast(std.c.getpid());
}

fn ownerAlive(pid: u64) bool {
    if (pid == 0) return false;
    if (builtin.os.tag == .windows) return true;
    const result = std.c.kill(@intCast(pid), @fromBackingInt(@intCast(0)));
    return result == 0 or std.c.errno(result) == .PERM;
}

fn staleSharedMemory(name: []const u8) bool {
    var existing = SharedMemory.open(name, @sizeOf(RingMemory)) catch return false;
    defer existing.deinit();
    const ring: *const RingMemory = @ptrCast(@alignCast(existing.memory.ptr));
    validateRingHeader(&ring.header) catch return false;
    if (ring.header.owner_pid == 0) return ring.header.published_seq != 0;
    return !ownerAlive(ring.header.owner_pid);
}

comptime {
    if (@offsetOf(RingHeader, "published_seq") % @alignOf(u64) != 0)
        @compileError("shared telemetry sequence must be naturally aligned");
    if (@offsetOf(RingSlot, "frame") % @alignOf(TelemetryFrame) != 0)
        @compileError("shared telemetry frame must be naturally aligned");
}

pub const LatestFrame = struct {
    frame: *const TelemetryFrame,
    slot: *const RingSlot,
    expected_seq: u64,

    /// Call after consuming `frame`; false means the producer wrapped onto this slot.
    pub fn valid(self: LatestFrame) bool {
        return @atomicLoad(u64, &self.slot.seq, .acquire) == self.expected_seq;
    }
};

pub const TelemetrySubscriber = struct {
    shared: SharedMemory,
    ring: *const RingMemory,

    pub fn open(name: []const u8) !TelemetrySubscriber {
        var shared = try SharedMemory.open(name, @sizeOf(RingMemory));
        errdefer shared.deinit();
        const ring: *const RingMemory = @ptrCast(@alignCast(shared.memory.ptr));
        try validateRingHeader(&ring.header);
        return .{ .shared = shared, .ring = ring };
    }

    pub fn deinit(self: *TelemetrySubscriber) void {
        self.shared.deinit();
        self.* = undefined;
    }

    pub fn latest(self: *const TelemetrySubscriber) ?LatestFrame {
        const seq = @atomicLoad(u64, &self.ring.header.published_seq, .acquire);
        if (seq == 0) return null;
        const slot = &self.ring.slots[seq % telemetry_ring_slot_count];
        const expected = seq *% 2;
        if (@atomicLoad(u64, &slot.seq, .acquire) != expected) return null;
        return .{ .frame = &slot.frame, .slot = slot, .expected_seq = expected };
    }
};

fn validateRingHeader(header: *const RingHeader) !void {
    if (!std.mem.eql(u8, &header.magic, "AUVS")) return error.BadMagic;
    if (header.abi_version != telemetry_frame_abi_version) return error.UnsupportedAbiVersion;
    if (header.layout_fingerprint != telemetry_layout_fingerprint) return error.LayoutMismatch;
    if (header.slot_count != telemetry_ring_slot_count or header.slot_size != @sizeOf(RingSlot))
        return error.LayoutMismatch;
}

const live_config_recv_buf_len = 4096;
const outbound_buf_len = 64 * 1024;

pub const UdpPublisher = struct {
    socket: Io.net.Socket,
    dest: Io.net.IpAddress,

    fn init(io: Io, config: MissionConfig.MissionConfig) !UdpPublisher {
        const dest = try Io.net.IpAddress.parseLiteral(try config.telemetryAddr());
        const bind_addr: Io.net.IpAddress = .{ .ip4 = .unspecified(0) };
        return .{
            .socket = try bind_addr.bind(io, .{ .mode = .dgram }),
            .dest = dest,
        };
    }

    fn publishEncoded(self: *UdpPublisher, io: Io, encoded: []const u8) void {
        self.socket.send(io, &self.dest, encoded) catch {};
    }

    fn deinit(self: *UdpPublisher, io: Io) void {
        self.socket.close(io);
        self.* = undefined;
    }
};

pub const SharedMemoryPublisher = struct {
    shared: SharedMemory,
    ring: *RingMemory,

    fn init(name: []const u8) !SharedMemoryPublisher {
        var shared = SharedMemory.create(name, @sizeOf(RingMemory)) catch |err| switch (err) {
            error.AlreadyExists => blk: {
                if (!staleSharedMemory(name)) return err;
                SharedMemory.unlinkName(name);
                break :blk try SharedMemory.create(name, @sizeOf(RingMemory));
            },
            else => return err,
        };
        errdefer {
            shared.unlink();
            shared.deinit();
        }
        const ring: *RingMemory = @ptrCast(@alignCast(shared.memory.ptr));
        ring.header = .{
            .magic = "AUVS".*,
            .abi_version = telemetry_frame_abi_version,
            .layout_fingerprint = telemetry_layout_fingerprint,
            .owner_pid = ownerPid(),
            .slot_count = telemetry_ring_slot_count,
            .slot_size = @sizeOf(RingSlot),
            .published_seq = 0,
        };
        return .{ .shared = shared, .ring = ring };
    }

    fn publishFrame(self: *SharedMemoryPublisher, frame: TelemetryFrame) void {
        publishTelemetryFrame(self.ring, frame);
    }

    fn deinit(self: *SharedMemoryPublisher) void {
        self.shared.unlink();
        self.shared.deinit();
        self.* = undefined;
    }
};

pub const TelemetryPublisher = union(TelemetryTransport) {
    udp: UdpPublisher,
    shared_memory: SharedMemoryPublisher,

    fn init(io: Io, config: MissionConfig.MissionConfig) !TelemetryPublisher {
        return switch (config.telemetry_transport) {
            .udp => .{ .udp = try .init(io, config) },
            .shared_memory => .{ .shared_memory = try .init(try config.telemetryShmName()) },
        };
    }

    fn publishEncoded(self: *TelemetryPublisher, io: Io, frame: TelemetryFrame, encoded: ?[]const u8) void {
        switch (self.*) {
            .udp => |*publisher| if (encoded) |bytes| publisher.publishEncoded(io, bytes),
            .shared_memory => |*publisher| publisher.publishFrame(frame),
        }
    }

    fn deinit(self: *TelemetryPublisher, io: Io) void {
        switch (self.*) {
            .udp => |*publisher| publisher.deinit(io),
            .shared_memory => |*publisher| publisher.deinit(),
        }
        self.* = undefined;
    }
};

pub fn encodeWritePacket(buf: []u8, output: WritePacket) ![]const u8 {
    var writer: Io.Writer = .fixed(buf);
    var packet = output;
    packet.objects = packet.objects[0..@min(packet.objects.len, std.math.maxInt(u8))];
    packet.boxes = packet.boxes[0..@min(packet.boxes.len, std.math.maxInt(u8))];
    packet.thrusters = packet.thrusters[0..@min(packet.thrusters.len, std.math.maxInt(u8))];
    packet.step = packet.step[0..@min(packet.step.len, std.math.maxInt(u8))];
    packet.goals = packet.goals[0..@min(packet.goals.len, std.math.maxInt(u8))];
    try SerialBinary.write(&writer, packet);
    return writer.buffered();
}

pub fn encodeMissionCommand(buf: []u8, command: MissionCommand) ![]const u8 {
    var writer: Io.Writer = .fixed(buf);
    var packet = command;
    try packet.validate();
    try SerialBinary.write(&writer, packet);
    return writer.buffered();
}

fn encodeTelemetryFrame(buf: []u8, frame: *const TelemetryFrame) ![]const u8 {
    var objects: [Auv.Frame.max_objects]Auv.Object = undefined;
    for (frame.objects[0..frame.objects_len], objects[0..frame.objects_len]) |source, *dest| {
        dest.* = .{
            .bbox = .{
                .pose = .{ .pos = source.pose.pos, .quat = source.pose.quat },
                .size = source.size,
            },
            .id = source.id,
            .cls = source.cls,
        };
    }

    var boxes: [Auv.Frame.max_objects]Auv.Object2d = undefined;
    for (frame.boxes[0..frame.boxes_len], boxes[0..frame.boxes_len]) |source, *dest| {
        dest.* = .{
            .top_left = .{ .x = source.top_left.x, .y = source.top_left.y },
            .bottom_right = .{ .x = source.bottom_right.x, .y = source.bottom_right.y },
            .id = source.id,
            .cls = source.cls,
        };
    }

    var goals: [MissionConfig.queue_capacity_max]Goal = undefined;
    for (frame.goals[0..frame.goals_len], goals[0..frame.goals_len]) |source, *dest| {
        dest.* = .{ .position = source.position, .look_at = source.look_at };
    }

    return encodeWritePacket(buf, .{
        .magic = frame.magic,
        .timestamp = frame.timestamp,
        .camera_pose = .{ .pos = frame.camera_pose.pos, .quat = frame.camera_pose.quat },
        .position_ok = .{ .x = frame.position_ok.x, .y = frame.position_ok.y, .z = frame.position_ok.z },
        .attitude_ok = .{ .x = frame.attitude_ok.x, .y = frame.attitude_ok.y, .z = frame.attitude_ok.z },
        .depth_ok = frame.depth_ok,
        .depth = frame.depth,
        .objects = objects[0..frame.objects_len],
        .boxes = boxes[0..frame.boxes_len],
        .thrusters = frame.thrusters[0..frame.thrusters_len],
        .step = frame.step[0..frame.step_len],
        .force_error = frame.force_error,
        .goals = goals[0..frame.goals_len],
    });
}

fn printControlDebug(snapshot: ControlDebugSnapshot) void {
    if (snapshot.clear_screen) std.debug.print("\x1b[2J\x1b[H", .{});
    std.log.debug("done in {} us", .{snapshot.cycle_us});
    std.log.debug("control:", .{});
    math.debug6f("\tkp", snapshot.kp);
    math.debug6f("\tki", snapshot.ki);
    math.debug6f("\tkd", snapshot.kd);
    for (snapshot.tam[0..snapshot.tam_len]) |row| math.debug6f("\ttam", row);
    if (snapshot.has_dt) {
        std.log.debug("dt = {d:6.2} ms", .{snapshot.dt * 1000});
    } else {
        std.log.debug("dt = unavailable", .{});
    }
    std.log.debug("timestamp_ns = {d}", .{snapshot.timestamp_ns});
    math.debug6f("pose", snapshot.pose);
    std.log.debug("position ok: {any} attitude ok: {any}", .{ snapshot.position_ok, snapshot.attitude_ok });
    math.debug3f("translation error", snapshot.translation_error);
    std.log.debug("target orientation: {any}", .{snapshot.target_orientation});
    math.debug3f("attitude error body", snapshot.attitude_error);
    math.debug3f("rotation limit", snapshot.rotation_limit);
    std.log.debug("target quaternion: {any}", .{snapshot.target_quaternion});
    math.debug6f("\tinput", snapshot.input);
    std.log.debug("\tthruster_values = {any}", .{snapshot.thrusters[0..snapshot.thrusters_len]});
}

fn processTelemetry(
    io: Io,
    publisher: *TelemetryPublisher,
    log_file: ?Io.File,
    event: IoEvent,
    encoded_buf: *[outbound_buf_len]u8,
) void {
        if (event.control_debug) |debug| printControlDebug(debug);

        const encoded: ?[]const u8 = if (publisher.* == .udp or log_file != null)
            encodeTelemetryFrame(encoded_buf, &event.frame) catch |err| blk: {
                std.log.err("telemetry encode failed: {s}", .{@errorName(err)});
                break :blk null;
            }
        else
            null;

        publisher.publishEncoded(io, event.frame, encoded);
        if (log_file) |file| {
            if (encoded) |bytes| file.writeStreamingAll(io, bytes) catch |err| {
                std.log.err("telemetry log write failed: {s}", .{@errorName(err)});
            };
        }
}

/// Decode the legacy `AUV1` UDP/log representation into the same fixed frame
/// used by shared memory. This keeps viewer code independent of slice lifetimes.
pub fn decodeWritePacket(buf: []const u8, frame: *TelemetryFrame) !void {
    var reader: Io.Reader = .fixed(buf);
    frame.* = std.mem.zeroes(TelemetryFrame);
    frame.magic = try SerialBinary.read(&reader, [4]u8);
    if (!std.mem.eql(u8, &frame.magic, "AUV1")) return error.BadMagic;
    frame.abi_version = telemetry_frame_abi_version;
    frame.timestamp = try SerialBinary.read(&reader, u64);
    const pose = try SerialBinary.read(&reader, math.Pose);
    frame.camera_pose = .{ .pos = pose.pos, .quat = pose.quat };
    const position_ok = try SerialBinary.read(&reader, Auv.AxisOk);
    frame.position_ok = .{
        .x = position_ok.x,
        .y = position_ok.y,
        .z = position_ok.z,
        ._pad = 0,
    };
    const attitude_ok = try SerialBinary.read(&reader, Auv.AxisOk);
    frame.attitude_ok = .{
        .x = attitude_ok.x,
        .y = attitude_ok.y,
        .z = attitude_ok.z,
        ._pad = 0,
    };
    frame.depth_ok = try SerialBinary.read(&reader, bool);
    frame.depth = try SerialBinary.read(&reader, f32);

    const objects_len = try SerialBinary.read(&reader, u8);
    frame.objects_len = @intCast(@min(@as(usize, objects_len), frame.objects.len));
    for (0..objects_len) |i| {
        const source = try SerialBinary.read(&reader, Auv.Object);
        if (i < frame.objects.len) frame.objects[i] = .{
            .pose = .{ .pos = source.bbox.pose.pos, .quat = source.bbox.pose.quat },
            .size = source.bbox.size,
            .id = source.id,
            .cls = source.cls,
            ._pad = .{ 0, 0, 0 },
        };
    }

    const boxes_len = try SerialBinary.read(&reader, u8);
    frame.boxes_len = @intCast(@min(@as(usize, boxes_len), frame.boxes.len));
    for (0..boxes_len) |i| {
        const source = try SerialBinary.read(&reader, Auv.Object2d);
        if (i < frame.boxes.len) frame.boxes[i] = .{
            .top_left = .{ .x = source.top_left.x, .y = source.top_left.y },
            .bottom_right = .{ .x = source.bottom_right.x, .y = source.bottom_right.y },
            .id = source.id,
            .cls = source.cls,
            ._pad = .{ 0, 0, 0 },
        };
    }

    const thrusters_len = try SerialBinary.read(&reader, u8);
    frame.thrusters_len = @intCast(@min(@as(usize, thrusters_len), frame.thrusters.len));
    for (0..thrusters_len) |i| {
        const value = try SerialBinary.read(&reader, f32);
        if (i < frame.thrusters.len) frame.thrusters[i] = value;
    }

    const step_len = try SerialBinary.read(&reader, u8);
    frame.step_len = @intCast(@min(@as(usize, step_len), frame.step.len));
    for (0..step_len) |i| {
        const byte = try SerialBinary.read(&reader, u8);
        if (i < frame.step.len) frame.step[i] = byte;
    }
    frame.force_error = try SerialBinary.read(&reader, math.Vector3f);

    // Goals were appended after the original packet format; accept old senders.
    if (reader.seek == buf.len) return;
    const goals_len = try SerialBinary.read(&reader, u8);
    frame.goals_len = @intCast(@min(@as(usize, goals_len), frame.goals.len));
    for (0..goals_len) |i| {
        const source = try SerialBinary.read(&reader, Goal);
        if (i < frame.goals.len)
            frame.goals[i] = .{ .position = source.position, .look_at = source.look_at };
    }
}

pub const TeleopCommand = struct {
    translation: [3]f32,
    orientation: [4]f32,
};

pub const MissionCommandKind = enum(u8) {
    point,
    object,
};

pub const MissionManeuver = enum(u8) {
    move,
    pass,
    circle,
};

pub const MissionCommand = struct {
    magic: [4]u8 = mission_command_magic,
    wire_version: u16 = mission_command_wire_version,
    kind: MissionCommandKind,
    maneuver: MissionManeuver,
    position: math.Vector3f = @splat(0),
    look_direction: math.Vector3f = .{ 0, 0, 1 },
    object_class: u8 = 0,

    pub const DecodeError = error{
        BufferTooShort,
        InvalidLength,
        BadMagic,
        UnsupportedWireVersion,
        InvalidEnum,
        NonFinite,
        InvalidLookDirection,
        InvalidPointManeuver,
    };

    pub fn decode(buf: []const u8) DecodeError!MissionCommand {
        if (buf.len < 6) return error.BufferTooShort;
        if (!std.mem.eql(u8, buf[0..4], &mission_command_magic)) return error.BadMagic;
        if (std.mem.readInt(u16, buf[4..6], .little) != mission_command_wire_version)
            return error.UnsupportedWireVersion;

        const parsed = SerialBinary.readFromBuffer(MissionCommand, buf) catch |err| switch (err) {
            error.InvalidEnum => return error.InvalidEnum,
            else => return error.BufferTooShort,
        };
        if (parsed.consumed != buf.len) return error.InvalidLength;

        var command = parsed.value;
        try command.validate();
        return command;
    }

    pub fn validate(self: *MissionCommand) DecodeError!void {
        if (!std.mem.eql(u8, &self.magic, &mission_command_magic)) return error.BadMagic;
        if (self.wire_version != mission_command_wire_version) return error.UnsupportedWireVersion;
        if (self.kind == .point and self.maneuver != .move)
            return error.InvalidPointManeuver;

        inline for (0..3) |i| {
            if (!std.math.isFinite(self.position[i])) return error.NonFinite;
            if (!std.math.isFinite(self.look_direction[i])) return error.NonFinite;
        }

        if (self.kind == .point) {
            const length = math.length3f(self.look_direction);
            if (!std.math.isFinite(length) or length <= 1e-6)
                return error.InvalidLookDirection;
            self.look_direction /= @as(math.Vector3f, @splat(length));
        }
    }
};

const mission_command_magic: [4]u8 = "AUM1".*;
const mission_command_wire_version: u16 = 1;

pub const Telemetry = struct {
    publisher: TelemetryPublisher,
    log_file: ?Io.File,
    encoded_buf: [outbound_buf_len]u8 = undefined,
    control_socket: Io.net.Socket = undefined,
    teleop_command: ?TeleopCommand = null,
    mission_command: ?MissionCommand = null,
    read_packet: ReadPacket = undefined,
    initial_read_packet: ReadPacket = undefined,

    fn syncConfigFields(self: *Telemetry, config: MissionConfig.MissionConfig) void {
        self.read_packet = ReadPacket.fromConfig(&config);
    }

    pub fn init(_: std.mem.Allocator, io: Io, config: MissionConfig.MissionConfig) !Telemetry {
        var publisher = try TelemetryPublisher.init(io, config);
        errdefer publisher.deinit(io);

        var control_addr_buf: [48]u8 = undefined;
        const control_addr_text = try std.fmt.bufPrint(&control_addr_buf, "0.0.0.0:{d}", .{config.teleop_port});
        const control_bind = try Io.net.IpAddress.parseLiteral(control_addr_text);
        const control_socket = control_bind.bind(io, .{ .mode = .dgram, .protocol = .udp }) catch |err| switch (err) {
            error.AddressInUse => {
                std.log.err("control UDP port {d} is already in use", .{config.teleop_port});
                return error.AddressInUse;
            },
            else => return err,
        };
        errdefer control_socket.close(io);

        const log_file: ?Io.File = if (try config.telemetryLogPath()) |path|
            try Io.Dir.cwd().createFile(io, path, .{})
        else
            null;
        errdefer if (log_file) |file| file.close(io);

        var telemetry: Telemetry = .{
            .publisher = publisher,
            .log_file = log_file,
            .control_socket = control_socket,
            .initial_read_packet = ReadPacket.fromConfig(&config),
        };
        telemetry.syncConfigFields(config);
        return telemetry;
    }

    pub fn deinit(self: *Telemetry, io: Io) void {
        if (self.log_file) |file| file.close(io);
        self.publisher.deinit(io);
        self.control_socket.close(io);
        self.* = undefined;
    }

    pub fn warnInitOnlyFields(self: *const Telemetry, config: MissionConfig.MissionConfig) void {
        const init_telemetry = self.initial_read_packet.telemetry_addr.slice() catch "";
        const new_telemetry = config.telemetryAddr() catch {
            std.log.err("live config has invalid telemetry_addr", .{});
            return;
        };
        if (!std.mem.eql(u8, new_telemetry, init_telemetry)) {
            std.log.warn("config telemetry_addr changed; restart required", .{});
        }
        if (config.telemetry_transport != self.initial_read_packet.telemetry_transport) {
            std.log.warn("config telemetry_transport changed; restart required", .{});
        }
        const init_shm_name = self.initial_read_packet.telemetry_shm_name.slice() catch "";
        const new_shm_name = config.telemetryShmName() catch {
            std.log.err("live config has invalid telemetry_shm_name", .{});
            return;
        };
        if (!std.mem.eql(u8, new_shm_name, init_shm_name)) {
            std.log.warn("config telemetry_shm_name changed; restart required", .{});
        }
        const init_log_path = self.initial_read_packet.telemetry_log_path.slice() catch "";
        const new_log_path = config.telemetryLogPath() catch {
            std.log.err("live config has invalid telemetry_log_path", .{});
            return;
        } orelse "";
        if (init_log_path.len != new_log_path.len or !std.mem.eql(u8, new_log_path, init_log_path)) {
            std.log.warn("config telemetry_log_path changed; restart required", .{});
        }
    }

    pub fn takeTeleopCommand(self: *Telemetry) ?TeleopCommand {
        const command = self.teleop_command;
        self.teleop_command = null;
        return command;
    }

    pub fn takeMissionCommand(self: *Telemetry) ?MissionCommand {
        const command = self.mission_command;
        self.mission_command = null;
        return command;
    }

    pub fn pollLiveConfig(
        self: *Telemetry,
        io: Io,
        base: MissionConfig.MissionConfig,
        print_stuff: bool,
        merged_out: *MissionConfig.MissionConfig,
    ) bool {
        const timeout: std.Io.Timeout = .{ .duration = .{
            .clock = .awake,
            .raw = .fromNanoseconds(0),
        } };
        var recv_buf: [live_config_recv_buf_len]u8 = undefined;
        var applied = false;
        var working = base;
        while (true) {
            const msg = self.control_socket.receiveTimeout(io, &recv_buf, timeout) catch |err| switch (err) {
                error.Timeout => break,
                else => {
                    if (print_stuff) std.log.err("live config UDP recv: {s}", .{@errorName(err)});
                    break;
                },
            };
            if (msg.data.len >= mission_command_magic.len and
                std.mem.eql(u8, msg.data[0..mission_command_magic.len], &mission_command_magic))
            {
                self.mission_command = MissionCommand.decode(msg.data) catch |err| {
                    if (print_stuff) std.log.debug("mission command rejected: {s}", .{@errorName(err)});
                    continue;
                };
                continue;
            }
            if (msg.data.len == 7 * @sizeOf(f32)) {
                var values: [7]f32 = undefined;
                for (0..values.len) |i| {
                    const bits = std.mem.readInt(u32, msg.data[i * 4 ..][0..4], .little);
                    values[i] = @bitCast(bits);
                }
                self.teleop_command = .{
                    .translation = .{ values[0], values[1], values[2] },
                    .orientation = .{ values[3], values[4], values[5], values[6] },
                };
                continue;
            }
            const packet = ReadPacket.decode(msg.data) catch |err| {
                if (print_stuff) std.log.debug("live config UDP decode: {s}", .{@errorName(err)});
                continue;
            };
            ReadPacket.validate(&packet) catch |err| {
                std.log.warn("live config UDP rejected: {s}", .{@errorName(err)});
                continue;
            };
            ReadPacket.mergeInto(&working, packet);
            if (print_stuff) std.log.debug("applied live config UDP on control port", .{});
            applied = true;
        }
        if (applied) {
            self.syncConfigFields(working);
            merged_out.* = working;
        }
        return applied;
    }

    pub fn pushToIoQueue(
        self: *Telemetry,
        io: Io,
        output: WritePacket,
        control_debug: ?ControlDebugSnapshot,
    ) void {
        var frame: TelemetryFrame = undefined;
        frame.writeFromPacket(output);
        processTelemetry(io, &self.publisher, self.log_file, .{
            .frame = frame,
            .control_debug = control_debug,
        }, &self.encoded_buf);
    }
};

test "mission command round trips and normalizes point direction" {
    var buf: [64]u8 = undefined;
    const encoded = try encodeMissionCommand(&buf, .{
        .kind = .point,
        .maneuver = .move,
        .position = .{ 1, 2, 3 },
        .look_direction = .{ 0, 0, 5 },
    });
    const decoded = try MissionCommand.decode(encoded);

    try std.testing.expectEqual(MissionCommandKind.point, decoded.kind);
    try std.testing.expectEqual(MissionManeuver.move, decoded.maneuver);
    try std.testing.expectEqual(@as(f32, 1), decoded.position[0]);
    try std.testing.expectEqual(@as(f32, 2), decoded.position[1]);
    try std.testing.expectEqual(@as(f32, 3), decoded.position[2]);
    try std.testing.expectEqual(@as(f32, 1), decoded.look_direction[2]);
}

test "mission command rejects invalid point maneuvers and directions" {
    var buf: [64]u8 = undefined;
    const encoded = try encodeMissionCommand(&buf, .{
        .kind = .point,
        .maneuver = .move,
        .position = .{ 1, 2, 3 },
        .look_direction = .{ 0, 0, 1 },
    });

    buf[7] = @backingInt(MissionManeuver.pass);
    try std.testing.expectError(error.InvalidPointManeuver, MissionCommand.decode(encoded));

    buf[7] = @backingInt(MissionManeuver.move);
    for (20..32) |i| buf[i] = 0;
    try std.testing.expectError(error.InvalidLookDirection, MissionCommand.decode(encoded));
}

test "mission command accepts every object maneuver" {
    var buf: [64]u8 = undefined;
    for (std.enums.values(MissionManeuver)) |maneuver| {
        const encoded = try encodeMissionCommand(&buf, .{
            .kind = .object,
            .maneuver = maneuver,
            .object_class = 2,
        });
        const decoded = try MissionCommand.decode(encoded);
        try std.testing.expectEqual(MissionCommandKind.object, decoded.kind);
        try std.testing.expectEqual(maneuver, decoded.maneuver);
        try std.testing.expectEqual(@as(u8, 2), decoded.object_class);
    }
}
