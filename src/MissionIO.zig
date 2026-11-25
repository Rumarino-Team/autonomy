const std = @import("std");
const builtin = @import("builtin");
const Io = std.Io;
const math = @import("math.zig");
const Auv = @import("Auv.zig");
const MissionConfig = @import("MissionConfig.zig");
const reflection = @import("simulator/ReflectionWriter.zig");

pub const LiveConfigPacket = struct {
    revision: u64 = 0,
    config: MissionConfig.MissionConfig,

    pub fn fromRuntimeConfig(config: *const MissionConfig.MissionConfig) LiveConfigPacket {
        return .{ .config = config.* };
    }

    pub fn validate(self: *const LiveConfigPacket) !void {
        try self.config.validate();
    }
};

pub const LiveConfigDoc = struct {
    revision: u64,
    config: MissionConfig.MissionConfig,
};
pub const live_config_schema_hash = reflection.documentSchemaHash(LiveConfigDoc);

pub const RuntimeState = enum(u8) { idle, starting, running, stopping };
pub const RuntimeResult = enum(u8) { none, completed, stopped, failed };
pub const RuntimeAction = enum(u8) { query, start, stop, restart };

pub const RuntimeRequestRow = struct {
    client_id: u64,
    request_id: u64,
    action: RuntimeAction,
};
pub const RuntimeRequest = RuntimeRequestRow;
pub const RuntimeRequestDoc = struct { request: RuntimeRequestRow };

pub const RuntimeStatusRow = struct {
    state: RuntimeState,
    result: RuntimeResult,
    revision: u64,
    accepted_client_id: u64,
    accepted_request_id: u64,
    config_reply_revision: u64,
    config_rejected: bool,
    active: bool,
    desired: MissionConfig.MissionConfig,
    active_config: MissionConfig.MissionConfig,
    last_error: MissionConfig.FixedText(256),
    run_id: u64,
};
pub const RuntimeStatus = RuntimeStatusRow;
pub const RuntimeStatusDoc = struct { status: RuntimeStatusRow };

pub const MissionCommandKind = enum(u8) { point, object };
pub const MissionManeuver = enum(u8) { move, pass, circle };

pub const MissionCommandRow = struct {
    kind: MissionCommandKind,
    maneuver: MissionManeuver,
    position: [3]f32,
    look_direction: [3]f32,
    object_class: u8 = 0,

    pub fn validate(self: *MissionCommandRow) !void {
        if (self.kind == .point and self.maneuver != .move) return error.InvalidPointManeuver;
        for (self.position) |c| if (!std.math.isFinite(c)) return error.NonFinite;
        for (self.look_direction) |c| if (!std.math.isFinite(c)) return error.NonFinite;
        if (self.kind == .point) {
            var dir: math.Vector3f = .{ self.look_direction[0], self.look_direction[1], self.look_direction[2] };
            const length = math.length3f(dir);
            if (!std.math.isFinite(length) or length <= 1e-6) return error.InvalidLookDirection;
            dir /= @as(math.Vector3f, @splat(length));
            self.look_direction = .{ @floatCast(dir[0]), @floatCast(dir[1]), @floatCast(dir[2]) };
        }
    }
};
pub const MissionCommandDoc = struct { command: MissionCommandRow };

pub fn matchesControlDocument(comptime Doc: type, bytes: []const u8) bool {
    return reflection.matchesDocument(Doc, bytes);
}

pub fn decodeLiveConfig(bytes: []const u8) !LiveConfigPacket {
    const doc = try reflection.read(LiveConfigDoc, bytes);
    try doc.config.validate();
    return .{ .revision = doc.revision, .config = doc.config };
}

pub fn decodeRuntimeRequest(bytes: []const u8) !RuntimeRequest {
    return (try reflection.read(RuntimeRequestDoc, bytes)).request;
}

pub fn decodeRuntimeStatus(bytes: []const u8) !RuntimeStatus {
    const doc = try reflection.read(RuntimeStatusDoc, bytes);
    try doc.status.desired.validate();
    return doc.status;
}

pub fn decodeMissionCommand(bytes: []const u8) !MissionCommandRow {
    var row = (try reflection.read(MissionCommandDoc, bytes)).command;
    try row.validate();
    return row;
}

/// The compiled profile definitions shared by the runtime and viewer.
pub const CompiledConfig = MissionConfig;
pub const Math = math;
pub const TelemetryTransport = CompiledConfig.TelemetryTransport;

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

        const loaded: Auv = .{
            .init_impl = loader.dynlib.lookup(
                *const Auv.InitFunc,
                "auv_init",
            ) orelse return error.MissingAuvInit,
            .yield_until_next_frame_impl = loader.dynlib.lookup(
                *const Auv.YieldUntilNextFrameFunc,
                "auv_yield_until_next_frame",
            ) orelse return error.MissingAuvYieldUntilNextFrame,
            .deinit_impl = loader.dynlib.lookup(
                *const Auv.DeinitFunc,
                "auv_deinit",
            ) orelse return error.MissingAuvDeinit,
            .stonefish_temporal_thrusters = loader.dynlib.lookup(
                *const Auv.StonefishTemporalSetThrusters,
                @import("StonefishTemporalBridge.zig").symbol,
            ),
        };
        return loaded;
    }
};

pub const Goal = struct {
    position: math.Vector3f,
    look_at: math.Vector3f,
};

pub const telemetry_step_max = 64;

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

pub const StdoutPublisher = struct {
    file: Io.File,
    clear_before_frame: bool,

    fn init(io: Io) !StdoutPublisher {
        const file = Io.File.stdout();
        return .{
            .file = file,
            .clear_before_frame = try file.isTty(io),
        };
    }

    fn publishEncoded(self: *StdoutPublisher, io: Io, bytes: []const u8) void {
        if (self.clear_before_frame) {
            self.file.writeStreamingAll(io, "\x1b[2J\x1b[H") catch |err| {
                std.log.err("telemetry stdout clear failed: {s}", .{@errorName(err)});
                return;
            };
        }
        self.file.writeStreamingAll(io, bytes) catch |err| {
            std.log.err("telemetry stdout write failed: {s}", .{@errorName(err)});
        };
    }

    fn deinit(self: *StdoutPublisher) void {
        // stdout is owned by the process, not by the telemetry publisher.
        self.* = undefined;
    }
};

/// Runtime-owned transport selected from the compiled startup configuration.
pub const TelemetryPublisher = union(TelemetryTransport) {
    udp: UdpPublisher,
    stdout: StdoutPublisher,
    disabled: void,

    fn init(io: Io, config: MissionConfig.MissionConfig) !TelemetryPublisher {
        return switch (config.runtime.telemetry_transport) {
            .udp => .{ .udp = try .init(io, config) },
            .stdout => .{ .stdout = try .init(io) },
            .disabled => .disabled,
        };
    }

    fn publishEncoded(
        self: *TelemetryPublisher,
        io: Io,
        encoded: ?[]const u8,
        ascii_encoded: ?[]const u8,
    ) void {
        switch (self.*) {
            .udp => |*publisher| if (encoded) |bytes| publisher.publishEncoded(io, bytes),
            .stdout => |*publisher| if (ascii_encoded) |bytes| publisher.publishEncoded(io, bytes),
            .disabled => {},
        }
    }

    fn deinit(self: *TelemetryPublisher, io: Io) void {
        switch (self.*) {
            .udp => |*publisher| publisher.deinit(io),
            .stdout => |*publisher| publisher.deinit(),
            .disabled => {},
        }
        self.* = undefined;
    }
};

pub fn missionCommandToRow(command: MissionCommand) MissionCommandRow {
    return .{
        .kind = command.kind,
        .maneuver = command.maneuver,
        .position = .{
            @floatCast(command.position[0]),
            @floatCast(command.position[1]),
            @floatCast(command.position[2]),
        },
        .look_direction = .{
            @floatCast(command.look_direction[0]),
            @floatCast(command.look_direction[1]),
            @floatCast(command.look_direction[2]),
        },
        .object_class = command.object_class,
    };
}

pub fn missionCommandFromRow(row: MissionCommandRow) MissionCommand {
    return .{
        .kind = row.kind,
        .maneuver = row.maneuver,
        .position = .{ row.position[0], row.position[1], row.position[2] },
        .look_direction = .{ row.look_direction[0], row.look_direction[1], row.look_direction[2] },
        .object_class = row.object_class,
    };
}

pub fn encodeControl(buf: []align(8) u8, value: anytype) ![]const u8 {
    const V = @TypeOf(value);
    const wrapped = if (V == LiveConfigPacket)
        LiveConfigDoc{ .revision = value.revision, .config = value.config }
    else if (V == RuntimeRequest)
        RuntimeRequestDoc{ .request = value }
    else if (V == RuntimeStatus)
        RuntimeStatusDoc{ .status = value }
    else if (V == MissionCommand) blk: {
        var cmd = value;
        try cmd.validate();
        break :blk MissionCommandDoc{ .command = missionCommandToRow(cmd) };
    } else
        value;
    var fallback = std.heap.FixedBufferAllocator.init(buf);
    return reflection.write(fallback.allocator(), @TypeOf(wrapped), wrapped) catch error.NoSpaceLeft;
}

pub fn encodeTelemetryFrame(buf: []align(8) u8, frame: anytype) ![]const u8 {
    return encodeControl(buf, frame);
}

pub fn decodeTelemetryFrame(comptime Doc: type, buf: []const u8) !Doc {
    return reflection.read(Doc, buf) catch error.UnsupportedAbiVersion;
}

fn roundDisplayFloat(value: anytype) @TypeOf(value) {
    const T = @TypeOf(value);
    const scale: T = 100000.0;
    return @round(value * scale) / scale;
}

fn writeAsciiFloatList(writer: *Io.Writer, values: anytype) !void {
    try writer.writeAll("[");
    for (values, 0..) |value, i| {
        if (i != 0) try writer.writeAll(",");
        try writer.print("{d:.5}", .{roundDisplayFloat(value)});
    }
    try writer.writeAll("]");
}

fn writeProjected(writer: *Io.Writer, comptime prefix: []const u8, value: anytype) !void {
    const T = @TypeOf(value);
    switch (@typeInfo(T)) {
        .int, .bool => try writer.print("{s}={}\n", .{ prefix, value }),
        .float => try writer.print("{s}={d:.5}\n", .{ prefix, roundDisplayFloat(value) }),
        .array => |arr| {
            if (arr.child == u8) {
                try writer.print("{s}={s}\n", .{ prefix, std.mem.sliceTo(&value, 0) });
                return;
            }
            if (arr.child == f32 or arr.child == f64) {
                try writer.print("{s}=", .{prefix});
                try writeAsciiFloatList(writer, value);
                try writer.writeAll("\n");
                return;
            }
            if (arr.len > 8) {
                try writer.print("{s}.len={d}\n", .{ prefix, arr.len });
                for (value) |item| {
                    try writeBare(writer, item);
                    try writer.writeAll("\n");
                }
                return;
            }
            inline for (0..arr.len) |i| {
                try writeProjected(writer, prefix ++ std.fmt.comptimePrint("[{}]", .{i}), value[i]);
            }
        },
        .@"struct" => {
            inline for (@typeInfo(T).@"struct".field_names) |name| {
                try writeProjected(writer, prefix ++ "." ++ name, @field(value, name));
            }
        },
        .pointer => |ptr| {
            if (ptr.size != .slice) {
                try writer.print("{s}=?\n", .{prefix});
                return;
            }
            if (ptr.child == u8) {
                try writer.print("{s}={s}\n", .{ prefix, value });
                return;
            }
            try writer.print("{s}.len={}\n", .{ prefix, value.len });
            for (value) |item| {
                try writeBare(writer, item);
                try writer.writeAll("\n");
            }
        },
        else => try writer.print("{s}=?\n", .{prefix}),
    }
}

fn writeBare(writer: *Io.Writer, value: anytype) !void {
    const T = @TypeOf(value);
    switch (@typeInfo(T)) {
        .int, .bool => try writer.print("{}", .{value}),
        .float => try writer.print("{d:.5}", .{roundDisplayFloat(value)}),
        .array => |arr| {
            if (arr.child == f32 or arr.child == f64) {
                try writeAsciiFloatList(writer, value);
                return;
            }
            try writer.writeAll("[");
            inline for (0..arr.len) |i| {
                if (i != 0) try writer.writeAll(",");
                try writeBare(writer, value[i]);
            }
            try writer.writeAll("]");
        },
        .@"struct" => |info| {
            try writer.writeAll("{");
            inline for (info.field_names, 0..) |name, i| {
                if (i != 0) try writer.writeAll(" ");
                try writer.print("{s}=", .{name});
                try writeBare(writer, @field(value, name));
            }
            try writer.writeAll("}");
        },
        else => try writer.writeAll("?"),
    }
}

pub fn encodeTelemetryFrameAscii(buf: []u8, frame: anytype) ![]const u8 {
    var writer: Io.Writer = .fixed(buf);
    try writer.writeAll("FRAME_BEGIN\n");
    inline for (@typeInfo(@TypeOf(frame)).@"struct".field_names) |name| {
        try writeProjected(&writer, name, @field(frame, name));
    }
    try writer.writeAll("FRAME_END\n");
    return writer.buffered();
}

fn processTelemetry(
    io: Io,
    publisher: *TelemetryPublisher,
    log_file: ?Io.File,
    frame: anytype,
    encoded_buf: *align(8) [outbound_buf_len]u8,
    ascii_buf: *[outbound_buf_len]u8,
    stdout_enabled: bool,
) void {
    const encoded: ?[]const u8 = if (publisher.* == .udp or log_file != null)
        encodeTelemetryFrame(encoded_buf, frame) catch |err| blk: {
            std.log.err("telemetry encode failed: {s}", .{@errorName(err)});
            break :blk null;
        }
    else
        null;
    const ascii_encoded: ?[]const u8 = if (publisher.* == .stdout and stdout_enabled)
        encodeTelemetryFrameAscii(ascii_buf, frame) catch |err| blk: {
            std.log.err("telemetry ASCII encode failed: {s}", .{@errorName(err)});
            break :blk null;
        }
    else
        null;

    publisher.publishEncoded(io, encoded, ascii_encoded);
    if (log_file) |file| {
        if (encoded) |bytes| file.writeStreamingAll(io, bytes) catch |err| {
            std.log.err("telemetry log write failed: {s}", .{@errorName(err)});
        };
    }
}

pub const MissionCommand = struct {
    kind: MissionCommandKind,
    maneuver: MissionManeuver,
    position: math.Vector3f = @splat(0),
    look_direction: math.Vector3f = .{ 0, 0, 1 },
    object_class: u8 = 0,

    pub fn validate(self: *MissionCommand) !void {
        var row = missionCommandToRow(self.*);
        try row.validate();
        self.look_direction = .{ row.look_direction[0], row.look_direction[1], row.look_direction[2] };
    }
};

pub const Telemetry = struct {
    state: RuntimeState = .idle,
    result: RuntimeResult = .none,
    revision: u64 = 1,
    last_error: MissionConfig.FixedText(256) = .{},
    telemetry_error: MissionConfig.FixedText(256) = .{},
    run_id: u64 = 0,
    direct_values: [MissionConfig.thruster_count]f32 = @splat(0),
    direct_len: u8 = 0,
    direct_last_received: ?Io.Clock.Timestamp = null,
    teleop_packet: [1024]u8 = undefined,
    teleop_packet_len: usize = 0,
    dna_queue: [8][1024]u8 = undefined,
    dna_len: [8]u16 = @splat(0),
    dna_count: u8 = 0,
    dna_read: u8 = 0,
    mission_command: ?MissionCommand = null,
    pending_lifecycle: ?error{ MissionRestart, MissionStopped } = null,
    accepted_client_id: u64 = 0,
    accepted_request_id: u64 = 0,
    clients: [16]struct { id: u64 = 0, request: u64 = 0 } = @splat(.{}),
    client_slot: usize = 0,

    publisher: TelemetryPublisher,
    log_file: ?Io.File,
    encoded_buf: [outbound_buf_len]u8 align(8) = undefined,
    ascii_buf: [outbound_buf_len]u8 = undefined,
    stdout_log_counter: u64 = 0,
    stdout_log_freq_div: u16,
    control_socket: ?Io.net.Socket = null,

    initial_config: MissionConfig.MissionConfig = undefined,

    pub fn init(io: Io, config: MissionConfig.MissionConfig, enable_udp_control: bool) !Telemetry {
        var publisher = try TelemetryPublisher.init(io, config);
        errdefer publisher.deinit(io);

        const control_socket: ?Io.net.Socket = if (enable_udp_control) blk: {
            var control_addr_buf: [48]u8 = undefined;
            const control_addr_text = try std.fmt.bufPrint(&control_addr_buf, "0.0.0.0:{d}", .{config.runtime.teleop_port});
            const control_bind = try Io.net.IpAddress.parseLiteral(control_addr_text);
            break :blk control_bind.bind(io, .{ .mode = .dgram, .protocol = .udp }) catch |err| switch (err) {
                error.AddressInUse => {
                    std.log.err("control UDP port {d} is already in use", .{config.runtime.teleop_port});
                    return error.AddressInUse;
                },
                else => return err,
            };
        } else null;
        errdefer if (control_socket) |socket| socket.close(io);

        const log_file: ?Io.File = if (try config.telemetryLogPath()) |path|
            try Io.Dir.cwd().createFile(io, path, .{})
        else
            null;
        errdefer if (log_file) |file| file.close(io);

        const telemetry: Telemetry = .{
            .publisher = publisher,
            .log_file = log_file,
            .stdout_log_freq_div = config.log_freq_div,
            .control_socket = control_socket,
            .initial_config = config,
        };
        return telemetry;
    }

    pub fn deinit(self: *Telemetry, io: Io) void {
        if (self.log_file) |file| file.close(io);
        self.publisher.deinit(io);
        if (self.control_socket) |socket| socket.close(io);
        self.* = undefined;
    }

    /// Replace outputs transactionally; the listener never changes ownership.
    pub fn reconfigure(self: *Telemetry, io: Io, config: MissionConfig.MissionConfig) !void {
        const previous = self.initial_config;
        const publisher_changed = config.runtime.telemetry_transport != previous.runtime.telemetry_transport or
            (config.runtime.telemetry_transport == .udp and
                !MissionConfig.textEqual(config.runtime.telemetry_addr, previous.runtime.telemetry_addr));
        var replacement: ?TelemetryPublisher = if (publisher_changed) try TelemetryPublisher.init(io, config) else null;
        errdefer if (replacement) |*publisher| publisher.deinit(io);
        const log_changed = !MissionConfig.textEqual(config.runtime.telemetry_log_path, previous.runtime.telemetry_log_path);
        const log_file: ?Io.File = if (log_changed) blk: {
            if (try config.telemetryLogPath()) |path| break :blk try Io.Dir.cwd().createFile(io, path, .{ .truncate = false });
            break :blk null;
        } else self.log_file;
        if (replacement) |publisher| {
            self.publisher.deinit(io);
            self.publisher = publisher;
        }
        if (log_changed) {
            if (self.log_file) |file| file.close(io);
            self.log_file = log_file;
        }
        self.stdout_log_freq_div = config.log_freq_div;
        self.initial_config = config;
    }

    pub fn report(self: *Telemetry, err: anyerror) void {
        const detail = if (err == error.SensorInitFailed or err == error.SensorFrameFailed) Auv.lastError() else "";
        setText(&self.last_error, if (detail.len != 0) detail else @errorName(err));
        std.log.err("mission: {s}", .{self.last_error.slice() catch @errorName(err)});
    }

    pub fn clearCommands(self: *Telemetry) void {
        self.direct_len = 0;
        self.direct_last_received = null;
        self.teleop_packet_len = 0;
        self.dna_count = 0;
        self.mission_command = null;
    }

    pub fn storeDnaPacket(self: *Telemetry, bytes: []const u8) void {
        if (bytes.len == 0 or bytes.len > self.dna_queue[0].len) return;
        if (self.dna_count == self.dna_queue.len) {
            self.dna_read = (self.dna_read + 1) % @as(u8, @intCast(self.dna_queue.len));
            self.dna_count -= 1;
        }
        const slot = (self.dna_read + self.dna_count) % @as(u8, @intCast(self.dna_queue.len));
        @memcpy(self.dna_queue[slot][0..bytes.len], bytes);
        self.dna_len[slot] = @intCast(bytes.len);
        self.dna_count += 1;
    }

    pub fn takeDnaPacket(self: *Telemetry) ?[]const u8 {
        if (self.dna_count == 0) return null;
        const slot = self.dna_read;
        const len = self.dna_len[slot];
        self.dna_read = (self.dna_read + 1) % @as(u8, @intCast(self.dna_queue.len));
        self.dna_count -= 1;
        return self.dna_queue[slot][0..len];
    }

    pub fn takeTeleopPacket(self: *Telemetry) ?[]const u8 {
        if (self.teleop_packet_len == 0) return null;
        const packet = self.teleop_packet[0..self.teleop_packet_len];
        self.teleop_packet_len = 0;
        return packet;
    }

    pub fn storeTeleopPacket(self: *Telemetry, bytes: []const u8) void {
        if (self.state != .running or self.pending_lifecycle != null) return;
        if (bytes.len > self.teleop_packet.len) return;
        @memcpy(self.teleop_packet[0..bytes.len], bytes);
        self.teleop_packet_len = bytes.len;
    }

    pub fn takeMissionCommand(self: *Telemetry) ?MissionCommand {
        const command = self.mission_command;
        self.mission_command = null;
        return command;
    }

    pub fn applyConfig(self: *Telemetry, io: Io, current: *MissionConfig.MissionConfig, packet: LiveConfigPacket) !void {
        try packet.validate();
        if (packet.revision != self.revision) {
            var ack = LiveConfigPacket.fromRuntimeConfig(current);
            ack.revision = packet.revision;
            if (std.meta.eql(ack.config, packet.config) and ack.revision == packet.revision) return;
            return error.StaleConfig;
        }
        const next = packet.config;
        const restart = !MissionConfig.MissionConfig.operationallyEqual(current, &next);
        // Open replacement telemetry resources before committing the new config.
        try self.reconfigure(io, next);
        self.last_error = .{};
        self.telemetry_error = .{};
        current.* = next;
        self.revision += 1;
        if (restart) self.pending_lifecycle = error.MissionRestart;
    }

    pub fn acceptRequest(self: *Telemetry, request: RuntimeRequest) void {
        if (request.action == .query) {
            self.accepted_client_id = request.client_id;
            self.accepted_request_id = 0;
            for (self.clients) |client| if (client.id == request.client_id) {
                self.accepted_request_id = client.request;
                break;
            };
            return;
        }
        if (request.client_id == 0 or request.request_id == 0) {
            self.report(error.InvalidRequestId);
            return;
        }
        var slot: ?usize = null;
        for (self.clients, 0..) |client, i| if (client.id == request.client_id) {
            slot = i;
            break;
        };
        const index = slot orelse blk: {
            const index = self.client_slot;
            self.client_slot = (self.client_slot + 1) % self.clients.len;
            self.clients[index] = .{ .id = request.client_id };
            break :blk index;
        };
        if (request.request_id <= self.clients[index].request) {
            self.accepted_client_id = request.client_id;
            self.accepted_request_id = self.clients[index].request;
            return;
        }
        self.clients[index].request = request.request_id;
        self.accepted_client_id = request.client_id;
        self.accepted_request_id = request.request_id;
        switch (request.action) {
            .query => unreachable,
            .start => if (self.state == .idle or (if (self.pending_lifecycle) |pending| pending == error.MissionStopped else false)) {
                self.pending_lifecycle = error.MissionRestart;
            },
            .stop => self.pending_lifecycle = error.MissionStopped,
            .restart => self.pending_lifecycle = error.MissionRestart,
        }
    }

    pub fn status(self: *const Telemetry, current: *const MissionConfig.MissionConfig, config_reply_revision: u64, config_rejected: bool) RuntimeStatus {
        var active_value = current.*;
        active_value.runtime.telemetry_transport = self.initial_config.runtime.telemetry_transport;
        active_value.runtime.telemetry_addr = self.initial_config.runtime.telemetry_addr;
        active_value.runtime.telemetry_log_path = self.initial_config.runtime.telemetry_log_path;
        return .{
            .run_id = self.run_id,
            .state = if (self.pending_lifecycle != null and self.state != .idle) .stopping else self.state,
            .result = self.result,
            .revision = self.revision,
            .accepted_client_id = self.accepted_client_id,
            .accepted_request_id = self.accepted_request_id,
            .config_reply_revision = config_reply_revision,
            .config_rejected = config_rejected,
            .active = self.state != .idle,
            .desired = current.*,
            .active_config = active_value,
            .last_error = if (self.last_error.len != 0) self.last_error else self.telemetry_error,
        };
    }

    fn reply(self: *Telemetry, io: Io, current: *const MissionConfig.MissionConfig, destination: *const Io.net.IpAddress, config_reply_revision: u64, config_rejected: bool) void {
        const socket = self.control_socket orelse return;
        var buffer: [8192]u8 align(8) = undefined;
        var fallback = std.heap.FixedBufferAllocator.init(&buffer);
        const status_row = self.status(current, config_reply_revision, config_rejected);
        const bytes = reflection.write(fallback.allocator(), RuntimeStatusDoc, RuntimeStatusDoc{ .status = status_row }) catch return;
        socket.send(io, destination, bytes) catch {};
    }

    pub fn acceptThrusters(self: *Telemetry, io: Io, current: *const MissionConfig.MissionConfig, values: []const f32) void {
        if (self.state != .running or self.pending_lifecycle != null or
            current.mission_id != .thruster_control or values.len != current.tam_len or
            values.len > self.direct_values.len) return;
        for (values) |value| {
            if (!std.math.isFinite(value) or value < -1 or value > 1) return;
        }
        @memcpy(self.direct_values[0..values.len], values);
        self.direct_len = @intCast(values.len);
        self.direct_last_received = .now(io, .awake);
    }

    pub fn acceptMission(self: *Telemetry, input: MissionCommand) !void {
        var command = input;
        try command.validate();
        self.queueMission(command);
    }

    fn queueMission(self: *Telemetry, command: MissionCommand) void {
        if (self.state == .running and self.pending_lifecycle == null) self.mission_command = command;
    }

    /// Drain a bounded batch before unwinding, so only the latest requested
    /// configuration is opened by the next mission attempt.
    pub fn pollControl(self: *Telemetry, io: Io, current: *MissionConfig.MissionConfig) !void {
        if (self.direct_last_received) |last| {
            if (last.untilNow(io).raw.toMilliseconds() >= 2000) {
                setText(&self.last_error, "Thruster control connection timed out");
                self.clearCommands();
                return error.MissionStopped;
            }
        }
        var buffer: [4096]u8 = undefined;
        const timeout: Io.Timeout = .{ .duration = .{ .clock = .awake, .raw = .fromNanoseconds(0) } };
        if (self.control_socket) |socket| {
            for (0..64) |_| {
                const message = socket.receiveTimeout(io, &buffer, timeout) catch |err| switch (err) {
                    error.Timeout => break,
                    else => {
                        self.report(err);
                        break;
                    },
                };
                var config_reply_revision: u64 = 0;
                var config_rejected = false;
                if (reflection.matchesDocument(RuntimeRequestDoc, message.data)) {
                    const request = decodeRuntimeRequest(message.data) catch |err| {
                        self.report(err);
                        self.reply(io, current, &message.from, 0, false);
                        continue;
                    };
                    self.acceptRequest(request);
                } else if (reflection.matchesDocument(LiveConfigDoc, message.data)) {
                    const packet = decodeLiveConfig(message.data) catch |err| {
                        self.report(err);
                        self.reply(io, current, &message.from, 0, false);
                        continue;
                    };
                    config_reply_revision = packet.revision;
                    self.applyConfig(io, current, packet) catch |err| {
                        self.report(err);
                        config_rejected = true;
                    };
                } else if (reflection.matchesDocument(MissionCommandDoc, message.data)) {
                    const row = decodeMissionCommand(message.data) catch |err| {
                        self.report(err);
                        self.reply(io, current, &message.from, 0, false);
                        continue;
                    };
                    self.queueMission(missionCommandFromRow(row));
                    continue;
                } else if (reflection.dnaSectionHash(message.data) != null) {
                    self.storeDnaPacket(message.data);
                    continue;
                } else {
                    self.report(error.InvalidSchema);
                    continue;
                }
                self.reply(io, current, &message.from, config_reply_revision, config_rejected);
            }
        }
        const lifecycle = self.pending_lifecycle;
        self.pending_lifecycle = null;
        if (lifecycle) |err| {
            self.clearCommands();
            return err;
        }
    }

    /// Publish one runtime telemetry frame synchronously.
    pub fn publishTelemetry(
        self: *Telemetry,
        io: Io,
        frame: anytype,
    ) void {
        if (self.publisher == .disabled and self.log_file == null) return;
        self.stdout_log_counter +%= 1;
        const stdout_enabled = self.stdout_log_freq_div != 0 and
            self.stdout_log_counter % self.stdout_log_freq_div == 0;
        processTelemetry(
            io,
            &self.publisher,
            self.log_file,
            frame.*,
            &self.encoded_buf,
            &self.ascii_buf,
            stdout_enabled,
        );
    }
};


pub fn setText(destination: anytype, message: []const u8) void {
    destination.* = .{};
    const len = @min(destination.bytes.len, message.len);
    @memcpy(destination.bytes[0..len], message[0..len]);
    destination.len = @intCast(len);
}

test "telemetry sender round trip" {
    const Row = struct { depth: f64, flag: bool };
    const Sample = struct { row: Row };
    const frame = Sample{ .row = .{ .depth = 1.25, .flag = true } };
    var storage: [4096]u8 align(8) = undefined;
    const encoded = try encodeTelemetryFrame(&storage, frame);
    try std.testing.expectEqual(comptime reflection.documentSchemaHash(Sample), reflection.dnaSectionHash(encoded).?);
    const decoded = try decodeTelemetryFrame(Sample, encoded);
    try std.testing.expectEqual(frame.row.depth, decoded.row.depth);
    try std.testing.expect(decoded.row.flag);
    var bad = try std.testing.allocator.dupe(u8, encoded);
    defer std.testing.allocator.free(bad);
    bad[0] ^= 0xff;
    try std.testing.expectError(error.UnsupportedAbiVersion, decodeTelemetryFrame(Sample, bad));
}

test "telemetry accepts equivalent row layouts from another build" {
    const SenderRow = struct { depth: f64, flag: bool };
    const ViewerRow = struct { depth: f64, flag: bool };
    const ViewerDoc = struct { row: ViewerRow };
    var storage: [4096]u8 align(8) = undefined;
    const encoded = try encodeTelemetryFrame(&storage, .{ .row = SenderRow{ .depth = 2.5, .flag = true } });
    try std.testing.expect(reflection.dnaSectionHash(encoded) != comptime reflection.documentSchemaHash(ViewerDoc));
    const decoded = try decodeTelemetryFrame(ViewerDoc, encoded);
    try std.testing.expectEqual(@as(f64, 2.5), decoded.row.depth);
    try std.testing.expect(decoded.row.flag);

    const ChangedRow = struct { temperature: f64, flag: bool };
    const ChangedDoc = struct { row: ChangedRow };
    try std.testing.expectError(error.UnsupportedAbiVersion, decodeTelemetryFrame(ChangedDoc, encoded));
}

test "lifecycle requests deduplicate and the latest request wins" {
    var telemetry: Telemetry = .{ .publisher = undefined, .log_file = null, .stdout_log_freq_div = 1, .state = .running };
    telemetry.acceptRequest(.{ .client_id = 1, .request_id = 1, .action = .start });
    try std.testing.expect(telemetry.pending_lifecycle == null);
    telemetry.acceptRequest(.{ .client_id = 1, .request_id = 2, .action = .restart });
    try std.testing.expectEqual(error.MissionRestart, telemetry.pending_lifecycle.?);
    telemetry.acceptRequest(.{ .client_id = 1, .request_id = 3, .action = .stop });
    telemetry.acceptRequest(.{ .client_id = 1, .request_id = 2, .action = .restart });
    try std.testing.expectEqual(error.MissionStopped, telemetry.pending_lifecycle.?);
    telemetry.acceptRequest(.{ .client_id = 1, .request_id = 3, .action = .query });
    try std.testing.expectEqual(@as(u64, 3), telemetry.accepted_request_id);
    telemetry.acceptRequest(.{ .client_id = 1, .request_id = 4, .action = .start });
    try std.testing.expectEqual(error.MissionRestart, telemetry.pending_lifecycle.?);
}

test "direct control works without UDP and clears commands on restart" {
    const io = std.testing.io;
    var config = MissionConfig.startup_config;
    config.runtime.telemetry_transport = .disabled;
    var telemetry: Telemetry = try .init(io, config, false);
    defer telemetry.deinit(io);
    try std.testing.expect(telemetry.control_socket == null);

    telemetry.state = .running;
    try telemetry.acceptMission(.{ .kind = .point, .maneuver = .move, .look_direction = .{ 0, 4, 0 } });
    try std.testing.expectEqual(math.Vector3f{ 0, 1, 0 }, telemetry.takeMissionCommand().?.look_direction);

    var edit = LiveConfigPacket.fromRuntimeConfig(&config);
    edit.revision = telemetry.revision;
    edit.config.mission_id = .teleop;
    try telemetry.applyConfig(io, &config, edit);
    telemetry.storeTeleopPacket("not-a-frame");
    try std.testing.expect(telemetry.takeTeleopPacket() == null);
    try std.testing.expectError(error.MissionRestart, telemetry.pollControl(io, &config));
    try std.testing.expectEqual(MissionConfig.MissionId.teleop, config.mission_id);
}
