const std = @import("std");
const builtin = @import("builtin");
const Io = std.Io;
const math = @import("math.zig");
const Auv = @import("Auv.zig");
const MissionConfig = @import("MissionConfig.zig");

var stop_requested = std.atomic.Value(bool).init(false);

fn requestStop() void {
    stop_requested.store(true, .monotonic);
}

pub fn stopRequested() bool {
    return stop_requested.load(.monotonic);
}

const windows_ctrl_c_event: u32 = 0;
const windows_ctrl_break_event: u32 = 1;
const windows_ctrl_close_event: u32 = 2;
const windows_ctrl_logoff_event: u32 = 5;
const windows_ctrl_shutdown_event: u32 = 6;

fn windowsConsoleHandler(ctrl_type: u32) callconv(.winapi) std.os.windows.BOOL {
    switch (ctrl_type) {
        windows_ctrl_c_event,
        windows_ctrl_break_event,
        windows_ctrl_close_event,
        windows_ctrl_logoff_event,
        windows_ctrl_shutdown_event,
        => requestStop(),
        else => return .FALSE,
    }
    return .TRUE;
}

extern "kernel32" fn SetConsoleCtrlHandler(
    handler: ?*const fn (u32) callconv(.winapi) std.os.windows.BOOL,
    add: std.os.windows.BOOL,
) callconv(.winapi) std.os.windows.BOOL;

pub fn installStopHandlers() void {
    if (builtin.os.tag == .windows) {
        _ = SetConsoleCtrlHandler(&windowsConsoleHandler, .TRUE);
    } else {
        const stop: std.posix.Sigaction = .{
            .handler = .{ .handler = struct {
                fn handle(_: std.posix.SIG) callconv(.c) void {
                    requestStop();
                }
            }.handle },
            .mask = std.posix.sigemptyset(),
            .flags = 0,
        };
        std.posix.sigaction(.INT, &stop, null);
        std.posix.sigaction(.TERM, &stop, null);
    }
}

const SharedMemory = struct {
    const is_windows = builtin.os.tag == .windows;
    const page_alignment = std.heap.page_size_min;
    const NativeHandle = if (is_windows) std.os.windows.HANDLE else std.posix.fd_t;

    handle: NativeHandle,
    memory: []align(page_alignment) u8,
    owner: bool,
    name: [128:0]u8,
    name_len: usize,

    pub fn create(name: []const u8, size: usize) !SharedMemory {
        if (size == 0) return error.InvalidSize;
        var normalized: [128:0]u8 = undefined;
        const normalized_len = try normalizeName(name, &normalized);
        if (is_windows) {
            return createWindows(normalized, normalized_len, size);
        }
        return createPosix(normalized, normalized_len, size);
    }

    pub fn open(name: []const u8, size: usize) !SharedMemory {
        if (size == 0) return error.InvalidSize;
        var normalized: [128:0]u8 = undefined;
        const normalized_len = try normalizeName(name, &normalized);
        if (is_windows) {
            return openWindows(normalized, normalized_len, size);
        }
        return openPosix(normalized, normalized_len, size);
    }

    pub fn unlinkName(name: []const u8) void {
        var normalized: [128:0]u8 = undefined;
        const normalized_len = normalizeName(name, &normalized) catch return;
        if (is_windows) return;
        _ = std.c.shm_unlink(normalized[0..normalized_len :0].ptr);
    }

    pub fn deinit(self: *SharedMemory) void {
        if (is_windows) {
            _ = UnmapViewOfFile(self.memory.ptr);
            _ = CloseHandle(self.handle);
        } else {
            std.posix.munmap(self.memory);
            _ = std.c.close(self.handle);
        }
        self.* = undefined;
    }

    pub fn unlink(self: *SharedMemory) void {
        if (is_windows or !self.owner) return;
        _ = std.c.shm_unlink(self.name[0..self.name_len :0].ptr);
    }

    fn normalizeName(name: []const u8, out: *[128:0]u8) !usize {
        if (name.len == 0) return error.InvalidName;
        const prefix = if (is_windows) "Local\\" else "/";
        const skip_prefix = if (is_windows)
            std.mem.startsWith(u8, name, "Local\\")
        else
            std.mem.startsWith(u8, name, "/");
        const needed = name.len + if (skip_prefix) 0 else prefix.len;
        if (needed > out.len) return error.NameTooLong;
        var at: usize = 0;
        if (!skip_prefix) {
            @memcpy(out[0..prefix.len], prefix);
            at = prefix.len;
        }
        @memcpy(out[at..needed], name);
        out[needed] = 0;
        return needed;
    }

    fn createPosix(name: [128:0]u8, name_len: usize, size: usize) !SharedMemory {
        if (is_windows) unreachable;
        const zname = name[0..name_len :0];
        const flags: std.posix.O = .{ .ACCMODE = .RDWR, .CREAT = true, .EXCL = true };
        const fd = std.c.shm_open(zname.ptr, @bitCast(flags), 0o600);
        if (fd < 0) return if (std.c.errno(fd) == .EXIST) error.AlreadyExists else error.CreateFailed;
        errdefer _ = std.c.close(fd);
        if (std.c.ftruncate(fd, @intCast(size)) != 0) return error.ResizeFailed;
        const memory = try std.posix.mmap(
            null,
            size,
            .{ .READ = true, .WRITE = true },
            .{ .TYPE = .SHARED },
            fd,
            0,
        );
        errdefer std.posix.munmap(memory);
        @memset(memory, 0);
        return .{
            .handle = fd,
            .memory = memory,
            .owner = true,
            .name = name,
            .name_len = name_len,
        };
    }

    fn openPosix(name: [128:0]u8, name_len: usize, size: usize) !SharedMemory {
        if (is_windows) unreachable;
        const zname = name[0..name_len :0];
        const flags: std.posix.O = .{ .ACCMODE = .RDONLY };
        const fd = std.c.shm_open(zname.ptr, @bitCast(flags), 0);
        if (fd < 0) return error.OpenFailed;
        errdefer _ = std.c.close(fd);
        const memory = try std.posix.mmap(
            null,
            size,
            .{ .READ = true },
            .{ .TYPE = .SHARED },
            fd,
            0,
        );
        return .{
            .handle = fd,
            .memory = memory,
            .owner = false,
            .name = name,
            .name_len = name_len,
        };
    }

    const invalid_handle_value: std.os.windows.HANDLE = @ptrFromInt(std.math.maxInt(usize));
    const page_readwrite: u32 = 0x04;
    const file_map_all_access: u32 = 0x000f001f;
    const file_map_read: u32 = 0x00000004;

    fn createWindows(name: [128:0]u8, name_len: usize, size: usize) !SharedMemory {
        if (!is_windows) unreachable;
        const size64: u64 = @intCast(size);
        const handle = CreateFileMappingA(
            invalid_handle_value,
            null,
            page_readwrite,
            @intCast(size64 >> 32),
            @truncate(size64),
            name[0..name_len :0].ptr,
        ) orelse return error.CreateFailed;
        errdefer _ = CloseHandle(handle);
        if (GetLastError() == 183) return error.AlreadyExists;
        const raw = MapViewOfFile(handle, file_map_all_access, 0, 0, size) orelse return error.MapFailed;
        const memory = @as([*]align(page_alignment) u8, @ptrCast(@alignCast(raw)))[0..size];
        @memset(memory, 0);
        return .{
            .handle = handle,
            .memory = memory,
            .owner = true,
            .name = name,
            .name_len = name_len,
        };
    }

    fn openWindows(name: [128:0]u8, name_len: usize, size: usize) !SharedMemory {
        if (!is_windows) unreachable;
        const handle = OpenFileMappingA(file_map_read, .FALSE, name[0..name_len :0].ptr) orelse
            return error.OpenFailed;
        errdefer _ = CloseHandle(handle);
        const raw = MapViewOfFile(handle, file_map_read, 0, 0, size) orelse return error.MapFailed;
        const memory = @as([*]align(page_alignment) u8, @ptrCast(@alignCast(raw)))[0..size];
        return .{
            .handle = handle,
            .memory = memory,
            .owner = false,
            .name = name,
            .name_len = name_len,
        };
    }

    extern "kernel32" fn CreateFileMappingA(
        file: std.os.windows.HANDLE,
        attributes: ?*anyopaque,
        protect: u32,
        maximum_size_high: u32,
        maximum_size_low: u32,
        name: [*:0]const u8,
    ) callconv(.winapi) ?std.os.windows.HANDLE;
    extern "kernel32" fn OpenFileMappingA(
        desired_access: u32,
        inherit_handle: std.os.windows.BOOL,
        name: [*:0]const u8,
    ) callconv(.winapi) ?std.os.windows.HANDLE;
    extern "kernel32" fn MapViewOfFile(
        mapping: std.os.windows.HANDLE,
        desired_access: u32,
        offset_high: u32,
        offset_low: u32,
        bytes_to_map: usize,
    ) callconv(.winapi) ?*anyopaque;
    extern "kernel32" fn UnmapViewOfFile(address: *const anyopaque) callconv(.winapi) std.os.windows.BOOL;
    extern "kernel32" fn CloseHandle(handle: std.os.windows.HANDLE) callconv(.winapi) std.os.windows.BOOL;
    extern "kernel32" fn GetLastError() callconv(.winapi) u32;
};

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

pub const SerialBinary = struct {
    // Little-endian field order. Slices use a u8 length followed by their items.
    // Struct fields omit C layout padding.
    pub fn write(w: *Io.Writer, value: anytype) !void {
        const T = @TypeOf(value);
        switch (@typeInfo(T)) {
            .int => try w.writeInt(T, value, .little),
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
            .int => return try r.takeInt(T, .little),
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

/// Transport-neutral message kinds. UDP carries these as datagrams; TCP adds
/// a length prefix because TCP itself is a byte stream.
pub const LogicalMessageKind = enum(u8) {
    telemetry = 1,
    teleop = 2,
    mission_command = 3,
    live_config = 4,
    command_ack = 5,
};

pub const LogicalMessage = struct {
    kind: LogicalMessageKind,
    sequence: u32,
    payload: []const u8,
};

pub const CommandAckStatus = enum(u8) {
    rejected = 0,
    accepted = 1,
};

pub fn decodeCommandAck(payload: []const u8) !CommandAckStatus {
    if (payload.len != 1) return error.InvalidLength;
    return std.enums.fromInt(CommandAckStatus, payload[0]) orelse error.InvalidKind;
}

pub const LogicalMessageDecodeError = error{
    BufferTooShort,
    InvalidLength,
    BadMagic,
    InvalidKind,
    FrameTooLarge,
};

const logical_message_magic: [4]u8 = "AUVP".*;
const logical_message_header_len = 4 + 1 + 4 + 2;
pub const logical_message_max_len = 64 * 1024;

pub fn encodeLogicalMessage(
    buf: []u8,
    kind: LogicalMessageKind,
    sequence: u32,
    payload: []const u8,
) ![]const u8 {
    if (payload.len > std.math.maxInt(u16)) return error.MessageTooLarge;
    const total_len = logical_message_header_len + payload.len;
    if (buf.len < total_len) return error.BufferTooSmall;
    @memcpy(buf[0..4], &logical_message_magic);
    buf[4] = @backingInt(kind);
    std.mem.writeInt(u32, buf[5..9], sequence, .little);
    std.mem.writeInt(u16, buf[9..11], @intCast(payload.len), .little);
    @memcpy(buf[11..total_len], payload);
    return buf[0..total_len];
}

pub fn encodePacket(
    buffer: []u8,
    kind: LogicalMessageKind,
    sequence: u32,
    value: anytype,
) ![]const u8 {
    if (buffer.len < logical_message_header_len) return error.BufferTooSmall;

    var payload_writer: Io.Writer = .fixed(buffer[logical_message_header_len..]);
    try SerialBinary.write(&payload_writer, value);
    const payload = payload_writer.buffered();
    if (payload.len > std.math.maxInt(u16)) return error.MessageTooLarge;

    @memcpy(buffer[0..4], &logical_message_magic);
    buffer[4] = @backingInt(kind);
    std.mem.writeInt(u32, buffer[5..9], sequence, .little);
    std.mem.writeInt(u16, buffer[9..11], @intCast(payload.len), .little);
    return buffer[0 .. logical_message_header_len + payload.len];
}

pub fn decodeLogicalMessage(buf: []const u8) LogicalMessageDecodeError!LogicalMessage {
    if (buf.len < logical_message_header_len) return error.BufferTooShort;
    if (!std.mem.eql(u8, buf[0..4], &logical_message_magic)) return error.BadMagic;
    const kind = std.enums.fromInt(LogicalMessageKind, buf[4]) orelse return error.InvalidKind;
    const payload_len = std.mem.readInt(u16, buf[9..11], .little);
    if (payload_len > logical_message_max_len - logical_message_header_len)
        return error.FrameTooLarge;
    if (buf.len != logical_message_header_len + payload_len) return error.InvalidLength;
    return .{
        .kind = kind,
        .sequence = std.mem.readInt(u32, buf[5..9], .little),
        .payload = buf[logical_message_header_len..],
    };
}

pub const TcpFrameDecoder = struct {
    bytes: [logical_message_max_len + 4]u8 = undefined,
    len: usize = 0,

    pub fn feed(self: *TcpFrameDecoder, data: []const u8) !void {
        if (data.len > self.bytes.len - self.len) return error.FrameTooLarge;
        @memcpy(self.bytes[self.len..][0..data.len], data);
        self.len += data.len;
    }

    pub fn peek(self: *TcpFrameDecoder) !?struct {
        message: LogicalMessage,
        consumed: usize,
    } {
        if (self.len < 4) return null;

        const frame_len_raw = std.mem.readInt(u32, self.bytes[0..4], .little);
        const frame_len: usize = @intCast(frame_len_raw);
        if (frame_len < logical_message_header_len or frame_len > logical_message_max_len)
            return error.FrameTooLarge;

        const total = 4 + frame_len;
        if (self.len < total) return null;
        return .{
            .message = try decodeLogicalMessage(self.bytes[4..total]),
            .consumed = total,
        };
    }

    pub fn consume(self: *TcpFrameDecoder, count: usize) void {
        std.debug.assert(count <= self.len);
        const remaining = self.len - count;
        std.mem.copyForwards(u8, self.bytes[0..remaining], self.bytes[count..self.len]);
        self.len = remaining;
    }
};

pub fn setSocketNonBlocking(socket: *const Io.net.Socket) !void {
    if (builtin.os.tag == .windows) return;
    const current_raw = std.c.fcntl(socket.handle, std.c.F.GETFL);
    if (current_raw < 0) return error.SetNonBlockingFailed;
    var current: std.c.O = @bitCast(current_raw);
    current.NONBLOCK = true;
    if (std.c.fcntl(socket.handle, std.c.F.SETFL, @as(c_int, @bitCast(current))) < 0)
        return error.SetNonBlockingFailed;
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

/// Mutable control values sent by the viewer over the control socket.
pub const LiveConfigPacket = struct {
    magic: [4]u8 = magic_value,
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

    const magic_value: [4]u8 = "MCF1".*;

    const DecodeError = error{
        BufferTooShort,
        BadMagic,
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

    fn decode(buf: []const u8) DecodeError!LiveConfigPacket {
        if (buf.len < 4) return error.BufferTooShort;
        if (!std.mem.eql(u8, buf[0..4], &magic_value)) return error.BadMagic;

        const parsed = SerialBinary.readFromBuffer(LiveConfigPacket, buf) catch return error.BufferTooShort;
        if (!std.mem.eql(u8, &parsed.value.magic, &magic_value)) return error.BadMagic;
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

    fn validate(packet: *const LiveConfigPacket) ValidateError!void {
        if (packet.tam_len == 0 or packet.tam_len > MissionConfig.thruster_count)
            return error.InvalidTamLen;
        if (packet.queue_capacity == 0 or packet.queue_capacity > MissionConfig.queue_capacity_max)
            return error.InvalidQueueCapacity;
        try validateTam(&packet.tam, packet.tam_len);
        try validateOdometry(&packet.odometry);
        try validateIbvs(&packet.ibvs);
        try requireFinite(packet.goal_dist_threshold);
        try requireFinite(packet.thrustor_saturate);
        switch (packet.telemetry_transport) {
            .udp => {
                const address = try packet.telemetry_addr.slice();
                if (address.len == 0) return error.InvalidTextLength;
            },
            .shared_memory => {
                const shm_name = try packet.telemetry_shm_name.slice();
                if (shm_name.len == 0) return error.InvalidTextLength;
            },
            .stdout => {},
        }
        _ = try packet.telemetry_log_path.slice();
    }

    pub fn fromRuntimeConfig(config: *const MissionConfig.MissionConfig) LiveConfigPacket {
        return .{
            .magic = magic_value,
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

    fn applyToRuntimeConfig(dst: *MissionConfig.MissionConfig, packet: LiveConfigPacket) void {
        const control_port = dst.control_port;
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

        dst.control_port = control_port;
        dst.teleop_port = teleop_port;
    }
};

pub const Goal = struct {
    position: math.Vector3f,
    look_at: math.Vector3f,
};

/// Runtime telemetry data produced by the mission loop for publication.
pub const telemetry_step_max = 64;
pub const telemetry_ring_slot_count = 32;

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

/// Canonical fixed-layout runtime telemetry shared with the viewer.
/// All variable-length data is stored inline with explicit lengths so the same
/// type can be used for runtime publication, shared memory, UDP, and logs.
pub const TelemetryFrame = extern struct {
    magic: [4]u8,
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
    clear_screen: bool,
    _debug_pad: [7]u8,
    cycle_us: u64,
    has_dt: bool,
    _dt_pad: [3]u8,
    dt: f32,
    kp: [6]f32,
    ki: [6]f32,
    kd: [6]f32,
    tam: [MissionConfig.thruster_count][6]f32,
    tam_len: u8,
    _tam_pad: [7]u8,
    control_pose: [6]f32,
    translation_error: [3]f32,
    target_orientation: [4]f32,
    target_quaternion: [4]f32,
    attitude_error: [3]f32,
    rotation_limit: [3]f32,
    input: [6]f32,
};

const RingHeader = extern struct {
    magic: [4]u8,
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

fn publishTelemetryFrame(ring: *RingMemory, frame: *const TelemetryFrame) void {
    const previous = @atomicLoad(u64, &ring.header.published_seq, .monotonic);
    const seq = previous +% 1;
    const slot = &ring.slots[seq % telemetry_ring_slot_count];
    @atomicStore(u64, &slot.seq, seq *% 2 -% 1, .release);
    slot.frame = frame.*;
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
    // LiveConfigPacket is serialized field-by-field by SerialBinary. Keep
    // these anchors in order when changing the control wire schema.
    const live_fields = @typeInfo(LiveConfigPacket).@"struct".field_names;
    if (live_fields.len != 17 or
        !std.mem.eql(u8, live_fields[1], "camera_forward") or
        !std.mem.eql(u8, live_fields[13], "telemetry_transport") or
        !std.mem.eql(u8, live_fields[16], "telemetry_log_path"))
        @compileError("live config packet control fields changed order");
    // TelemetryFrame is shared-memory ABI, so its fixed offsets are part of
    // the viewer/runtime contract.
    if (@offsetOf(TelemetryFrame, "timestamp") != 8)
        @compileError("telemetry timestamp ABI drift");
    if (@offsetOf(TelemetryFrame, "objects") <= @offsetOf(TelemetryFrame, "goals_len"))
        @compileError("telemetry payload ABI drift");
    if (@offsetOf(TelemetryFrame, "cycle_us") <= @offsetOf(TelemetryFrame, "clear_screen") or
        @offsetOf(TelemetryFrame, "input") <= @offsetOf(TelemetryFrame, "rotation_limit"))
        @compileError("telemetry diagnostics ABI drift");
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
    if (header.slot_count != telemetry_ring_slot_count or header.slot_size != @sizeOf(RingSlot))
        return error.LayoutMismatch;
}

const outbound_buf_len = 64 * 1024;

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
            .owner_pid = ownerPid(),
            .slot_count = telemetry_ring_slot_count,
            .slot_size = @sizeOf(RingSlot),
            .published_seq = 0,
        };
        return .{ .shared = shared, .ring = ring };
    }

    fn deinit(self: *SharedMemoryPublisher) void {
        self.shared.unlink();
        self.shared.deinit();
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
    udp: void,
    shared_memory: SharedMemoryPublisher,
    stdout: StdoutPublisher,

    fn init(io: Io, config: MissionConfig.MissionConfig) !TelemetryPublisher {
        return switch (config.telemetry_transport) {
            .udp => .{ .udp = {} },
            .shared_memory => .{ .shared_memory = try .init(try config.telemetryShmName()) },
            .stdout => .{ .stdout = try .init(io) },
        };
    }

    fn deinit(self: *TelemetryPublisher, io: Io) void {
        _ = io;
        switch (self.*) {
            .udp => {},
            .shared_memory => |*publisher| publisher.deinit(),
            .stdout => |*publisher| publisher.deinit(),
        }
        self.* = undefined;
    }
};

pub fn encodeTelemetryFrame(buf: []u8, frame: TelemetryFrame) ![]const u8 {
    var writer: Io.Writer = .fixed(buf);
    try SerialBinary.write(&writer, frame);
    return writer.buffered();
}

fn roundDisplayFloat(value: f32) f32 {
    return @round(value * 100000.0) / 100000.0;
}

fn roundDisplayArray(comptime len: usize, value: [len]f32) [len]f32 {
    var rounded: [len]f32 = undefined;
    for (value, &rounded) |source, *dest| dest.* = roundDisplayFloat(source);
    return rounded;
}

fn roundDisplayPose(value: TelemetryPose) TelemetryPose {
    return .{
        .pos = roundDisplayArray(3, value.pos),
        .quat = roundDisplayArray(4, value.quat),
    };
}

fn writeAsciiFloatList(writer: *Io.Writer, values: anytype) !void {
    try writer.writeAll("[");
    for (values, 0..) |value, i| {
        if (i != 0) try writer.writeAll(",");
        try writer.print("{d:.5}", .{roundDisplayFloat(value)});
    }
    try writer.writeAll("]");
}

fn writeAsciiIntList(writer: *Io.Writer, values: anytype) !void {
    try writer.writeAll("[");
    for (values, 0..) |value, i| {
        if (i != 0) try writer.writeAll(",");
        try writer.print("{}", .{value});
    }
    try writer.writeAll("]");
}

pub fn encodeTelemetryFrameAscii(buf: []u8, frame: TelemetryFrame) ![]const u8 {
    var objects: [Auv.Frame.max_objects]struct {
        pose: TelemetryPose,
        size: [3]f32,
        id: u32,
        cls: u8,
    } = undefined;
    for (frame.objects[0..frame.objects_len], objects[0..frame.objects_len]) |source, *dest| {
        dest.* = .{
            .pose = roundDisplayPose(source.pose),
            .size = roundDisplayArray(3, source.size),
            .id = source.id,
            .cls = source.cls,
        };
    }

    var writer: Io.Writer = .fixed(buf);
    try writer.writeAll("FRAME_BEGIN\n");
    try writer.print("magic={s}\ntimestamp={}\n", .{
        frame.magic[0..],
        frame.timestamp,
    });
    try writer.writeAll("camera_pose.pos=");
    try writeAsciiFloatList(&writer, frame.camera_pose.pos);
    try writer.writeAll("\ncamera_pose.quat=");
    try writeAsciiFloatList(&writer, frame.camera_pose.quat);
    try writer.print("\nposition_ok={},{},{}\nattitude_ok={},{},{}\n", .{
        frame.position_ok.x,
        frame.position_ok.y,
        frame.position_ok.z,
        frame.attitude_ok.x,
        frame.attitude_ok.y,
        frame.attitude_ok.z,
    });
    try writer.print("depth_ok={}\ndepth={d:.5}\n", .{
        frame.depth_ok,
        roundDisplayFloat(frame.depth),
    });
    try writer.print("objects_len={}\n", .{frame.objects_len});
    for (objects[0..frame.objects_len], 0..) |object, i| {
        try writer.print("object[{}].pose.pos=", .{i});
        try writeAsciiFloatList(&writer, object.pose.pos);
        try writer.print("\nobject[{}].pose.quat=", .{i});
        try writeAsciiFloatList(&writer, object.pose.quat);
        try writer.print("\nobject[{}].size=", .{i});
        try writeAsciiFloatList(&writer, object.size);
        try writer.print("\nobject[{}].id={}\nobject[{}].cls={}\n", .{
            i,
            object.id,
            i,
            object.cls,
        });
    }
    try writer.print("boxes_len={}\n", .{frame.boxes_len});
    for (frame.boxes[0..frame.boxes_len], 0..) |box, i| {
        try writer.print("box[{}].top_left=[{},{}]\nbox[{}].bottom_right=[{},{}]\nbox[{}].id={}\nbox[{}].cls={}\n", .{
            i,
            box.top_left.x,
            box.top_left.y,
            i,
            box.bottom_right.x,
            box.bottom_right.y,
            i,
            box.id,
            i,
            box.cls,
        });
    }
    try writer.print("thrusters_len={}\nthrusters=", .{frame.thrusters_len});
    try writeAsciiFloatList(&writer, frame.thrusters[0..frame.thrusters_len]);
    try writer.print("\nstep_len={}\nstep_bytes=", .{frame.step_len});
    try writeAsciiIntList(&writer, frame.step[0..frame.step_len]);
    try writer.writeAll("\nforce_error=");
    try writeAsciiFloatList(&writer, frame.force_error);
    try writer.print("\ngoals_len={}\n", .{frame.goals_len});
    for (frame.goals[0..frame.goals_len], 0..) |goal, i| {
        try writer.print("goal[{}].position=", .{i});
        try writeAsciiFloatList(&writer, goal.position);
        try writer.print("\ngoal[{}].look_at=", .{i});
        try writeAsciiFloatList(&writer, goal.look_at);
        try writer.writeAll("\n");
    }
    try writer.print(
        "clear_screen={}\ncycle_us={}\nhas_dt={}\ndt={d:.5}\n",
        .{ frame.clear_screen, frame.cycle_us, frame.has_dt, roundDisplayFloat(frame.dt) },
    );
    try writer.writeAll("kp=");
    try writeAsciiFloatList(&writer, frame.kp);
    try writer.writeAll("\nki=");
    try writeAsciiFloatList(&writer, frame.ki);
    try writer.writeAll("\nkd=");
    try writeAsciiFloatList(&writer, frame.kd);
    try writer.print("\ntam_len={}\n", .{frame.tam_len});
    for (frame.tam[0..frame.tam_len], 0..) |row, i| {
        try writer.print("tam[{}]=", .{i});
        try writeAsciiFloatList(&writer, row);
        try writer.writeAll("\n");
    }
    try writer.writeAll("control_pose=");
    try writeAsciiFloatList(&writer, frame.control_pose);
    try writer.writeAll("\ntranslation_error=");
    try writeAsciiFloatList(&writer, frame.translation_error);
    try writer.writeAll("\ntarget_orientation=");
    try writeAsciiFloatList(&writer, frame.target_orientation);
    try writer.writeAll("\ntarget_quaternion=");
    try writeAsciiFloatList(&writer, frame.target_quaternion);
    try writer.writeAll("\nattitude_error=");
    try writeAsciiFloatList(&writer, frame.attitude_error);
    try writer.writeAll("\nrotation_limit=");
    try writeAsciiFloatList(&writer, frame.rotation_limit);
    try writer.writeAll("\ninput=");
    try writeAsciiFloatList(&writer, frame.input);
    try writer.writeAll("\nFRAME_END\n");
    return writer.buffered();
}

/// Decode the fixed telemetry frame used by both UDP/log and shared memory.
pub fn decodeTelemetryFrame(buf: []const u8, frame: *TelemetryFrame) !void {
    var reader: Io.Reader = .fixed(buf);
    frame.* = try SerialBinary.read(&reader, TelemetryFrame);
    if (!std.mem.eql(u8, &frame.magic, "AUV1")) return error.BadMagic;
    if (frame.objects_len > frame.objects.len or frame.boxes_len > frame.boxes.len or
        frame.thrusters_len > frame.thrusters.len or frame.step_len > frame.step.len or
        frame.goals_len > frame.goals.len or frame.tam_len > frame.tam.len)
        return error.InvalidTelemetryLengths;
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
    kind: MissionCommandKind,
    maneuver: MissionManeuver,
    position: math.Vector3f = @splat(0),
    look_direction: math.Vector3f = .{ 0, 0, 1 },
    object_class: u8 = 0,

    pub const DecodeError = error{
        BufferTooShort,
        InvalidLength,
        BadMagic,
        InvalidEnum,
        NonFinite,
        InvalidLookDirection,
        InvalidPointManeuver,
    };

    pub fn decode(buf: []const u8) DecodeError!MissionCommand {
        if (buf.len < 4) return error.BufferTooShort;
        if (!std.mem.eql(u8, buf[0..4], &mission_command_magic)) return error.BadMagic;

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

const teleop_timeout_ns: i96 = 250 * std.time.ns_per_ms;

fn teleopExpired(received_at: Io.Clock.Timestamp, now: Io.Clock.Timestamp) bool {
    return received_at.durationTo(now).raw.nanoseconds > teleop_timeout_ns;
}

pub const Telemetry = struct {
    publisher: TelemetryPublisher,
    log_file: ?Io.File,
    encoded_buf: [outbound_buf_len]u8 = undefined,
    ascii_buf: [outbound_buf_len]u8 = undefined,
    stdout_log_counter: u64 = 0,
    stdout_log_freq_div: u16,
    telemetry_sequence: u32 = 1,
    udp: Io.net.Socket,
    udp_destination: Io.net.IpAddress,
    server: Io.net.Server,
    client: ?Io.net.Stream = null,
    tcp: TcpFrameDecoder = .{},
    teleop_command: ?TeleopCommand = null,
    last_teleop_at: ?Io.Clock.Timestamp = null,
    mission_command: ?MissionCommand = null,
    live_config_packet: LiveConfigPacket = undefined,
    initial_config_packet: LiveConfigPacket = undefined,

    fn refreshLiveConfigPacket(self: *Telemetry, config: MissionConfig.MissionConfig) void {
        self.live_config_packet = LiveConfigPacket.fromRuntimeConfig(&config);
    }

    pub fn init(_: std.mem.Allocator, io: Io, config: MissionConfig.MissionConfig) !Telemetry {
        var publisher = try TelemetryPublisher.init(io, config);
        errdefer publisher.deinit(io);

        const udp_destination = try Io.net.IpAddress.parseLiteral(try config.telemetryAddr());
        const udp_bind: Io.net.IpAddress = .{ .ip4 = .unspecified(config.teleop_port) };
        var udp = udp_bind.bind(io, .{ .mode = .dgram, .protocol = .udp }) catch |err| switch (err) {
            error.AddressInUse => {
                std.log.err("realtime UDP port {d} is already in use", .{config.teleop_port});
                return error.AddressInUse;
            },
            else => return err,
        };
        errdefer udp.close(io);

        const tcp_bind: Io.net.IpAddress = .{ .ip4 = .unspecified(config.control_port) };
        var server = tcp_bind.listen(io, .{
            .mode = .stream,
            .protocol = .tcp,
            .reuse_address = true,
        }) catch |err| switch (err) {
            error.AddressInUse => {
                std.log.err("TCP control port {d} is already in use", .{config.control_port});
                return error.AddressInUse;
            },
            else => return err,
        };
        errdefer server.deinit(io);
        try setSocketNonBlocking(&server.socket);

        const log_file: ?Io.File = if (try config.telemetryLogPath()) |path|
            try Io.Dir.cwd().createFile(io, path, .{})
        else
            null;
        errdefer if (log_file) |file| file.close(io);

        var telemetry: Telemetry = .{
            .publisher = publisher,
            .log_file = log_file,
            .stdout_log_freq_div = config.log_freq_div,
            .udp = udp,
            .udp_destination = udp_destination,
            .server = server,
            .initial_config_packet = LiveConfigPacket.fromRuntimeConfig(&config),
        };
        telemetry.refreshLiveConfigPacket(config);
        return telemetry;
    }

    pub fn deinit(self: *Telemetry, io: Io) void {
        if (self.log_file) |file| file.close(io);
        self.publisher.deinit(io);
        self.closeClient(io);
        self.server.deinit(io);
        self.udp.close(io);
        self.* = undefined;
    }

    pub fn warnImmutableFields(self: *const Telemetry, config: MissionConfig.MissionConfig) void {
        const initial_transport = self.initial_config_packet.telemetry_transport;
        if (initial_transport == .udp or config.telemetry_transport == .udp) {
            const init_telemetry = self.initial_config_packet.telemetry_addr.slice() catch "";
            const new_telemetry = config.telemetryAddr() catch {
                std.log.err("live config has invalid telemetry_addr", .{});
                return;
            };
            if (!std.mem.eql(u8, new_telemetry, init_telemetry)) {
                std.log.warn("config telemetry_addr changed; restart required", .{});
            }
        }
        if (config.telemetry_transport != self.initial_config_packet.telemetry_transport) {
            std.log.warn("config telemetry_transport changed; restart required", .{});
        }
        if (initial_transport == .shared_memory or config.telemetry_transport == .shared_memory) {
            const init_shm_name = self.initial_config_packet.telemetry_shm_name.slice() catch "";
            const new_shm_name = config.telemetryShmName() catch {
                std.log.err("live config has invalid telemetry_shm_name", .{});
                return;
            };
            if (!std.mem.eql(u8, new_shm_name, init_shm_name)) {
                std.log.warn("config telemetry_shm_name changed; restart required", .{});
            }
        }
        const init_log_path = self.initial_config_packet.telemetry_log_path.slice() catch "";
        const new_log_path = config.telemetryLogPath() catch {
            std.log.err("live config has invalid telemetry_log_path", .{});
            return;
        } orelse "";
        if (init_log_path.len != new_log_path.len or !std.mem.eql(u8, new_log_path, init_log_path)) {
            std.log.warn("config telemetry_log_path changed; restart required", .{});
        }
    }

    fn closeClient(self: *Telemetry, io: Io) void {
        if (self.client) |*client| client.close(io);
        self.client = null;
        self.tcp = .{};
    }

    fn sendTcp(
        self: *Telemetry,
        io: Io,
        kind: LogicalMessageKind,
        sequence: u32,
        value: anytype,
    ) !void {
        const client = &(self.client orelse return error.NotConnected);
        var frame: [logical_message_max_len + 4]u8 = undefined;
        const packet = try encodePacket(frame[4..], kind, sequence, value);
        std.mem.writeInt(u32, frame[0..4], @intCast(packet.len), .little);
        var writer_buffer: [1024]u8 = undefined;
        var writer = Io.net.Stream.writer(client.*, io, &writer_buffer);
        try writer.interface.writeAll(frame[0 .. 4 + packet.len]);
        try writer.interface.flush();
    }

    pub fn takeTeleopCommand(self: *Telemetry, io: Io) ?TeleopCommand {
        const command = self.teleop_command;
        self.teleop_command = null;
        if (self.last_teleop_at) |received_at| {
            const now = Io.Clock.Timestamp.now(io, .awake);
            if (teleopExpired(received_at, now)) {
                self.last_teleop_at = null;
                return .{
                    .translation = @splat(0),
                    .orientation = .{ 0, 0, 0, 1 },
                };
            }
        }
        if (command == null) return null;
        return command;
    }

    fn sendCommandAck(self: *Telemetry, io: Io, sequence: u32, accepted: bool) void {
        const status: CommandAckStatus = if (accepted) .accepted else .rejected;
        self.sendTcp(io, .command_ack, sequence, status) catch |err| {
            std.log.debug("TCP command acknowledgement failed: {s}", .{@errorName(err)});
            self.closeClient(io);
        };
    }

    fn processTcpMessage(
        self: *Telemetry,
        io: Io,
        message: LogicalMessage,
        base: *MissionConfig.MissionConfig,
        print_stuff: bool,
    ) bool {
        switch (message.kind) {
            .mission_command => {
                self.mission_command = MissionCommand.decode(message.payload) catch |err| {
                    if (print_stuff) std.log.debug("mission command rejected: {s}", .{@errorName(err)});
                    self.sendCommandAck(io, message.sequence, false);
                    return false;
                };
                self.sendCommandAck(io, message.sequence, true);
            },
            .live_config => {
                const packet = LiveConfigPacket.decode(message.payload) catch |err| {
                    if (print_stuff) std.log.debug("live config rejected: {s}", .{@errorName(err)});
                    self.sendCommandAck(io, message.sequence, false);
                    return false;
                };
                LiveConfigPacket.validate(&packet) catch |err| {
                    std.log.warn("live config rejected: {s}", .{@errorName(err)});
                    self.sendCommandAck(io, message.sequence, false);
                    return false;
                };
                LiveConfigPacket.applyToRuntimeConfig(base, packet);
                if (print_stuff) std.log.debug("applied live config on TCP control connection", .{});
                self.sendCommandAck(io, message.sequence, true);
                return true;
            },
            else => {
                self.sendCommandAck(io, message.sequence, false);
            },
        }
        return false;
    }

    fn processUdpMessage(self: *Telemetry, io: Io, message: LogicalMessage, print_stuff: bool) void {
        if (message.kind != .teleop) return;
        const parsed = SerialBinary.readFromBuffer(TeleopCommand, message.payload) catch |err| {
            if (print_stuff) std.log.debug("teleop packet rejected: {s}", .{@errorName(err)});
            return;
        };
        if (parsed.consumed != message.payload.len) {
            if (print_stuff) std.log.debug("teleop packet has trailing bytes", .{});
            return;
        }
        for (parsed.value.translation ++ parsed.value.orientation) |value| {
            if (!std.math.isFinite(value)) {
                if (print_stuff) std.log.debug("teleop packet contains non-finite values", .{});
                return;
            }
        }
        self.teleop_command = parsed.value;
        self.last_teleop_at = Io.Clock.Timestamp.now(io, .awake);
    }

    fn pollTcp(
        self: *Telemetry,
        io: Io,
        working: *MissionConfig.MissionConfig,
        print_stuff: bool,
    ) bool {
        const incoming = self.server.accept(io) catch |err| switch (err) {
            error.WouldBlock => null,
            else => blk: {
                if (print_stuff) std.log.warn("TCP control accept failed: {s}", .{@errorName(err)});
                break :blk null;
            },
        };
        if (incoming) |stream| {
            self.closeClient(io);
            var client = stream;
            setSocketNonBlocking(&client.socket) catch {
                client.close(io);
                return false;
            };
            self.client = client;
        }

        const stream = &(self.client orelse return false);
        var read_buf: [4096]u8 = undefined;
        var applied = false;
        var read_count: usize = 0;
        var parsed_messages: usize = 0;
        while (read_count < 16 and parsed_messages < 16) : (read_count += 1) {
            var chunks: [1][]u8 = .{&read_buf};
            const count = stream.read(io, &chunks) catch |err| switch (err) {
                error.ConnectionTimedOut => break,
                else => {
                    if (print_stuff) std.log.debug("TCP control read failed: {s}", .{@errorName(err)});
                    self.closeClient(io);
                    break;
                },
            };
            if (count == 0) {
                self.closeClient(io);
                break;
            }
            self.tcp.feed(read_buf[0..count]) catch |err| {
                if (print_stuff) std.log.debug("TCP control buffer failed: {s}", .{@errorName(err)});
                self.closeClient(io);
                break;
            };

            var invalid = false;
            while (parsed_messages < 16) {
                const maybe_frame = self.tcp.peek() catch |err| {
                    if (print_stuff) std.log.debug("TCP control frame rejected: {s}", .{@errorName(err)});
                    invalid = true;
                    break;
                };
                const frame = maybe_frame orelse break;
                if (self.processTcpMessage(io, frame.message, working, print_stuff)) applied = true;
                if (self.client == null) break;
                self.tcp.consume(frame.consumed);
                parsed_messages += 1;
            }
            if (invalid or self.client == null) break;
        }
        return applied;
    }

    fn pollUdp(self: *Telemetry, io: Io, print_stuff: bool) void {
        const timeout: std.Io.Timeout = .{ .duration = .{
            .clock = .awake,
            .raw = .fromNanoseconds(0),
        } };
        var recv_buf: [logical_message_max_len]u8 = undefined;
        for (0..16) |_| {
            const msg = self.udp.receiveTimeout(io, &recv_buf, timeout) catch |err| switch (err) {
                error.Timeout => break,
                else => {
                    if (print_stuff) std.log.err("realtime UDP recv: {s}", .{@errorName(err)});
                    break;
                },
            };
            const message = decodeLogicalMessage(msg.data) catch |err| {
                if (print_stuff) std.log.debug("realtime UDP packet rejected: {s}", .{@errorName(err)});
                continue;
            };
            self.processUdpMessage(io, message, print_stuff);
        }
    }

    pub fn pollLiveConfig(
        self: *Telemetry,
        io: Io,
        base: MissionConfig.MissionConfig,
        print_stuff: bool,
        merged_out: *MissionConfig.MissionConfig,
    ) bool {
        var working = base;
        self.pollUdp(io, print_stuff);
        const applied = self.pollTcp(io, &working, print_stuff);
        if (applied) {
            self.refreshLiveConfigPacket(working);
            self.stdout_log_freq_div = working.log_freq_div;
            merged_out.* = working;
        }
        return applied;
    }

    pub fn takeMissionCommand(self: *Telemetry) ?MissionCommand {
        const command = self.mission_command;
        self.mission_command = null;
        return command;
    }

    /// Publish one runtime telemetry frame synchronously.
    pub fn publishTelemetry(
        self: *Telemetry,
        io: Io,
        frame: *const TelemetryFrame,
    ) void {
        switch (self.publisher) {
            .shared_memory => |*publisher| {
                publishTelemetryFrame(publisher.ring, frame);
                if (self.log_file == null) return;
            },
            else => {},
        }

        self.stdout_log_counter +%= 1;
        const stdout_enabled = self.stdout_log_freq_div != 0 and
            self.stdout_log_counter % self.stdout_log_freq_div == 0;

        if (self.log_file) |file| {
            const encoded = encodeTelemetryFrame(&self.encoded_buf, frame.*) catch |err| {
                std.log.err("telemetry encode failed: {s}", .{@errorName(err)});
                return;
            };
            file.writeStreamingAll(io, encoded) catch |err| {
                std.log.err("telemetry log write failed: {s}", .{@errorName(err)});
            };
        }

        switch (self.publisher) {
            .udp => {
                var datagram: [logical_message_max_len]u8 = undefined;
                const encoded = encodePacket(
                    &datagram,
                    .telemetry,
                    self.telemetry_sequence,
                    frame.*,
                ) catch |err| {
                    std.log.err("UDP telemetry encode failed: {s}", .{@errorName(err)});
                    return;
                };
                self.telemetry_sequence +%= 1;
                self.udp.send(io, &self.udp_destination, encoded) catch {};
            },
            .stdout => |*publisher| if (stdout_enabled) {
                const encoded = encodeTelemetryFrameAscii(&self.ascii_buf, frame.*) catch |err| {
                    std.log.err("telemetry ASCII encode failed: {s}", .{@errorName(err)});
                    return;
                };
                publisher.publishEncoded(io, encoded);
            },
            .shared_memory => {},
        }
    }
};