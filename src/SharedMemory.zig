//! Cross-platform named shared-memory mapping.
const std = @import("std");
const builtin = @import("builtin");

const is_windows = builtin.os.tag == .windows;
const page_alignment = std.heap.page_size_min;
const NativeHandle = if (is_windows) std.os.windows.HANDLE else std.posix.fd_t;

const SharedMemory = @This();

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

pub fn bytes(self: *SharedMemory) []u8 {
    return self.memory;
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
