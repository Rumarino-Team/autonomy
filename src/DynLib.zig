const std = @import("std");
const builtin = @import("builtin");

pub const DynLib = if (builtin.os.tag == .windows) WindowsDynLib else std.DynLib;

pub fn loadError() [:0]const u8 {
    if (builtin.os.tag == .windows) return "LoadLibrary failed";
    return posixLoadError();
}

fn posixLoadError() [:0]const u8 {
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
extern "kernel32" fn GetProcAddress(module: std.os.windows.HMODULE, name: [*:0]const u8) callconv(.winapi) ?*anyopaque;

const dll_load_dir: u32 = 0x00000100;
const dll_default_dirs: u32 = 0x00001000;

fn load(path: [*:0]const u16) ?std.os.windows.HMODULE {
    // A path with a directory must be absolute. Otherwise Windows looks for
    // libsimulator_core.dll next to src_3.exe instead of next to the plugin.
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
