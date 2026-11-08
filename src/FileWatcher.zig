const std = @import("std");
const builtin = @import("builtin");

const FileWatcher = @This();

impl: Impl,

const Impl = if (builtin.os.tag == .linux) Linux else Poll;

pub fn init(arena: std.mem.Allocator, path: []const u8) !FileWatcher {
    return .{ .impl = try Impl.init(arena, path) };
}

pub fn deinit(self: *FileWatcher) void {
    self.impl.deinit();
}

pub fn changed(self: *FileWatcher) !bool {
    return self.impl.changed();
}

pub fn directory(self: *const FileWatcher) []const u8 {
    return self.impl.dir;
}

pub fn fileName(self: *const FileWatcher) []const u8 {
    return self.impl.name;
}

const Linux = struct {
    const Watcher = @This();
    const linux = std.os.linux;

    fd: std.posix.fd_t,
    wd: i32,

    dir: []const u8,
    name: []const u8,

    buf: []u8,
    len: usize,
    off: usize,

    const RawEvent = extern struct {
        wd: i32,
        mask: u32,
        cookie: u32,
        len: u32,
    };

    const Event = struct {
        raw: RawEvent,
        name: []const u8,
    };

    pub fn init(arena: std.mem.Allocator, path: []const u8) !Watcher {
        const dir = std.fs.path.dirname(path) orelse ".";

        const fd_result = linux.inotify_init1(
            linux.IN.NONBLOCK | linux.IN.CLOEXEC,
        );

        if (linux.errno(fd_result) != .SUCCESS) {
            return error.InotifyInitFailed;
        }

        const fd: std.posix.fd_t = @intCast(fd_result);
        errdefer _ = linux.close(fd);

        return .{
            .fd = fd,
            .wd = try addWatch(fd, dir),
            .dir = dir,
            .name = std.fs.path.basename(path),
            .buf = try arena.alloc(u8, 4096),
            .len = 0,
            .off = 0,
        };
    }

    pub fn deinit(self: *Watcher) void {
        if (self.fd >= 0) {
            _ = linux.close(self.fd);
            self.fd = -1;
        }
    }

    pub fn changed(self: *Watcher) !bool {
        while (true) {
            if (self.off >= self.len) {
                const result = linux.read(self.fd, self.buf.ptr, self.buf.len);

                if (linux.errno(result) != .SUCCESS) {
                    return switch (linux.errno(result)) {
                        .AGAIN, .INTR => false,
                        else => error.InotifyReadFailed,
                    };
                }

                if (result == 0) {
                    return false;
                }

                self.len = @intCast(result);
                self.off = 0;
            }

            const event = self.nextEvent() orelse {
                return error.InotifyMalformedEvent;
            };

            if (event.raw.wd != self.wd) {
                continue;
            }

            if (!std.mem.eql(u8, event.name, self.name)) {
                continue;
            }

            if ((event.raw.mask & linux.IN.MOVED_TO) != 0) {
                return true;
            }

            if ((event.raw.mask & linux.IN.CLOSE_WRITE) != 0) {
                return true;
            }
        }
    }

    fn nextEvent(self: *Watcher) ?Event {
        const header_size = @sizeOf(RawEvent);

        if (self.len - self.off < header_size) {
            return null;
        }

        const start = self.off;

        const event_ptr: *align(1) const RawEvent =
            @ptrCast(self.buf[start..].ptr);

        const event = event_ptr.*;

        const total_size =
            header_size + @as(usize, event.len);

        if (self.len - start < total_size) {
            return null;
        }

        const name_buf =
            self.buf[start + header_size ..][0..event.len];

        const nul =
            std.mem.indexOfScalar(u8, name_buf, 0) orelse name_buf.len;

        const name = name_buf[0..nul];

        self.off += std.mem.alignForward(
            usize,
            total_size,
            @alignOf(RawEvent),
        );

        return .{
            .raw = event,
            .name = name,
        };
    }

    fn addWatch(fd: std.posix.fd_t, path: []const u8) !i32 {
        var path_buf: [std.fs.max_path_bytes]u8 = undefined;

        const path_z = std.fmt.bufPrintSentinel(
            &path_buf,
            "{s}",
            .{path},
            0,
        ) catch return error.PathTooLong;

        const result = linux.inotify_add_watch(
            fd,
            path_z.ptr,
            linux.IN.CLOSE_WRITE | linux.IN.MOVED_TO,
        );

        if (linux.errno(result) != .SUCCESS) {
            return error.InotifyAddWatchFailed;
        }

        return @intCast(result);
    }
};

const Poll = struct {
    const Watcher = @This();

    path: []const u8,
    dir: []const u8,
    name: []const u8,
    mtime: u64,

    pub fn init(arena: std.mem.Allocator, path: []const u8) !Watcher {
        const owned = try arena.dupe(u8, path);
        return .{
            .path = owned,
            .dir = std.fs.path.dirname(owned) orelse ".",
            .name = std.fs.path.basename(owned),
            .mtime = writeTime(owned),
        };
    }

    pub fn deinit(self: *Watcher) void {
        _ = self;
    }

    pub fn changed(self: *Watcher) !bool {
        const now = writeTime(self.path);
        if (now == self.mtime) return false;
        self.mtime = now;
        return true;
    }
};

const FileAttributeData = extern struct {
    attributes: std.os.windows.DWORD,
    creation: std.os.windows.FILETIME,
    access: std.os.windows.FILETIME,
    write: std.os.windows.FILETIME,
    size_high: std.os.windows.DWORD,
    size_low: std.os.windows.DWORD,
};

extern "kernel32" fn GetFileAttributesExW(
    name: [*:0]const u16,
    level: u32,
    info: *FileAttributeData,
) callconv(.winapi) std.os.windows.BOOL;

fn writeTime(path: []const u8) u64 {
    var wide: [std.os.windows.PATH_MAX_WIDE]u16 = undefined;
    const len = std.unicode.wtf8ToWtf16Le(wide[0 .. wide.len - 1], path) catch return 0;
    if (len >= wide.len) return 0;
    wide[len] = 0;
    var info: FileAttributeData = undefined;
    if (!GetFileAttributesExW(wide[0..len :0].ptr, 0, &info).toBool()) return 0;
    return (@as(u64, info.write.dwHighDateTime) << 32) | info.write.dwLowDateTime;
}
