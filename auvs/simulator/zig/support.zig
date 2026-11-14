const std = @import("std");
const builtin = @import("builtin");
const c = @import("sim_c");

pub const initialized = 0x53494d35;
pub const max_scene_bytes: usize = 128 * 1024 * 1024;

threadlocal var fail_msg: [512]u8 = undefined;
threadlocal var fail_len: usize = 0;
threadlocal var fail_code: c.SimResult = c.SIM_INVALID;

pub fn boom(code: c.SimResult, msg: []const u8) error{Failed} {
    const len = @min(msg.len, fail_msg.len);
    @memcpy(fail_msg[0..len], msg[0..len]);
    fail_len = len;
    fail_code = code;
    return error.Failed;
}

pub fn require(ok: bool, msg: []const u8, code: c.SimResult) error{Failed}!void {
    if (!ok) return boom(code, msg);
}

pub fn check(result: c.SimResult, msg: []const u8) error{Failed}!void {
    if (result != c.SIM_OK) return boom(result, if (msg.len == 0) "simulator call failed" else msg);
}

fn put(e: ?[*]u8, n: u32, msg: []const u8) void {
    const p = e orelse return;
    if (n == 0) return;
    const len = @min(msg.len, n - 1);
    @memcpy(p[0..len], msg[0..len]);
    p[len] = 0;
}

extern "c" fn aligned_alloc(alignment: usize, size: usize) ?*anyopaque;

pub fn guard(e: ?[*]u8, n: u32, result: anyerror!void) c.SimResult {
    result catch |err| {
        if (err == error.OutOfMemory) {
            put(e, n, "allocation failed");
            return c.SIM_NO_MEMORY;
        }
        if (err == error.Failed) {
            put(e, n, fail_msg[0..fail_len]);
            return fail_code;
        }
        put(e, n, "unknown backend failure");
        return c.SIM_BACKEND_ERROR;
    };
    if (e) |p| {
        if (n > 0) p[0] = 0;
    }
    return c.SIM_OK;
}

pub fn bytes(count: usize, width: usize) error{Failed}!usize {
    try require(width == 0 or count <= std.math.maxInt(usize) / width, "size overflow", c.SIM_CAPACITY);
    return count * width;
}

pub fn defaultAlloc(_: ?*anyopaque, n: usize, alignment: usize) callconv(.c) ?*anyopaque {
    var align_to = alignment;
    if (align_to < @alignOf(*anyopaque)) align_to = @alignOf(*anyopaque);
    if (n > std.math.maxInt(usize) - align_to - @sizeOf(*anyopaque)) return null;
    if (builtin.os.tag == .windows) {
        const raw = std.c.malloc(n + align_to + @sizeOf(*anyopaque)) orelse return null;
        var aligned = @intFromPtr(raw) + @sizeOf(*anyopaque);
        aligned = (aligned + align_to - 1) & ~(align_to - 1);
        const slot: [*]?*anyopaque = @ptrFromInt(aligned);
        slot[-1] = raw;
        return @ptrFromInt(aligned);
    }
    const size = (n + align_to - 1) & ~(align_to - 1);
    return aligned_alloc(align_to, size);
}

pub fn defaultFree(_: ?*anyopaque, p: ?*anyopaque, _: usize, _: usize) callconv(.c) void {
    const memory = p orelse return;
    if (builtin.os.tag == .windows) {
        const slot: [*]?*anyopaque = @ptrCast(@alignCast(memory));
        std.c.free(slot[-1]);
        return;
    }
    std.c.free(memory);
}

pub fn resolveAllocator(a: c.SimAllocator) error{Failed}!c.SimAllocator {
    const has_alloc = a.allocate != null;
    const has_free = a.deallocate != null;
    try require(has_alloc == has_free, "allocator callbacks must be paired", c.SIM_INVALID);
    if (a.allocate == null) return .{ .user = null, .allocate = defaultAlloc, .deallocate = defaultFree };
    return a;
}

pub const Alloc = c.SimAllocator;

pub fn allocate(a: Alloc, size: usize, alignment: usize) error{ OutOfMemory, Failed }!*anyopaque {
    const p = a.allocate.?(a.user, size, alignment) orelse return error.OutOfMemory;
    if (alignment != 0 and @intFromPtr(p) % alignment != 0) {
        a.deallocate.?(a.user, p, size, alignment);
        return boom(c.SIM_INVALID, "allocator returned misaligned memory");
    }
    return p;
}

pub fn Buffer(comptime T: type) type {
    return struct {
        const Self = @This();
        alloc: Alloc = std.mem.zeroes(Alloc),
        ptr: ?[*]T = null,
        n: usize = 0,
        cap: usize = 0,

        pub fn deinit(self: *Self) void {
            if (self.ptr) |p| {
                const size = self.cap * @sizeOf(T);
                self.alloc.deallocate.?(self.alloc.user, @ptrCast(p), size, @alignOf(T));
            }
            self.* = .{};
        }

        pub fn reserve(self: *Self, count: usize) !void {
            if (count <= self.cap) return;
            const size = try bytes(count, @sizeOf(T));
            try require(size <= max_scene_bytes, "buffer exceeds size limit", c.SIM_CAPACITY);
            const raw = try allocate(self.alloc, size, @alignOf(T));
            const fresh: [*]T = @ptrCast(@alignCast(raw));
            @memset(fresh[0..count], std.mem.zeroes(T));
            if (self.n != 0) @memcpy(fresh[0..self.n], self.ptr.?[0..self.n]);
            if (self.ptr) |p| {
                const old = self.cap * @sizeOf(T);
                self.alloc.deallocate.?(self.alloc.user, @ptrCast(p), old, @alignOf(T));
            }
            self.ptr = fresh;
            self.cap = count;
        }

        pub fn resize(self: *Self, count: usize) !void {
            if (count > self.cap) {
                const grown = if (self.cap == 0) @max(count, 8) else @max(count, self.cap * 2);
                try self.reserve(grown);
            }
            if (count > self.n) @memset(self.ptr.?[self.n..count], std.mem.zeroes(T));
            self.n = count;
        }

        pub fn items(self: *Self) []T {
            return if (self.ptr) |p| p[0..self.n] else &.{};
        }

        pub fn constItems(self: *const Self) []const T {
            return if (self.ptr) |p| p[0..self.n] else &.{};
        }

        pub fn data(self: *Self) ?[*]T {
            return self.ptr;
        }
    };
}

pub fn v(x: f64, y: f64, z: f64) c.MathVector3f {
    @setFloatMode(.strict);
    return .{ .buf = .{ @floatCast(x), @floatCast(y), @floatCast(z) } };
}

pub fn q(x: f64, y: f64, z: f64, w: f64) c.MathQuaternionf {
    @setFloatMode(.strict);
    return .{ .buf = .{ @floatCast(x), @floatCast(y), @floatCast(z), @floatCast(w) } };
}

pub fn add(a: c.MathVector3f, b: c.MathVector3f) c.MathVector3f {
    @setFloatMode(.strict);
    return v(a.buf[0] + b.buf[0], a.buf[1] + b.buf[1], a.buf[2] + b.buf[2]);
}

pub fn sub(a: c.MathVector3f, b: c.MathVector3f) c.MathVector3f {
    @setFloatMode(.strict);
    return v(a.buf[0] - b.buf[0], a.buf[1] - b.buf[1], a.buf[2] - b.buf[2]);
}

pub fn scale(a: c.MathVector3f, s: f64) c.MathVector3f {
    @setFloatMode(.strict);
    return v(@as(f64, a.buf[0]) * s, @as(f64, a.buf[1]) * s, @as(f64, a.buf[2]) * s);
}

pub fn dot(a: c.MathVector3f, b: c.MathVector3f) f64 {
    @setFloatMode(.strict);
    return @as(f64, a.buf[0]) * b.buf[0] + @as(f64, a.buf[1]) * b.buf[1] + @as(f64, a.buf[2]) * b.buf[2];
}

pub fn norm(a: c.MathVector3f) f64 {
    @setFloatMode(.strict);
    return @sqrt(dot(a, a));
}

pub fn cross(a: c.MathVector3f, b: c.MathVector3f) c.MathVector3f {
    @setFloatMode(.strict);
    return v(
        @as(f64, a.buf[1]) * b.buf[2] - @as(f64, a.buf[2]) * b.buf[1],
        @as(f64, a.buf[2]) * b.buf[0] - @as(f64, a.buf[0]) * b.buf[2],
        @as(f64, a.buf[0]) * b.buf[1] - @as(f64, a.buf[1]) * b.buf[0],
    );
}

pub fn conjugate(a: c.MathQuaternionf) c.MathQuaternionf {
    return q(-a.buf[0], -a.buf[1], -a.buf[2], a.buf[3]);
}

pub fn multiply(a: c.MathQuaternionf, b: c.MathQuaternionf) c.MathQuaternionf {
    @setFloatMode(.strict);
    return q(
        @as(f64, a.buf[3]) * b.buf[0] + @as(f64, a.buf[0]) * b.buf[3] + @as(f64, a.buf[1]) * b.buf[2] - @as(f64, a.buf[2]) * b.buf[1],
        @as(f64, a.buf[3]) * b.buf[1] - @as(f64, a.buf[0]) * b.buf[2] + @as(f64, a.buf[1]) * b.buf[3] + @as(f64, a.buf[2]) * b.buf[0],
        @as(f64, a.buf[3]) * b.buf[2] + @as(f64, a.buf[0]) * b.buf[1] - @as(f64, a.buf[1]) * b.buf[0] + @as(f64, a.buf[2]) * b.buf[3],
        @as(f64, a.buf[3]) * b.buf[3] - @as(f64, a.buf[0]) * b.buf[0] - @as(f64, a.buf[1]) * b.buf[1] - @as(f64, a.buf[2]) * b.buf[2],
    );
}

pub fn rotate(a: c.MathQuaternionf, b: c.MathVector3f) c.MathVector3f {
    @setFloatMode(.strict);
    const u = v(a.buf[0], a.buf[1], a.buf[2]);
    const t = scale(cross(u, b), 2);
    return add(b, add(scale(t, a.buf[3]), cross(u, t)));
}

pub fn point(a: c.MathPose, b: c.MathVector3f) c.MathVector3f {
    return add(a.pos, rotate(a.quat, b));
}

pub fn compose(a: c.MathPose, b: c.MathPose) c.MathPose {
    return .{ .pos = point(a, b.pos), .quat = multiply(a.quat, b.quat) };
}

pub fn finite3(a: c.MathVector3f) bool {
    return std.math.isFinite(a.buf[0]) and std.math.isFinite(a.buf[1]) and std.math.isFinite(a.buf[2]);
}

pub fn finite4(a: c.MathQuaternionf) bool {
    return std.math.isFinite(a.buf[0]) and std.math.isFinite(a.buf[1]) and std.math.isFinite(a.buf[2]) and std.math.isFinite(a.buf[3]);
}

pub fn validPose(a: c.MathPose) error{Failed}!void {
    @setFloatMode(.strict);
    var n: f64 = 0;
    for (a.quat.buf) |x| n += @as(f64, x) * x;
    try require(finite3(a.pos) and finite4(a.quat) and @abs(n - 1) < 1e-4, "pose requires finite position and unit xyzw quaternion", c.SIM_INVALID);
}

pub fn canonical(x: f32) f32 {
    return if (x == 0) 0 else x;
}

pub const Hash = struct {
    value: u64 = 1469598103934665603,

    pub fn raw(self: *Hash, data: []const u8) void {
        for (data) |b| {
            self.value ^= b;
            self.value *%= 1099511628211;
        }
    }

    pub fn scalar(self: *Hash, comptime T: type, n: T) void {
        var value = n;
        if (T == f32 or T == f64) {
            if (value == 0) value = 0;
        }
        self.raw(std.mem.asBytes(&value));
    }

    pub fn vec(self: *Hash, a: c.MathVector3f) void {
        for (a.buf) |f| self.scalar(f32, f);
    }

    pub fn quat(self: *Hash, a: c.MathQuaternionf) void {
        for (a.buf) |f| self.scalar(f32, f);
    }

    pub fn state(self: *Hash, s: c.SimBodyState) void {
        self.scalar(u32, s.id);
        self.vec(s.pose.pos);
        self.quat(s.pose.quat);
        self.vec(s.center_of_mass);
        self.vec(s.linear_velocity);
        self.vec(s.angular_velocity);
    }
};

pub fn addForce(w: anytype, f: c.MathVector3f, p: c.MathVector3f, com: c.MathVector3f) void {
    @setFloatMode(.strict);
    w.*.force = add(w.*.force, f);
    w.*.torque = add(w.*.torque, cross(sub(p, com), f));
}

pub fn span(s: [*c]const u8) []const u8 {
    return std.mem.span(s);
}

pub fn text(dst: []u8, s: []const u8) error{Failed}!void {
    try require(s.len < dst.len, "name too long", c.SIM_INVALID);
    @memset(dst, 0);
    @memcpy(dst[0..s.len], s);
}
