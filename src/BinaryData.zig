//! AUVDATA1: versioned, bounded, field-based application data. Never a memory dump.
//! Tags: null, bool, signed, unsigned, float, bytes, sequence, record, enum.
const std = @import("std");
const Io = std.Io;
pub const Kind = enum(u32) { state = 1, campaign = 2, case = 3, summary = 4, totals = 5, scene = 6, metadata = 7, run = 8 };
pub const max_bytes = 64 * 1024 * 1024;
pub const max_elements = 1000000;
pub const Tag = enum(u8) { null_value, boolean, signed, unsigned, float, bytes, sequence, record, enumeration };
pub const Encoder = struct {
    allocator: std.mem.Allocator,
    bytes: std.ArrayList(u8) = .empty,
    pub fn deinit(e: *Encoder) void {
        e.bytes.deinit(e.allocator);
    }
    fn raw(e: *Encoder, b: []const u8) !void {
        if (b.len > max_bytes -| e.bytes.items.len) return error.BinaryLimit;
        try e.bytes.appendSlice(e.allocator, b);
    }
    fn number(e: *Encoder, comptime T: type, n: T) !void {
        var b: [@sizeOf(T)]u8 = undefined;
        std.mem.writeInt(T, &b, n, .little);
        try e.raw(&b);
    }
    fn tag(e: *Encoder, t: Tag) !void {
        try e.number(u8, @intFromEnum(t));
    }
    fn text(e: *Encoder, b: []const u8) !void {
        if (b.len > max_bytes) return error.BinaryLimit;
        try e.number(u32, @intCast(b.len));
        try e.raw(b);
    }
    pub fn value(e: *Encoder, v: anytype) anyerror!void {
        const T = @TypeOf(v);
        switch (@typeInfo(T)) {
            .null => try e.tag(.null_value),
            .bool => {
                try e.tag(.boolean);
                try e.number(u8, @intFromBool(v));
            },
            .int => |i| {
                if (i.signedness == .signed) {
                    try e.tag(.signed);
                    try e.number(i64, @intCast(v));
                } else {
                    try e.tag(.unsigned);
                    try e.number(u64, @intCast(v));
                }
            },
            .comptime_int => {
                try e.tag(.signed);
                try e.number(i64, v);
            },
            .float, .comptime_float => {
                try e.tag(.float);
                try e.number(u64, @bitCast(@as(f64, v)));
            },
            .@"enum" => {
                try e.tag(.enumeration);
                try e.text(@tagName(v));
            },
            .optional => {
                if (v) |x| try e.value(x) else try e.tag(.null_value);
            },
            .array => |a| {
                if (a.child == u8) {
                    try e.tag(.bytes);
                    try e.text(&v);
                } else {
                    try e.tag(.sequence);
                    try e.number(u32, @intCast(a.len));
                    for (v) |x| try e.value(x);
                }
            },
            .vector => |a| {
                const xs: [a.len]a.child = v;
                try e.value(xs);
            },
            .pointer => |p| {
                if (p.size == .one and @typeInfo(p.child) == .array) return e.value(v.*);
                if (p.size != .slice) @compileError("binary data cannot contain pointers: " ++ @typeName(T));
                if (v.len > max_elements) return error.BinaryLimit;
                if (p.child == u8) {
                    try e.tag(.bytes);
                    try e.text(v);
                } else {
                    try e.tag(.sequence);
                    try e.number(u32, @intCast(v.len));
                    for (v) |x| try e.value(x);
                }
            },
            .@"struct" => |s| {
                try e.tag(.record);
                if (@hasField(@TypeOf(s), "fields")) {
                    try e.number(u32, @intCast(s.fields.len));
                    inline for (s.fields) |f| {
                        try e.text(f.name);
                        try e.value(@field(v, f.name));
                    }
                } else {
                    try e.number(u32, @intCast(s.field_names.len));
                    inline for (s.field_names) |name| {
                        try e.text(name);
                        try e.value(@field(v, name));
                    }
                }
            },
            else => @compileError("unsupported binary type: " ++ @typeName(T)),
        }
    }
};
pub const Decoder = struct {
    allocator: std.mem.Allocator,
    bytes: []const u8,
    offset: usize = 0,
    depth: usize = 0,
    allocated: usize = 0,
    nodes: usize = 0,
    fn raw(d: *Decoder, n: usize) ![]const u8 {
        if (n > d.bytes.len -| d.offset) return error.TruncatedBinary;
        const out = d.bytes[d.offset..][0..n];
        d.offset += n;
        return out;
    }
    fn number(d: *Decoder, comptime T: type) !T {
        return std.mem.readInt(T, (try d.raw(@sizeOf(T)))[0..@sizeOf(T)], .little);
    }
    fn tag(d: *Decoder) !Tag {
        return std.enums.fromInt(Tag, try d.number(u8)) orelse error.InvalidBinaryTag;
    }
    fn expect(d: *Decoder, expected: Tag) !void {
        if (try d.tag() != expected) return error.BinaryTypeMismatch;
    }
    fn text(d: *Decoder) ![]const u8 {
        const n = try d.number(u32);
        if (n > max_bytes) return error.BinaryLimit;
        return d.raw(n);
    }
    fn count(d: *Decoder) !u32 {
        const n = try d.number(u32);
        if (n > max_elements or n > d.bytes.len -| d.offset) return error.BinaryLimit;
        return n;
    }
    pub fn value(d: *Decoder, comptime T: type) anyerror!T {
        if (d.depth >= 64 or d.nodes >= max_elements) return error.BinaryLimit;
        d.nodes += 1;
        d.depth += 1;
        defer d.depth -= 1;
        switch (@typeInfo(T)) {
            .bool => {
                try d.expect(.boolean);
                const b = try d.number(u8);
                if (b > 1) return error.InvalidBinaryBoolean;
                return b == 1;
            },
            .int => |i| {
                try d.expect(if (i.signedness == .signed) .signed else .unsigned);
                const v = if (i.signedness == .signed) try d.number(i64) else try d.number(u64);
                return std.math.cast(T, v) orelse error.BinaryOverflow;
            },
            .float => {
                try d.expect(.float);
                return @floatCast(@as(f64, @bitCast(try d.number(u64))));
            },
            .@"enum" => {
                try d.expect(.enumeration);
                return std.meta.stringToEnum(T, try d.text()) orelse error.InvalidBinaryEnum;
            },
            .optional => |o| {
                if (d.offset < d.bytes.len and d.bytes[d.offset] == @intFromEnum(Tag.null_value)) {
                    d.offset += 1;
                    return null;
                }
                return try d.value(o.child);
            },
            .vector => |v| {
                const a = try d.value([v.len]v.child);
                return a;
            },
            .array => |a| {
                var out: T = undefined;
                if (a.child == u8) {
                    try d.expect(.bytes);
                    const s = try d.text();
                    if (s.len != a.len) return error.BinaryTypeMismatch;
                    @memcpy(&out, s);
                } else {
                    try d.expect(.sequence);
                    if (try d.count() != a.len) return error.BinaryTypeMismatch;
                    for (&out) |*x| x.* = try d.value(a.child);
                }
                return out;
            },
            .pointer => |p| {
                if (p.size != .slice) @compileError("binary data cannot restore pointers");
                if (p.child == u8) {
                    try d.expect(.bytes);
                    const text_value = try d.text();
                    if (text_value.len + 1 > max_bytes -| d.allocated) return error.BinaryLimit;
                    d.allocated += text_value.len + 1;
                    if (p.sentinel_ptr != null) return d.allocator.dupeSentinel(u8, text_value, 0);
                    return d.allocator.dupe(u8, text_value);
                }
                try d.expect(.sequence);
                const n = try d.count();
                const needed = std.math.mul(usize, @sizeOf(p.child), n) catch return error.BinaryOverflow;
                if (needed > max_bytes -| d.allocated) return error.BinaryLimit;
                d.allocated += needed;
                const out = try d.allocator.alloc(p.child, n);
                for (out) |*x| x.* = try d.value(p.child);
                return out;
            },
            .@"struct" => |s| {
                try d.expect(.record);
                var out: T = undefined;
                if (@hasField(@TypeOf(s), "fields")) {
                    if (try d.count() != s.fields.len) return error.BinaryTypeMismatch;
                    inline for (s.fields) |f| {
                        if (!std.mem.eql(u8, try d.text(), f.name)) return error.BinaryFieldMismatch;
                        @field(out, f.name) = try d.value(f.type);
                    }
                } else {
                    if (try d.count() != s.field_names.len) return error.BinaryTypeMismatch;
                    inline for (s.field_names, s.field_types) |name, Field| {
                        if (!std.mem.eql(u8, try d.text(), name)) return error.BinaryFieldMismatch;
                        @field(out, name) = try d.value(Field);
                    }
                }
                return out;
            },
            else => @compileError("unsupported binary restore type " ++ @typeName(T)),
        }
    }
};
pub const Payload = struct { kind: Kind, schema: u32, bytes: []const u8 };
pub fn pack(allocator: std.mem.Allocator, kind: Kind, schema: u32, v: anytype) ![]u8 {
    var e: Encoder = .{ .allocator = allocator };
    defer e.deinit();
    try e.value(v);
    var out: Encoder = .{ .allocator = allocator };
    errdefer out.deinit();
    try out.raw("AUVDATA1");
    try out.number(u32, @intFromEnum(kind));
    try out.number(u32, schema);
    try out.number(u64, e.bytes.items.len);
    try out.number(u32, std.hash.Crc32.hash(e.bytes.items));
    try out.raw(e.bytes.items);
    return out.bytes.toOwnedSlice(allocator);
}
pub fn unpack(bytes: []const u8) !Payload {
    var d: Decoder = .{ .allocator = undefined, .bytes = bytes };
    if (!std.mem.eql(u8, try d.raw(8), "AUVDATA1")) return error.UnsupportedBinaryVersion;
    const kind = std.enums.fromInt(Kind, try d.number(u32)) orelse return error.UnsupportedBinaryKind;
    const schema = try d.number(u32);
    const n = try d.number(u64);
    const crc = try d.number(u32);
    if (n > max_bytes or n != bytes.len -| d.offset) return error.TruncatedBinary;
    const data = try d.raw(@intCast(n));
    if (std.hash.Crc32.hash(data) != crc) return error.BinaryChecksum;
    return .{ .kind = kind, .schema = schema, .bytes = data };
}
pub fn decode(allocator: std.mem.Allocator, comptime T: type, bytes: []const u8, kind: Kind, schema: u32) !T {
    const p = try unpack(bytes);
    if (p.kind != kind or p.schema != schema) return error.UnsupportedBinarySchema;
    var d: Decoder = .{ .allocator = allocator, .bytes = p.bytes };
    const out = try d.value(T);
    if (d.offset != d.bytes.len) return error.TrailingBinaryData;
    return out;
}
pub fn writeFile(io: Io, allocator: std.mem.Allocator, path: []const u8, kind: Kind, v: anytype) !void {
    const data = try pack(allocator, kind, 1, v);
    defer allocator.free(data);
    // Reserve the final name first: a campaign cannot overwrite existing results.
    const reserved = try Io.Dir.cwd().createFile(io, path, .{ .exclusive = true });
    reserved.close(io);
    errdefer Io.Dir.cwd().deleteFile(io, path) catch {};
    const temporary = try std.fmt.allocPrint(allocator, "{s}.partial", .{path});
    defer allocator.free(temporary);
    const file = try Io.Dir.cwd().createFile(io, temporary, .{ .exclusive = true });
    errdefer Io.Dir.cwd().deleteFile(io, temporary) catch {};
    {
        defer file.close(io);
        try file.writeStreamingAll(io, data);
    }
    try Io.Dir.cwd().rename(temporary, Io.Dir.cwd(), path, io);
}
test "binary round trip excludes padding and bounds malformed input" {
    const T = struct { x: [3]f32, optional: ?u64, name: []const u8 };
    var arena = std.heap.ArenaAllocator.init(std.testing.allocator);
    defer arena.deinit();
    const a = arena.allocator();
    const data = try pack(a, .state, 1, T{ .x = .{ 1, 2, 3 }, .optional = 9, .name = "test" });
    const out = try decode(a, T, data, .state, 1);
    try std.testing.expectEqualStrings("test", out.name);
    try std.testing.expectEqual(@as(?u64, 9), out.optional);
    try std.testing.expectError(error.TruncatedBinary, unpack(data[0 .. data.len - 1]));
    data[data.len - 1] ^= 1;
    try std.testing.expectError(error.BinaryChecksum, unpack(data));
}

fn dump(d: *Decoder, w: *Io.Writer, depth: usize) anyerror!void {
    if (depth >= 64) return error.BinaryLimit;
    switch (try d.tag()) {
        .null_value => try w.writeAll("null"),
        .boolean => {
            const b = try d.number(u8);
            if (b > 1) return error.InvalidBinaryBoolean;
            try w.print("{}", .{b == 1});
        },
        .signed => try w.print("{d}", .{try d.number(i64)}),
        .unsigned => try w.print("{d}", .{try d.number(u64)}),
        .float => try w.print("{d}", .{@as(f64, @bitCast(try d.number(u64)))}),
        .bytes, .enumeration => try w.print("{f}", .{std.zig.fmtString(try d.text())}),
        .sequence => {
            const n = try d.count();
            try w.writeAll("[\n");
            for (0..n) |_| {
                try w.splatByteAll(' ', (depth + 1) * 2);
                try dump(d, w, depth + 1);
                try w.writeAll(",\n");
            }
            try w.splatByteAll(' ', depth * 2);
            try w.writeAll("]");
        },
        .record => {
            const n = try d.count();
            try w.writeAll("{\n");
            for (0..n) |_| {
                try w.splatByteAll(' ', (depth + 1) * 2);
                try w.print("{s} = ", .{try d.text()});
                try dump(d, w, depth + 1);
                try w.writeAll(",\n");
            }
            try w.splatByteAll(' ', depth * 2);
            try w.writeAll("}");
        },
    }
}
pub fn inspect(io: Io, allocator: std.mem.Allocator, path: []const u8) !void {
    const bytes = try Io.Dir.cwd().readFileAlloc(io, path, allocator, .limited(max_bytes + 28));
    defer allocator.free(bytes);
    const payload = try unpack(bytes);
    var buffer: [4096]u8 = undefined;
    var writer = Io.File.stdout().writer(io, &buffer);
    try writer.interface.print("AUVDATA1 kind={s} schema={d}\n", .{ @tagName(payload.kind), payload.schema });
    var reader: Decoder = .{ .allocator = allocator, .bytes = payload.bytes };
    try dump(&reader, &writer.interface, 0);
    if (reader.offset != reader.bytes.len) return error.TrailingBinaryData;
    try writer.interface.writeAll("\n");
    try writer.interface.flush();
}

test "Zig encoder matches cross-language golden bytes" {
    const T = struct { x: [3]f32, optional: ?u64, name: []const u8 };
    const bytes = try pack(std.testing.allocator, .metadata, 1, T{ .x = .{ 1, 2, 3 }, .optional = 9, .name = "test" });
    defer std.testing.allocator.free(bytes);
    try std.testing.expectEqualSlices(u8, @embedFile("../auvs/simulator/tests/binary-golden.bin"), bytes);
}
