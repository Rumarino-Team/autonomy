//! AUTODNA1 documents: the schema section records Zig row layout at build time; row bytes
//! follow those structs. Reads require a matching schema fingerprint.
const std = @import("std");
const abi_mod = @import("simulator/abi.zig");

pub const ColumnSpan = abi_mod.ColumnSpan;
pub const rna = abi_mod.rna;

pub const magic = "AUTODNA1";

pub const max_types = 64;
pub const max_fields = 384;
pub const max_names = 256;

const header_size: usize = 16;
const type_rec_size: usize = 16;
const field_rec_size: usize = 10;

const TypeDesc = struct {
    name_index: u16,
    size: u16,
    alignment: u8,
    elem_type: u16 = 0,
    elem_count: u16 = 0,
    field_start: u16 = 0,
    field_count: u16 = 0,
    /// For integers and enum tags: Zig `int.bits`. Zero for other types.
    logical_bits: u16 = 0,
};

const Layout = struct {
    const Field = struct {
        name_index: u16,
        type_index: u16,
        offset: u16,
        size: u16,
        flags: u16 = 0,
    };

    name_len: usize,
    names: [max_names][]const u8,
    type_len: usize,
    types: [max_types]TypeDesc,
    field_len: usize,
    fields: [max_fields]Field,
    root_type_index: u16,
    schema_len: usize,
};

/// Comptime workspace for `compileLayout`; not retained at runtime.
const CompileLayoutCtx = struct {
    names: [max_names][]const u8 = @splat(""),
    name_len: usize = 0,
    type_keys: [max_types]type = undefined,
    types: [max_types]TypeDesc = undefined,
    type_len: usize = 0,
    fields: [max_fields]Layout.Field = undefined,
    field_len: usize = 0,
    visiting: [max_types]bool = @splat(false),
};

fn compileLayoutInternName(ctx: *CompileLayoutCtx, name: []const u8) u16 {
    @setEvalBranchQuota(100_000);
    for (ctx.names[0..ctx.name_len], 0..) |existing, i| {
        if (std.mem.eql(u8, existing, name)) return @intCast(i);
    }
    if (ctx.name_len >= max_names) @compileError("too many schema names");
    ctx.names[ctx.name_len] = name;
    const index: u16 = @intCast(ctx.name_len);
    ctx.name_len += 1;
    return index;
}

fn compileLayoutIntern(ctx: *CompileLayoutCtx, comptime T: type) u16 {
    @setEvalBranchQuota(100_000);
    for (ctx.type_keys[0..ctx.type_len], 0..) |key, i| {
        if (key == T) {
            if (ctx.visiting[i]) @compileError("recursive schema type " ++ @typeName(T));
            return @intCast(i);
        }
    }
    if (ctx.type_len >= max_types) @compileError("too many schema types");
    const index: u16 = @intCast(ctx.type_len);
    ctx.type_len += 1;
    ctx.type_keys[index] = T;
    ctx.visiting[index] = true;

    const type_info = @typeInfo(T);
    var desc: TypeDesc = .{
        .name_index = undefined,
        .size = fitU16(@sizeOf(T), @typeName(T)),
        .alignment = fitU8(@alignOf(T), @typeName(T)),
    };
    switch (type_info) {
        .int => |int| {
            if (int.bits == 0) @compileError("schema integers must be runtime-sized: " ++ @typeName(T));
            desc.logical_bits = fitU16(int.bits, @typeName(T));
        },
        .float => |float| {
            if (float.bits != 32 and float.bits != 64) @compileError("schema floats are f32 or f64: " ++ @typeName(T));
        },
        .bool => {},
        .@"enum" => |e| {
            if (@typeInfo(e.tag_type) != .int) @compileError("schema enums need integer tags: " ++ @typeName(T));
            const tag = @typeInfo(e.tag_type).int;
            if (tag.bits == 0) @compileError("schema enum tag must be runtime-sized: " ++ @typeName(T));
            desc.logical_bits = fitU16(tag.bits, @typeName(T));
        },
        .array => |arr| {
            if (arr.sentinel() != null) @compileError("schema arrays cannot have sentinels: " ++ @typeName(T));
            desc.elem_type = compileLayoutIntern(ctx, arr.child);
            desc.elem_count = fitU16(arr.len, @typeName(T));
        },
        .vector => |vec| {
            desc.elem_type = compileLayoutIntern(ctx, vec.child);
            desc.elem_count = fitU16(vec.len, @typeName(T));
        },
        .optional => |opt| {
            desc.elem_type = compileLayoutIntern(ctx, opt.child);
        },
        .@"struct" => |s| {
            if (s.is_tuple) @compileError("schema rows must be structs: " ++ @typeName(T));
            desc.field_start = fitU16(ctx.field_len, @typeName(T));
            inline for (s.field_names, s.field_types, s.field_attrs) |field_name, FieldType, attrs| {
                if (attrs.@"comptime") continue;
                const field_type_index = compileLayoutIntern(ctx, FieldType);
                if (ctx.field_len >= max_fields) @compileError("too many schema fields");
                ctx.fields[ctx.field_len] = .{
                    .name_index = compileLayoutInternName(ctx, field_name),
                    .type_index = field_type_index,
                    .offset = fitU16(@offsetOf(T, field_name), field_name),
                    .size = fitU16(@sizeOf(FieldType), @typeName(FieldType)),
                    .flags = 0,
                };
                ctx.field_len += 1;
            }
            desc.field_count = fitU16(ctx.field_len - desc.field_start, @typeName(T));
        },
        else => @compileError("unsupported schema type " ++ @typeName(T)),
    }
    desc.name_index = compileLayoutInternName(ctx, schemaTypeName(T));
    ctx.types[index] = desc;
    ctx.visiting[index] = false;
    return index;
}

/// `@typeName` without the module path, which differs with the compilation root
/// (`abi.Entity` vs `simulator.abi.Entity`) and would change the schema hash.
fn schemaTypeName(comptime T: type) []const u8 {
    const full = @typeName(T);
    const end = std.mem.indexOfScalar(u8, full, '(') orelse full.len;
    const dot = std.mem.lastIndexOfScalar(u8, full[0..end], '.') orelse return full;
    return full[dot + 1 ..];
}

fn compileLayout(comptime Root: type) Layout {
    @setEvalBranchQuota(1_000_000);
    const info = @typeInfo(Root).@"struct";
    if (info.is_tuple) @compileError("schema document must be a struct");

    var ctx: CompileLayoutCtx = .{};
    const root_name = schemaTypeName(Root);
    const root_name_index = compileLayoutInternName(&ctx, root_name);

    const field_start = ctx.field_len;
    inline for (info.field_names, info.field_types, info.field_attrs) |field_name, FieldType, attrs| {
        if (attrs.@"comptime") continue;
        if (@typeInfo(FieldType) == .pointer) @compileError("schema documents cannot contain slices: " ++ field_name);
        const type_index = compileLayoutIntern(&ctx, FieldType);
        if (ctx.field_len >= max_fields) @compileError("too many schema fields");
        ctx.fields[ctx.field_len] = .{
            .name_index = compileLayoutInternName(&ctx, field_name),
            .type_index = type_index,
            .offset = @offsetOf(Root, field_name),
            .size = fitU16(@sizeOf(FieldType), field_name),
            .flags = 0,
        };
        ctx.field_len += 1;
    }

    if (ctx.type_len >= max_types) @compileError("too many schema types");
    const index: u16 = @intCast(ctx.type_len);
    ctx.type_len += 1;
    ctx.type_keys[index] = Root;
    ctx.types[index] = .{
        .name_index = root_name_index,
        .size = fitU16(@sizeOf(Root), root_name),
        .alignment = fitU8(@alignOf(Root), root_name),
        .field_start = fitU16(field_start, root_name),
        .field_count = fitU16(ctx.field_len - field_start, root_name),
    };
    var layout: Layout = .{
        .name_len = ctx.name_len,
        .names = ctx.names,
        .type_len = ctx.type_len,
        .types = ctx.types,
        .field_len = ctx.field_len,
        .fields = ctx.fields,
        .root_type_index = index,
        .schema_len = 0,
    };
    layout.schema_len = schemaBytes(layout);
    return layout;
}

fn validateAutodnaHeader(comptime Root: type, bytes: []const u8) !void {
    const layout = DocumentLayout(Root).layout;
    if (bytes.len < header_size or !std.mem.eql(u8, bytes[0..magic.len], magic)) return error.InvalidSchema;
    const schema_len = std.mem.readInt(u32, bytes[magic.len..][0..4], .little);
    if (schema_len != layout.schema_len) return error.IncompatibleSchema;
    const root_type_index = std.mem.readInt(u32, bytes[12..][0..4], .little);
    if (root_type_index != layout.root_type_index) return error.IncompatibleSchema;
    const expected_hash = comptime documentSchemaHash(Root);
    const hash = dnaSectionHash(bytes) orelse return error.InvalidSchema;
    if (hash != expected_hash) return error.IncompatibleSchema;
}

fn fitU16(value: usize, label: []const u8) u16 {
    if (value > std.math.maxInt(u16)) @compileError("schema size exceeds u16: " ++ label);
    return @intCast(value);
}

fn fitU8(value: usize, label: []const u8) u8 {
    if (value > std.math.maxInt(u8)) @compileError("schema alignment exceeds u8: " ++ label);
    return @intCast(value);
}

fn schemaBytes(comptime layout: Layout) usize {
    var size: usize = 4;
    for (layout.names[0..layout.name_len]) |name| {
        size += 2 + name.len;
        if (name.len % 2 != 0) size += 1;
    }
    size += 4 + layout.type_len * type_rec_size;
    size += 4 + layout.field_len * field_rec_size;
    return size;
}

fn DocumentLayout(comptime Root: type) type {
    return struct {
        const layout = compileLayout(Root);
        const schema_end = header_size + layout.schema_len;
        const payload = std.mem.alignForward(usize, schema_end, @alignOf(Root));
        const prefix = blk: {
            @setEvalBranchQuota(1_000_000);
            var bytes: [payload]u8 = @splat(0);
            @memcpy(bytes[0..magic.len], magic);
            std.mem.writeInt(u32, bytes[8..12], @intCast(layout.schema_len), .little);
            std.mem.writeInt(u32, bytes[12..16], layout.root_type_index, .little);
            writeSchemaSection(bytes[header_size..schema_end], layout);
            break :blk bytes;
        };
    };
}

/// Fingerprint of the schema section `write` emits for a document struct.
pub fn documentSchemaHash(comptime Root: type) u64 {
    @setEvalBranchQuota(1_000_000);
    const D = DocumentLayout(Root);
    return comptime std.hash.Wyhash.hash(0, D.prefix[header_size..D.schema_end]);
}

/// Wyhash of the DNA schema bytes in a finished AUTODNA blob (`header_size`..schema end), or null if not AUTODNA.
pub fn dnaSectionHash(bytes: []const u8) ?u64 {
    if (bytes.len < header_size or !std.mem.eql(u8, bytes[0..magic.len], magic)) return null;
    const schema_len = std.mem.readInt(u32, bytes[magic.len..][0..4], .little);
    const end = header_size + @as(usize, schema_len);
    if (end > bytes.len) return null;
    return std.hash.Wyhash.hash(0, bytes[header_size..end]);
}

fn writeSchemaSection(dst: []u8, comptime layout: Layout) void {
    var cursor: usize = 0;
    std.mem.writeInt(u32, dst[cursor..][0..4], @intCast(layout.name_len), .little);
    cursor += 4;
    for (layout.names[0..layout.name_len]) |name| {
        std.mem.writeInt(u16, dst[cursor..][0..2], @intCast(name.len), .little);
        cursor += 2;
        @memcpy(dst[cursor..][0..name.len], name);
        cursor += name.len;
        if (name.len % 2 != 0) {
            dst[cursor] = 0;
            cursor += 1;
        }
    }
    std.mem.writeInt(u32, dst[cursor..][0..4], @intCast(layout.type_len), .little);
    cursor += 4;
    for (layout.types[0..layout.type_len]) |desc| {
        std.mem.writeInt(u16, dst[cursor..][0..2], desc.name_index, .little);
        std.mem.writeInt(u16, dst[cursor + 2 ..][0..2], desc.size, .little);
        dst[cursor + 4] = desc.alignment;
        dst[cursor + 5] = 0; // reserved (v2: type class inferred from name and field/elem counts)
        std.mem.writeInt(u16, dst[cursor + 6 ..][0..2], desc.elem_type, .little);
        std.mem.writeInt(u16, dst[cursor + 8 ..][0..2], desc.elem_count, .little);
        std.mem.writeInt(u16, dst[cursor + 10 ..][0..2], desc.field_start, .little);
        std.mem.writeInt(u16, dst[cursor + 12 ..][0..2], desc.field_count, .little);
        std.mem.writeInt(u16, dst[cursor + 14 ..][0..2], desc.logical_bits, .little);
        cursor += type_rec_size;
    }
    std.mem.writeInt(u32, dst[cursor..][0..4], @intCast(layout.field_len), .little);
    cursor += 4;
    for (layout.fields[0..layout.field_len]) |field| {
        std.mem.writeInt(u16, dst[cursor..][0..2], field.name_index, .little);
        std.mem.writeInt(u16, dst[cursor + 2 ..][0..2], field.type_index, .little);
        std.mem.writeInt(u16, dst[cursor + 4 ..][0..2], field.offset, .little);
        std.mem.writeInt(u16, dst[cursor + 6 ..][0..2], field.size, .little);
        std.mem.writeInt(u16, dst[cursor + 8 ..][0..2], field.flags, .little);
        cursor += field_rec_size;
    }
}

fn isColumnRoot(comptime Root: type) bool {
    const info = @typeInfo(Root);
    if (info != .@"struct" or info.@"struct".is_tuple) return false;
    var any = false;
    for (info.@"struct".field_types, info.@"struct".field_attrs) |Field, attrs| {
        if (attrs.@"comptime") continue;
        if (Field != ColumnSpan) return false;
        any = true;
    }
    return any;
}

pub fn write(allocator: std.mem.Allocator, comptime Root: type, value: anytype) ![]align(@alignOf(u64)) u8 {
    if (comptime isColumnRoot(Root)) return writeColumns(allocator, Root, value);
    if (@TypeOf(value) != Root) @compileError("document value does not match " ++ @typeName(Root));
    const out = try allocator.alignedAlloc(u8, .of(u64), DocumentLayout(Root).payload + @sizeOf(Root));
    _ = writeRowInto(out, value) catch unreachable;
    return out;
}

/// Encode a fixed row into caller-owned storage. The document type is inferred from the row.
pub fn writeRowInto(buf: []u8, value: anytype) error{NoSpaceLeft}![]u8 {
    const Root = @TypeOf(value);
    if (comptime isColumnRoot(Root)) @compileError("use write with column slices for " ++ @typeName(Root));
    const payload = DocumentLayout(Root).payload;
    const total = payload + @sizeOf(Root);
    if (buf.len < total) return error.NoSpaceLeft;
    const out = buf[0..total];
    @memcpy(out[0..payload], &DocumentLayout(Root).prefix);
    @memset(out[payload..], 0);

    const info = @typeInfo(Root).@"struct";
    inline for (info.field_names, info.field_types, info.field_attrs) |field_name, FieldType, attrs| {
        if (attrs.@"comptime") continue;
        const field = @field(value, field_name);
        const at = payload + @offsetOf(Root, field_name);
        @memcpy(out[at..][0..@sizeOf(FieldType)], std.mem.asBytes(&field));
    }
    return out;
}

pub fn read(comptime Root: type, bytes: []const u8) !Root {
    try validateAutodnaHeader(Root, bytes);
    const payload = DocumentLayout(Root).payload;
    if (payload + @sizeOf(Root) > bytes.len) return error.InvalidSchema;

    const info = @typeInfo(Root).@"struct";
    var out: Root = undefined;
    inline for (info.field_names, info.field_types, info.field_attrs) |field_name, FieldType, attrs| {
        if (attrs.@"comptime") continue;
        const at = payload + @offsetOf(Root, field_name);
        @memcpy(std.mem.asBytes(&@field(out, field_name)), bytes[at..][0..@sizeOf(FieldType)]);
    }
    return out;
}

/// Same-build documents: schema section Wyhash must match `documentSchemaHash(Root)`.
pub fn matchesDocument(comptime Root: type, bytes: []const u8) bool {
    validateAutodnaHeader(Root, bytes) catch return false;
    return true;
}

/// Decode the root spans and return the byte offset they are relative to.
pub fn readColumnDocument(comptime Doc: type, bytes: []const u8) !struct { row: Doc, payload: usize } {
    if (comptime !isColumnRoot(Doc)) @compileError("expected a column document: " ++ @typeName(Doc));
    return .{
        .row = try read(Doc, bytes),
        .payload = DocumentLayout(Doc).payload,
    };
}

fn writeColumns(allocator: std.mem.Allocator, comptime Doc: type, build: anytype) ![]align(@alignOf(u64)) u8 {
    const info = @typeInfo(Doc).@"struct";
    const payload = DocumentLayout(Doc).payload;

    var tail: usize = payload + @sizeOf(Doc);
    var col_off: [max_fields]u32 = undefined;
    var col_cnt: [max_fields]u32 = undefined;
    var col_n: usize = 0;

    inline for (info.field_names, info.field_types) |field_name, FieldType| {
        if (FieldType != ColumnSpan) @compileError("column document fields must be ColumnSpan");
        const source = @field(build, field_name);
        const Child = std.meta.Child(@TypeOf(source));
        const byte_len: usize = if (Child == u8)
            source.len
        else
            source.len * @sizeOf(Child);
        if (byte_len > std.math.maxInt(u32)) return error.SchemaTooLarge;
        const elem_align: usize = if (Child == u8) 1 else @alignOf(Child);
        tail = std.mem.alignForward(usize, tail, elem_align);
        if (col_n >= col_off.len) return error.SchemaTooLarge;
        const rel = tail - payload;
        if (rel > std.math.maxInt(u32) or tail + byte_len > std.math.maxInt(u32)) return error.SchemaTooLarge;
        col_off[col_n] = @intCast(rel);
        col_cnt[col_n] = if (Child == u8) @intCast(byte_len) else @intCast(source.len);
        tail += byte_len;
        col_n += 1;
    }

    const out = try allocator.alignedAlloc(u8, .of(u64), tail);
    @memcpy(out[0..payload], &DocumentLayout(Doc).prefix);
    @memset(out[payload..], 0);

    col_n = 0;
    inline for (info.field_names, info.field_types) |field_name, FieldType| {
        _ = FieldType;
        const at = payload + @offsetOf(Doc, field_name);
        const span = ColumnSpan{ .count = col_cnt[col_n], .offset = col_off[col_n] };
        @memcpy(out[at..][0..@sizeOf(ColumnSpan)], std.mem.asBytes(&span));
        const source = @field(build, field_name);
        const Child = std.meta.Child(@TypeOf(source));
        if (col_cnt[col_n] != 0) {
            const start = payload + col_off[col_n];
            if (Child == u8) {
                @memcpy(out[start..][0..source.len], source);
            } else {
                @memcpy(out[start..][0..std.mem.sliceAsBytes(source).len], std.mem.sliceAsBytes(source));
            }
        }
        col_n += 1;
    }
    return out;
}

/// Struct of the named fields of `Src`, with the same field types.
pub fn projectRow(comptime Src: type, comptime names: []const []const u8) type {
    var field_names: [names.len][]const u8 = undefined;
    var field_types: [names.len]type = undefined;
    var field_attrs: [names.len]std.builtin.Type.Struct.FieldAttributes = @splat(.{});
    for (names, 0..) |name, i| {
        const Field = @FieldType(Src, name);
        if (@typeInfo(Field) == .pointer) @compileError("project rows cannot be slices: " ++ name);
        field_names[i] = name;
        field_types[i] = Field;
    }
    return @Struct(.auto, null, &field_names, &field_types, &field_attrs);
}

pub fn copyProject(comptime Src: type, comptime names: []const []const u8, src: *const Src) projectRow(Src, names) {
    const Row = projectRow(Src, names);
    var out: Row = undefined;
    inline for (names) |name| @field(out, name) = @field(src, name);
    return out;
}