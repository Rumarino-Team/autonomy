const std = @import("std");
const c = @import("sim_c");

pub export fn arena_init(a: ?*c.ArenaAllocator, p: ?*anyopaque, n: usize) callconv(.c) void {
    const arena = a orelse return;
    arena.data = @ptrCast(p);
    arena.capacity = if (p != null) n else 0;
    arena.offset = 0;
}

pub export fn arena_alloc(a: ?*c.ArenaAllocator, n: usize, alignment: usize) callconv(.c) ?*anyopaque {
    const arena = a orelse return null;
    if (arena.data == null or n == 0 or alignment == 0 or (alignment & (alignment - 1)) != 0 or arena.offset > arena.capacity)
        return null;
    const base = @intFromPtr(arena.data);
    if (arena.offset > std.math.maxInt(usize) - base) return null;
    const current = base + arena.offset;
    const padding = (~current +% 1) & (alignment - 1);
    const remaining = arena.capacity - arena.offset;
    if (padding > remaining or n > remaining - padding) return null;
    if (padding > std.math.maxInt(usize) - current or n > std.math.maxInt(usize) - current - padding) return null;
    const result: [*]u8 = @ptrCast(arena.data);
    arena.offset += padding + n;
    return result + arena.offset - n;
}

pub export fn arena_reset(a: ?*c.ArenaAllocator) callconv(.c) void {
    if (a) |arena| arena.offset = 0;
}

pub export fn pool_init(
    p: ?*c.PoolAllocator,
    memory: ?*anyopaque,
    nbytes: usize,
    size: usize,
    alignment: usize,
    count: u32,
) callconv(.c) c_int {
    const pool = p orelse return 0;
    pool.* = std.mem.zeroes(c.PoolAllocator);
    if (size == 0 or alignment == 0 or (alignment & (alignment - 1)) != 0 or size > std.math.maxInt(usize) - (alignment - 1))
        return 0;
    const stride = (size + alignment - 1) & ~(alignment - 1);
    if ((count != 0 and memory == null) or (count != 0 and stride != 0 and count > std.math.maxInt(usize) / stride) or nbytes < @as(usize, count) * stride or (@intFromPtr(memory) & (alignment - 1)) != 0)
        return 0;
    arena_init(&pool.arena, memory, nbytes);
    pool.element_size = size;
    pool.alignment = alignment;
    pool.stride = stride;
    pool.capacity = count;
    return 1;
}

pub export fn pool_alloc(p: ?*c.PoolAllocator) callconv(.c) ?*anyopaque {
    const pool = p orelse return null;
    if (pool.count == pool.capacity) return null;
    const value = arena_alloc(&pool.arena, pool.stride, pool.alignment) orelse return null;
    @memset(@as([*]u8, @ptrCast(value))[0..pool.element_size], 0);
    pool.count += 1;
    return value;
}

pub export fn pool_reset(p: ?*c.PoolAllocator) callconv(.c) void {
    const pool = p orelse return;
    arena_reset(&pool.arena);
    pool.count = 0;
}
