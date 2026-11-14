const std = @import("std");
const builtin = @import("builtin");
const c = @import("sim_c");
const S = @import("support.zig");

const RTLD_NOW: c_int = 2;
const RTLD_LOCAL: c_int = 0;

const Dl_info = extern struct {
    dli_fname: ?[*:0]const u8,
    dli_fbase: ?*anyopaque,
    dli_sname: ?[*:0]const u8,
    dli_saddr: ?*anyopaque,
};

extern "c" fn dlopen(path: [*:0]const u8, flags: c_int) ?*anyopaque;
extern "c" fn dlsym(handle: ?*anyopaque, name: [*:0]const u8) ?*anyopaque;
extern "c" fn dlclose(handle: ?*anyopaque) c_int;
extern "c" fn dladdr(address: *const anyopaque, info: *Dl_info) c_int;

export fn dynlib_open(path: ?[*:0]const u8) callconv(.c) ?*anyopaque {
    const name = path orelse return null;
    if (builtin.os.tag == .windows) {
        const lib = std.DynLib.open(std.mem.span(name)) catch return null;
        return lib.inner.dll;
    }
    return dlopen(name, RTLD_NOW | RTLD_LOCAL);
}

export fn dynlib_symbol(library: ?*anyopaque, name: ?[*:0]const u8) callconv(.c) ?*anyopaque {
    const module = library orelse return null;
    const symbol = name orelse return null;
    if (builtin.os.tag == .windows) {
        var lib = std.DynLib{ .inner = .{ .dll = @ptrCast(module) } };
        return lib.lookup(*anyopaque, std.mem.span(symbol));
    }
    return dlsym(module, symbol);
}

export fn dynlib_close(library: ?*anyopaque) callconv(.c) void {
    const module = library orelse return;
    if (builtin.os.tag == .windows) {
        var lib = std.DynLib{ .inner = .{ .dll = @ptrCast(module) } };
        lib.close();
        return;
    }
    _ = dlclose(module);
}

export fn dynlib_module_path(address: ?*anyopaque, out: ?[*]u8, out_len: usize) callconv(.c) c_int {
    const symbol = address orelse return 0;
    const buffer = out orelse return 0;
    if (out_len < 2) return 0;
    if (builtin.os.tag == .windows) return 0;
    var info = std.mem.zeroes(Dl_info);
    if (dladdr(symbol, &info) == 0 or info.dli_fname == null) return 0;
    const path = std.mem.span(info.dli_fname.?);
    if (path.len + 1 > out_len) return 0;
    @memcpy(buffer[0..path.len], path);
    buffer[path.len] = 0;
    return 1;
}

const GetApi = *const fn (u32, u32, *c.PhysicsBackendApi) callconv(.c) c.SimResult;

export fn physics_backend_open(b: ?*c.PhysicsBackend, path: ?[*:0]const u8, e: ?[*]u8, n: u32) callconv(.c) c.SimResult {
    return S.guard(e, n, struct {
        fn run(backend: ?*c.PhysicsBackend, file: ?[*:0]const u8) !void {
            try S.require(backend != null and backend.?.library == null and file != null, "invalid backend loader arguments", c.SIM_INVALID);
            const lib = dynlib_open(file) orelse return S.boom(c.SIM_BACKEND_ERROR, "cannot load backend shared library");
            errdefer dynlib_close(lib);
            const symbol = dynlib_symbol(lib, "sim_backend_get_api") orelse return S.boom(c.SIM_UNSUPPORTED, "missing backend entrypoint");
            const get: GetApi = @ptrCast(symbol);
            var api = std.mem.zeroes(c.PhysicsBackendApi);
            try S.check(get(5, @sizeOf(c.PhysicsBackendApi), &api), "backend ABI v5 required");
            try S.require(api.struct_size == @sizeOf(c.PhysicsBackendApi) and api.abi_version == 5 and api.world_create != null and api.world_destroy != null and api.reset != null and api.step != null and api.read_state != null and api.read_contacts != null and api.state_hash != null, "incomplete backend API", c.SIM_UNSUPPORTED);
            backend.?.api = api;
            backend.?.library = lib;
        }
    }.run(b, path));
}

export fn physics_backend_close(b: ?*c.PhysicsBackend) callconv(.c) void {
    const backend = b orelse return;
    if (backend.library) |lib| dynlib_close(lib);
    backend.* = std.mem.zeroes(c.PhysicsBackend);
}
