const std = @import("std");
const c = @import("sim_c");
const auv = @import("auv_c");
const S = @import("support.zig");

extern fn dynlib_module_path(address: ?*const anyopaque, out: [*]u8, out_len: usize) callconv(.c) c_int;
extern fn sim_robot_find(name: [*:0]const u8) callconv(.c) ?*const c.Robot;
extern fn simulation_scene_create(s: *c.Simulation, scene: [*:0]const u8, robot: *const c.Robot, e: [*]u8, n: u32) callconv(.c) c.SimResult;
extern fn simulation_scene_load(s: *c.Simulation, path: [*:0]const u8, robot: *const c.Robot, e: [*]u8, n: u32) callconv(.c) c.SimResult;
extern fn simulation_destroy(s: *c.Simulation) callconv(.c) void;
extern fn simulation_validate(s: *const c.Simulation, e: [*]u8, n: u32) callconv(.c) c.SimResult;
extern fn physics_backend_open(b: *c.PhysicsBackend, path: [*:0]const u8, e: [*]u8, n: u32) callconv(.c) c.SimResult;
extern fn physics_backend_close(b: *c.PhysicsBackend) callconv(.c) void;
extern fn simulation_runtime_create(m: *const c.Simulation, b: *const c.PhysicsBackend, registry: ?*const c.ForceRegistry, out: *?*anyopaque, e: [*]u8, n: u32) callconv(.c) c.SimResult;
extern fn simulation_runtime_destroy(r: ?*anyopaque) callconv(.c) void;
extern fn simulation_runtime_step(r: *anyopaque, input: [*]const f64, count: u32, e: [*]u8, n: u32) callconv(.c) c.SimResult;
extern fn simulation_runtime_states(r: *const anyopaque, p: *?[*]const c.SimBodyState, n: *u32) callconv(.c) c.SimResult;
extern fn simulation_runtime_observe(r: *anyopaque, out: *c.Observation, e: [*]u8, n: u32) callconv(.c) c.SimResult;
extern fn simulation_runtime_tick(r: ?*const anyopaque) callconv(.c) u64;
extern fn simulation_runtime_hash(r: ?*const anyopaque) callconv(.c) u64;
extern "c" fn getenv(name: [*:0]const u8) ?[*:0]u8;
extern "c" fn system(command: [*:0]const u8) c_int;
extern "c" fn printf(format: [*:0]const u8, ...) callconv(.c) c_int;
extern "c" fn fflush(stream: ?*anyopaque) c_int;
extern "c" fn socket(domain: c_int, ty: c_int, protocol: c_int) c_int;
extern "c" fn sendto(sock: c_int, buf: [*]const u8, len: usize, flags: c_int, addr: *const SockAddr, addrlen: u32) isize;
extern "c" fn close(fd: c_int) c_int;
extern "c" fn clock_gettime(clk: i32, tp: *Timespec) c_int;
extern "c" fn access(path: [*:0]const u8, mode: c_int) c_int;
extern "c" fn unlink(path: [*:0]const u8) c_int;

const Timespec = extern struct { sec: i64, nsec: i64 };

fn zprint(buf: []u8, comptime fmt: []const u8, args: anytype) ![:0]u8 {
    return std.fmt.bufPrintSentinel(buf, fmt, args, 0);
}

extern "c" fn nanosleep(req: *const Timespec, rem: ?*Timespec) c_int;

fn sleepNs(ns: u64) void {
    var req = Timespec{ .sec = @intCast(ns / 1_000_000_000), .nsec = @intCast(ns % 1_000_000_000) };
    _ = nanosleep(&req, null);
}

fn monoNs() i128 {
    var ts = Timespec{ .sec = 0, .nsec = 0 };
    _ = clock_gettime(1, &ts);
    return @as(i128, ts.sec) * 1_000_000_000 + ts.nsec;
}

fn vec3(value: c.MathVector3f) auv.MathVector3f {
    return .{ .buf = value.buf };
}
fn quat(value: c.MathQuaternionf) auv.MathQuaternionf {
    return .{ .buf = value.buf };
}
fn pose(value: c.MathPose) auv.MathPose {
    return .{ .pos = vec3(value.pos), .quat = quat(value.quat) };
}

const SockAddr = extern struct {
    family: u16,
    port: u16,
    addr: u32,
    zero: [8]u8 = @splat(0),
};

const Link = struct {
    fd: c_int = -1,
    port: u16 = 9760,
    next_time: f64 = 0,

    fn openEnv(self: *Link) void {
        self.closeSocket();
        const flag = getenv("AUTONOMY_SIM_BLENDER") orelse return;
        if (flag[0] == 0 or flag[0] == '0') return;
        var port: u16 = 9760;
        if (getenv("AUTONOMY_SIM_BLENDER_PORT")) |text| {
            const parsed = std.fmt.parseInt(u32, std.mem.span(text), 10) catch 0;
            if (parsed > 0 and parsed <= 65535) port = @intCast(parsed);
        }
        self.open(true, port);
    }

    fn open(self: *Link, enabled: bool, port: u16) void {
        self.closeSocket();
        self.port = port;
        if (!enabled or port == 0) return;
        const fd = socket(2, 2 | 0x800 | 0x80000, 0);
        if (fd < 0) return;
        self.fd = fd;
    }

    fn closeSocket(self: *Link) void {
        if (self.fd >= 0) _ = close(self.fd);
        self.fd = -1;
        self.next_time = 0;
    }

    fn publish(self: *Link, runtime: *anyopaque, model: *const c.Simulation, observation: c.Observation, time_ns: u64) void {
        if (self.fd < 0) return;
        const time = @as(f64, @floatFromInt(time_ns)) * 1e-9;
        if (time + 1e-9 < self.next_time) return;
        var states: ?[*]const c.SimBodyState = null;
        var count: u32 = 0;
        if (simulation_runtime_states(runtime, &states, &count) != c.SIM_OK or states == null) return;
        var packet: [8192]u8 = undefined;
        var used: usize = 0;
        var poses: u16 = 0;
        const pushPose = struct {
            fn add(buf: []u8, len: *usize, nposes: *u16, id: u32, name: []const u8, body_pose: c.MathPose) void {
                if (name.len == 0 or nposes.* == 64 or len.* + 92 > buf.len) return;
                nposes.* += 1;
                std.mem.writeInt(u32, buf[len.*..][0..4], id, .little);
                len.* += 4;
                @memset(buf[len.*..][0..32], 0);
                @memcpy(buf[len.*..][0..@min(name.len, 31)], name[0..@min(name.len, 31)]);
                len.* += 32;
                const numbers = [_]f64{
                    -body_pose.pos.buf[0], body_pose.pos.buf[1], -body_pose.pos.buf[2],
                    body_pose.quat.buf[3], -body_pose.quat.buf[0], body_pose.quat.buf[1], -body_pose.quat.buf[2],
                };
                for (numbers) |value| {
                    std.mem.writeInt(u64, buf[len.*..][0..8], @bitCast(value), .little);
                    len.* += 8;
                }
            }
        }.add;
        var i: u32 = 0;
        while (i < model.object_count and i < count) : (i += 1) {
            const body = model.object_table[i].*.body;
            if (body.motion_type != c.SIM_DYNAMIC and body.tracked == 0 and body.fuzz == 0) continue;
            const name = std.mem.sliceTo(&body.name, 0);
            pushPose(&packet, &used, &poses, body.id, name, states.?[i].pose);
            if (std.mem.eql(u8, name, "gate opening")) pushPose(&packet, &used, &poses, body.id, "gate", states.?[i].pose);
        }
        pushPose(&packet, &used, &poses, std.math.maxInt(u32), "camera", observation.camera);
        var header: [16 + 8192]u8 = undefined;
        header[0..6].* = .{ 'A', 'M', 'J', 'B', 2, 0 };
        std.mem.writeInt(u16, header[6..8], poses, .little);
        std.mem.writeInt(u64, header[8..16], @bitCast(time), .little);
        @memcpy(header[16..][0..used], packet[0..used]);
        var address = SockAddr{ .family = 2, .port = std.mem.nativeToBig(u16, self.port), .addr = std.mem.nativeToBig(u32, 0x7f000001) };
        _ = sendto(self.fd, &header, 16 + used, 0x40, &address, @sizeOf(SockAddr));
        self.next_time = time + 1.0 / 30.0;
    }
};

const Context = struct {
    owned: c.Simulation = std.mem.zeroes(c.Simulation),
    model: ?*const c.Simulation = null,
    backend: c.PhysicsBackend = std.mem.zeroes(c.PhysicsBackend),
    runtime: ?*anyopaque = null,
    status: auv.AuvSimulationStatus = std.mem.zeroes(auv.AuvSimulationStatus),
    commands: [8]f64 = @splat(0),
    limit: u64 = 21600,
    frame_ticks: u64 = 6,
    min_depth: f64 = 0,
    max_depth: f64 = 0,
    realtime: bool = false,
    print_summary: bool = true,
    started_ns: i128 = 0,
    link: Link = .{},

    fn now(_: *Context) i128 {
        return monoNs();
    }

    fn release(self: *Context) void {
        self.link.closeSocket();
        simulation_runtime_destroy(self.runtime);
        self.runtime = null;
        physics_backend_close(&self.backend);
        simulation_destroy(&self.owned);
        self.model = null;
    }
};

threadlocal var ctx: Context = .{};

fn environment(key: [*:0]const u8, fallback: [*:0]const u8) [*:0]const u8 {
    const value = getenv(key) orelse return fallback;
    return if (value[0] == 0) fallback else value;
}

fn setting(key: [*:0]const u8, fallback: u64) !u64 {
    const text = getenv(key) orelse return fallback;
    return std.fmt.parseInt(u64, std.mem.span(text), 10) catch S.boom(c.SIM_INVALID, "invalid simulator integer setting");
}

fn failStatus(message: []const u8) void {
    ctx.status.code = 2;
    ctx.status.tick = simulation_runtime_tick(ctx.runtime);
    const len = @min(message.len, ctx.status.message.len - 1);
    @memcpy(ctx.status.message[0..len], message[0..len]);
    ctx.status.message[len] = 0;
    ctx.commands = @splat(0);
}

fn endsWith(value: []const u8, suffix: []const u8) bool {
    if (value.len < suffix.len) return false;
    for (suffix, 0..) |right, i| {
        var left = value[value.len - suffix.len + i];
        if (left >= 'A' and left <= 'Z') left = left - 'A' + 'a';
        var expected = right;
        if (expected >= 'A' and expected <= 'Z') expected = expected - 'A' + 'a';
        if (left != expected) return false;
    }
    return true;
}

fn timestamp() u64 {
    return simulation_runtime_tick(ctx.runtime) * ctx.model.?.settings.dt_numerator * 1000000000 / ctx.model.?.settings.dt_denominator;
}

fn validateTiming() !void {
    try S.require(ctx.frame_ticks != 0 and ctx.limit != 0 and ctx.limit % ctx.frame_ticks == 0, "tick budget must be positive and divisible by frame ticks", c.SIM_INVALID);
    const scale = @as(u64, ctx.model.?.settings.dt_numerator) * 1000000000;
    try S.require(ctx.model.?.settings.dt_numerator != 0 and ctx.model.?.settings.dt_denominator != 0 and scale != 0 and ctx.limit <= @as(u64, std.math.maxInt(i64)) / scale, "simulation timestamp overflow", c.SIM_INVALID);
}

fn depth() !void {
    var states: ?[*]const c.SimBodyState = null;
    var count: u32 = 0;
    try S.check(simulation_runtime_states(ctx.runtime.?, &states, &count), "cannot read depth");
    const z = states.?[ctx.model.?.robot.object.id.index].pose.pos.buf[2];
    ctx.min_depth = @min(ctx.min_depth, z);
    ctx.max_depth = @max(ctx.max_depth, z);
}

fn zeroFrame(frame: *auv.AuvFrame) void {
    frame.* = std.mem.zeroes(auv.AuvFrame);
    frame.camera_pose.quat = quat(S.q(0, 0, 0, 1));
    frame.imu_quat = frame.camera_pose.quat;
}

fn fillFrame(frame: *auv.AuvFrame, observation: c.Observation, time: u64) !void {
    zeroFrame(frame);
    frame.timestamp = time;
    frame.camera_pose = pose(observation.body);
    frame.position_ok = .{ .x = observation.position_axes & 1 != 0, .y = observation.position_axes & 2 != 0, .z = observation.position_axes & 4 != 0 };
    frame.attitude_ok = .{ .x = observation.attitude_axes & 1 != 0, .y = observation.attitude_axes & 2 != 0, .z = observation.attitude_axes & 4 != 0 };
    frame.imu_quat = quat(observation.imu);
    frame.accel = vec3(observation.accel);
    frame.gyro = vec3(observation.gyro);
    frame.pressure_depth = @floatCast(observation.depth);
    frame.pressure_depth_ok = observation.depth_ok != 0;
    frame.image_width = observation.image_width;
    frame.image_height = observation.image_height;
    var i: u32 = 0;
    const limit = @min(observation.object_count, auv.AUV_FRAME_MAX_OBJECTS);
    while (i < limit) : (i += 1) {
        const object = observation.objects[i];
        try S.require(object.cls <= 255, "object class cannot be represented by AuvFrame", c.SIM_INVALID);
        frame.objects[frame.objects_len] = .{ .bbox = .{ .pose = pose(object.pose), .size = vec3(object.size) }, .id = object.id, .cls = @intCast(object.cls) };
        frame.objects_len += 1;
        frame.objects2d[frame.objects2d_len] = .{
            .top_left = .{ .x = object.x0, .y = object.y0 },
            .bottom_right = .{ .x = object.x1, .y = object.y1 },
            .id = object.id,
            .cls = @intCast(object.cls),
        };
        frame.objects2d_len += 1;
    }
}

fn moduleDir(buffer: []u8) ![]const u8 {
    try S.require(dynlib_module_path(&auv_init, buffer.ptr, buffer.len) != 0, "cannot locate simulator plugin", c.SIM_INVALID);
    const path = std.mem.sliceTo(buffer, 0);
    return std.fs.path.dirname(path) orelse ".";
}

fn quote(out: []u8, path: []const u8) ![]const u8 {
    if (out.len < path.len + 2) return S.boom(c.SIM_INVALID, "path too long");
    out[0] = '"';
    var n: usize = 1;
    for (path) |ch| {
        if (ch == '"') {
            if (n + 2 >= out.len) return S.boom(c.SIM_INVALID, "path too long");
            out[n] = '\\';
            n += 1;
        }
        out[n] = ch;
        n += 1;
    }
    out[n] = '"';
    return out[0 .. n + 1];
}

fn findScript(start: []const u8, out: []u8) ?[]const u8 {
    var dir = start;
    var i: u32 = 0;
    while (i < 8) : (i += 1) {
        const suffixes = [_][]const u8{ "/tools/blender/export_snapshot.py", "/share/autonomy/tools/blender/export_snapshot.py" };
        for (suffixes) |suffix| {
            const path = std.fmt.bufPrint(out[0 .. out.len - 1], "{s}{s}", .{ dir, suffix }) catch continue;
            out[path.len] = 0;
            if (access(out[0..path.len :0], 0) == 0) return path;
        }
        dir = std.fs.path.dirname(dir) orelse break;
    }
    return null;
}

fn blenderExe() []const u8 {
    if (getenv("AUTONOMY_BLENDER")) |configured| {
        if (configured[0] != 0 and access(configured, 0) == 0) return std.mem.span(configured);
    }
    return "blender";
}

fn exportBlend(blend: []const u8, module_directory: []const u8, output: []u8) ![]const u8 {
    var blend_z: [4096]u8 = undefined;
    const blend_path = zprint(&blend_z, "{s}", .{blend}) catch return S.boom(c.SIM_INVALID, "path too long");
    if (access(blend_path, 0) != 0) return S.boom(c.SIM_INVALID, "cannot read blend file");
    var script_buf: [4096]u8 = undefined;
    const script = findScript(module_directory, &script_buf) orelse return S.boom(c.SIM_INVALID, "cannot find blend snapshot exporter");
    const name = std.fmt.bufPrint(output, "/tmp/autonomy-scene-{d}.bin", .{monoNs()}) catch return S.boom(c.SIM_INVALID, "path too long");
    var command: [8192]u8 = undefined;
    var quoted_blend: [4096]u8 = undefined;
    var quoted_script: [4096]u8 = undefined;
    var quoted_output: [4096]u8 = undefined;
    var quoted_blender: [4096]u8 = undefined;
    const text = zprint(&command, "{s} --background --factory-startup --disable-autoexec {s} --python {s} -- --output {s}", .{
        try quote(&quoted_blender, blenderExe()),
        try quote(&quoted_blend, blend),
        try quote(&quoted_script, script),
        try quote(&quoted_output, name),
    }) catch return S.boom(c.SIM_INVALID, "blend command too long");
    try S.require(system(text.ptr) == 0, "blend export failed", c.SIM_INVALID);
    var name_z: [512]u8 = undefined;
    const produced = zprint(&name_z, "{s}", .{name}) catch return S.boom(c.SIM_INVALID, "path too long");
    if (access(produced, 0) != 0) return S.boom(c.SIM_INVALID, "blend export failed");
    return name;
}

fn loadOwned(scene: []const u8, robot: *const c.Robot, message: []u8) !void {
    if (endsWith(scene, ".blend") or endsWith(scene, ".bin")) {
        var module_path: [32768]u8 = undefined;
        const directory = try moduleDir(&module_path);
        var output: [512]u8 = undefined;
        var source: []const u8 = scene;
        var temporary: []const u8 = "";
        if (endsWith(scene, ".blend")) {
            temporary = try exportBlend(scene, directory, &output);
            source = temporary;
        }
        defer if (temporary.len != 0) {
            var doomed: [512]u8 = undefined;
            if (zprint(&doomed, "{s}", .{temporary})) |z| _ = unlink(z) else |_| {}
        };
        var path_z: [1024]u8 = undefined;
        const z = zprint(&path_z, "{s}", .{source}) catch return S.boom(c.SIM_INVALID, "path too long");
        try S.check(simulation_scene_load(&ctx.owned, z, robot, message.ptr, @intCast(message.len)), std.mem.sliceTo(message, 0));
    } else {
        var scene_z: [1024]u8 = undefined;
        const z = zprint(&scene_z, "{s}", .{scene}) catch return S.boom(c.SIM_INVALID, "path too long");
        try S.check(simulation_scene_create(&ctx.owned, z, robot, message.ptr, @intCast(message.len)), std.mem.sliceTo(message, 0));
    }
}

pub export fn auv_simulator_load_scene(model: ?*c.Simulation, path: ?[*:0]const u8, robot: ?*const c.Robot, error_out: ?[*]u8, capacity: u32) callconv(.c) c.SimResult {
    return S.guard(error_out, capacity, struct {
        fn run(sim: ?*c.Simulation, file: ?[*:0]const u8, preset: ?*const c.Robot) !void {
            try S.require(sim != null and file != null and preset != null, "missing scene arguments", c.SIM_INVALID);
            var message: [512]u8 = @splat(0);
            const source = std.mem.span(file.?);
            var module_path: [4096]u8 = undefined;
            var output: [512]u8 = undefined;
            var temporary: []const u8 = "";
            var load_path: []const u8 = source;
            if (endsWith(source, ".blend")) {
                const directory = try moduleDir(&module_path);
                temporary = try exportBlend(source, directory, &output);
                load_path = temporary;
            }
            defer if (temporary.len != 0) {
            var doomed: [512]u8 = undefined;
            if (zprint(&doomed, "{s}", .{temporary})) |z| _ = unlink(z) else |_| {}
        };
            var path_z: [1024]u8 = undefined;
            const z = zprint(&path_z, "{s}", .{load_path}) catch return S.boom(c.SIM_INVALID, "path too long");
            try S.check(simulation_scene_load(sim.?, z, preset.?, &message, message.len), std.mem.sliceTo(&message, 0));
        }
    }.run(model, path, robot));
}

pub export fn auv_simulator_configure_blender(enabled: u32, port: u16) callconv(.c) void {
    ctx.link.open(enabled != 0, port);
}

pub export fn auv_simulator_rebase_pacing() callconv(.c) void {
    const elapsed: i128 = if (ctx.runtime != null) @intCast(timestamp()) else 0;
    ctx.started_ns = ctx.now() - elapsed;
}

pub export fn auv_deinit() callconv(.c) void {
    if (ctx.runtime != null and ctx.print_summary) {
        _ = printf("simulation steps=%llu hash=%llu min_depth=%.9g max_depth=%.9g status=%u\n", simulation_runtime_tick(ctx.runtime), simulation_runtime_hash(ctx.runtime), ctx.min_depth, ctx.max_depth, ctx.status.code);
        _ = fflush(null);
    }
    ctx.release();
    ctx.commands = @splat(0);
}

pub export fn auv_init() callconv(.c) void {
    auv_deinit();
    ctx.status = std.mem.zeroes(auv.AuvSimulationStatus);
    var message: [512]u8 = @splat(0);
    const result = S.guard(&message, message.len, struct {
        fn run(note: *[512]u8) !void {
            const robot = sim_robot_find(environment("AUTONOMY_SIM_ROBOT", "hydrus")) orelse return S.boom(c.SIM_INVALID, "unknown robot preset");
            try loadOwned(std.mem.span(environment("AUTONOMY_SIM_SCENE", "robosub_course")), robot, note);
            ctx.model = &ctx.owned;
            var module_path: [32768]u8 = undefined;
            const directory = try moduleDir(&module_path);
            var backend_path: [4096]u8 = undefined;
            const joined = zprint(&backend_path, "{s}/libsimulator_backend_mujoco.so", .{directory}) catch return S.boom(c.SIM_INVALID, "path too long");
            try S.check(physics_backend_open(&ctx.backend, environment("AUTONOMY_SIM_BACKEND", joined), note, note.len), std.mem.sliceTo(note, 0));
            ctx.limit = try setting("AUTONOMY_SIM_MAX_TICKS", 21600);
            ctx.frame_ticks = try setting("AUTONOMY_SIM_FRAME_TICKS", 6);
            try validateTiming();
            const realtime = try setting("AUTONOMY_SIM_REALTIME", 0);
            try S.require(realtime <= 1, "realtime setting must be 0 or 1", c.SIM_INVALID);
            ctx.realtime = realtime != 0;
            ctx.print_summary = true;
            try S.check(simulation_runtime_create(ctx.model.?, &ctx.backend, null, &ctx.runtime, note, note.len), std.mem.sliceTo(note, 0));
            ctx.min_depth = ctx.model.?.robot.object.body.initial_pose.pos.buf[2];
            ctx.max_depth = ctx.min_depth;
            ctx.started_ns = ctx.now();
            ctx.link.openEnv();
        }
    }.run(&message));
    if (result != c.SIM_OK) {
        failStatus(std.mem.sliceTo(&message, 0));
        ctx.release();
    }
}

pub export fn auv_set_thrustor_values(values: ?[*]const f32, count: u8) callconv(.c) void {
    if (ctx.status.code != 0) return;
    if (ctx.runtime == null or values == null or count != ctx.model.?.robot.thruster_count) {
        failStatus("thruster command count does not match robot");
        return;
    }
    var i: u32 = 0;
    while (i < count) : (i += 1) if (!std.math.isFinite(values.?[i]) or values.?[i] < -1 or values.?[i] > 1) {
        failStatus("thruster command must be finite and in [-1,1]");
        return;
    };
    i = 0;
    while (i < count) : (i += 1) ctx.commands[i] = values.?[i];
}

pub export fn auv_yield_until_next_frame(frame: ?*auv.AuvFrame) callconv(.c) void {
    const out = frame orelse {
        failStatus("null AUV frame");
        return;
    };
    zeroFrame(out);
    if (ctx.status.code != 0) return;
    var message: [512]u8 = @splat(0);
    const result = S.guard(&message, message.len, struct {
        fn run(out_frame: *auv.AuvFrame, note: *[512]u8) !void {
            try S.require(ctx.runtime != null, "simulator is not initialized", c.SIM_INVALID);
            if (simulation_runtime_tick(ctx.runtime) >= ctx.limit) {
                ctx.status.code = 1;
                const text = "simulation tick budget exhausted";
                @memcpy(ctx.status.message[0..text.len], text);
                ctx.status.message[text.len] = 0;
                ctx.commands = @splat(0);
                return;
            }
            var i: u64 = 0;
            while (i < ctx.frame_ticks) : (i += 1) {
                try S.check(simulation_runtime_step(ctx.runtime.?, &ctx.commands, ctx.model.?.robot.thruster_count, note, note.len), std.mem.sliceTo(note, 0));
                try depth();
            }
            ctx.status.tick = simulation_runtime_tick(ctx.runtime);
            if (ctx.realtime) {
                const due = ctx.started_ns + @as(i128, @intCast(timestamp()));
                const current = ctx.now();
                if (due > current) sleepNs(@intCast(due - current));
            }
            var observation = std.mem.zeroes(c.Observation);
            try S.check(simulation_runtime_observe(ctx.runtime.?, &observation, note, note.len), std.mem.sliceTo(note, 0));
            try fillFrame(out_frame, observation, timestamp());
            ctx.link.publish(ctx.runtime.?, ctx.model.?, observation, timestamp());
        }
    }.run(out, &message));
    if (result != c.SIM_OK) {
        failStatus(std.mem.sliceTo(&message, 0));
        zeroFrame(out);
    }
}

pub export fn auv_simulator_init_scene(model: ?*const c.Simulation, options: ?*const c.AuvSimulatorOptions, message: ?[*]u8, capacity: u32) callconv(.c) c.SimResult {
    ctx.release();
    ctx.status = std.mem.zeroes(auv.AuvSimulationStatus);
    ctx.commands = @splat(0);
    const result = S.guard(message, capacity, struct {
        fn run(scene: ?*const c.Simulation, opts: ?*const c.AuvSimulatorOptions) !void {
            try S.require(opts != null and opts.?.struct_size == @sizeOf(c.AuvSimulatorOptions) and opts.?.version == c.AUV_SIMULATOR_EXTENSION_VERSION, "simulator extension v1 required", c.SIM_UNSUPPORTED);
            var detail: [256]u8 = @splat(0);
            try S.require(scene != null, "missing scene", c.SIM_INVALID);
            try S.check(simulation_validate(scene.?, &detail, detail.len), std.mem.sliceTo(&detail, 0));
            try S.require(scene.?.frozen != 0, "backend requires finalized model", c.SIM_INVALID);
            try S.require(scene.?.robot.registered != 0, "scene requires a registered robot", c.SIM_INVALID);
            try S.require(opts.?.backend_path != null and opts.?.realtime <= 1 and opts.?.print_summary <= 1, "invalid simulator options", c.SIM_INVALID);
            ctx.model = scene.?;
            ctx.limit = opts.?.max_ticks;
            ctx.frame_ticks = opts.?.frame_ticks;
            ctx.realtime = opts.?.realtime != 0;
            ctx.print_summary = opts.?.print_summary != 0;
            try validateTiming();
            var backend_detail: [512]u8 = @splat(0);
            try S.check(physics_backend_open(&ctx.backend, opts.?.backend_path, &backend_detail, backend_detail.len), std.mem.sliceTo(&backend_detail, 0));
            try S.require(std.mem.eql(u8, std.mem.sliceTo(&ctx.backend.api.name, 0), "mujoco"), "scene extension requires MuJoCo", c.SIM_UNSUPPORTED);
            try S.check(simulation_runtime_create(scene.?, &ctx.backend, null, &ctx.runtime, &backend_detail, backend_detail.len), std.mem.sliceTo(&backend_detail, 0));
            ctx.min_depth = scene.?.robot.object.body.initial_pose.pos.buf[2];
            ctx.max_depth = ctx.min_depth;
            ctx.started_ns = ctx.now();
            ctx.link.openEnv();
        }
    }.run(model, options));
    if (result != c.SIM_OK) {
        failStatus(if (message) |text| std.mem.sliceTo(text, 0) else "scene initialization failed");
        ctx.release();
    }
    return result;
}

pub export fn auv_simulator_read_result(out: ?*c.AuvSimulatorResult, size: u32, message: ?[*]u8, capacity: u32) callconv(.c) c.SimResult {
    return S.guard(message, capacity, struct {
        fn run(result: ?*c.AuvSimulatorResult, result_size: u32) !void {
            try S.require(result != null and result_size == @sizeOf(c.AuvSimulatorResult), "invalid simulator result size", c.SIM_UNSUPPORTED);
            result.?.* = std.mem.zeroes(c.AuvSimulatorResult);
            result.?.struct_size = @sizeOf(c.AuvSimulatorResult);
            result.?.version = c.AUV_SIMULATOR_EXTENSION_VERSION;
            result.?.status = ctx.status.code;
            result.?.tick = ctx.status.tick;
            result.?.message = ctx.status.message;
            if (ctx.runtime == null) return;
            result.?.tick = simulation_runtime_tick(ctx.runtime);
            result.?.hash = simulation_runtime_hash(ctx.runtime);
            result.?.min_depth = ctx.min_depth;
            result.?.max_depth = ctx.max_depth;
            var states: ?[*]const c.SimBodyState = null;
            var count: u32 = 0;
            try S.check(simulation_runtime_states(ctx.runtime.?, &states, &count), "cannot read result states");
            const index = ctx.model.?.robot.object.id.index;
            if (index < count) {
                result.?.robot = states.?[index];
                result.?.has_final_state = 1;
            }
        }
    }.run(out, size));
}

pub export fn auv_simulation_status(out: ?*auv.AuvSimulationStatus) callconv(.c) bool {
    const status = out orelse return false;
    status.* = ctx.status;
    return true;
}
