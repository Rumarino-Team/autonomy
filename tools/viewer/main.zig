const std = @import("std");
const rl = @import("raylib");
const MissionIO = @import("MissionIO");
const MissionConfig = MissionIO.CompiledConfig;
const math = MissionIO.Math;
const Io = std.Io;

extern fn video_open(source: [*:0]const u8, width: *c_int, height: *c_int) callconv(.c) bool;
extern fn video_frame(out: [*]u8) callconv(.c) bool;
extern fn video_close() callconv(.c) void;

const default_stream_source = "127.0.0.1:5610";
const teleop_step: f32 = 0.05;
const teleop_period: f64 = 0.05;
const mission_target_step: f32 = 0.25;
const mission_look_step: f32 = 0.1;
const tune_step: f32 = 0.1;
const tune_period: f64 = 0.1;

const tune_names = [_][:0]const u8{
    "kp_x", "kp_y", "kp_z", "kp_roll", "kp_pitch", "kp_yaw",
    "ki_x", "ki_y", "ki_z", "ki_roll", "ki_pitch", "ki_yaw",
    "kd_x", "kd_y", "kd_z", "kd_roll", "kd_pitch", "kd_yaw",
};
const pid_group_names = [_][:0]const u8{ "kp", "ki", "kd" };
const pid_axis_names = [_][:0]const u8{ "x", "y", "z", "roll", "pitch", "yaw" };
const pid_slider_max = [_]f32{ 10, 2, 10 };

const PidSliderLayout = struct {
    x: f32,
    width: f32,
    height: f32,
    row_y: f32,
    row_step: f32,
    col_step: f32,
};

const Config = struct {
    video_source: [:0]const u8 = default_stream_source,
    mission_control: bool = false,
};

fn parseArgs(args: []const [:0]const u8) !Config {
    var config = Config{};
    var i: usize = 1;
    while (i < args.len) {
        const arg = args[i];
        if (std.mem.eql(u8, arg, "--video")) {
            i += 1;
            if (i >= args.len) return error.MissingVideoSource;
            config.video_source = args[i];
        } else if (std.mem.eql(u8, arg, "--mission-control")) {
            config.mission_control = true;
        } else if (std.mem.eql(u8, arg, "--help") or std.mem.eql(u8, arg, "-h")) {
            std.debug.print(
                "usage: viewer [--video source|none] [--mission-control]\n",
                .{},
            );
            return error.HelpRequested;
        } else {
            std.debug.print("unknown argument: {s}\n", .{arg});
            return error.UnknownArgument;
        }
        i += 1;
    }
    return config;
}

fn videoDisabled(source: []const u8) bool {
    return std.mem.eql(u8, source, "none") or
        std.mem.eql(u8, source, "off") or
        std.mem.eql(u8, source, "-");
}

fn tuningValue(packet: *MissionIO.LiveConfigPacket, index: usize) *f32 {
    const group = index / 6;
    const axis = index % 6;
    return switch (group) {
        0 => &packet.odometry.kp[axis],
        1 => &packet.odometry.ki[axis],
        2 => &packet.odometry.kd[axis],
        else => unreachable,
    };
}

fn pidSliderLayout() PidSliderLayout {
    const width_scale = @max(1.0, @as(f32, @floatFromInt(rl.getScreenWidth())) / 1280.0);
    const height_scale = @max(1.0, @as(f32, @floatFromInt(rl.getScreenHeight())) / 1000.0);
    return .{
        .x = 955,
        .width = 45 * width_scale,
        .height = 18 * height_scale,
        .row_y = 740,
        .row_step = 90 * height_scale,
        .col_step = 51 * width_scale,
    };
}

fn pidSliderRect(index: usize, layout: PidSliderLayout) rl.Rectangle {
    const group = index / 6;
    const axis = index % 6;
    return .{
        .x = layout.x + @as(f32, @floatFromInt(axis)) * layout.col_step,
        .y = layout.row_y + @as(f32, @floatFromInt(group)) * layout.row_step,
        .width = layout.width,
        .height = layout.height,
    };
}

fn pidSliderValue(index: usize, mouse_x: f32, layout: PidSliderLayout) f32 {
    const rect = pidSliderRect(index, layout);
    const ratio = @min(1.0, @max(0.0, (mouse_x - rect.x) / rect.width));
    return ratio * pid_slider_max[index / 6];
}

fn sendTuning(
    io: Io,
    socket: *Io.net.Socket,
    destination: *const Io.net.IpAddress,
    packet: MissionIO.LiveConfigPacket,
) !void {
    var buffer: [4096]u8 = undefined;
    var writer: Io.Writer = .fixed(&buffer);
    try MissionIO.SerialBinary.write(&writer, packet);
    try socket.send(io, destination, writer.buffered());
}

fn sendMissionCommand(
    io: Io,
    socket: *Io.net.Socket,
    destination: *const Io.net.IpAddress,
    command: MissionIO.MissionCommand,
) !void {
    var buffer: [64]u8 = undefined;
    const packet = try MissionIO.encodeMissionCommand(&buffer, command);
    try socket.send(io, destination, packet);
}

fn emptyTelemetry() MissionIO.TelemetryFrame {
    var frame = std.mem.zeroes(MissionIO.TelemetryFrame);
    frame.magic = "AUV1".*;
    frame.abi_version = MissionIO.telemetry_frame_abi_version;
    frame.camera_pose.quat[3] = 1;
    return frame;
}

fn toRay(v: [3]f32) rl.Vector3 {
    return .{ .x = v[0], .y = -v[2], .z = -v[1] };
}

fn add3(a: [3]f32, b: [3]f32) [3]f32 {
    return .{ a[0] + b[0], a[1] + b[1], a[2] + b[2] };
}

fn scaled3(v: [3]f32, scale: f32) [3]f32 {
    return .{ v[0] * scale, v[1] * scale, v[2] * scale };
}

fn drawArrow(origin: [3]f32, vector: [3]f32, radius: f32, color: rl.Color) void {
    const from = toRay(origin);
    const to = toRay(add3(origin, vector));
    rl.drawCylinderEx(from, to, radius, radius, 8, color);
    rl.drawSphere(to, radius * 2.5, color);
}

fn drawBox(pos: [3]f32, quat: [4]f32, size: [3]f32, color: rl.Color) void {
    var corners: [8]rl.Vector3 = undefined;
    var n: usize = 0;
    const signs = [_]f32{ -1, 1 };
    for (signs) |sx| {
        for (signs) |sy| {
            for (signs) |sz| {
                const local = [3]f32{
                    sx * size[0] * 0.5,
                    sy * size[1] * 0.5,
                    sz * size[2] * 0.5,
                };
                corners[n] = toRay(add3(pos, math.quaternionRotate(quat, local)));
                n += 1;
            }
        }
    }
    const edges = [_][2]usize{
        .{ 0, 1 }, .{ 2, 3 }, .{ 4, 5 }, .{ 6, 7 },
        .{ 0, 2 }, .{ 1, 3 }, .{ 4, 6 }, .{ 5, 7 },
        .{ 0, 4 }, .{ 1, 5 }, .{ 2, 6 }, .{ 3, 7 },
    };
    for (edges) |edge| rl.drawLine3D(corners[edge[0]], corners[edge[1]], color);
}

fn objectColor(cls: u8) rl.Color {
    return rl.colorFromHSV(@floatFromInt((@as(u16, cls) * 47) % 360), 0.75, 0.95);
}

fn axisAngle(axis: [3]f32, angle: f32) [4]f32 {
    const half = angle * 0.5;
    const sine = @sin(half);
    return .{ axis[0] * sine, axis[1] * sine, axis[2] * sine, @cos(half) };
}

fn normalizeQuaternion(quat: *[4]f32) void {
    const value: math.Quaternionf = quat.*;
    const length = @sqrt(@reduce(.Add, value * value));
    if (length > 1e-6)
        quat.* = value / @as(math.Quaternionf, @splat(length));
}

fn normalizeTranslation(translation: *[3]f32) void {
    const length = @sqrt(
        translation[0] * translation[0] +
            translation[1] * translation[1] +
            translation[2] * translation[2],
    );
    if (length > 1e-6) {
        for (translation) |*axis| axis.* /= length;
    }
}

fn encodeGoal(goal: [7]f32) [28]u8 {
    var bytes: [28]u8 = undefined;
    for (goal, 0..) |value, i| {
        const bits: u32 = @bitCast(value);
        bytes[i * 4 + 0] = @truncate(bits);
        bytes[i * 4 + 1] = @truncate(bits >> 8);
        bytes[i * 4 + 2] = @truncate(bits >> 16);
        bytes[i * 4 + 3] = @truncate(bits >> 24);
    }
    return bytes;
}

fn drawFollowingGrid(sub: rl.Vector3) void {
    const center_x = @floor(sub.x);
    const center_z = @floor(sub.z);
    for (0..21) |index| {
        const offset: f32 = @as(f32, @floatFromInt(index)) - 10;
        rl.drawLine3D(
            .{ .x = center_x + offset, .y = 0, .z = center_z - 10 },
            .{ .x = center_x + offset, .y = 0, .z = center_z + 10 },
            rl.Color.gray,
        );
        rl.drawLine3D(
            .{ .x = center_x - 10, .y = 0, .z = center_z + offset },
            .{ .x = center_x + 10, .y = 0, .z = center_z + offset },
            rl.Color.gray,
        );
    }
}

fn stepText(frame: *const MissionIO.TelemetryFrame, buffer: *[MissionIO.telemetry_step_max + 1]u8) [:0]const u8 {
    const length = @min(@as(usize, frame.step_len), frame.step.len);
    @memcpy(buffer[0..length], frame.step[0..length]);
    buffer[length] = 0;
    return buffer[0..length :0];
}

pub fn main(init: std.process.Init) !void {
    const args = try init.minimal.args.toSlice(init.arena.allocator());
    const config = parseArgs(args) catch |err| {
        if (err == error.HelpRequested) return;
        std.debug.print("viewer argument error: {s}\n", .{@errorName(err)});
        return err;
    };
    const compiled_config = MissionConfig.compiledProfile(MissionConfig.viewer_profile);
    const telemetry_shm_name = compiled_config.telemetryShmName() catch "";
    const telemetry_addr = compiled_config.telemetryAddr() catch "";

    var subscriber: ?MissionIO.TelemetrySubscriber = null;
    defer if (subscriber) |*value| value.deinit();
    var udp_socket: ?Io.net.Socket = null;
    defer if (udp_socket) |*socket| socket.close(init.io);
    switch (compiled_config.telemetry_transport) {
        .shared_memory => {},
        .udp => {
            const bind_addr = try Io.net.IpAddress.parseLiteral(telemetry_addr);
            udp_socket = bind_addr.bind(init.io, .{ .mode = .dgram, .protocol = .udp }) catch {
                std.debug.print("can't listen on telemetry address {s}\n", .{telemetry_addr});
                return error.UdpOpenFailed;
            };
        },
        .stdout => {},
    }

    const teleop_dest = blk: {
        var address_buf: [48]u8 = undefined;
        const text = try std.fmt.bufPrint(&address_buf, "127.0.0.1:{d}", .{compiled_config.teleop_port});
        break :blk try Io.net.IpAddress.parseLiteral(text);
    };
    var config_packet = MissionIO.LiveConfigPacket.fromRuntimeConfig(compiled_config);
    const teleop_bind: Io.net.IpAddress = .{ .ip4 = .unspecified(0) };
    var teleop_socket: ?Io.net.Socket = teleop_bind.bind(init.io, .{ .mode = .dgram, .protocol = .udp }) catch null;
    defer if (teleop_socket) |*socket| socket.close(init.io);
    if (teleop_socket == null)
        std.debug.print("can't send control commands to 127.0.0.1:{d}\n", .{compiled_config.teleop_port})
    else if (config.mission_control)
        std.debug.print("mission commands -> 127.0.0.1:{d}\n", .{compiled_config.teleop_port})
    else
        std.debug.print("teleop command -> 127.0.0.1:{d}\n", .{compiled_config.teleop_port});

    const read_video = !videoDisabled(config.video_source);
    rl.setConfigFlags(.{ .window_resizable = true });
    rl.initWindow(1280, 1000, "viewer");
    defer rl.closeWindow();
    rl.setWindowMinSize(1000, 700);
    rl.setTargetFPS(60);

    const scene_w: i32 = 900;
    const scene_h: i32 = if (read_video) 470 else 990;
    const scene = try rl.loadRenderTexture(scene_w, scene_h);
    defer rl.unloadRenderTexture(scene);
    const view3d = rl.Rectangle{
        .x = 0,
        .y = if (read_video) 520 else 0,
        .width = @floatFromInt(scene_w),
        .height = @floatFromInt(scene_h),
    };

    var orbit_yaw: f32 = 0.8;
    var orbit_pitch: f32 = 0.55;
    var orbit_dist: f32 = 8;
    var video_w: c_int = 0;
    var video_h: c_int = 0;
    var video: rl.Texture2D = std.mem.zeroes(rl.Texture2D);
    var video_image: ?rl.Image = null;
    var pixels: ?[*]u8 = null;
    defer {
        if (video.id != 0) rl.unloadTexture(video);
        if (video_image) |image| rl.unloadImage(image);
        video_close();
    }

    var packet: [64 * 1024]u8 = undefined;
    var udp_telemetry = emptyTelemetry();
    var telemetry: *const MissionIO.TelemetryFrame = &udp_telemetry;
    var translation = [3]f32{ 0, 0, 0 };
    var orientation = [4]f32{ 0, 0, 0, 1 };
    var last_timestamp: ?u64 = null;
    var last_packet: f64 = -1;
    var last_frame: f64 = -1;
    var last_video_try: f64 = -10;
    var last_source_try: f64 = -10;
    var last_teleop: f64 = -1;
    var mission_target_initialized = false;
    var mission_kind: MissionIO.MissionCommandKind = .point;
    var mission_maneuver: MissionIO.MissionManeuver = .move;
    var mission_position: math.Vector3f = .{ 0, 0, 0 };
    var mission_look_direction: math.Vector3f = .{ 0, 1, 0 };
    var mission_look_yaw: f32 = 0;
    var mission_look_pitch: f32 = 0;
    var mission_object_class: u8 = 0;
    var mission_control = config.mission_control;
    var show_command_legend = false;
    var tune_selection: usize = 0;
    var last_tune_adjust: f64 = -1;
    var tune_error_logged = false;
    var mission_error_logged = false;
    var teleop_error_logged = false;
    var pid_drag: ?usize = null;

    while (!rl.windowShouldClose()) {
        telemetry = &udp_telemetry;
        var shared_view: ?MissionIO.LatestFrame = null;
        const now = rl.getTime();
        const pid_layout = pidSliderLayout();
        if (read_video and video_w == 0 and now - last_video_try > 2) {
            last_video_try = now;
            var width: c_int = 0;
            var height: c_int = 0;
            if (video_open(config.video_source.ptr, &width, &height)) {
                const blank = rl.genImageColor(width, height, rl.Color.black);
                video = rl.loadTextureFromImage(blank) catch {
                    rl.unloadImage(blank);
                    video_close();
                    continue;
                };
                video_w = width;
                video_h = height;
                pixels = @ptrCast(blank.data);
                video_image = blank;
                std.debug.print("zed stream connected: {s}\n", .{config.video_source});
            }
        }

        switch (compiled_config.telemetry_transport) {
            .shared_memory => {
                if (subscriber == null and now - last_source_try > 2) {
                    last_source_try = now;
                    subscriber = MissionIO.TelemetrySubscriber.open(telemetry_shm_name) catch |err| blk: {
                        std.debug.print("waiting for shared telemetry '{s}': {s}\n", .{ telemetry_shm_name, @errorName(err) });
                        break :blk null;
                    };
                }
                if (subscriber) |*source| {
                    var attempts: usize = 0;
                    while (attempts < 2) : (attempts += 1) {
                        const latest = source.latest() orelse break;
                        if (!latest.valid()) continue;
                        shared_view = latest;
                        telemetry = latest.frame;
                        if (last_timestamp == null or telemetry.timestamp != last_timestamp.?) {
                            last_timestamp = telemetry.timestamp;
                            last_packet = now;
                        }
                        break;
                    }
                }
            },
            .udp => {
                const timeout: Io.Timeout = .{ .duration = .{
                    .clock = .awake,
                    .raw = .fromNanoseconds(0),
                } };
                while (true) {
                    const message = udp_socket.?.receiveTimeout(init.io, &packet, timeout) catch |err| switch (err) {
                        error.Timeout => break,
                        else => break,
                    };
                    MissionIO.decodeTelemetryFrame(message.data, &udp_telemetry) catch continue;
                    {
                        last_timestamp = udp_telemetry.timestamp;
                        last_packet = now;
                    }
                }
            },
            .stdout => {},
        }

        if (pixels) |video_pixels| {
            if (video_frame(video_pixels)) {
                rl.updateTexture(video, video_pixels);
                last_frame = now;
            }
        }

        if (rl.isKeyPressed(.t)) {
            mission_control = !mission_control;
            if (mission_control) mission_target_initialized = false;
            std.debug.print(
                "control mode: {s}\n",
                .{if (mission_control) "mission-control" else "teleop"},
            );
        }
        if (rl.isKeyPressed(.h)) show_command_legend = !show_command_legend;

        if (mission_control and !mission_target_initialized and last_timestamp != null) {
            mission_position = telemetry.camera_pose.pos + @as(math.Vector3f, .{ 0, 2, 0 });
            mission_target_initialized = true;
        }

        if (mission_control) {
            if (rl.isKeyPressed(.p)) mission_kind = .point;
            if (rl.isKeyPressed(.o)) mission_kind = .object;
            if (rl.isKeyPressed(.m)) mission_maneuver = .move;
            if (rl.isKeyPressed(.b)) mission_maneuver = .pass;
            if (rl.isKeyPressed(.c)) mission_maneuver = .circle;

            if (mission_kind == .point) {
                if (rl.isKeyDown(.d)) mission_position[0] += mission_target_step;
                if (rl.isKeyDown(.a)) mission_position[0] -= mission_target_step;
                if (rl.isKeyDown(.w)) mission_position[1] += mission_target_step;
                if (rl.isKeyDown(.s)) mission_position[1] -= mission_target_step;
                if (rl.isKeyDown(.r)) mission_position[2] += mission_target_step;
                if (rl.isKeyDown(.f)) mission_position[2] -= mission_target_step;
            } else {
                if (rl.isKeyPressed(.q)) mission_object_class -%= 1;
                if (rl.isKeyPressed(.e)) mission_object_class +%= 1;
            }

            if (rl.isKeyDown(.right)) mission_look_yaw -= mission_look_step;
            if (rl.isKeyDown(.left)) mission_look_yaw += mission_look_step;
            if (rl.isKeyDown(.up)) mission_look_pitch = @min(1.4, mission_look_pitch + mission_look_step);
            if (rl.isKeyDown(.down)) mission_look_pitch = @max(-1.4, mission_look_pitch - mission_look_step);
            mission_look_direction = .{
                @sin(mission_look_yaw) * @cos(mission_look_pitch),
                @cos(mission_look_yaw) * @cos(mission_look_pitch),
                @sin(mission_look_pitch),
            };

            if (rl.isKeyPressed(.enter) and teleop_socket != null) {
                const command: MissionIO.MissionCommand = switch (mission_kind) {
                    .point => .{
                        .kind = .point,
                        .maneuver = .move,
                        .position = mission_position,
                        .look_direction = mission_look_direction,
                    },
                    .object => .{
                        .kind = .object,
                        .maneuver = mission_maneuver,
                        .object_class = mission_object_class,
                    },
                };
                sendMissionCommand(init.io, &teleop_socket.?, &teleop_dest, command) catch {
                    if (!mission_error_logged) {
                        std.debug.print("mission command send failed\n", .{});
                        mission_error_logged = true;
                    }
                };
            }
        } else if (teleop_socket != null and now - last_teleop >= teleop_period) {
            last_teleop = now;
            translation = .{ 0, 0, 0 };
            if (rl.isKeyDown(.d)) translation[0] += 1;
            if (rl.isKeyDown(.a)) translation[0] -= 1;
            if (rl.isKeyDown(.w)) translation[1] += 1;
            if (rl.isKeyDown(.s)) translation[1] -= 1;
            if (rl.isKeyDown(.r)) translation[2] += 1;
            if (rl.isKeyDown(.f)) translation[2] -= 1;
            normalizeTranslation(&translation);

            if (rl.isKeyDown(.right)) orientation = math.quaternionMul(axisAngle(.{ 0, 0, 1 }, -teleop_step), orientation);
            if (rl.isKeyDown(.left)) orientation = math.quaternionMul(axisAngle(.{ 0, 0, 1 }, teleop_step), orientation);
            if (rl.isKeyDown(.up)) orientation = math.quaternionMul(orientation, axisAngle(.{ 1, 0, 0 }, -teleop_step));
            if (rl.isKeyDown(.down)) orientation = math.quaternionMul(orientation, axisAngle(.{ 1, 0, 0 }, teleop_step));
            if (rl.isKeyDown(.i)) orientation = math.quaternionMul(orientation, axisAngle(.{ 0, 1, 0 }, teleop_step));
            if (rl.isKeyDown(.k)) orientation = math.quaternionMul(orientation, axisAngle(.{ 0, 1, 0 }, -teleop_step));
            normalizeQuaternion(&orientation);

            const command = [7]f32{
                translation[0], translation[1], translation[2],
                orientation[0], orientation[1], orientation[2],
                orientation[3],
            };
            const goal_packet = encodeGoal(command);
            teleop_socket.?.send(init.io, &teleop_dest, &goal_packet) catch {
                if (!teleop_error_logged) {
                    std.debug.print("teleop send failed\n", .{});
                    teleop_error_logged = true;
                }
            };
        }

        if (!mission_control) {
            if (rl.isKeyPressed(.q)) {
                tune_selection = (tune_selection + tune_names.len - 1) % tune_names.len;
            }
            if (rl.isKeyPressed(.e)) {
                tune_selection = (tune_selection + 1) % tune_names.len;
            }
        }

        var tune_changed = false;
        if (rl.isMouseButtonPressed(.left)) {
            const mouse = rl.getMousePosition();
            for (0..tune_names.len) |index| {
                if (rl.checkCollisionPointRec(mouse, pidSliderRect(index, pid_layout))) {
                    tune_selection = index;
                    pid_drag = index;
                    break;
                }
            }
        }
        if (!rl.isMouseButtonDown(.left)) pid_drag = null;
        if (pid_drag) |index| {
            if (now - last_tune_adjust >= tune_period) {
                const value = pidSliderValue(index, rl.getMousePosition().x, pid_layout);
                const target = tuningValue(&config_packet, index);
                if (@abs(target.* - value) > 1e-5) {
                    target.* = value;
                    last_tune_adjust = now;
                    tune_changed = true;
                }
            }
        }
        if (now - last_tune_adjust >= tune_period) {
            var delta: f32 = 0;
            if (rl.isKeyDown(.z)) delta -= tune_step;
            if (rl.isKeyDown(.x)) delta += tune_step;
            if (delta != 0) {
                tuningValue(&config_packet, tune_selection).* += delta;
                last_tune_adjust = now;
                tune_changed = true;
            }
        }
        if (tune_changed and teleop_socket != null) {
            sendTuning(init.io, &teleop_socket.?, &teleop_dest, config_packet) catch {
                if (!tune_error_logged) {
                    std.debug.print("live tuning send failed\n", .{});
                    tune_error_logged = true;
                }
            };
        }

        if (rl.checkCollisionPointRec(rl.getMousePosition(), view3d)) {
            if (rl.isMouseButtonDown(.right)) {
                const delta = rl.getMouseDelta();
                orbit_yaw -= delta.x * 0.01;
                orbit_pitch = @min(1.35, @max(0.15, orbit_pitch - delta.y * 0.01));
            }
            orbit_dist = @max(1.5, orbit_dist - rl.getMouseWheelMove());
        }

        const sub = toRay(telemetry.camera_pose.pos);
        const camera = rl.Camera3D{
            .position = .{
                .x = sub.x + orbit_dist * @cos(orbit_pitch) * @sin(orbit_yaw),
                .y = sub.y + orbit_dist * @sin(orbit_pitch),
                .z = sub.z + orbit_dist * @cos(orbit_pitch) * @cos(orbit_yaw),
            },
            .target = sub,
            .up = .{ .x = 0, .y = 1, .z = 0 },
            .fovy = 50,
            .projection = .perspective,
        };

        rl.beginTextureMode(scene);
        rl.clearBackground(.{ .r = 12, .g = 18, .b = 28, .a = 255 });
        rl.beginMode3D(camera);
        drawFollowingGrid(sub);
        rl.drawLine3D(sub, .{ .x = sub.x, .y = 0, .z = sub.z }, rl.fade(rl.Color.sky_blue, 0.45));
        drawBox(telemetry.camera_pose.pos, telemetry.camera_pose.quat, .{ 0.35, 0.8, 0.22 }, rl.Color.sky_blue);

        const nose = math.quaternionRotate(telemetry.camera_pose.quat, [3]f32{ 0, 0.7, 0 });
        rl.drawLine3D(sub, toRay(add3(telemetry.camera_pose.pos, nose)), rl.Color.orange);
        const command_forward = math.quaternionRotate(orientation, [3]f32{ 0, 2, 0 });
        drawArrow(telemetry.camera_pose.pos, command_forward, 0.02, rl.Color.green);
        const command_move = math.quaternionRotate(orientation, translation);
        if (math.dot3f(command_move, command_move) > 1e-6)
            drawArrow(telemetry.camera_pose.pos, scaled3(command_move, 1.2), 0.045, rl.Color.magenta);
        if (mission_control and mission_target_initialized) {
            rl.drawSphere(toRay(mission_position), 0.18, rl.Color.yellow);
            drawArrow(mission_position, mission_look_direction, 0.035, rl.Color.yellow);
        }
        const force_error = math.quaternionRotate(telemetry.camera_pose.quat, telemetry.force_error);
        if (math.dot3f(force_error, force_error) > 1e-6)
            drawArrow(telemetry.camera_pose.pos, force_error, 0.03, rl.Color.yellow);

        for (telemetry.objects[0..telemetry.objects_len]) |object| {
            var size = object.size;
            for (&size) |*axis| if (axis.* < 0.05) {
                axis.* = 0.3;
            };
            drawBox(object.pose.pos, object.pose.quat, size, objectColor(object.cls));
        }
        for (telemetry.goals[0..telemetry.goals_len], 0..) |goal, i| {
            const at = toRay(goal.position);
            const look = toRay(goal.look_at);
            const color = if (i == 0) rl.Color.white else rl.Color.lime;
            if (i + 1 < telemetry.goals_len)
                rl.drawLine3D(at, toRay(telemetry.goals[i + 1].position), rl.Color.lime);
            rl.drawLine3D(at, look, rl.fade(color, 0.65));
            rl.drawSphere(at, if (i == 0) 0.18 else 0.1, color);
        }
        rl.endMode3D();
        rl.drawText("sub blue   objects colored by coco class   right-drag orbit   wheel zoom", 8, 8, 16, rl.Color.light_gray);
        rl.drawText("y forward   x right   z down", 8, 28, 16, rl.Color.gray);
        rl.drawText("orange nose   green cmd   pink move   yellow err   lime goals", 8, 48, 16, rl.Color.gray);
        rl.endTextureMode();

        rl.beginDrawing();
        rl.clearBackground(rl.Color.black);
        if (read_video and video_w != 0) {
            var scale: f32 = 900.0 / @as(f32, @floatFromInt(video_w));
            if (@as(f32, @floatFromInt(video_h)) * scale > 500)
                scale = 500.0 / @as(f32, @floatFromInt(video_h));
            rl.drawTextureEx(video, .{ .x = 0, .y = 0 }, 0, scale, rl.Color.white);
            for (telemetry.boxes[0..telemetry.boxes_len]) |box| {
                rl.drawRectangleLinesEx(.{
                    .x = @as(f32, @floatFromInt(box.top_left.x)) * scale,
                    .y = @as(f32, @floatFromInt(box.top_left.y)) * scale,
                    .width = @as(f32, @floatFromInt(box.bottom_right.x - box.top_left.x)) * scale,
                    .height = @as(f32, @floatFromInt(box.bottom_right.y - box.top_left.y)) * scale,
                }, 2, rl.Color.yellow);
            }
            if (last_frame < 0 or now - last_frame > 1)
                rl.drawText(rl.textFormat("no frames from %s", .{config.video_source.ptr}), 20, 20, 20, rl.Color.red);
        } else if (read_video) {
            rl.drawText(rl.textFormat("video: connecting to %s", .{config.video_source.ptr}), 20, 20, 20, rl.Color.yellow);
        }

        const x: i32 = 920;
        const source_ok = last_packet >= 0 and now - last_packet <= 1;
        switch (compiled_config.telemetry_transport) {
            .shared_memory => {
                rl.drawText(
                    rl.textFormat(if (source_ok) "shm %s" else "no telemetry on shm %s", .{telemetry_shm_name.ptr}),
                    x,
                    20,
                    20,
                    if (source_ok) rl.Color.green else rl.Color.red,
                );
            },
            .udp => rl.drawText(
                rl.textFormat(if (source_ok) "udp %s" else "no telemetry on udp %s", .{telemetry_addr.ptr}),
                x,
                20,
                20,
                if (source_ok) rl.Color.green else rl.Color.red,
            ),
            .stdout => rl.drawText(
                "telemetry stdout (producer-only)",
                x,
                20,
                20,
                rl.Color.gray,
            ),
        }

        const zed_color = if (!read_video) rl.Color.gray else if (video_w != 0) rl.Color.green else rl.Color.orange;
        rl.drawText(
            if (read_video) rl.textFormat("zed  %s", .{config.video_source.ptr}) else "zed  off",
            x,
            40,
            18,
            zed_color,
        );
        var step_buffer: [MissionIO.telemetry_step_max + 1]u8 = undefined;
        rl.drawText(rl.textFormat("step  %s", .{stepText(telemetry, &step_buffer).ptr}), x, 60, 20, rl.Color.ray_white);
        rl.drawText(rl.textFormat("pos   %.2f %.2f %.2f", .{
            @as(f64, telemetry.camera_pose.pos[0]),
            @as(f64, telemetry.camera_pose.pos[1]),
            @as(f64, telemetry.camera_pose.pos[2]),
        }), x, 90, 20, rl.Color.ray_white);
        rl.drawText(rl.textFormat("quat  %.2f %.2f %.2f %.2f", .{
            @as(f64, telemetry.camera_pose.quat[0]),
            @as(f64, telemetry.camera_pose.quat[1]),
            @as(f64, telemetry.camera_pose.quat[2]),
            @as(f64, telemetry.camera_pose.quat[3]),
        }), x, 120, 20, rl.Color.ray_white);
        rl.drawText(rl.textFormat("objs  %d", .{@as(c_int, telemetry.objects_len)}), x, 150, 20, rl.Color.ray_white);

        const pos_ok = telemetry.position_ok;
        rl.drawText(rl.textFormat("pos xyz  %s %s %s", .{
            if (pos_ok.x) "ok" else "--",
            if (pos_ok.y) "ok" else "--",
            if (pos_ok.z) "ok" else "--",
        }), x, 180, 20, if (pos_ok.x and pos_ok.y and pos_ok.z) rl.Color.green else rl.Color.orange);
        const att_ok = telemetry.attitude_ok;
        rl.drawText(rl.textFormat("att xyz  %s %s %s", .{
            if (att_ok.x) "ok" else "--",
            if (att_ok.y) "ok" else "--",
            if (att_ok.z) "ok" else "--",
        }), x, 210, 20, if (att_ok.x and att_ok.y and att_ok.z) rl.Color.green else rl.Color.orange);
        rl.drawText(
            if (telemetry.depth_ok)
                rl.textFormat("depth %.2f m", .{@as(f64, telemetry.depth)})
            else
                "depth -",
            x,
            240,
            20,
            rl.Color.ray_white,
        );
        if (mission_control) {
            const kind_name = switch (mission_kind) {
                .point => "point",
                .object => "object",
            };
            const maneuver_name = switch (mission_maneuver) {
                .move => "move",
                .pass => "pass",
                .circle => "circle",
            };
            rl.drawText(
                rl.textFormat("mission %s/%s  port %d", .{ kind_name.ptr, maneuver_name.ptr, compiled_config.teleop_port }),
                x,
                270,
                18,
                if (teleop_socket != null) rl.Color.green else rl.Color.red,
            );
            rl.drawText(rl.textFormat("target %.2f %.2f %.2f", .{
                @as(f64, mission_position[0]),
                @as(f64, mission_position[1]),
                @as(f64, mission_position[2]),
            }), x, 296, 16, rl.Color.ray_white);
            rl.drawText(rl.textFormat("look   %.2f %.2f %.2f", .{
                @as(f64, mission_look_direction[0]),
                @as(f64, mission_look_direction[1]),
                @as(f64, mission_look_direction[2]),
            }), x, 318, 16, rl.Color.ray_white);
            rl.drawText(rl.textFormat("class  %d", .{@as(c_int, mission_object_class)}), x, 340, 16, rl.Color.ray_white);
            rl.drawText("P/O mode  M/B/C move/pass/circle  Enter send", x, 360, 14, rl.Color.gray);
            rl.drawText("WASD/RF target  arrows look  Q/E class", x, 378, 14, rl.Color.gray);
        } else {
            rl.drawText(rl.textFormat("move  %.2f %.2f %.2f", .{
                @as(f64, translation[0]),
                @as(f64, translation[1]),
                @as(f64, translation[2]),
            }), x, 270, 20, if (teleop_socket != null) rl.Color.ray_white else rl.Color.red);
            rl.drawText(rl.textFormat("cmd   %.2f %.2f %.2f %.2f", .{
                @as(f64, orientation[0]),
                @as(f64, orientation[1]),
                @as(f64, orientation[2]),
                @as(f64, orientation[3]),
            }), x, 300, 20, rl.Color.ray_white);
            rl.drawText(rl.textFormat("err   %.2f %.2f %.2f", .{
                @as(f64, telemetry.force_error[0]),
                @as(f64, telemetry.force_error[1]),
                @as(f64, telemetry.force_error[2]),
            }), x, 330, 20, rl.Color.yellow);
            rl.drawText("teleop W/S surge  A/D sway  R/F heave  I/K roll", x, 360, 16, rl.Color.gray);
            rl.drawText("arrows yaw and pitch", x, 378, 16, rl.Color.gray);
        }
        rl.drawText(
            rl.textFormat("tune %s %.3f", .{
                tune_names[tune_selection].ptr,
                @as(f64, tuningValue(&config_packet, tune_selection).*),
            }),
            x,
            396,
            18,
            if (teleop_socket != null and !tune_error_logged) rl.Color.green else rl.Color.red,
        );
        rl.drawText("tune on teleop port  Q/E select  Z/X adjust", x, 418, 16, rl.Color.gray);
        rl.drawText("H show mission command legend", x, 448, 14, rl.Color.gray);
        rl.drawText("PID gains  drag sliders", x, @intFromFloat(pid_layout.row_y - 50), 18, rl.Color.ray_white);
        for (0..3) |group| {
            const group_y: i32 = @intFromFloat(
                pid_layout.row_y - 40 + @as(f32, @floatFromInt(group)) * pid_layout.row_step,
            );
            rl.drawText(pid_group_names[group], x, group_y, 16, rl.Color.light_gray);
            for (0..6) |axis| {
                const index = group * 6 + axis;
                const rect = pidSliderRect(index, pid_layout);
                const value = tuningValue(&config_packet, index).*;
                const ratio = @min(1.0, @max(0.0, value / pid_slider_max[group]));
                const track_x: i32 = @intFromFloat(rect.x);
                const track_y: i32 = @intFromFloat(rect.y);
                const track_width: i32 = @intFromFloat(rect.width);
                const track_height: i32 = @intFromFloat(rect.height);
                const filled_width: i32 = @intFromFloat(rect.width * ratio);
                const color = if (index == tune_selection) rl.Color.orange else rl.Color.sky_blue;

                rl.drawText(pid_axis_names[axis], track_x, group_y + 18, 12, rl.Color.gray);
                rl.drawText(rl.textFormat("%.2f", .{@as(f64, value)}), track_x, group_y + 60, 12, color);
                rl.drawRectangle(track_x, track_y, track_width, track_height, rl.Color.dark_gray);
                if (filled_width > 0)
                    rl.drawRectangle(track_x, track_y, filled_width, track_height, color);
                rl.drawRectangleLinesEx(rect, 1, rl.Color.gray);
            }
        }
        rl.drawTextureRec(scene.texture, .{
            .x = 0,
            .y = 0,
            .width = @floatFromInt(scene_w),
            .height = -@as(f32, @floatFromInt(scene_h)),
        }, .{ .x = view3d.x, .y = view3d.y }, rl.Color.white);
        rl.drawRectangleLinesEx(view3d, 1, rl.Color.gray);

        for (telemetry.thrusters[0..telemetry.thrusters_len], 0..) |thruster, index| {
            const y: i32 = 450 + @as(i32, @intCast(index)) * 30;
            const width: i32 = @intFromFloat(thruster * 100);
            rl.drawText(rl.textFormat("t%d %+.2f", .{ @as(c_int, @intCast(index)), @as(f64, thruster) }), x, y, 20, rl.Color.ray_white);
            rl.drawRectangle(if (width < 0) x + 240 + width else x + 240, y, @intCast(@abs(width)), 20, rl.Color.sky_blue);
            rl.drawLine(x + 240, y - 2, x + 240, y + 22, rl.Color.gray);
        }
        if (show_command_legend) {
            const legend_y: i32 = 438;
            rl.drawRectangle(x - 8, legend_y - 8, 365, 150, rl.fade(rl.Color.black, 0.95));
            rl.drawText("mission command legend", x, legend_y, 16, rl.Color.light_gray);
            rl.drawText("T  teleop / mission-control", x, legend_y + 20, 14, rl.Color.gray);
            rl.drawText("P/O  point or object target", x, legend_y + 38, 14, rl.Color.gray);
            rl.drawText("M/B/C  move, pass, or circle", x, legend_y + 56, 14, rl.Color.gray);
            rl.drawText("WASD/RF  edit point target", x, legend_y + 74, 14, rl.Color.gray);
            rl.drawText("arrows  edit look direction", x, legend_y + 92, 14, rl.Color.gray);
            rl.drawText("Q/E class   Enter send", x, legend_y + 110, 14, rl.Color.gray);
        }
        rl.endDrawing();
        if (shared_view) |view| {
            if (!view.valid()) last_packet = -1;
        }
    }
}
