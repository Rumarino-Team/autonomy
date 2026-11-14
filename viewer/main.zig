const std = @import("std");
const rl = @import("raylib");
const ui = @import("ui.zig");
const MissionIO = @import("MissionIO");
const MissionConfig = MissionIO.CompiledConfig;
const math = MissionIO.Math;
const Io = std.Io;

extern fn video_default_source() callconv(.c) [*:0]const u8;
extern fn video_open(source: [*:0]const u8, width: *c_int, height: *c_int) callconv(.c) bool;
extern fn video_get_size(width: *c_int, height: *c_int) callconv(.c) bool;
extern fn video_frame(out: [*]u8, capacity: usize, width: c_int, height: c_int) callconv(.c) bool;
extern fn video_has_error() callconv(.c) bool;
extern fn video_close() callconv(.c) void;

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
const pid_slider_max = [_]f32{ 10, 2, 10 };

const Config = struct {
    video_source: [:0]const u8,
    mission_control: bool = false,
};

fn parseArgs(args: []const [:0]const u8) !Config {
    var config = Config{ .video_source = std.mem.span(video_default_source()) };
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

fn pidSliderValue(index: usize, mouse_x: f32, layout: ui.Layout) f32 {
    const rect = layout.pidRect(index);
    const ratio = std.math.clamp((mouse_x - rect.x) / rect.width, 0, 1);
    return ratio * pid_slider_max[index / 6];
}

fn sendTcpPacket(
    io: Io,
    stream: *Io.net.Stream,
    kind: MissionIO.LogicalMessageKind,
    sequence: u32,
    value: anytype,
) !void {
    var frame: [MissionIO.logical_message_max_len + 4]u8 = undefined;
    const packet = try MissionIO.encodePacket(frame[4..], kind, sequence, value);
    std.mem.writeInt(u32, frame[0..4], @intCast(packet.len), .little);
    var writer_buffer: [1024]u8 = undefined;
    var writer = Io.net.Stream.writer(stream.*, io, &writer_buffer);
    try writer.interface.writeAll(frame[0 .. 4 + packet.len]);
    try writer.interface.flush();
}

fn sendTeleop(
    io: Io,
    socket: *Io.net.Socket,
    destination: *const Io.net.IpAddress,
    sequence: u32,
    command: MissionIO.TeleopCommand,
) !void {
    var datagram: [MissionIO.logical_message_max_len]u8 = undefined;
    const encoded = try MissionIO.encodePacket(
        &datagram,
        .teleop,
        sequence,
        command,
    );
    try socket.send(io, destination, encoded);
}

fn emptyTelemetry() MissionIO.TelemetryFrame {
    var frame = std.mem.zeroes(MissionIO.TelemetryFrame);
    frame.magic = "AUV1".*;
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

fn validFrame(frame: *const MissionIO.TelemetryFrame) bool {
    return std.mem.eql(u8, &frame.magic, "AUV1") and
        frame.objects_len <= frame.objects.len and frame.boxes_len <= frame.boxes.len and
        frame.thrusters_len <= frame.thrusters.len and frame.step_len <= frame.step.len and
        frame.goals_len <= frame.goals.len and frame.tam_len <= frame.tam.len;
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
    const realtime_bind = try Io.net.IpAddress.parseLiteral(telemetry_addr);
    var realtime_socket = realtime_bind.bind(init.io, .{ .mode = .dgram, .protocol = .udp }) catch {
        std.debug.print("can't open realtime UDP socket at {s}\n", .{telemetry_addr});
        return error.UdpOpenFailed;
    };
    defer realtime_socket.close(init.io);

    const teleop_dest = blk: {
        var address_buf: [48]u8 = undefined;
        const text = try std.fmt.bufPrint(&address_buf, "127.0.0.1:{d}", .{compiled_config.teleop_port});
        break :blk try Io.net.IpAddress.parseLiteral(text);
    };
    const control_dest = blk: {
        var address_buf: [48]u8 = undefined;
        const text = try std.fmt.bufPrint(&address_buf, "127.0.0.1:{d}", .{compiled_config.control_port});
        break :blk try Io.net.IpAddress.parseLiteral(text);
    };
    var config_packet = MissionIO.LiveConfigPacket.fromRuntimeConfig(compiled_config);
    var tcp_stream: ?Io.net.Stream = null;
    defer if (tcp_stream) |*stream| stream.close(init.io);
    var tcp_decoder: MissionIO.TcpFrameDecoder = .{};
    var tcp_sequence: u32 = 1;
    var tcp_last_connect: f64 = -10;
    std.debug.print(
        "realtime UDP -> 127.0.0.1:{d}; TCP control -> 127.0.0.1:{d}\n",
        .{ compiled_config.teleop_port, compiled_config.control_port },
    );

    const read_video = !videoDisabled(config.video_source);
    rl.setConfigFlags(.{ .window_resizable = true });
    rl.initWindow(1280, 1000, "viewer");
    defer rl.closeWindow();
    rl.setWindowMinSize(1000, 700);
    rl.setTargetFPS(60);

    const embedded_font = @embedFile("assets/IBMPlexSans-Regular.ttf");
    const loaded_font: ?rl.Font = rl.loadFontFromMemory(".ttf", embedded_font, 32, null) catch null;
    const font = if (loaded_font) |value| value else try rl.getFontDefault();
    if (loaded_font != null) rl.setTextureFilter(font.texture, .bilinear);
    defer if (loaded_font) |value| rl.unloadFont(value);

    var scene_w: i32 = 900;
    var scene_h: i32 = 470;
    var scene = try rl.loadRenderTexture(scene_w, scene_h);
    defer rl.unloadRenderTexture(scene);

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
    var video_started = false;
    var last_source_try: f64 = -10;
    var source_opened_at: f64 = -1;
    var last_teleop: f64 = -1;
    var teleop_sequence: u32 = 1;
    var mission_target_initialized = false;
    var mission_kind: MissionIO.MissionCommandKind = .point;
    var mission_maneuver: MissionIO.MissionManeuver = .move;
    var mission_position: math.Vector3f = .{ 0, 0, 0 };
    var mission_look_direction: math.Vector3f = .{ 0, 1, 0 };
    var mission_look_yaw: f32 = 0;
    var mission_look_pitch: f32 = 0;
    var mission_object_class: u8 = 0;
    var mission_control = config.mission_control;
    var workspace: ui.Workspace = if (config.mission_control) .mission else .operation;
    var font_percent: u8 = 100;
    var mission_sent = false;
    var tune_sent = false;
    var object_scroll: usize = 0;
    var goal_scroll: usize = 0;
    var tune_selection: usize = 0;
    var last_tune_adjust: f64 = -1;
    var tune_error_logged = false;
    var mission_error_logged = false;
    var teleop_error_logged = false;
    var tcp_error_logged = false;
    var pid_drag: ?usize = null;
    defer if (!mission_control) {
        const neutral = MissionIO.TeleopCommand{
            .translation = .{ 0, 0, 0 },
            .orientation = orientation,
        };
        sendTeleop(init.io, &realtime_socket, &teleop_dest, teleop_sequence, neutral) catch {};
    };

    while (!rl.windowShouldClose()) {
        telemetry = &udp_telemetry;
        const now = rl.getTime();
        if (tcp_stream == null and now - tcp_last_connect >= 1) {
            tcp_last_connect = now;
            if (control_dest.connect(init.io, .{ .mode = .stream, .protocol = .tcp })) |stream| {
                var connected = stream;
                MissionIO.setSocketNonBlocking(&connected.socket) catch {
                    connected.close(init.io);
                    return error.SetNonBlockingFailed;
                };
                tcp_stream = connected;
                tcp_decoder = .{};
                tcp_error_logged = false;
            } else |err| {
                if (!tcp_error_logged) {
                    std.debug.print("TCP control connection failed: {s}\n", .{@errorName(err)});
                    tcp_error_logged = true;
                }
            }
        }
        if (tcp_stream) |*stream| {
            var close_tcp = false;
            var recv_buf: [4096]u8 = undefined;
            var read_count: usize = 0;
            var parsed_messages: usize = 0;
            while (read_count < 16 and parsed_messages < 16) : (read_count += 1) {
                var chunks: [1][]u8 = .{&recv_buf};
                const count = stream.read(init.io, &chunks) catch |err| switch (err) {
                    error.ConnectionTimedOut => break,
                    else => {
                        close_tcp = true;
                        break;
                    },
                };
                if (count == 0) {
                    close_tcp = true;
                    break;
                }
                tcp_decoder.feed(recv_buf[0..count]) catch {
                    close_tcp = true;
                    break;
                };

                while (parsed_messages < 16) {
                    const frame = tcp_decoder.peek() catch {
                        close_tcp = true;
                        break;
                    } orelse break;
                    if (frame.message.kind == .command_ack) {
                        const status = MissionIO.decodeCommandAck(frame.message.payload) catch .rejected;
                        if (status == .accepted) {
                            tcp_error_logged = false;
                        } else {
                            std.debug.print("TCP control command rejected (sequence {d})\n", .{frame.message.sequence});
                        }
                    }
                    tcp_decoder.consume(frame.consumed);
                    parsed_messages += 1;
                }
                if (close_tcp) break;
            }
            if (close_tcp) {
                stream.close(init.io);
                tcp_stream = null;
                tcp_decoder = .{};
            }
        }
        const layout = ui.Layout.init(@floatFromInt(rl.getScreenWidth()), @floatFromInt(rl.getScreenHeight()), read_video);
        if (rl.isKeyPressed(.equal) or rl.isKeyPressed(.kp_add)) font_percent = ui.adjustFontPercent(font_percent, 5);
        if (rl.isKeyPressed(.minus) or rl.isKeyPressed(.kp_subtract)) font_percent = ui.adjustFontPercent(font_percent, -5);
        if (rl.isKeyPressed(.zero)) font_percent = 100;
        if (rl.isKeyPressed(.one)) workspace = .operation;
        if (rl.isKeyPressed(.two)) workspace = .mission;
        if (rl.isKeyPressed(.three)) workspace = .tuning;
        if (rl.isMouseButtonPressed(.left)) {
            const mouse = rl.getMousePosition();
            if (rl.checkCollisionPointRec(mouse, layout.fontButton(false))) font_percent = ui.adjustFontPercent(font_percent, -5);
            if (rl.checkCollisionPointRec(mouse, layout.fontButton(true))) font_percent = ui.adjustFontPercent(font_percent, 5);
            for (0..3) |i| if (rl.checkCollisionPointRec(rl.getMousePosition(), layout.navRect(i))) {
                workspace = @fromBackingInt(@intCast(i));
                pid_drag = null;
                break;
            };
        }
        const view3d = layout.sceneContent(if (workspace == .mission) .mission else .operation);
        const next_w: i32 = @max(1, @as(i32, @intFromFloat(view3d.width)));
        const next_h: i32 = @max(1, @as(i32, @intFromFloat(view3d.height)));
        if (next_w != scene_w or next_h != scene_h) {
            const replacement = try rl.loadRenderTexture(next_w, next_h);
            rl.unloadRenderTexture(scene);
            scene = replacement;
            scene_w = next_w;
            scene_h = next_h;
        }
        if (read_video and !video_started and now - last_video_try > 2) {
            last_video_try = now;
            var width: c_int = 0;
            var height: c_int = 0;
            video_started = video_open(config.video_source.ptr, &width, &height);
        }
        if (video_started) {
            var width: c_int = 0;
            var height: c_int = 0;
            if (video_get_size(&width, &height) and width > 0 and height > 0 and
                (width != video_w or height != video_h))
            {
                const blank = rl.genImageColor(width, height, rl.Color.black);
                const new_video = rl.loadTextureFromImage(blank) catch {
                    rl.unloadImage(blank);
                    continue;
                };
                if (video.id != 0) rl.unloadTexture(video);
                if (video_image) |old| rl.unloadImage(old);
                video = new_video;
                video_w = width;
                video_h = height;
                pixels = @ptrCast(blank.data);
                video_image = blank;
                last_frame = -1;
                std.debug.print("video dimensions: {d}x{d}\n", .{ width, height });
            }
        }

        switch (compiled_config.telemetry_transport) {
            .shared_memory => {
                // A restarted publisher can replace the shared-memory object. Reopen
                // stale mappings so the viewer can follow the new publisher.
                const last_activity = if (last_packet >= source_opened_at) last_packet else source_opened_at;
                if (subscriber != null and last_activity >= 0 and now - last_activity > 2) {
                    subscriber.?.deinit();
                    subscriber = null;
                }
                if (subscriber == null and now - last_source_try > 2) {
                    last_source_try = now;
                    subscriber = MissionIO.TelemetrySubscriber.open(telemetry_shm_name) catch |err| blk: {
                        std.debug.print("waiting for shared telemetry '{s}': {s}\n", .{ telemetry_shm_name, @errorName(err) });
                        break :blk null;
                    };
                    if (subscriber != null) source_opened_at = now;
                }
                if (subscriber) |*source| {
                    var attempts: usize = 0;
                    while (attempts < 2) : (attempts += 1) {
                        const latest = source.latest() orelse break;
                        if (!latest.valid()) continue;
                        const snapshot = latest.frame.*;
                        if (!latest.valid() or !validFrame(&snapshot)) continue;
                        udp_telemetry = snapshot;
                        if (last_timestamp == null or snapshot.timestamp != last_timestamp.?) {
                            last_timestamp = snapshot.timestamp;
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
                for (0..16) |_| {
                    const datagram = realtime_socket.receiveTimeout(init.io, &packet, timeout) catch |err| switch (err) {
                        error.Timeout => break,
                        else => break,
                    };
                    const message = MissionIO.decodeLogicalMessage(datagram.data) catch continue;
                    if (message.kind != .telemetry) continue;
                    var snapshot: MissionIO.TelemetryFrame = undefined;
                    MissionIO.decodeTelemetryFrame(message.payload, &snapshot) catch continue;
                    udp_telemetry = snapshot;
                    last_timestamp = snapshot.timestamp;
                    last_packet = now;
                }
            },
            .stdout => {},
        }

        const list_capacity = ui.Layout.listCapacity(layout.objects);
        const object_max = @as(usize, telemetry.objects_len) -| list_capacity;
        const goal_max = @as(usize, telemetry.goals_len) -| list_capacity;
        object_scroll = @min(object_scroll, object_max);
        goal_scroll = @min(goal_scroll, goal_max);
        if (workspace == .mission and rl.isWindowFocused()) {
            const wheel = rl.getMouseWheelMove();
            if (wheel != 0) {
                const mouse = rl.getMousePosition();
                if (rl.checkCollisionPointRec(mouse, layout.objects)) {
                    object_scroll = if (wheel > 0) object_scroll -| 1 else @min(object_scroll + 1, object_max);
                } else if (rl.checkCollisionPointRec(mouse, layout.goals)) {
                    goal_scroll = if (wheel > 0) goal_scroll -| 1 else @min(goal_scroll + 1, goal_max);
                }
            }
        }

        if (pixels) |video_pixels| {
            const capacity = @as(usize, @intCast(video_w)) * @as(usize, @intCast(video_h)) * 4;
            if (video_frame(video_pixels, capacity, video_w, video_h)) {
                rl.updateTexture(video, video_pixels);
                last_frame = now;
            }
        }

        if (rl.isKeyPressed(.t)) {
            // Release any held teleoperation translation before leaving teleop mode.
            if (!mission_control) {
                const neutral = MissionIO.TeleopCommand{
                    .translation = .{ 0, 0, 0 },
                    .orientation = orientation,
                };
                sendTeleop(init.io, &realtime_socket, &teleop_dest, teleop_sequence, neutral) catch {
                    teleop_error_logged = true;
                };
                teleop_sequence +%= 1;
                translation = .{ 0, 0, 0 };
            }
            mission_control = !mission_control;
            if (mission_control) mission_target_initialized = false;
            std.debug.print(
                "control mode: {s}\n",
                .{if (mission_control) "mission-control" else "teleop"},
            );
        }
        const focused = rl.isWindowFocused();
        const mouse_click = focused and workspace == .mission and rl.isMouseButtonPressed(.left);
        if (mouse_click and mission_control) {
            if (rl.checkCollisionPointRec(rl.getMousePosition(), layout.missionButton(0))) mission_kind = .point;
            if (rl.checkCollisionPointRec(rl.getMousePosition(), layout.missionButton(1))) mission_kind = .object;
        }

        if (mission_control and !mission_target_initialized and last_packet >= 0 and now - last_packet <= 1) {
            mission_position = telemetry.camera_pose.pos + @as(math.Vector3f, .{ 0, 2, 0 });
            mission_target_initialized = true;
        }

        if (mission_control and focused and workspace == .mission) {
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

            const clicked_send = mouse_click and rl.checkCollisionPointRec(rl.getMousePosition(), layout.sendButton());
            if (workspace == .mission and (rl.isKeyPressed(.enter) or clicked_send) and tcp_stream != null and (mission_kind != .point or (mission_target_initialized and last_packet >= 0 and now - last_packet <= 1))) {
                var command: MissionIO.MissionCommand = switch (mission_kind) {
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
                if (command.validate()) |_| {
                    if (sendTcpPacket(init.io, &tcp_stream.?, .mission_command, tcp_sequence, command)) |_| {
                        tcp_sequence +%= 1;
                        mission_error_logged = false;
                        mission_sent = true;
                    } else |_| {
                        if (!mission_error_logged) std.debug.print("mission command send failed\n", .{});
                        mission_error_logged = true;
                        if (tcp_stream) |*stream| stream.close(init.io);
                        tcp_stream = null;
                        tcp_decoder = .{};
                    }
                } else |err| {
                    if (!mission_error_logged) std.debug.print("mission command validation failed: {s}\n", .{@errorName(err)});
                    mission_error_logged = true;
                }
            }
        } else if (!mission_control and now - last_teleop >= teleop_period) {
            last_teleop = now;
            translation = .{ 0, 0, 0 };
            if (focused and rl.isKeyDown(.d)) translation[0] += 1;
            if (focused and rl.isKeyDown(.a)) translation[0] -= 1;
            if (focused and rl.isKeyDown(.w)) translation[1] += 1;
            if (focused and rl.isKeyDown(.s)) translation[1] -= 1;
            if (focused and rl.isKeyDown(.r)) translation[2] += 1;
            if (focused and rl.isKeyDown(.f)) translation[2] -= 1;
            normalizeTranslation(&translation);

            if (focused and rl.isKeyDown(.right)) orientation = math.quaternionMul(axisAngle(.{ 0, 0, 1 }, -teleop_step), orientation);
            if (focused and rl.isKeyDown(.left)) orientation = math.quaternionMul(axisAngle(.{ 0, 0, 1 }, teleop_step), orientation);
            if (focused and rl.isKeyDown(.up)) orientation = math.quaternionMul(orientation, axisAngle(.{ 1, 0, 0 }, -teleop_step));
            if (focused and rl.isKeyDown(.down)) orientation = math.quaternionMul(orientation, axisAngle(.{ 1, 0, 0 }, teleop_step));
            if (focused and rl.isKeyDown(.i)) orientation = math.quaternionMul(orientation, axisAngle(.{ 0, 1, 0 }, teleop_step));
            if (focused and rl.isKeyDown(.k)) orientation = math.quaternionMul(orientation, axisAngle(.{ 0, 1, 0 }, -teleop_step));
            normalizeQuaternion(&orientation);

            const command = [7]f32{
                translation[0], translation[1], translation[2],
                orientation[0], orientation[1], orientation[2],
                orientation[3],
            };
            const teleop_command = MissionIO.TeleopCommand{
                .translation = .{ command[0], command[1], command[2] },
                .orientation = .{ command[3], command[4], command[5], command[6] },
            };
            if (sendTeleop(init.io, &realtime_socket, &teleop_dest, teleop_sequence, teleop_command)) |_| {
                teleop_sequence +%= 1;
                teleop_error_logged = false;
            } else |_| {
                if (!teleop_error_logged) std.debug.print("teleop send failed\n", .{});
                teleop_error_logged = true;
            }
        }

        if (!mission_control and focused and workspace == .tuning) {
            if (rl.isKeyPressed(.q)) {
                tune_selection = (tune_selection + tune_names.len - 1) % tune_names.len;
            }
            if (rl.isKeyPressed(.e)) {
                tune_selection = (tune_selection + 1) % tune_names.len;
            }
        }

        var tune_changed = false;
        if (focused and workspace == .tuning and rl.isMouseButtonPressed(.left)) {
            const mouse = rl.getMousePosition();
            for (0..tune_names.len) |index| {
                if (rl.checkCollisionPointRec(mouse, layout.pidRect(index))) {
                    tune_selection = index;
                    pid_drag = index;
                    break;
                }
            }
        }
        if (!focused or workspace != .tuning or !rl.isMouseButtonDown(.left)) pid_drag = null;
        if (pid_drag) |index| {
            if (now - last_tune_adjust >= tune_period) {
                const value = pidSliderValue(index, rl.getMousePosition().x, layout);
                const target = tuningValue(&config_packet, index);
                if (@abs(target.* - value) > 1e-5) {
                    target.* = value;
                    last_tune_adjust = now;
                    tune_changed = true;
                }
            }
        }
        if (focused and workspace == .tuning and now - last_tune_adjust >= tune_period) {
            var delta: f32 = 0;
            if (rl.isKeyDown(.z)) delta -= tune_step;
            if (rl.isKeyDown(.x)) delta += tune_step;
            if (delta != 0) {
                tuningValue(&config_packet, tune_selection).* += delta;
                last_tune_adjust = now;
                tune_changed = true;
            }
        }
        if (tune_changed and tcp_stream != null) {
            if (sendTcpPacket(init.io, &tcp_stream.?, .live_config, tcp_sequence, config_packet)) |_| {
                tcp_sequence +%= 1;
                tune_error_logged = false;
                tune_sent = true;
            } else |_| {
                if (!tune_error_logged) std.debug.print("live tuning send failed\n", .{});
                tune_error_logged = true;
                if (tcp_stream) |*stream| stream.close(init.io);
                tcp_stream = null;
                tcp_decoder = .{};
            }
        }

        if (focused and workspace != .tuning and rl.checkCollisionPointRec(rl.getMousePosition(), view3d)) {
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

        const nose = math.quaternionRotate(telemetry.camera_pose.quat, [3]f32{ 0, 1, 0 });
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

        for (telemetry.objects[0..telemetry.objects_len], 0..) |object, i| {
            if (i == 0) rl.drawLine3D(sub, toRay(object.pose.pos), rl.fade(rl.Color.ray_white, 0.45));
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
        rl.endTextureMode();

        const telemetry_status: ui.Status = if (compiled_config.telemetry_transport == .stdout)
            .unavailable
        else if (last_packet < 0)
            .connecting
        else if (now - last_packet > 1)
            .stale
        else
            .live;
        const video_status: ui.Status = if (!read_video)
            .disabled
        else if (video_has_error())
            .unavailable
        else if (video_w == 0 or last_frame < 0)
            .connecting
        else if (now - last_frame > 1)
            .stale
        else
            .live;
        rl.beginDrawing();
        ui.draw(.{
            .font = font,
            .font_percent = font_percent,
            .workspace = workspace,
            .layout = layout,
            .frame = telemetry,
            .telemetry_status = telemetry_status,
            .video_status = video_status,
            .video = video,
            .video_w = video_w,
            .video_h = video_h,
            .scene = scene,
            .mission_control = mission_control,
            .socket_ok = tcp_stream != null,
            .send_error = teleop_error_logged or mission_error_logged or tune_error_logged or tcp_error_logged,
            .translation = translation,
            .orientation = orientation,
            .mission_kind = mission_kind,
            .mission_maneuver = mission_maneuver,
            .mission_position = mission_position,
            .mission_look = mission_look_direction,
            .mission_class = mission_object_class,
            .target_initialized = mission_target_initialized,
            .config = &config_packet,
            .tune_selection = tune_selection,
            .mission_sent = mission_sent,
            .tune_sent = tune_sent,
            .object_scroll = object_scroll,
            .goal_scroll = goal_scroll,
        });
        rl.endDrawing();
    }
}
