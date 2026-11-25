const std = @import("std");
const rl = @import("raylib");
const Visualizer = @import("Visualizer");
const MissionIO = Visualizer.MissionIO;
const AuvWire = Visualizer.AuvWire;
const MissionConfig = MissionIO.CompiledConfig;
const Io = std.Io;

extern fn video_default_source() callconv(.c) [*:0]const u8;
extern fn video_open(source: [*:0]const u8, width: *c_int, height: *c_int) callconv(.c) bool;
extern fn video_get_size(width: *c_int, height: *c_int) callconv(.c) bool;
extern fn video_frame(out: [*]u8, capacity: usize, width: c_int, height: c_int) callconv(.c) bool;
extern fn video_has_error() callconv(.c) bool;
extern fn video_close() callconv(.c) void;

const Config = struct {
    video_source: [:0]const u8,
    mission_control: bool = MissionConfig.startup_mission == .mission_control,
    teleop_host: []const u8 = "127.0.0.1",
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
        } else if (std.mem.eql(u8, arg, "--teleop-host")) {
            i += 1;
            if (i >= args.len) return error.MissingTeleopHost;
            config.teleop_host = args[i];
        } else if (std.mem.eql(u8, arg, "--mission-control")) {
            config.mission_control = true;
        } else if (std.mem.eql(u8, arg, "--help") or std.mem.eql(u8, arg, "-h")) {
            std.debug.print(
                "usage: viewer [--video source|none] [--teleop-host IP] [--mission-control]\n",
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

const nonblocking: Io.Timeout = .{ .duration = .{ .clock = .awake, .raw = .zero } };

/// Owns all viewer UDP I/O; the Visualizer only sees decoded Input and emits Actions.
const ViewerHost = struct {
    control: Io.net.Socket,
    destination: Io.net.IpAddress,
    client_id: u64,
    telemetry: ?Io.net.Socket = null,
    telemetry_transport: MissionConfig.TelemetryTransport = .disabled,
    telemetry_addr: MissionConfig.FixedText(MissionConfig.telemetry_addr_max) = .{},
    frame: AuvWire.TelemetryDoc = AuvWire.emptyTelemetry(),
    arena: std.heap.ArenaAllocator = undefined,
    arena_ready: bool = false,
    last_telemetry: f64 = -1,
    last_telemetry_bind: f64 = -10,
    last_send: f64 = -1,
    send_error: bool = false,

    fn init(io: Io, teleop_host: []const u8, teleop_port: u16) !ViewerHost {
        var address_buf: [128]u8 = undefined;
        const destination = try Io.net.IpAddress.parseLiteral(
            try std.fmt.bufPrint(&address_buf, "{s}:{d}", .{ teleop_host, teleop_port }),
        );
        const bind: Io.net.IpAddress = .{ .ip4 = .unspecified(0) };
        const control = try bind.bind(io, .{ .mode = .dgram, .protocol = .udp });

        var client_id: u64 = 0;
        Io.random(io, std.mem.asBytes(&client_id));
        client_id |= 1;

        return .{
            .control = control,
            .destination = destination,
            .client_id = client_id,
        };
    }

    fn deinit(self: *ViewerHost, io: Io, viewer: *Visualizer) void {
        if (viewer.neutral()) |command| {
            self.sendControl(io, command) catch {};
        }
        self.closeTelemetry(io);
        self.control.close(io);
    }

    fn closeTelemetry(self: *ViewerHost, io: Io) void {
        if (self.arena_ready) {
            self.arena.deinit();
            self.arena_ready = false;
        }
        if (self.telemetry) |socket| socket.close(io);
        self.telemetry = null;
    }

    fn sendControl(self: *ViewerHost, io: Io, value: anytype) !void {
        var buffer: [4096]u8 align(8) = undefined;
        const packet = if (@TypeOf(value) == AuvWire.TeleopWire)
            try MissionIO.encodeControl(&buffer, AuvWire.TeleopDoc{ .command = value })
        else
            try MissionIO.encodeControl(&buffer, value);
        try self.control.send(io, &self.destination, packet);
    }

    fn pollControlStatus(self: *ViewerHost, io: Io, viewer: *Visualizer) void {
        var buffer: [8192]u8 = undefined;
        for (0..32) |_| {
            const message = self.control.receiveTimeout(io, &buffer, nonblocking) catch break;
            if (!MissionIO.matchesControlDocument(MissionIO.RuntimeStatusDoc, message.data)) continue;
            const status = MissionIO.decodeRuntimeStatus(message.data) catch continue;
            status.desired.validate() catch continue;
            viewer.acceptStatus(status);
        }
    }

    fn activePacket(viewer: *const Visualizer, initial: *const MissionIO.LiveConfigPacket) MissionIO.LiveConfigPacket {
        return if (viewer.runtimeStatus()) |status|
            .{ .revision = status.revision, .config = status.active_config }
        else
            initial.*;
    }

    fn rebindTelemetryIfNeeded(self: *ViewerHost, io: Io, config: *const MissionIO.LiveConfigPacket) void {
        if (self.telemetry_transport == config.config.runtime.telemetry_transport and
            MissionConfig.textEqual(self.telemetry_addr, config.config.runtime.telemetry_addr))
        {
            return;
        }
        self.closeTelemetry(io);
        self.telemetry_transport = config.config.runtime.telemetry_transport;
        self.telemetry_addr = config.config.runtime.telemetry_addr;
        self.last_telemetry = -1;
        self.last_telemetry_bind = -10;
    }

    fn pollTelemetry(self: *ViewerHost, io: Io, config: *const MissionIO.LiveConfigPacket, now: f64) Visualizer.Status {
        self.rebindTelemetryIfNeeded(io, config);
        if (self.telemetry_transport != .udp) return .unavailable;
        if (self.telemetry == null and now - self.last_telemetry_bind >= 1) {
            self.last_telemetry_bind = now;
            const address = Io.net.IpAddress.parseLiteral(self.telemetry_addr.slice() catch "") catch return .unavailable;
            self.telemetry = address.bind(io, .{ .mode = .dgram, .protocol = .udp }) catch return .unavailable;
        }
        const socket = self.telemetry orelse return .unavailable;
        if (!self.arena_ready) {
            self.arena = .init(std.heap.page_allocator);
            self.arena_ready = true;
        }
        _ = self.arena.reset(.retain_capacity);
        var buffer: [64 * 1024]u8 = undefined;
        for (0..64) |_| {
            const message = socket.receiveTimeout(io, &buffer, nonblocking) catch break;
            const bytes = self.arena.allocator().dupe(u8, message.data) catch continue;
            const frame = MissionIO.decodeTelemetryFrame(AuvWire.TelemetryDoc, bytes) catch continue;
            self.frame = frame;
            self.last_telemetry = now;
        }
        return if (self.last_telemetry < 0) .connecting else if (now - self.last_telemetry > 1) .stale else .live;
    }

    const PollResult = struct {
        telemetry_status: Visualizer.Status,
        frame: ?*const AuvWire.TelemetryDoc,
        send_error: bool,
    };

    fn poll(self: *ViewerHost, io: Io, viewer: *Visualizer, initial: *const MissionIO.LiveConfigPacket, now: f64) PollResult {
        self.pollControlStatus(io, viewer);
        const active = ViewerHost.activePacket(viewer, initial);
        const telemetry_status = self.pollTelemetry(io, &active, now);
        return .{
            .telemetry_status = telemetry_status,
            .frame = if (self.last_telemetry >= 0) &self.frame else null,
            .send_error = self.send_error,
        };
    }

    fn flush(self: *ViewerHost, io: Io, now: f64, actions: Visualizer.Actions) void {
        if (now - self.last_send >= 0.25) {
            self.last_send = now;
            self.send_error = false;
            if (actions.config) |packet| self.sendControl(io, packet) catch {
                self.send_error = true;
            };
            if (actions.thrusters) |command| self.sendControl(io, command) catch {
                self.send_error = true;
            };
            const request: MissionIO.RuntimeRequest = actions.lifecycle orelse .{
                .client_id = self.client_id,
                .request_id = 0,
                .action = .query,
            };
            self.sendControl(io, request) catch {
                self.send_error = true;
            };
        }
        if (actions.mission) |command| self.sendControl(io, command) catch {
            self.send_error = true;
        };
        if (actions.teleop) |command| self.sendControl(io, command) catch {
            self.send_error = true;
        };
    }
};

const VideoCapture = struct {
    source: [:0]const u8,
    started: bool = false,
    last_try: f64 = -10,
    last_frame: f64 = -1,
    image: ?rl.Image = null,
    view: Visualizer.Video = .{},

    fn deinit(self: *VideoCapture) void {
        if (self.view.texture.id != 0) rl.unloadTexture(self.view.texture);
        if (self.image) |image| rl.unloadImage(image);
        if (self.view.enabled) video_close();
    }

    fn update(self: *VideoCapture, now: f64) Visualizer.Video {
        if (!self.view.enabled) return self.view;
        if (!self.started and now - self.last_try > 2) {
            self.last_try = now;
            var width: c_int = 0;
            var height: c_int = 0;
            self.started = video_open(self.source.ptr, &width, &height);
        }
        if (self.started) {
            var width: c_int = 0;
            var height: c_int = 0;
            if (video_get_size(&width, &height) and width > 0 and height > 0 and
                (width != self.view.width or height != self.view.height))
            {
                const image = rl.genImageColor(width, height, rl.Color.black);
                if (rl.loadTextureFromImage(image)) |texture| {
                    if (self.view.texture.id != 0) rl.unloadTexture(self.view.texture);
                    if (self.image) |old| rl.unloadImage(old);
                    self.image = image;
                    self.view.texture = texture;
                    self.view.width = width;
                    self.view.height = height;
                    self.last_frame = -1;
                } else |_| rl.unloadImage(image);
            }
        }
        if (self.image) |image| {
            const pixels: [*]u8 = @ptrCast(image.data);
            const capacity = @as(usize, @intCast(self.view.width)) * @as(usize, @intCast(self.view.height)) * 4;
            if (video_frame(pixels, capacity, self.view.width, self.view.height)) {
                rl.updateTexture(self.view.texture, pixels);
                self.last_frame = now;
            }
        }
        self.view.status = if (video_has_error()) .unavailable else if (self.last_frame < 0) .connecting else if (now - self.last_frame > 1) .stale else .live;
        return self.view;
    }
};

pub fn main(init: std.process.Init) !void {
    const args = try init.minimal.args.toSlice(init.arena.allocator());
    const config = parseArgs(args) catch |err| switch (err) {
        error.HelpRequested => return,
        else => return err,
    };
    const startup = MissionConfig.startup_config;

    var host = try ViewerHost.init(init.io, config.teleop_host, startup.runtime.teleop_port);
    var viewer = try Visualizer.init(&startup, host.client_id, config.mission_control, false);
    defer viewer.deinit();
    defer host.deinit(init.io, &viewer);

    var video: VideoCapture = .{ .source = config.video_source, .view = .{ .enabled = !videoDisabled(config.video_source) } };
    defer video.deinit();

    const initial_config = MissionIO.LiveConfigPacket.fromRuntimeConfig(&startup);
    while (!Visualizer.shouldClose()) {
        const now = rl.getTime();
        const polled = host.poll(init.io, &viewer, &initial_config, now);
        const actions = try viewer.tick(.{
            .frame = polled.frame,
            .telemetry_status = polled.telemetry_status,
            .video = video.update(now),
            .send_error = polled.send_error,
        });
        host.flush(init.io, now, actions);
    }
}