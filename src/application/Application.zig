//! Platform/runtime shell: I/O, telemetry, visualizer, and the per-frame pipeline.
const std = @import("std");
const Io = std.Io;

const Auv = @import("../Auv.zig");
const MissionConfig = @import("../MissionConfig.zig");
const MissionIO = @import("../MissionIO.zig");
const Serial = @import("../Serial.zig");
const AuvRobot = @import("../AuvRobot.zig");
const reflection = @import("../ReflectionWriter.zig");

const simulator_enabled = @import("build_options").simulator;
const SimSession = if (simulator_enabled) @import("../simulator/Session.zig").Session else void;

const Visualizer = if (simulator_enabled) @import("../Visualizer.zig") else void;
const VisualizerMeshCache = if (simulator_enabled) @import("../simulator/VisualizerMeshCache.zig").VisualizerMeshCache else void;
const SceneMeshDraw = if (simulator_enabled) @import("../simulator/VisualizerMeshCache.zig").DrawContext else void;
const drawSceneMeshes = if (simulator_enabled) @import("../simulator/VisualizerMeshCache.zig").drawEmbedded else void;
const EmbeddedView = if (simulator_enabled) struct {
    viewer: Visualizer,
    scene_meshes: VisualizerMeshCache = undefined,
    has_scene_meshes: bool = false,
    mesh_draw: SceneMeshDraw = .{ .cache = undefined, .bodies = &.{} },
    last_idle_draw: ?Io.Clock.Timestamp = null,
    config_reply_revision: u64 = 0,
    config_rejected: bool = false,
} else void;

pub const frame_pipeline_version: u32 = 2;

pub const SensorFrame = Auv.Frame;
pub const Observations = AuvRobot.Observations;
pub const ActuatorCommand = AuvRobot.ActuatorCommand;

pub const AuvApplication = Application(AuvRobot);

pub fn Application(comptime Robot: type) type {
    return struct {
        const Self = @This();
        const Wire = Robot.Wire;
        robot: Robot = .{},
        mission_active: bool = false,
        pending: ActuatorCommand = .{},
        pending_valid: bool = false,

        io: Io = undefined,
        frame: Auv.Frame = undefined,
        config: MissionConfig.MissionConfig = undefined,
        telemetry_doc: Wire.TelemetryDoc = Wire.emptyTelemetry(),
        snapshot_buffers: Wire.SnapshotBuffers = .{},
        telemetry: *MissionIO.Telemetry = undefined,
        visualizer: ?*EmbeddedView = null,
        previous_frame_ns: ?u64 = null,

        session: SimSession = undefined,
        auv: if (simulator_enabled) void else ?Auv = if (simulator_enabled) {} else null,
        loader: if (simulator_enabled) void else ?MissionIO.MissionDiynLoader = if (simulator_enabled) {} else null,
        serial: if (simulator_enabled) void else ?Serial.Port = if (simulator_enabled) {} else null,
        parser: if (simulator_enabled) void else Serial.Parser = if (simulator_enabled) {} else .{},
        hardware_frame_time: if (simulator_enabled) void else ?Io.Clock.Timestamp = if (simulator_enabled) {} else null,

        pub fn initProcess(self: *Self, allocator: std.mem.Allocator, io: Io, windowed: bool) !void {
            self.* = .{ .io = io, .config = MissionConfig.startup_config };
            if (windowed) self.config.runtime.telemetry_transport = .disabled;

            var bootstrap = self.config;
            bootstrap.runtime.telemetry_transport = .disabled;
            bootstrap.runtime.telemetry_log_path = .{};
            const enable_udp_control = !windowed and self.config.runtime.mission_profile != .stonefish_sim;
            self.telemetry = try allocator.create(MissionIO.Telemetry);
            errdefer allocator.destroy(self.telemetry);
            self.telemetry.* = try .init(io, bootstrap, enable_udp_control);
            errdefer self.telemetry.deinit(io);
            self.telemetry.reconfigure(io, self.config) catch |err| {
                self.telemetry.report(err);
                self.telemetry.telemetry_error = self.telemetry.last_error;
            };

            if (comptime simulator_enabled) {
                if (windowed) {
                    const embedded = try allocator.create(EmbeddedView);
                    embedded.* = .{
                        .viewer = Visualizer.init(&self.config, 1, self.config.mission_id == .mission_control, true) catch |err| {
                            std.log.err("could not open the simulator visualizer ({s}); use --headless to run without a display", .{@errorName(err)});
                            return err;
                        },
                    };
                    self.visualizer = embedded;
                }
            }
        }

        pub fn deinitProcess(self: *Self) void {
            deinitMission(self);
            if (comptime simulator_enabled) {
                if (self.visualizer) |embedded| {
                    if (embedded.has_scene_meshes) embedded.scene_meshes.deinit();
                    embedded.viewer.deinit();
                }
            }
            self.telemetry.deinit(self.io);
        }

        fn pollCommands(self: *Self) !void {
            try self.telemetry.pollControl(self.io, &self.config);
            while (self.telemetry.takeDnaPacket()) |bytes| {
                if (reflection.matchesDocument(Wire.TeleopDoc, bytes)) {
                    self.telemetry.storeTeleopPacket(bytes);
                } else if (reflection.matchesDocument(Wire.ThrusterDoc, bytes)) {
                    const decoded = reflection.read(Wire.ThrusterDoc, bytes) catch continue;
                    self.telemetry.acceptThrusters(self.io, &self.config, decoded.values[0..decoded.len]);
                }
            }
        }

        pub fn serviceWithoutFrame(self: *Self) !void {
            try present(self, null);
            try pollCommands(self);
        }

        fn present(self: *Self, frame: ?*const Wire.TelemetryDoc) !void {
            if (comptime !simulator_enabled) return;
            const embedded = self.visualizer orelse return;
            if (Visualizer.shouldClose()) return error.WindowClosed;

            if (frame == null) {
                if (embedded.last_idle_draw) |last| {
                    if (last.untilNow(self.io).raw.toNanoseconds() < 16_666_667) return;
                }
                embedded.last_idle_draw = .now(self.io, .awake);
            } else {
                embedded.last_idle_draw = null;
            }

            var scene_draw: ?Visualizer.SceneDraw = null;
            if (embedded.has_scene_meshes) {
                if (comptime simulator_enabled) {
                    embedded.mesh_draw.bodies = self.session.bodyStates();
                    scene_draw = .{ .ctx = &embedded.mesh_draw, .draw = drawSceneMeshes };
                }
            }
            const actions = try embedded.viewer.tick(.{
                .status = self.telemetry.status(&self.config, embedded.config_reply_revision, embedded.config_rejected),
                .frame = frame,
                .telemetry_status = if (frame != null) .live else if (self.telemetry.state == .starting) .connecting else .disabled,
                .scene_draw = scene_draw,
            });
            embedded.config_reply_revision = 0;
            embedded.config_rejected = false;
            if (actions.config) |packet| {
                embedded.config_reply_revision = packet.revision;
                self.telemetry.applyConfig(self.io, &self.config, packet) catch |err| {
                    self.telemetry.report(err);
                    embedded.config_rejected = true;
                };
            }
            if (actions.lifecycle) |request| self.telemetry.acceptRequest(request);
            if (actions.thrusters) |command| self.telemetry.acceptThrusters(self.io, &self.config, command.values[0..command.len]);
            if (actions.teleop) |command| {
                var scratch: [1024]u8 align(8) = undefined;
                const bytes = reflection.writeRowInto(&scratch, Wire.TeleopDoc{ .command = command }) catch null;
                if (bytes) |packet| self.telemetry.storeTeleopPacket(packet);
            }
            if (actions.mission) |command| self.telemetry.acceptMission(command) catch |err| self.telemetry.report(err);
        }

        fn waitForReadyOnBackend(self: *Self) !void {
            const max_retries = 1000;
            for (0..max_retries + 1) |attempt| {
                try serviceWithoutFrame(self);
                if (try readSerialReplies(self)) return;
                if (attempt < max_retries) try Io.sleep(self.io, .fromMilliseconds(10), .awake);
            }
            return error.SerialReadyTimeout;
        }

        fn publishFrameAfterTick(self: *Self, goals: *const std.Deque(MissionIO.Goal), step: []const u8) !void {
            self.telemetry_doc = self.snapshot_buffers.capture(
                &self.robot,
                &self.frame,
                &self.config,
                goals,
                step,
                self.pending.values[0..self.pending.len],
            );
            self.telemetry.publishTelemetry(self.io, &self.telemetry_doc);
            if (comptime simulator_enabled) {
                if (self.visualizer != null) {
                    try present(self, &self.telemetry_doc);
                    try pollCommands(self);
                }
            }
        }

        pub fn runtimeLoop(self: *Self, goals: *const std.Deque(MissionIO.Goal), step: []const u8) !void {
            try pollCommands(self);
            try gatherSensors(self, &self.frame);
            const elapsed = frameElapsed(&self.previous_frame_ns, self.frame.timestamp);

            var observations: Observations = undefined;
            processPerception(&self.frame, &observations);
            self.robot.observe(&observations);

            var actuator_cmd: ActuatorCommand = .{};
            if (self.config.mission_id == .thruster_control) {
                actuator_cmd.len = self.telemetry.direct_len;
                @memcpy(actuator_cmd.values[0..actuator_cmd.len], self.telemetry.direct_values[0..actuator_cmd.len]);
            } else {
                self.robot.update(&observations, &self.config, elapsed, &actuator_cmd);
            }
            applyActuators(self, &actuator_cmd);
            try publishFrameAfterTick(self, goals, step);
        }

        pub fn initMission(self: *Self, allocator: std.mem.Allocator) !void {
            try serviceWithoutFrame(self);
            const io = self.io;
            const config = &self.config;
            try config.validate();
            const tam_len = config.tam_len;

            if (config.runtime.serial_port.len != 0) switch (config.runtime.mission_profile) {
                .proteus_hwd => {},
                .stonefish_sim, .mujoco_sim => return error.InvalidThrusterCount,
            };

            const realtime = self.visualizer != null or config.runtime.sim_realtime or config.runtime.serial_port.len != 0;
            if (comptime simulator_enabled) {
                self.session = undefined;
                try self.session.init(allocator, io, config, tam_len, realtime);
            } else {
                var dyn_loader = MissionIO.MissionDiynLoader.init(config.runtime.sensor_library.slice() catch return error.InvalidPath);
                const loaded = try dyn_loader.load();
                self.loader = dyn_loader;
                self.auv = loaded;
                self.auv.?.init();
            }
            self.mission_active = true;
            errdefer self.deinitMission();
            self.pending = .{};
            self.pending_valid = false;
            self.previous_frame_ns = null;
            self.telemetry_doc = Wire.emptyTelemetry();
            self.snapshot_buffers = .{};

            if (comptime !simulator_enabled) {
                const stonefish_temporal_thrusters = if (self.auv) |loaded| loaded.stonefish_temporal_thrusters != null else false;
                if (config.runtime.serial_port.len == 0 and !stonefish_temporal_thrusters) return error.MissingOutput;
                try pollCommands(self);
                var initial_frame: Auv.Frame = undefined;
                try gatherSensors(self, &initial_frame);
                try pollCommands(self);
                if (config.runtime.serial_port.len != 0) {
                    const path = config.runtime.serial_port.sentinelSlice() catch return error.InvalidPath;
                    try openSerial(self, path);
                    try waitForReadyOnBackend(self);
                    std.log.info("READY on {s} at 115200 baud", .{path});
                }
            }

            if (comptime simulator_enabled) {
                if (self.visualizer) |embedded| {
                    embedded.scene_meshes = try VisualizerMeshCache.init(allocator, &self.session.scene);
                    embedded.has_scene_meshes = true;
                    embedded.mesh_draw = .{ .cache = &embedded.scene_meshes, .bodies = &.{} };
                }
            }

            self.robot.reset();
            self.frame = std.mem.zeroes(Auv.Frame);
        }

        pub fn deinitMission(self: *Self) void {
            if (!self.mission_active) return;
            if (comptime simulator_enabled) {
                if (self.visualizer) |embedded| {
                    if (embedded.has_scene_meshes) {
                        embedded.scene_meshes.deinit();
                        embedded.has_scene_meshes = false;
                    }
                }
                self.session.deinit();
            } else {
                if (self.serial) |*port| port.deinit();
                if (self.auv) |auv| auv.deinit();
                if (self.loader) |*l| l.dynlib.close();
                self.serial = null;
                self.auv = null;
                self.loader = null;
                self.parser = .{};
                self.hardware_frame_time = null;
            }
            self.mission_active = false;
        }

        pub fn openSerial(self: *Self, path: [:0]const u8) !void {
            if (comptime simulator_enabled) return error.SimulatorNotBuilt;
            self.serial = try Serial.Port.init(self.io, path);
        }

        pub fn readSerialReplies(self: *Self) !bool {
            if (comptime simulator_enabled) return false;
            if (self.serial) |*port| return try port.readReplies(&self.parser);
            return false;
        }

        fn paceHardwareFrame(self: *Self) !void {
            if (comptime simulator_enabled) return;
            if (self.serial == null) return;
            if (self.hardware_frame_time) |previous| {
                const remaining = 16_666_667 - previous.untilNow(self.io).raw.toNanoseconds();
                if (remaining > 0) try Io.sleep(self.io, .fromNanoseconds(remaining), .awake);
            }
            self.hardware_frame_time = .now(self.io, .awake);
        }

        pub fn gatherSensors(self: *Self, out: *SensorFrame) !void {
            if (comptime simulator_enabled) {
                if (self.pending_valid and self.pending.len > 0) {
                    var buf: [MissionConfig.thruster_count]f32 = @splat(0);
                    @memcpy(buf[0..self.pending.len], self.pending.values[0..self.pending.len]);
                    try self.session.setThrusters(&buf);
                    self.pending_valid = false;
                } else if (self.pending_valid) {
                    self.pending_valid = false;
                }
                try self.session.nextFrame(out);
                return;
            }

            const auv = self.auv orelse return error.BackendNotInitialized;
            if (self.pending_valid and self.pending.len > 0) {
                const p = self.pending;
                auv.setStonefishTemporalThrusters(p.values[0..p.len]);
                if (self.serial) |*port| {
                    var channels: [Serial.protocol_v2_command_channel_count]f32 = @splat(0);
                    const n = @min(Serial.protocol_v2_command_channel_count, p.len);
                    @memcpy(channels[0..n], p.values[0..n]);
                    const packet = Serial.encode(&channels);
                    try port.writeAll(&packet, 100);
                }
                self.pending_valid = false;
            } else if (self.pending_valid) {
                self.pending_valid = false;
            }
            try auv.yieldUntilNextFrame(out);
            try paceHardwareFrame(self);
            _ = try readSerialReplies(self);
        }

        fn applyActuators(self: *Self, cmd: *const ActuatorCommand) void {
            self.pending = cmd.*;
            self.pending_valid = true;
        }
    };
}

fn frameElapsed(previous: *?u64, current: u64) ?f64 {
    const last = previous.*;
    previous.* = current;
    if (last) |timestamp| {
        if (current > timestamp) return @as(f64, @floatFromInt(current - timestamp)) * 1e-9;
    }
    return null;
}

fn processPerception(sensor: *const SensorFrame, out: *Observations) void {
    out.* = sensor.*;
}
