const std = @import("std");
const Io = std.Io;

const Auv = @import("Auv.zig");
const MissionConfig = @import("MissionConfig.zig");
const MissionIO = @import("MissionIO.zig");
const Serial = @import("Serial.zig");
const reflection = @import("ReflectionWriter.zig");

const simulator_enabled = @import("build_options").simulator;
const SimSession = if (simulator_enabled) @import("simulator/Session.zig").Session else void;

const Visualizer = if (simulator_enabled) @import("Visualizer.zig") else void;
const VisualizerMeshCache = if (simulator_enabled) @import("simulator/VisualizerMeshCache.zig").VisualizerMeshCache else void;
const SceneMeshDraw = if (simulator_enabled) @import("simulator/VisualizerMeshCache.zig").DrawContext else void;
const drawSceneMeshes = if (simulator_enabled) @import("simulator/VisualizerMeshCache.zig").drawEmbedded else void;
const EmbeddedView = if (simulator_enabled) struct {
    viewer: Visualizer,
    scene_meshes: VisualizerMeshCache = undefined,
    has_scene_meshes: bool = false,
    mesh_draw: SceneMeshDraw = .{ .cache = undefined, .bodies = &.{} },
    last_idle_draw: ?Io.Clock.Timestamp = null,
    config_reply_revision: u64 = 0,
    config_rejected: bool = false,
} else void;

pub fn Vehicle(comptime Robot: type) type {
    return struct {
        const Wire = Robot.Wire;

        pub var robot: Robot = .{};
        pub var mission_active: bool = false;
        pub var pending: Robot.ActuatorCommand = .{};
        pub var pending_valid: bool = false;

        pub var io: Io = undefined;
        pub var frame: Auv.Frame = undefined;
        pub var config: MissionConfig.MissionConfig = undefined;
        pub var telemetry_doc: Wire.TelemetryDoc = Wire.emptyTelemetry();
        pub var snapshot_buffers: Wire.SnapshotBuffers = .{};
        pub var telemetry: *MissionIO.Telemetry = undefined;
        pub var visualizer: ?*EmbeddedView = null;
        pub var previous_frame_ns: ?u64 = null;

        pub var session: SimSession = undefined;
        pub var auv: if (simulator_enabled) void else ?Auv = if (simulator_enabled) {} else null;
        pub var loader: if (simulator_enabled) void else ?MissionIO.MissionDiynLoader = if (simulator_enabled) {} else null;
        pub var serial: if (simulator_enabled) void else ?Serial.Port = if (simulator_enabled) {} else null;
        pub var parser: if (simulator_enabled) void else Serial.Parser = if (simulator_enabled) {} else .{};
        pub var hardware_frame_time: if (simulator_enabled) void else ?Io.Clock.Timestamp = if (simulator_enabled) {} else null;

        fn frameElapsed(previous: *?u64, current: u64) ?f64 {
            const last = previous.*;
            previous.* = current;
            if (last) |timestamp| {
                if (current > timestamp) return @as(f64, @floatFromInt(current - timestamp)) * 1e-9;
            }
            return null;
        }

        fn applyActuators(cmd: *const Robot.ActuatorCommand) void {
            pending = cmd.*;
            pending_valid = true;
        }

        fn pollCommands() !void {
            try telemetry.pollControl(io, &config);
            while (telemetry.takeDnaPacket()) |bytes| {
                if (reflection.matchesDocument(Wire.TeleopDoc, bytes)) {
                    telemetry.storeTeleopPacket(bytes);
                } else if (reflection.matchesDocument(Wire.ThrusterDoc, bytes)) {
                    const decoded = reflection.read(Wire.ThrusterDoc, bytes) catch continue;
                    telemetry.acceptThrusters(io, &config, decoded.values[0..decoded.len]);
                }
            }
        }

        fn presentTelemetryFrame(frame_ptr: ?*const Wire.TelemetryDoc) !void {
            if (comptime !simulator_enabled) return;
            const embedded = visualizer orelse return;
            if (Visualizer.shouldClose()) return error.WindowClosed;

            if (frame_ptr == null) {
                if (embedded.last_idle_draw) |last| {
                    if (last.untilNow(io).raw.toNanoseconds() < 16_666_667) return;
                }
                embedded.last_idle_draw = .now(io, .awake);
            } else {
                embedded.last_idle_draw = null;
            }

            var scene_draw: ?Visualizer.SceneDraw = null;
            if (embedded.has_scene_meshes) {
                if (comptime simulator_enabled) {
                    embedded.mesh_draw.bodies = session.bodyStates();
                    scene_draw = .{ .ctx = &embedded.mesh_draw, .draw = drawSceneMeshes };
                }
            }
            const actions = try embedded.viewer.tick(.{
                .status = telemetry.status(&config, embedded.config_reply_revision, embedded.config_rejected),
                .frame = frame_ptr,
                .telemetry_status = if (frame_ptr != null) .live else if (telemetry.state == .starting) .connecting else .disabled,
                .scene_draw = scene_draw,
            });
            embedded.config_reply_revision = 0;
            embedded.config_rejected = false;
            if (actions.config) |packet| {
                embedded.config_reply_revision = packet.revision;
                telemetry.applyConfig(io, &config, packet) catch |err| {
                    telemetry.report(err);
                    embedded.config_rejected = true;
                };
            }
            if (actions.thrusters) |command| telemetry.acceptThrusters(io, &config, command.values[0..command.len]);
            if (actions.teleop) |command| {
                var scratch: [1024]u8 align(8) = undefined;
                const bytes = reflection.writeRowInto(&scratch, Wire.TeleopDoc{ .command = command }) catch null;
                if (bytes) |packet| telemetry.storeTeleopPacket(packet);
            }
            if (actions.mission) |command| telemetry.acceptMission(command) catch |err| telemetry.report(err);
        }

        fn serviceWithoutFrame() !void {
            try presentTelemetryFrame(null);
            try pollCommands();
        }

        fn waitForReadyOnBackend() !void {
            const max_retries = 1000;
            for (0..max_retries + 1) |attempt| {
                try serviceWithoutFrame();
                if (try readSerialReplies()) return;
                if (attempt < max_retries) try Io.sleep(io, .fromMilliseconds(10), .awake);
            }
            return error.SerialReadyTimeout;
        }

        fn publishFrameAfterTick(goals: *const std.Deque(MissionIO.Goal), step: []const u8) !void {
            telemetry_doc = snapshot_buffers.capture(
                &robot,
                &frame,
                &config,
                goals,
                step,
                pending.values[0..pending.len],
            );
            telemetry.publishTelemetry(io, &telemetry_doc);
            if (comptime simulator_enabled) {
                if (visualizer != null) {
                    try presentTelemetryFrame(&telemetry_doc);
                    try pollCommands();
                }
            }
        }

        pub fn runtimeLoop(goals: *const std.Deque(MissionIO.Goal), step: []const u8) !void {
            try pollCommands();
            try gatherSensors(&frame);
            const elapsed = frameElapsed(&previous_frame_ns, frame.timestamp);

            const observations = frame;
            robot.observe(&observations);

            var actuator_cmd: Robot.ActuatorCommand = .{};
            if (config.mission_id == .thruster_control) {
                actuator_cmd.len = telemetry.direct_len;
                @memcpy(actuator_cmd.values[0..actuator_cmd.len], telemetry.direct_values[0..actuator_cmd.len]);
            } else {
                robot.update(&observations, &config, elapsed, &actuator_cmd);
            }
            applyActuators(&actuator_cmd);
            try publishFrameAfterTick(goals, step);
        }

        pub fn initApp(allocator: std.mem.Allocator, io_in: Io, windowed: bool, startup: bool) !void {
            if (startup) {
                io = io_in;
                config = MissionConfig.startup_config;
                if (windowed) config.runtime.telemetry_transport = .disabled;

                var bootstrap = config;
                bootstrap.runtime.telemetry_transport = .disabled;
                bootstrap.runtime.telemetry_log_path = .{};
                const enable_udp_control = !windowed and config.runtime.mission_profile != .stonefish_sim;
                telemetry = try allocator.create(MissionIO.Telemetry);
                errdefer allocator.destroy(telemetry);
                telemetry.* = try MissionIO.Telemetry.init(io_in, bootstrap, enable_udp_control);
                errdefer telemetry.deinit(io_in);
                telemetry.reconfigure(io_in, config) catch |err| {
                    telemetry.report(err);
                    telemetry.telemetry_error = telemetry.last_error;
                };

                if (comptime simulator_enabled) {
                    if (windowed) {
                        const embedded = try allocator.create(EmbeddedView);
                        errdefer allocator.destroy(embedded);
                        embedded.* = .{
                            .viewer = Visualizer.init(&config, 1, config.mission_id == .mission_control, true) catch |err| {
                                std.log.err("could not open the simulator visualizer ({s}); use --headless to run without a display", .{@errorName(err)});
                                return err;
                            },
                        };
                        visualizer = embedded;
                    }
                }
            }

            try serviceWithoutFrame();
            try config.validate();
            const tam_len = config.tam_len;

            if (config.runtime.serial_port.len != 0) switch (config.runtime.mission_profile) {
                .proteus_hwd => {},
                .stonefish_sim, .mujoco_sim => return error.InvalidThrusterCount,
            };

            const realtime = visualizer != null or config.runtime.sim_realtime or config.runtime.serial_port.len != 0;
            if (comptime simulator_enabled) {
                session = undefined;
                try session.init(allocator, io_in, &config, tam_len, realtime);
            } else {
                var dyn_loader = MissionIO.MissionDiynLoader.init(config.runtime.sensor_library.slice() catch return error.InvalidPath);
                const loaded = try dyn_loader.load();
                loader = dyn_loader;
                auv = loaded;
                auv.?.init();
            }
            mission_active = true;
            errdefer deinitMission();
            pending = .{};
            pending_valid = false;
            previous_frame_ns = null;
            telemetry_doc = Wire.emptyTelemetry();
            snapshot_buffers = .{};

            if (comptime !simulator_enabled) {
                const stonefish_temporal_thrusters = if (auv) |loaded| loaded.stonefish_temporal_thrusters != null else false;
                if (config.runtime.serial_port.len == 0 and !stonefish_temporal_thrusters) return error.MissingOutput;
                try pollCommands();
                var initial_frame: Auv.Frame = undefined;
                try gatherSensors(&initial_frame);
                try pollCommands();
                if (config.runtime.serial_port.len != 0) {
                    const path = config.runtime.serial_port.sentinelSlice() catch return error.InvalidPath;
                    serial = try Serial.Port.init(io, path);
                    try waitForReadyOnBackend();
                    std.log.info("READY on {s} at 115200 baud", .{path});
                }
            }

            if (comptime simulator_enabled) {
                if (visualizer) |embedded| {
                    embedded.scene_meshes = try VisualizerMeshCache.init(allocator, &session.scene);
                    embedded.has_scene_meshes = true;
                    embedded.mesh_draw = .{ .cache = &embedded.scene_meshes, .bodies = &.{} };
                }
            }

            robot.reset();
            frame = std.mem.zeroes(Auv.Frame);
        }

        pub fn deinitApp(allocator: std.mem.Allocator) void {
            deinitMission();
            if (comptime simulator_enabled) {
                if (visualizer) |embedded| {
                    if (embedded.has_scene_meshes) embedded.scene_meshes.deinit();
                    embedded.viewer.deinit();
                    allocator.destroy(embedded);
                    visualizer = null;
                }
            }
            telemetry.deinit(io);
            allocator.destroy(telemetry);
        }

        pub fn deinitMission() void {
            if (!mission_active) return;
            if (comptime simulator_enabled) {
                if (visualizer) |embedded| {
                    if (embedded.has_scene_meshes) {
                        embedded.scene_meshes.deinit();
                        embedded.has_scene_meshes = false;
                    }
                }
                session.deinit();
            } else {
                if (serial) |*port| port.deinit();
                if (auv) |loaded| loaded.deinit();
                if (loader) |*l| l.dynlib.close();
                serial = null;
                auv = null;
                loader = null;
                parser = .{};
                hardware_frame_time = null;
            }
            mission_active = false;
        }

        fn readSerialReplies() !bool {
            if (comptime simulator_enabled) return false;
            if (serial) |*port| return try port.readReplies(&parser);
            return false;
        }

        fn paceHardwareFrame() !void {
            if (comptime simulator_enabled) return;
            if (serial == null) return;
            if (hardware_frame_time) |previous| {
                const remaining = 16_666_667 - previous.untilNow(io).raw.toNanoseconds();
                if (remaining > 0) try Io.sleep(io, .fromNanoseconds(remaining), .awake);
            }
            hardware_frame_time = .now(io, .awake);
        }

        fn gatherSensors(out: *Auv.Frame) !void {
            if (comptime simulator_enabled) {
                if (pending_valid and pending.len > 0) {
                    var buf: [MissionConfig.thruster_count]f32 = @splat(0);
                    @memcpy(buf[0..pending.len], pending.values[0..pending.len]);
                    try session.setThrusters(&buf);
                    pending_valid = false;
                } else if (pending_valid) {
                    pending_valid = false;
                }
                try session.nextFrame(out);
                return;
            }

            const auv_loaded = auv orelse return error.BackendNotInitialized;
            if (pending_valid and pending.len > 0) {
                const p = pending;
                auv_loaded.setStonefishTemporalThrusters(p.values[0..p.len]);
                if (serial) |*port| {
                    var channels: [Serial.protocol_v2_command_channel_count]f32 = @splat(0);
                    const n = @min(Serial.protocol_v2_command_channel_count, p.len);
                    @memcpy(channels[0..n], p.values[0..n]);
                    const packet = Serial.encode(&channels);
                    try port.writeAll(&packet, 100);
                }
                pending_valid = false;
            } else if (pending_valid) {
                pending_valid = false;
            }
            try auv_loaded.yieldUntilNextFrame(out);
            try paceHardwareFrame();
            _ = try readSerialReplies();
        }
    };
}

pub const App = Vehicle(@import("AuvRobot.zig"));
