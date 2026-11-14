const std = @import("std");
const c = @import("sim_c");
const S = @import("support.zig");

pub const Instance = struct {
    api: c.ForceModule = std.mem.zeroes(c.ForceModule),
    state: ?*anyopaque = null,
    created: u32 = 0,
};

pub const Runtime = struct {
    model: *const c.Simulation,
    backend: c.PhysicsBackendApi,
    world: ?*c.PhysicsWorld = null,
    bodies: S.Buffer(c.SimBodyState) = .{},
    contacts: S.Buffer(c.SimContact) = .{},
    wrenches: S.Buffer(c.SimWrench) = .{},
    fluid: S.Buffer(c.SimWrench) = .{},
    previous_linear: S.Buffer(c.MathVector3f) = .{},
    previous_angular: S.Buffer(c.MathVector3f) = .{},
    samples: S.Buffer(c.SensorSample) = .{},
    detections: S.Buffer(c.ObjectObservation) = .{},
    observation: c.Observation = std.mem.zeroes(c.Observation),
    modules: S.Buffer(Instance) = .{},
    scratch_memory: S.Buffer(u8) = .{},
    scratch: c.ArenaAllocator = std.mem.zeroes(c.ArenaAllocator),
    rotors: [8]c.RotorState = std.mem.zeroes([8]c.RotorState),
    commands: [8]f64 = @splat(0),
    input: [8]f64 = @splat(0),
    tick: u64 = 0,
};
