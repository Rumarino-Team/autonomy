//! Versioned JSON wire types. Schemas and defaults are derived from the same
//! Zig types used for dispatch, including every Motion and Navigation option.
const std = @import("std");
const math = @import("math.zig");
const Motion = @import("Motion.zig");
const Navigation = @import("Navigation.zig");
const Auv = @import("Auv.zig");
const Allocator = std.mem.Allocator;
pub const version = 1;
pub const max_datagram = 60000;
pub const max_plan_steps = 32;
pub const Empty = struct {};
pub const Selector = struct { id: ?u32 = null, class: ?Auv.ObjectCls = null };
pub const Step = struct { command: []const u8, args: std.json.Value };
pub const Frame = enum { world, body };
pub const Axis = enum { roll, pitch, yaw };
pub const Command = union(enum) {
    move_to: struct { position: math.Vector3f, move: Motion.MoveOptions = .{ .facing = .hold_current } },
    move_relative: struct { offset: math.Vector3f, frame: Frame = .body, move: Motion.MoveOptions = .{ .facing = .hold_current } },
    follow_path: struct { waypoints: []const math.Vector3f, options: Motion.PathOptions = .{ .move = .{ .facing = .face_path } } },
    move_to_object: struct { object: Selector, standoff: f32 = 2, depth: ?f32 = null, move: Motion.MoveOptions = .{ .facing = .face_goal } },
    follow_object: struct { object: Selector, standoff: f32 = 2, depth: ?f32 = null, duration: ?f32 = null, move: Motion.MoveOptions = .{ .facing = .face_goal } },
    orbit_object: struct { object: Selector, clearance: f32 = 2, options: Motion.PathOptions = .{ .move = .{ .facing = .face_goal, .attitude_policy = .allow_pitch } } },
    pass_gate: struct { object: Selector = .{ .class = .gate }, approach_distance: f32 = 2, exit_distance: f32 = 2, options: Motion.PathOptions = .{ .move = .{ .facing = .face_path, .translation = .align_first }, .corner_radius = 0 } },
    look_at_object: struct { object: Selector, move: Motion.MoveOptions = .{ .facing = .hold_current } },
    look_at_point: struct { point: math.Vector3f, move: Motion.MoveOptions = .{ .facing = .hold_current } },
    set_yaw: struct { yaw: f32, move: Motion.MoveOptions = .{ .facing = .hold_current } },
    set_attitude: struct { orientation: math.Quaternionf, move: Motion.MoveOptions = .{ .facing = .hold_current } },
    rotate: struct { axis: Axis, angle: f32, rate: f32 = 0.3, move: Motion.MoveOptions = .{ .facing = .hold_current } },
    search: Motion.SearchOptions,
    visual_target: struct { object: ?Selector = null, options: Motion.VisualOptions, move: Motion.MoveOptions = .{ .facing = .hold_current } },
    body_move: struct { options: Motion.BodyOptions, move: Motion.MoveOptions = .{ .facing = .hold_current } },
    hold: Empty,
    stop: Empty,
    cancel: Empty,
    pause: Empty,
    @"resume": Empty,
    status: Empty,
    objects: Empty,
    configure_motion: struct { options: std.json.Value },
    configure_navigation: struct { options: std.json.Value },
    reanchor: struct { transform: math.Pose, epoch: u32 },
    to_mission: struct { position: math.Vector3f },
    plan: struct { steps: []const Step },
};
pub const Packet = struct {
    version: u16,
    kind: enum { hello, request, heartbeat, bye },
    id: u64,
    session: ?[]const u8 = null,
    nonce: ?[]const u8 = null,
    command: ?std.json.Value = null,
};

pub fn description(name: []const u8) []const u8 {
    const tag = std.meta.stringToEnum(std.meta.Tag(Command), name) orelse return "Unknown command";
    return switch (tag) {
        .move_to => "Move to an absolute position in mission coordinates, metres; z positive down.",
        .move_relative => "Move a measured distance in body or mission coordinates. Body +Y forward, +X sideways, +Z down.",
        .follow_path => "Follow waypoints with configurable corner rounding, reference speed and recovery.",
        .move_to_object => "Approach a currently observed object to a horizontal standoff; null depth preserves current depth.",
        .follow_object => "Continuously follow the selected object at a horizontal standoff until cancelled or timed out.",
        .orbit_object => "Go around the object's bounding box with clearance and rounded corners, then return.",
        .pass_gate => "Cross a gate using approach and exit distances and a constant crossing heading.",
        .look_at_object => "Hold position and face a selected object.",
        .look_at_point => "Hold position and face a mission-frame point.",
        .set_yaw => "Hold position and set Euler yaw in radians: 0 faces +Y, -pi/2 faces +X.",
        .set_attitude => "Hold position and request a full body-to-world quaternion [x,y,z,w].",
        .rotate => "Perform a signed relative turn, including full revolutions. Angle and rate in radians and rad/s; body X roll, Y pitch, Z yaw. Position holding is best effort.",
        .search => "Look around or search for an object using yaw sweeps, optional body effort, pressure depth and detection settings.",
        .visual_target => "Use image centering and optional apparent target height for non-odometry object following.",
        .body_move => "Apply normalized body force effort for a duration; not a measured distance or speed.",
        .hold => "Replace the current plan with a hold at the measured pose.",
        .stop => "Cancel the plan and latch zero thruster output until a new motion command.",
        .cancel => "Cancel the current plan and hold the measured pose.",
        .pause => "Pause the retained plan and latch zero output; resume continues its saved progress.",
        .@"resume" => "Resume a paused or failed retained plan with a renewed recovery budget.",
        .status => "Read navigation, command status, route progress, settings and allocation feedback.",
        .objects => "List currently observed object IDs, classes, mission positions and image boxes.",
        .configure_motion => "Patch active MoveOptions; omitted values are preserved. Facing is a single-key tagged object.",
        .configure_navigation => "Patch all Navigation.Options at runtime; recapture a hold after applying calibration. Does not edit configuration files.",
        .reanchor => "Reconcile a localization epoch with a tracking-to-mission transform, then explicitly resume the retained plan.",
        .to_mission => "Convert a tracking-frame position to mission coordinates.",
        .plan => "Execute up to 32 movement steps in order. Continuous following must have a timeout to allow a later step.",
    };
}

pub fn jsonValue(allocator: Allocator, value: anytype) !std.json.Value {
    const bytes = try std.json.Stringify.valueAlloc(allocator, value, .{});
    return std.json.parseFromSliceLeaky(std.json.Value, allocator, bytes, .{});
}

/// Merge struct settings, replacing arrays and tagged unions as entire values.
pub fn patch(comptime T: type, allocator: Allocator, original: T, changes: std.json.Value) !T {
    var value = try jsonValue(allocator, original);
    try mergeStruct(T, allocator, &value, changes);
    return std.json.parseFromSliceLeaky(T, allocator, try std.json.Stringify.valueAlloc(allocator, value, .{}), .{});
}

fn mergeStruct(comptime T: type, allocator: Allocator, original: *std.json.Value, changes: std.json.Value) !void {
    if (changes != .object or original.* != .object) return error.ExpectedObject;
    var iterator = changes.object.iterator();
    while (iterator.next()) |entry| {
        inline for (@typeInfo(T).@"struct".fields) |field| {
            if (std.mem.eql(u8, field.name, entry.key_ptr.*)) {
                if (@typeInfo(field.type) == .@"struct" and field.type != std.json.Value) {
                    var child = original.object.get(field.name).?;
                    try mergeStruct(field.type, allocator, &child, entry.value_ptr.*);
                    try original.object.put(allocator, field.name, child);
                } else try original.object.put(allocator, field.name, entry.value_ptr.*);
                break;
            }
        } else return error.UnknownField;
    }
}

pub fn parseCommand(allocator: Allocator, source: std.json.Value) !Command {
    // Copy before expansion: raw patches must retain which fields were omitted.
    var copy = try jsonValue(allocator, source);
    try defaults(Command, allocator, &copy);
    return std.json.parseFromSliceLeaky(Command, allocator, try std.json.Stringify.valueAlloc(allocator, copy, .{}), .{});
}

fn defaults(comptime T: type, allocator: Allocator, value: *std.json.Value) anyerror!void {
    if (T == std.json.Value) return;
    switch (@typeInfo(T)) {
        .@"struct" => |info| {
            if (value.* != .object) return;
            inline for (info.fields) |field| {
                if (value.object.get(field.name)) |provided| {
                    var child = provided;
                    if (@typeInfo(field.type) == .@"struct" and field.type != std.json.Value) {
                        if (field.defaultValue()) |fallback| {
                            var base = try jsonValue(allocator, fallback);
                            try mergeStruct(field.type, allocator, &base, child);
                            child = base;
                        }
                    }
                    try defaults(field.type, allocator, &child);
                    try value.object.put(allocator, field.name, child);
                } else if (field.defaultValue()) |fallback| {
                    try value.object.put(allocator, field.name, try jsonValue(allocator, fallback));
                }
            }
        },
        .@"union" => |info| {
            if (value.* != .object or value.object.count() != 1) return;
            var iterator = value.object.iterator();
            const entry = iterator.next().?;
            inline for (info.fields) |field| {
                if (std.mem.eql(u8, field.name, entry.key_ptr.*)) {
                    try defaults(field.type, allocator, entry.value_ptr);
                    return;
                }
            }
        },
        .optional => |info| if (value.* != .null) try defaults(info.child, allocator, value),
        else => {},
    }
}

pub fn catalog(allocator: Allocator) !std.json.Value {
    var output: std.Io.Writer.Allocating = .init(allocator);
    var json: std.json.Stringify = .{ .writer = &output.writer };
    try json.beginObject();
    try json.objectField("commands");
    try json.beginObject();
    inline for (@typeInfo(Command).@"union".fields) |field| {
        try json.objectField(field.name);
        try json.beginObject();
        try json.objectField("description");
        try json.write(description(field.name));
        try json.objectField("schema");
        if (std.mem.eql(u8, field.name, "configure_motion")) {
            try patchSchema(Motion.MoveOptions, allocator, &json);
        } else if (std.mem.eql(u8, field.name, "configure_navigation")) {
            try patchSchema(Navigation.Options, allocator, &json);
        } else try schema(field.type, &json);
        try json.endObject();
    }
    try json.endObject();
    try json.objectField("definitions");
    try json.beginObject();
    inline for (.{ Motion.MoveOptions, Motion.PathOptions, Motion.Facing, Motion.RecoveryOptions, Motion.SearchOptions, Motion.VisualOptions, Motion.BodyOptions, Navigation.Options }) |T| {
        try json.objectField(sharedName(T).?);
        try schemaBody(T, &json);
    }
    try json.endObject();
    try json.endObject();
    return std.json.parseFromSliceLeaky(std.json.Value, allocator, output.written(), .{});
}

fn patchSchema(comptime T: type, allocator: Allocator, json: *std.json.Stringify) !void {
    try json.beginObject();
    try json.objectField("type");
    try json.write("object");
    try json.objectField("additionalProperties");
    try json.write(false);
    try json.objectField("required");
    try json.write([_][]const u8{"options"});
    try json.objectField("properties");
    try json.beginObject();
    try json.objectField("options");
    var output: std.Io.Writer.Allocating = .init(allocator);
    var inner: std.json.Stringify = .{ .writer = &output.writer };
    try schemaBody(T, &inner);
    var options = try std.json.parseFromSliceLeaky(std.json.Value, allocator, output.written(), .{});
    try options.object.put(allocator, "required", .{ .array = .init(allocator) });
    try json.write(options);
    try json.endObject();
    try json.endObject();
}

fn sharedName(comptime T: type) ?[]const u8 {
    if (T == Motion.MoveOptions) return "MoveOptions";
    if (T == Motion.PathOptions) return "PathOptions";
    if (T == Motion.Facing) return "Facing";
    if (T == Motion.RecoveryOptions) return "RecoveryOptions";
    if (T == Motion.SearchOptions) return "SearchOptions";
    if (T == Motion.VisualOptions) return "VisualOptions";
    if (T == Motion.BodyOptions) return "BodyOptions";
    if (T == Navigation.Options) return "NavigationOptions";
    return null;
}

fn schema(comptime T: type, json: *std.json.Stringify) anyerror!void {
    if (comptime sharedName(T)) |name| {
        try json.beginObject();
        try json.objectField("$ref");
        try json.write("#/definitions/" ++ name);
        try json.endObject();
    } else try schemaBody(T, json);
}

fn schemaBody(comptime T: type, json: *std.json.Stringify) anyerror!void {
    if (T == std.json.Value) {
        try json.beginObject();
        try json.endObject();
        return;
    }
    try json.beginObject();
    switch (@typeInfo(T)) {
        .void => {
            try json.objectField("type");
            try json.write("object");
            try json.objectField("properties");
            try json.beginObject();
            try json.endObject();
            try json.objectField("additionalProperties");
            try json.write(false);
        },
        .float, .comptime_float => {
            try json.objectField("type");
            try json.write("number");
        },
        .int => |info| {
            try json.objectField("type");
            try json.write("integer");
            if (info.signedness == .unsigned) {
                try json.objectField("minimum");
                try json.write(0);
            }
        },
        .bool => {
            try json.objectField("type");
            try json.write("boolean");
        },
        .@"enum" => |info| {
            try json.objectField("type");
            try json.write("string");
            try json.objectField("enum");
            try json.beginArray();
            inline for (info.fields) |field| try json.write(field.name);
            try json.endArray();
        },
        .optional => |info| {
            try json.objectField("anyOf");
            try json.beginArray();
            try schema(info.child, json);
            try json.beginObject();
            try json.objectField("type");
            try json.write("null");
            try json.endObject();
            try json.endArray();
        },
        .vector, .array => {
            const info = if (@typeInfo(T) == .vector) @typeInfo(T).vector else @typeInfo(T).array;
            try json.objectField("type");
            try json.write("array");
            try json.objectField("minItems");
            try json.write(info.len);
            try json.objectField("maxItems");
            try json.write(info.len);
            try json.objectField("items");
            try schema(info.child, json);
        },
        .pointer => |info| {
            if (info.size != .slice) @compileError("Unsupported command pointer");
            if (info.child == u8) {
                try json.objectField("type");
                try json.write("string");
            } else {
                try json.objectField("type");
                try json.write("array");
                try json.objectField("items");
                try schema(info.child, json);
            }
        },
        .@"struct" => |info| {
            try json.objectField("type");
            try json.write("object");
            try json.objectField("additionalProperties");
            try json.write(false);
            try json.objectField("properties");
            try json.beginObject();
            inline for (info.fields) |field| {
                try json.objectField(field.name);
                try schema(field.type, json);
            }
            try json.endObject();
            try json.objectField("required");
            try json.beginArray();
            inline for (info.fields) |field| if (field.default_value_ptr == null) {
                try json.write(field.name);
            };
            try json.endArray();
            try json.objectField("defaults");
            try json.beginObject();
            inline for (info.fields) |field| if (field.defaultValue()) |default| {
                try json.objectField(field.name);
                try json.write(default);
            };
            try json.endObject();
        },
        .@"union" => |info| {
            try json.objectField("anyOf");
            try json.beginArray();
            inline for (info.fields) |field| {
                try json.beginObject();
                try json.objectField("type");
                try json.write("object");
                try json.objectField("additionalProperties");
                try json.write(false);
                try json.objectField("properties");
                try json.beginObject();
                try json.objectField(field.name);
                try schema(field.type, json);
                try json.endObject();
                try json.objectField("required");
                try json.write([_][]const u8{field.name});
                try json.endObject();
            }
            try json.endArray();
        },
        else => @compileError("Unsupported command schema type: " ++ @typeName(T)),
    }
    try json.endObject();
}
