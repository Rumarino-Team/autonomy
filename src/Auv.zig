const std = @import("std");
const math = @import("MathTypes.zig");

pub const c = @import("auv_c");

pub const Object = extern struct {
    bbox: math.BoundingBox,
    id: u32,
    cls: u8,
};

pub const Point2u = extern struct {
    x: u32,
    y: u32,
};

pub const Object2d = extern struct {
    top_left: Point2u,
    bottom_right: Point2u,
    id: u32,
    cls: u8,
};

/// One flag per axis. False means that component of the pose must not be used.
pub const AxisOk = extern struct {
    x: bool,
    y: bool,
    z: bool,
};

pub const Frame = extern struct {
    pub const max_objects = c.AUV_FRAME_MAX_OBJECTS;

    objects: [max_objects]Object,
    objects_len: u8,
    camera_pose: math.Pose,
    timestamp: u64,
    /// World X, Y, Z. A false axis means `camera_pose.pos` on that axis is not usable.
    position_ok: AxisOk,
    /// Roll, pitch, yaw. A false axis means that attitude component is not usable.
    attitude_ok: AxisOk,
    accel: math.Vector3Storage,
    gyro: math.Vector3Storage,
    imu_quat: math.QuaternionStorage,
    objects2d: [max_objects]Object2d,
    objects2d_len: u8,
    image_width: u32,
    image_height: u32,
    pressure_depth: f32,
    pressure_depth_ok: bool,
};

pub const InitFunc = fn () callconv(.c) void;
pub const YieldUntilNextFrameFunc = fn (frame: *Frame) callconv(.c) void;
pub const SetThrustorValuesFunc = fn (thrustor_values: [*c]const f32, thrustor_values_len: u8) callconv(.c) void;
pub const DeinitFunc = fn () callconv(.c) void;

pub const SimulationStatusFunc = fn (status: *c.AuvSimulationStatus) callconv(.c) bool;

const Auv = @This();

init: *const InitFunc,
yieldUntilNextFrame: *const YieldUntilNextFrameFunc,
setThrustorValues: *const SetThrustorValuesFunc,
deinit: *const DeinitFunc,
simulationStatus: ?*const SimulationStatusFunc = null,

// below I'm double checking the ABI matches
fn assertSameLayout(comptime Zig: type, comptime C: type) void {
    if (@sizeOf(Zig) != @sizeOf(C))
        @compileError("size mismatch");

    if (@alignOf(Zig) != @alignOf(C))
        @compileError("alignment mismatch");

    const zig_info = @typeInfo(Zig);
    const c_info = @typeInfo(C);

    if (zig_info == .vector) {
        if (c_info != .@"struct")
            @compileError("expected C struct for Zig vector");

        if (structFieldCount(c_info.@"struct") != 1)
            @compileError("expected C struct with one field");

        const CField = structFieldType(c_info.@"struct", 0);
        const c_field_info = @typeInfo(CField);

        if (c_field_info != .array)
            @compileError("expected C vector field to be an array");

        if (c_field_info.array.child != zig_info.vector.child)
            @compileError("vector element type mismatch");

        return;
    }

    if (zig_info != .@"struct" or c_info != .@"struct")
        return;

    if (structFieldCount(zig_info.@"struct") != structFieldCount(c_info.@"struct"))
        @compileError("field count mismatch");

    inline for (0..structFieldCount(zig_info.@"struct")) |i| {
        const name = structFieldName(zig_info.@"struct", i);
        const ZigField = structFieldType(zig_info.@"struct", i);
        const CField = @FieldType(C, name);

        if (@offsetOf(Zig, name) != @offsetOf(C, name))
            @compileError("field offset mismatch");

        if (@sizeOf(ZigField) != @sizeOf(CField))
            @compileError("field size mismatch");

        assertSameLayout(ZigField, CField);
    }
}

fn structFieldCount(comptime info: anytype) usize {
    if (@hasField(@TypeOf(info), "fields")) return info.fields.len;
    return info.field_names.len;
}

fn structFieldName(comptime info: anytype, comptime index: usize) [:0]const u8 {
    if (@hasField(@TypeOf(info), "fields")) return info.fields[index].name;
    return info.field_names[index];
}

fn structFieldType(comptime info: anytype, comptime index: usize) type {
    if (@hasField(@TypeOf(info), "fields")) return info.fields[index].type;
    return info.field_types[index];
}

fn assertSameFunctionLayout(comptime Zig: type, comptime C: type) void {
    const zig_info = @typeInfo(Zig).@"fn";
    const c_info = @typeInfo(C).@"fn";
    const zig_params = if (@hasField(@TypeOf(zig_info), "params")) zig_info.params.len else zig_info.param_types.len;
    const c_params = if (@hasField(@TypeOf(c_info), "params")) c_info.params.len else c_info.param_types.len;

    if (zig_params != c_params)
        @compileError("parameter count mismatch");

    inline for (0..zig_params) |i| {
        const ZigParam = if (@hasField(@TypeOf(zig_info), "params"))
            zig_info.params[i].type orelse @compileError("missing Zig parameter type")
        else
            zig_info.param_types[i] orelse @compileError("missing Zig parameter type");
        const CParam = if (@hasField(@TypeOf(c_info), "params"))
            c_info.params[i].type orelse @compileError("missing C parameter type")
        else
            c_info.param_types[i] orelse @compileError("missing C parameter type");
        assertSameLayout(ZigParam, CParam);
    }

    assertSameLayout(
        zig_info.return_type orelse @compileError("missing Zig return type"),
        c_info.return_type orelse @compileError("missing C return type"),
    );
}

comptime {
    if (c.AUV_COCO_TOOTHBRUSH != 79)
        @compileError("coco class ids must stay in dataset order");
    if (c.AUV_COCO_COUNT != 80)
        @compileError("coco class count must be 80");
    if (c.AUV_ROBOSUB_GATE != 0)
        @compileError("robosub gate must be 0");

    assertSameLayout(math.Vector3f, c.MathVector3f);
    assertSameLayout(math.Quaternionf, c.MathQuaternionf);
    assertSameLayout(math.Vector6f, c.MathVector6f);
    assertSameLayout(math.Pose, c.MathPose);
    assertSameLayout(math.BoundingBox, c.MathBoundingBox);
    assertSameLayout(Object, c.AuvObject);
    assertSameLayout(Point2u, c.AuvPoint2u);
    assertSameLayout(Object2d, c.AuvObject2d);
    assertSameLayout(AxisOk, c.AuvAxisOk);
    assertSameLayout(Frame, c.AuvFrame);

    assertSameFunctionLayout(InitFunc, c.AuvInitFunc);
    assertSameFunctionLayout(YieldUntilNextFrameFunc, c.AuvYieldUntilNextFrameFunc);
    assertSameFunctionLayout(SetThrustorValuesFunc, c.AuvSetThrustorsInputFunc);
    assertSameFunctionLayout(DeinitFunc, c.AuvDeinitFunc);
}
