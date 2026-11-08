//! Shared ABI representations. Keep application math algorithms in math.zig.
pub const Vector2f = @Vector(2, f32);
pub const Vector3f = @Vector(3, f32);
pub const Vector6f = @Vector(6, f32);
pub const Quaternionf = @Vector(4, f32);

/// Zig 0.17 forbids `@Vector` fields in `extern` structs. These match `MathVector3f` and `MathQuaternionf`.
pub const Vector3Storage = extern struct {
    buf: [3]f32 align(16),
};

pub const QuaternionStorage = extern struct {
    buf: [4]f32 align(16),
};

pub const Pose = extern struct {
    pos: Vector3Storage,
    quat: QuaternionStorage,
};

pub const BoundingBox = extern struct {
    pose: Pose,
    size: Vector3Storage,
};
