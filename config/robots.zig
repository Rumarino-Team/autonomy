// Calibrated saved-asset provenance; hashes are unchanged.
pub const RobotMetadata = struct {
    name: []const u8,
    asset: []const u8,
    description: []const u8,
    asset_sha256: []const u8,
    source_configuration_sha256: []const u8,
    colliders: u32,
    thrusters: u32,
    hulls: u32,
    vertices: u32,
    triangles: u32,
    hull_vertices: u32,
};
pub const Manifest = struct { version: u32, representation: []const u8, robots: []const RobotMetadata };
pub const manifest: Manifest = .{ .version = 1, .representation = "C ABI v5; float geometry, poses and vectors; double scalar calibration", .robots = &.{ .{ .name = "hydrus", .asset = "robots/hydrus.blend", .description = "Existing calibrated Hydrus model; fixed standalone asset.", .asset_sha256 = "0285fef55d4d784c142677145381f5c24a0653f18761b0c54b1e9036e0aac422", .source_configuration_sha256 = "1e7a36ee8dbf5aef9568bfad78ce527e12fbc4381195c23f373648dc358208c7", .colliders = 5, .thrusters = 8, .hulls = 5, .vertices = 284, .triangles = 348, .hull_vertices = 11760 }, .{ .name = "proteus", .asset = "robots/proteus.blend", .description = "Physical parameters from the earlier simulator preset. Fixed convex collision/hydrodynamic envelope of the saved visual body; not a newly measured vehicle calibration.", .asset_sha256 = "dfd8a8fdc938c49125a24118cf9a987fb6ed36b545627fe7f7595f8ee69be87d", .source_configuration_sha256 = "1e402147361544eb1020b31d378b188abf28f7e4b7c06f7163e0297d47446058", .colliders = 1, .thrusters = 6, .hulls = 1, .vertices = 285, .triangles = 566, .hull_vertices = 1698 }, .{ .name = "bluerov", .asset = "robots/bluerov.blend", .description = "Physical parameters from the earlier simulator preset. Fixed convex collision/hydrodynamic envelope of the saved visual body; not a newly measured vehicle calibration.", .asset_sha256 = "f8875dd0c82dae5fc720c060546fe98bb573fcb538ab746dfdeb552481ef40da", .source_configuration_sha256 = "58fd1d6724832f784fa0316f52e3aefa6e2030d638a2c70c8d4b19ac86a9b73d", .colliders = 1, .thrusters = 8, .hulls = 1, .vertices = 422, .triangles = 838, .hull_vertices = 2514 }, .{ .name = "girona500", .asset = "robots/girona500.blend", .description = "Mass, augmented inertia, COM and displacement extracted from the pinned Stonefish Girona500 model. Fixed convex envelope; default portable rotor inertia, not a newly measured vehicle calibration.", .asset_sha256 = "a4140d55001e480cb439c8eab1e805cf3a18b7b7448a5e08c0a7e59b349e3a51", .source_configuration_sha256 = "8877f772ab57ecee2add5b11464531f62dad90fe3df10e3617c18c20d2b03617", .colliders = 1, .thrusters = 5, .hulls = 1, .vertices = 574, .triangles = 1144, .hull_vertices = 3432 } } };
