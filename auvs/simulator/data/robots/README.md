# Fixed vehicle assets

These four saved Blender files accompany the compiled C ABI v5 templates in
`../../robots.cpp`. They are copied unchanged from the previous simulator.
The manifest records their SHA-256
digests and the previous double-precision configuration digests as
`source_configuration_sha256`. Those source digests are provenance, not hashes
of the new float representation.

Hydrus retains the existing calibrated model, including five collision parts,
five separate fluid surfaces, and eight motors. Proteus and BlueROV retain the
earlier simulator presets and fixed convex envelopes of the saved visual bodies;
they are not new measured calibrations. Girona500 retains mass, augmented
inertia, COM, and displacement from the pinned Stonefish vehicle, its fixed
convex envelope, and the earlier portable rotor inertia assumption.

The migration used the existing native C++ loader to resolve the exact saved
collision meshes, local collider poses, triangle winding, fluid triangle soups,
motor order, and camera calibration. Scalars stay double; positions, vectors,
quaternions, and geometry are rounded to float. All four standalone scenes have
one body, ID 1, at NED depth 2 m with identity rotation. The construction API may
override body ID and initial pose without changing local calibration.

The compiled presets run independently of these files. A future saved-file
loader can associate an asset with its preset by the stored digest; that loader
is not implemented here. Editing a Blender file alone does not update the
compiled collision geometry or calibration. Changed geometry requires reviewing
calibration, rebuilding the corresponding geometry/definition, updating asset
provenance, and deliberately updating the source-reference test expectations.

Tests verify all asset digests and fingerprint every source physical field,
float geometry component, triangle index, motor, hull, and camera against the
pre-port native loader output. Both engines also run and cold-reset each preset.
