# Fixed robot assets

These are self-contained one-robot simulation scenes, each with a camera and
physical thruster mounts. Open any file directly in Blender to inspect it.

- `hydrus.blend`: Hydrus, 8 thrusters; existing detailed Hydrus calibration.
- `proteus.blend`: Proteus, 6 thrusters; earlier simulator physical parameters.
- `bluerov.blend`: BlueROV2, 8 thrusters; earlier simulator physical parameters.
- `girona500.blend`: Girona 500, 5 thrusters; mechanical properties extracted
  from the pinned Stonefish reference model.

The files contain the robot alone, initially underwater at NED depth 2 m. They
can run independently of the pool scene. Each has one active camera, respecting
the current simulator camera limit. The original pool scene is preserved.

## Code and asset agreement

`sim/blend/robot_registry.cpp` lists the four supported robots. Their fixed
physical definitions are C++ string literals in `sim/blend/robots/*_profile.inc`.
The literals are compiled into the native reader; no external or embedded
calibration profile can override these four registered names, including Hydrus
in the original pool scene. The pool's world objects remain editable; the exact
whole-file checksum requirement applies to the four standalone robot assets.

Each scene stores `sim_robot` and `sim_robot_config_sha256`. The registry stores
the matching configuration hash and the SHA-256 of the complete saved file.
`manifest.json` repeats those checksums for inspection and packaging. The full
file hash is stored outside the file to avoid a self-referential checksum.

Native loading verifies the file bytes, scene configuration hash and compiled
configuration hash. A moved or renamed copy loads normally; resaving or editing
its bytes requires rebuilding the asset and registry together. `simctl robots`
lists the registered checksums, body parameters and thruster mounts.

## Physical fidelity

Hydrus retains its five calibrated hull parts and detailed fluid surfaces.
Proteus and BlueROV inherit the previous presets' mass, augmented inertia,
buoyancy and motor parameters. Girona's mass, augmented inertia, COM and volume
were extracted from `vendor/stonefish/Tests/Data/girona500auv_console.scn` using
the pinned Stonefish library; its motor gains/coefficients come from that same
reference. Girona uses the portable default rotor inertia.

Proteus, BlueROV and Girona each use a convex envelope of their saved visual
body for collisions and a single approximate hydrodynamic surface. These are
functional simulation presets, not newly measured real-vehicle calibrations.
The convex envelopes fill body openings. Controller gains for those vehicles
have not been qualified by the asset tests.

## Authoring and verification

From the repository root:

```sh
blender --background --factory-startup --disable-autoexec \
  --python tools/blender/build_robot_assets.py
cmake --build build/portable --target sim_all -j4
ctest --test-dir build/portable --output-on-failure -R 'asset'
build/portable/sim/simctl robots
build/portable/sim/simctl validate auvs/stonefish_sim/data/robots/proteus.blend
build/portable/sim/simctl run build/portable/sim/libsim_backend_mujoco.so \
  auvs/stonefish_sim/data/robots/proteus.blend 720
```

The authoring script extracts the four existing vehicle models from
`pool_scene.blend`, reads the fixed calibration literals, and writes standalone
files plus updated literals, registry and manifest. It never saves the input
pool file. Rebuilding is an intentional change to the registered configuration.
Review and distribute the code, files and manifest together. Normal launches
read the saved file directly in C++; the authoring script is not a launch step.

Both backends are tested for deterministic replay, cold reset, concurrent
worlds and individual thruster response. Separate tests verify calibration,
mounts, exact checksums, relocation and rejection of a resaved mismatched file.
