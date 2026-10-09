# Proteus photo model

Created 2026-10-09 from the six supplied photographs. The active scene in
`data/proteus_photo.blend` contains 216 appearance components and six rotor
objects. It includes blue split rails and clamps, upper and lower acrylic
pressure tubes, red endplates, O-rings, bulkhead connectors, open thruster ducts,
motor spinners and stators, cables, electronics trays, boards, heatsinks,
an illustrative battery, the lower optical housing, and a slotted metal cradle.
The pool is preserved as a separate scene in this file. The robot scene is
for editing the asset; continue using `pool_scene.blend` as the simulator environment.

## Files and integration

- `data/proteus_photo.blend`: editable Blender model, in metres.
- `data/models/proteus_photo/specification.json`: component inventory, dimensions,
  frame conventions, six physical body proxies, six actuator definitions, sensor
  specifications, controller mapping, and the source of each assumption.
- `data/models/proteus_photo/*.obj`: twelve appearance groups, a rotating
  three-blade propeller, and the internal appearance proxy.
- `create_proteus_model.py`: reproducible Blender asset generator.
- `proteus_appearance.inc`: generated look and mesh registration used by `robot.cpp`.

`platform.json` already selects `proteus`. The native C++ builder loads the new
OBJ groups on the Proteus compound body. The older `scenarios/proteus_auv.scn`
is a legacy reference and is not used by this native loader; it does not contain
the new appearance. Other vehicle definitions are unchanged.

The body convention is +Y forward, +Z down. Blender positions are `(-x, y, -z)`.
The exports are already in the Stonefish body frame and use scale 1.

## Accuracy and physical behavior

This is a photo-based appearance reconstruction, not measured CAD. Scale comes
from the existing model: upper cabin radius 0.10 m and length 0.30 m, lower cabin
radius 0.05 m and length 0.31 m, and rails 0.45 m long. Frame details, connectors,
duct geometry, cradle and electronic layout are estimates. Internal component
identities, the battery and wire topology are illustrative.

The original two cylinders, two rail boxes and two support meshes continue to
supply collision, mass, inertia, hydrodynamic approximation and buoyancy.
The twelve appearance groups are internal, zero-mass and non-buoyant, so the
new details do not introduce an invented electronics mass budget or double-count
the displaced volume. Their proxy cubes are not part of the compound collision
shape. The default graphical view displays the new appearance; the legacy
external proxy shapes continue to participate in physics.

The four vertical actuator positions and all thrust dynamics are retained. The
two horizontal actuator mounts are photo estimates at body `(±0.205, 0, 0.13)` m,
placing the side pods below the blue rails. Their new lever arms affect pitch
and yaw torques; the controller gains remain unchanged and need later tuning.
Photo-estimated duct
outer diameter is 0.108 m and rotor visual diameter is about 0.09 m. The legacy
fluid-dynamics diameter is still 0.18 m, with 1000 RPM maximum, 0.2 s first-order
response, forward/reverse thrust coefficient 0.48 and torque coefficient 0.05.
The difference between the appearance diameter and the effective dynamics
diameter requires calibration when the real thruster model is supplied.

The existing camera remains at body `(0, 0.30, 0)` with RPY `(1.5708, 0, 3.14)`,
800 × 600 pixels, 60° horizontal FOV, and 21 Hz. Odometry remains 30 Hz; IMU
remains 200 Hz with the original noise/range parameters. The photographed lower
optical housing is visual geometry and does not relocate the simulated camera.

Standard Stonefish looks have no acrylic transmission setting, so the pressure
tubes are opaque pale acrylic in Stonefish. Blender uses a transparent material
to show the internal assembly. Sponsor stickers were not reconstructed.

The visual envelope is approximately 0.519 × 0.638 × 0.437 m including cables.
The legacy collision envelope does not include all the new appearance details.
The hardware controller file currently lists eight thrusters; the selected
simulation and reconstructed visible layout use six. This count needs checking
against the real wiring before changing the controller.

## Validation

The native plugin builds successfully. The Proteus smoke check verifies six
actuators, six external physics proxies, twelve zero-mass appearance groups,
unchanged aggregate mass/displacement/CG, finite bounds, physics stepping and
scenario restart. All twelve exported meshes match their inventory triangle counts.
The AUV API plugin check also passes: 36 frames, two tracked objects and an
800 × 600 camera image. The shutdown path now stops physics and releases robot
and pool graphics before destroying the OpenGL context, fixing the crash found
during this check.

The current engine reports mass **5.32290967021 kg**, displaced-volume parameter
**0.00483779523087 m³**, body CG approximately `(0, 0, -0.00972231220335)` m,
and principal inertia `(0.655972178224, 0.700133399801, 1.17491947485)` kg·m².
These are calculated legacy simulation properties, not measured robot values.
The shell-cylinder volume convention is inherited; do not treat it as verified
sealed-hull displacement.

```sh
cmake -S . -B build -DAUTONOMY_BLEND_CPP=ON
cmake --build build --target auv_stonefish_sim stonefish_blend_smoke -j4
build/stonefish_blend_smoke auvs/stonefish_sim/data auvs/stonefish_sim/stonefish_config.yaml proteus
```

To regenerate, open the existing pool file in Blender and run
`create_proteus_model.py` using Blender's Python, or use:

```sh
blender auvs/stonefish_sim/data/pool_scene.blend --background \
  --python auvs/stonefish_sim/create_proteus_model.py
```

This generates the separate model file, OBJ assets, JSON specification and C++
include. Rebuild and restart the simulator after changing robot assets; static
pool hot reload does not rebuild the robot. The authoritative construction
parameters currently live in the generator and `robot.cpp`; the JSON is an
exported specification, not a runtime configuration interface.

Next calibration inputs: overall dimensions, dry mass, CG, sealed volume,
thruster identity/thrust curves/mount poses, battery mass, camera and IMU poses.
