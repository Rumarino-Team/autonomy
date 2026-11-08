# Simulation authoring contract, version 1

`AutonomySource` in `pool_scene.blend` is the editable source. Its `sim_schema=1`
and `sim_collection="Simulation"` identify the native reader contract and collection.
Scene metre scale is required. Scene properties `sim_hz`, `sim_gravity`,
`sim_water_density` and `sim_hydro_period` define common world settings.

Every object inside that collection has an explicit `sim_role`. Objects outside
it, or explicitly marked `visual`, are presentation assets. Lights, water
rendering meshes, take curves and decorative robots do not silently become
physical obstacles. Unclassified objects inside the collection stop native loading.

## Objects and attachments

Each non-visual object has a unique, persistent string `sim_id`. Decimal body
IDs are preserved; other strings are mapped deterministically to uint32 IDs
with collision detection. Renaming an object does not change its ID. Optional
`sim_name` supplies a display label. The camera's presentation ID is reserved.

- `body`: rigid body origin; `sim_motion` is `static`, `dynamic` or `kinematic`.
  Mesh bodies supply their own geometry. Empty bodies own child collider objects.
  `sim_tracked`, numeric `sim_class` (0–255), and `sim_fuzz` are independent of
  collisions. Source bodies use static initial transforms, never take actions.
- `collider`: evaluated mesh with `sim_body` naming the owning body's string ID.
  Ownership is explicit; parenting controls the evaluated transform, not identity.
- `thruster`: mount empty with `sim_body` and a contiguous zero-based `sim_index`.
  Its position and local +X shaft direction determine the applied-force mount.
  The vehicle profile supplies the corresponding motor/propeller coefficients.
- `sensor`: a camera or mount empty with a camera child, `sim_body`, `sim_width`
  and `sim_height`. Both mount and camera-child edits affect the loaded optical
  pose; the camera datablock supplies horizontal FOV. One simulation camera is
  currently supported; other cameras are presentation-only.
- `instance`: a collection instancer with a stable ID. Standalone static mesh
  body prototypes are loaded as independent bodies with deterministic derived
  IDs. Dynamic/compound instances and unrealized modifier instances are rejected;
  realize those instances before running rather than silently losing geometry.
- `visual`: explicit exclusion from physical loading.

Use **Add simulation obstacle** for a correctly tagged collidable box. Ordinary
move, rotate, scale and delete operations update the next run.
Advanced objects can use Blender's Custom Properties to set the fields above.

## Geometry and collisions

`sim_shape` is `mesh`, `convex`, `box`, `sphere` or `cylinder`.
`sim_collision` is independent: a tracked mesh can be observed without generating
contacts. Enabled collisions require a primitive or explicit convex proxy.
Concave objects use multiple authored convex/primitive children. In particular,
the migrated gate has three separate bar colliders, and pool walls/floor are
explicit Blender boxes. Neither gate openings nor the pool are replaced by
enclosing collision hulls.

Native C++ loading reads saved mesh attributes, parent transforms and supported
static collection instances directly from the `.blend`. Apply modifiers, shape
keys and constraints before saving physical objects. Animated/driven physical
transforms are rejected; the reader does not execute Blender's dependency graph. It bakes scale/shear into mesh geometry and fixes
mirrored winding. Sheared boxes, nonuniform spheres and elliptical cylinders
must use convex geometry instead of being approximated as those primitives.
Friction and restitution use `sim_friction` and `sim_restitution`; backend
capability checks still apply (MuJoCo currently requires zero restitution).

Positions map from the pool's Blender frame to NED as `(-x,y,-z)`. Body/mount
orientations use `M R M`, with `M=diag(-1,1,-1)`. Camera evaluation also converts
Blender's -Z forward/+Y up optical convention. Source units and transforms are
validated before a run starts.

## Calibration profiles

Dynamic bodies select `sim_profile`. The registered names `hydrus`, `proteus`,
`bluerov` and `girona500` use definitions compiled from
`sim/blend/robots/*_profile.inc`. Embedded data and profile directories cannot
override those definitions. Standalone files in `data/robots` also bind the
complete saved file and configuration to SHA-256 hashes in the compiled registry.
See the [fixed asset contract](../../auvs/simulator/data/robots/README.md).

The original pool retains legacy reference-export calibration in
Text datablocks referenced by the source scene's `sim_profile_texts` properties.
Those references survive scene append and renaming. The data is versioned binary
MessagePack encoded as base64, with no executable script content. The source
`.blend` can be relocated and simulated without external profile files.

**Embed calibration profiles** refreshes this embedded data from `sim/profiles/`;
save the Blender file to persist it for legacy/reference tooling. Unregistered
legacy profiles can still resolve from that directory; their embedded values
take precedence over sidecars. Registered robot names always use code. Profiles contain
mass, added inertia, COM, displacement, buoyancy,
calibrated hydrodynamic surfaces/coefficients and motor specifications. They do
not contain a list of world bodies, initial placements or collision geometry.

The Hydrus profile is extracted from the measured constants used before this
migration. Its geometry calibration check compares saved physical shape points to
0.1 mm, tolerating Blender matrix roundoff. Changes that invalidate that
calibration check stop loading with an explicit recalibration error. Update measured
mass/inertia/displacement/fluid calibration and the profile fingerprint together
when deliberately redesigning the vehicle; suppressing the check is not part of
the normal launch path. World-object geometry can be edited independently.

The migrated rotor mounts use the visual model's +X shaft convention. Its axis
is reversed from the legacy model, so the profile converts motor command signs
to preserve the measured command-to-world force and reaction torque.

## Runs and replay

Every run reads its body list from the saved `.blend`. The add-on requires a saved file and rejects unsaved changes. Deleted objects cannot be
restored by profile data or the former `hydrus.json` preset. Objects and shapes
are ordered by stable IDs. Native validation must succeed before an engine world
is created. Normal launches pass the saved file path and source scene name to
C++; there is no Python-exported scene document or Blender scene socket.

`Scene::load` reads a saved `.blend` with CBlend directly in C++. Source scene
and frame metadata come from `AUTONOMY_SIM_SOURCE_SCENE` and
`AUTONOMY_SIM_SOURCE_FRAME` (defaults `AutonomySource`/`1`). Frames label the
initial condition; the native reader accepts static saved transforms and rejects
physical animation/drivers instead of evaluating them at another frame.
Registered robot names resolve to compiled calibration. Unregistered legacy
profiles can use embedded Text datablocks or `AUTONOMY_SIM_PROFILES` in C++.
Native loading supports little-endian, 64-bit
Blender 4.x. Missing data, invalid references and unsupported physical features
stop the run with an error. Blender is needed to author/test scenes, not to run a
saved simulation headlessly.

For internal backend IPC, resolved replay and tests, `Scene::load` also accepts
`-` for binary stdin or `fd:N`. This internal model transport is distinct from
the removed Python exporter startup path.

The internal model handoff starts with `ASIMSC02` and a little-endian uint64 payload
length. The payload is the scene header and its flat arrays copied as bytes:
bodies, shapes, vertices, triangles, thrusters, hulls, hull vertices, faults,
force-module names, and a command sequence. Records are naturally aligned,
little-endian, and limited to 128 MiB. Counts in the header are the schema.
Loading allocates each array to its count and copies the bytes in. Engines
receive that same handoff. It is not a file format and it is not checked in.
`simctl resolve <scene.blend> <seed>` writes one drawn handoff to stdout.
`simctl run ... - ... --resolved` reads it back.

The add-on validates the saved source with `simctl validate`, creates display
copies, and launches the mission with the `.blend` path. It never calls
`exporter.export_scene` or encodes a scene for physics. Calibration profiles are
resolved from the saved Text datablocks before native world creation. Seeded
randomization is applied to the resulting C++ model. An active world owns its
model; subsequent file edits apply to a later run. `simctl inspect` exposes native
calibration and mount values to read-only tools such as controller generation.

`AutonomyPreview` receives the live pose link while a process is running.
Source objects, mounts and their starting conditions stay untouched. Changing
the authoring scene creates a new preview. A saved take remembers the
seed, tick budget, and scene used to launch the run again.
