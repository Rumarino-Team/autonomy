# Simulation authoring contract, version 1

`AutonomySource` in `pool_scene.blend` is the editable source. Its `sim_schema=1`
and `sim_collection="Simulation"` identify the export contract and collection.
Scene metre scale is required. Scene properties `sim_hz`, `sim_gravity`,
`sim_water_density` and `sim_hydro_period` define common world settings.

Every object inside that collection has an explicit `sim_role`. Objects outside
it, or explicitly marked `visual`, are presentation assets. Lights, water
rendering meshes, take curves and decorative robots do not silently become
physical obstacles. Unclassified objects inside the collection stop export.

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
  and `sim_height`. Both mount and camera-child edits affect the exported optical
  pose; the camera datablock supplies horizontal FOV. One simulation camera is
  currently supported; other cameras are presentation-only.
- `instance`: a collection instancer with a stable ID. Standalone static mesh
  body prototypes are exported as independent bodies with deterministic derived
  IDs. Dynamic/compound instances and unrealized modifier instances are rejected;
  realize those instances before export rather than silently losing geometry.
- `visual`: explicit exclusion from physical export.

Use **Add simulation obstacle** for a correctly tagged collidable box. Ordinary
move, rotate, scale and delete operations update the next run's snapshot.
Advanced objects can use Blender's Custom Properties to set the fields above.

## Geometry and collisions

`sim_shape` is `mesh`, `convex`, `box`, `sphere` or `cylinder`.
`sim_collision` is independent: a tracked mesh can be observed without generating
contacts. Enabled collisions require a primitive or explicit convex proxy.
Concave objects use multiple authored convex/primitive children. In particular,
the migrated gate has three separate bar colliders, and pool walls/floor are
explicit Blender boxes. Neither gate openings nor the pool are replaced by
enclosing collision hulls.

The exporter evaluates meshes, modifiers, parents and supported instances through
Blender's dependency graph. It bakes scale/shear into mesh geometry and fixes
mirrored winding. Sheared boxes, nonuniform spheres and elliptical cylinders
must use convex geometry instead of being approximated as those primitives.
Friction and restitution use `sim_friction` and `sim_restitution`; backend
capability checks still apply (MuJoCo currently requires zero restitution).

Positions map from the pool's Blender frame to NED as `(-x,y,-z)`. Body/mount
orientations use `M R M`, with `M=diag(-1,1,-1)`. Camera export also converts
Blender's -Z forward/+Y up optical convention. Source units and transforms are
validated before writing a snapshot.

## Calibration profiles

Dynamic bodies select `sim_profile`. The migrated pool embeds calibration in
Text datablocks referenced by the source scene's `sim_profile_texts` properties.
Those references survive scene append and renaming. The data is versioned binary
MessagePack encoded as base64, with no executable script content. The source
`.blend` can be relocated and simulated without external profile files.

**Embed calibration profiles** refreshes this embedded data from `sim/profiles/`;
save the Blender file to persist it. Unembedded legacy scenes can still resolve
profiles from that directory. Embedded values take precedence. Profiles contain
mass, added inertia, COM, displacement, buoyancy,
calibrated hydrodynamic surfaces/coefficients and motor specifications. They do
not contain a list of world bodies, initial placements or collision geometry.

The Hydrus profile is extracted from the measured constants used before this
migration. Its geometry fingerprint checks evaluated physical shape points to
0.1 mm, tolerating Blender matrix roundoff. Changes that invalidate that
fingerprint stop export with an explicit recalibration error. Update measured
mass/inertia/displacement/fluid calibration and the profile fingerprint together
when deliberately redesigning the vehicle; suppressing the check is not part of
the normal launch path. World-object geometry can be edited independently.

The migrated rotor mounts use the visual model's +X shaft convention. Its axis
is reversed from the legacy model, so the profile converts motor command signs
to preserve the measured command-to-world force and reaction torque.

## Run snapshots and replay

Every run creates a fresh body list from Blender. Deleted objects cannot be
restored by profile data or the former `hydrus.json` preset. Objects and shapes
are ordered by stable IDs. Native validation must succeed before an engine world
is created. Normal launches transfer the evaluated scene over a binary socket;
they do not write or read an intermediate scene JSON file.

`Scene::load` accepts a saved `.blend`, an `.asim` snapshot, `-` for binary
stdin, or an inherited `fd:N` binary socket. `.blend` loading runs Blender in
background with automatic script execution disabled and the shared evaluated
collector. Scene name/frame come from
`AUTONOMY_SIM_SOURCE_SCENE`/`AUTONOMY_SIM_SOURCE_FRAME` (defaults
`AutonomySource`/`1`). `AUTONOMY_SIM_BLENDER_EXECUTABLE` selects Blender;
`AUTONOMY_SIM_BLENDER_SCRIPT` can override the installed loader script location.
Missing files, failed evaluation, timeouts and invalid transfers stop the run.

The snapshot header is `ASIMSC02` followed by a little-endian uint64 payload
length. The payload is the scene header and its flat arrays copied as bytes:
bodies, shapes, vertices, triangles, thrusters, hulls, hull vertices, faults,
force-module names, and a command sequence. Records are naturally aligned,
little-endian, and limited to 128 MiB. Counts in the header are the schema.
Loading allocates each array to its count and copies the bytes in. Engines
receive that same snapshot. A run stores that snapshot, the seed, and the tick
count. Replay launches the run again.

The add-on collects unsaved source data directly and sends the snapshot through
an inherited socket to the mission process. Command-line loading evaluates the
saved file. Fuzz batches resolve `.blend` once into a temporary `.asim`, then
independent workers reuse that immutable input. Preserved failures include
their snapshot so reproduction does not reopen Blender.

The snapshot records evaluated geometry, transforms, resolved calibration,
source scene/frame, and content hash. Calibration profiles stay authoring
inputs and are resolved into the body and shape records before the snapshot is
written. Fuzzing starts from that frozen snapshot and records its seed.
Runs never consult subsequent source edits. Physics replay runs the snapshot
again. After the reducer deletes faults or zeros commands, those edits are
saved in `reduced.asim` and replayed with `--resolved`.

`AutonomyPreview` receives the live pose link while a process is running.
Source objects, mounts and their starting conditions stay untouched. Changing
the authoring scene creates a new snapshot/preview. A saved take remembers the
seed, tick budget, and scene used to launch the run again. The legacy native CBlend
importer remains explicitly opt-in for old stored-mesh workflows and is not
used by these launch paths.
