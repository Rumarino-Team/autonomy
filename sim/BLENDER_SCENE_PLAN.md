# Make Blender authoritative for simulation objects

Status: implemented. See `tools/blender/SCENE_AUTHORING.md` for the versioned
contract and `tools/blender/README.md` for the exporter, launch and replay paths.
The subsequent direct-loading implementation accepts `.blend` natively and
uses binary IPC; JSON is now an optional export/legacy input and trace format.

## Intended behavior

Blender owns the entities, initial placements, geometry, collision shapes,
semantic labels, vehicle attachments and sensor mounts for a run. Both physics
backends receive the same owned, backend-independent scene snapshot generated
from Blender. JSON remains the snapshot/trace format rather than a second
manually maintained list of world objects.

Vehicle profiles retain calibrated mass, inertia, displacement, buoyancy,
hydrodynamic coefficients, motor parameters and controller tuning. Blender
objects explicitly select a profile. Those profiles must not silently add
world objects or override Blender placements. Geometry edits that invalidate
calibration must be reported, rather than silently assigning new physics.

Version one applies edits on the next run. An active run uses an immutable
snapshot; replay uses the recorded snapshot, irrespective of later edits.

## Original gaps addressed by this implementation

- `sim/auv_plugin.cpp` loads `sim/scenarios/hydrus.json` directly.
- `sim/blend_import.cpp` updates a base scenario by name. Deleted Blender
  objects can survive in the base; vehicle geometry bindings may go unmatched.
- The native reader handles stored Blender mesh data and a subset of legacy
  properties. It does not evaluate the active Blender dependency graph.
- Import currently conflates the convex flag with collision participation and
  reduces object classes to a gate/marker choice.
- The viewer and replay change Blender object transforms. Exporting those same
  objects without an authoring boundary could export a playback pose as the
  next run's initial condition, including visual rotor animation.

## 1. Define scene ownership and migrate the pool

Introduce a versioned, backend-neutral authoring contract:

- Stable `sim_id` identifiers, independent of object names and collection order.
- A designated simulation source scene/collection and an explicit role:
  rigid body, collision shape, thruster, sensor or visual-only object.
- Body motion type, collision enablement, semantic class and profile reference
  are separate properties. Collision shape type is separate from all of them.
- Collider, sensor and thruster objects identify their owning rigid body.
- Meshes, objects and mounts use documented metre/NED conversion conventions;
  simulation cameras are distinguished from presentation cameras.

Migrate `pool_scene.blend` to this contract. Convert legacy `stonefish_*`,
`geometry_slot` and `mujoco_geom` properties once, with a migration report.
Keep lights, materials, water rendering effects, paths and presentation cameras
outside the physical export. Require explicit classification for ambiguous
objects within the simulation collection.

Represent pool walls, the floor and gate bars using authored primitive or
convex compound collision proxies. Preserve openings rather than replacing
concave objects with a single enclosing convex hull. Reject unsupported
collision geometry instead of dropping collisions.

Deliverable: a migrated pool and documented property/collection contract.

## 2. Export evaluated Blender data into our scene structures

Create one Blender Python exporter shared by the add-on and background CLI.
Use Blender's evaluated dependency graph for geometry, modifiers, instances,
parent transforms, scale and the selected initial-condition frame. Export the
current authoring data, including unsaved edits when launched from the add-on.

Construct a fresh entity list on every export. Join physical profiles through
stable IDs/profile references; never merge the old preset's world-object list.
Vehicle origins and attachment transforms come from Blender. Keep visual
rotor rotation separate from the physical shaft mount.

Validate duplicate IDs, missing parents/profiles/bindings, non-finite or invalid
transforms, mesh topology, collision support, semantic classes and units.
Handle mirrored transforms with correct triangle winding. Unmatched required
bindings or unsupported features stop export with an object-specific error.

Sort entities/shapes by stable identifiers and write the validated snapshot
atomically. Hash the exported content and physics profiles; file timestamps
alone do not establish equivalence. Report the Blender/exporter versions,
authoring scene/frame and included/excluded objects.

The existing CBlend importer remains an explicitly selected legacy path; it
must not silently substitute for evaluated Blender export.

Deliverable: deterministic export to the existing portable scene structures,
plus the corresponding validation and provenance report.

## 3. Make every Blender-sourced launch use that snapshot

Change `Run live` and `Record run` to export and validate the designated source
scene before spawning a mission, then pass the exact generated path through
`AUTONOMY_SIM_SCENARIO`. Display the source scene, snapshot identity and object
count. A failed export prevents the mission from starting; it must never fall
back to `hydrus.json` or a stale export.

Provide a terminal command that runs the same exporter using background Blender
against a saved `.blend`, then starts the requested backend. The CLI must make
the source file and authoring frame explicit. Standalone snapshots remain
supported for replay, CI and machines without Blender, with their origin shown.

Split current vehicle presets into calibration profiles and generated scene
snapshots, and update the documented mission commands to use the Blender source
or its explicitly generated artifact.

Deliverable: consistent add-on and CLI launch paths with no hidden world preset.

## 4. Keep authoring, preview and replay transforms separate

Maintain editable source objects independently of objects used for live output
and takes, using a generated preview scene and ID-to-object mapping. Geometry
may be shared where read-only; streamed poses, rotor animation and take actions
must affect preview objects only. Repeated runs must not move the source robot,
gate, marker, collision proxies or mounts.

Bind incoming states to the exported IDs rather than relying on display names.
Version the presentation protocol as needed, and retain the current name-based
AMJB format as explicit legacy compatibility.

Attach the full resolved snapshot, source-content hash, profile hashes, exporter
version and fuzz seed to recordings. Apply seeded randomization only after the
authored snapshot is frozen. Playback and physics replay consume the captured
snapshot; changing the `.blend` afterwards cannot change an old recording.

Deliverable: reproducible runs and replay without changing authored conditions.

## 5. Prove source ownership on both backends

Add integration fixtures that modify Blender data and inspect exported objects,
backend model/state data, contacts and common sensor observations:

1. Move/rotate the gate; both backends start with its exported transform and
   detections use the updated geometry.
2. Add a collidable box; it appears in the scene inventory and produces contact
   on both engines. Disable collisions; it remains present without contacts.
3. Delete the marker; it disappears from the snapshot and detections. No preset
   object may restore it. A mission requiring it may correctly exhaust its
   search budget.
4. Modify mesh geometry, a modifier, a parent transform or an instance; evaluated
   geometry and bounds agree with Blender after coordinate conversion.
5. Move the robot, a thruster mount and the simulation camera; initial poses,
   lever arms and camera observations follow the authored attachments.
6. Export an unchanged authoring scene repeatedly, including after a live run or
   take playback; scene/profile hashes and initial conditions remain equal.
7. Replay a recorded run after editing/deleting the source file; recorded
   physics states remain reproducible without consulting the changed source.
8. Reject duplicate IDs, missing profiles and unsupported collision geometry
   before creating either backend world.

Check semantic scene equivalence across engines and deterministic replay within
each engine. Different engines need not produce identical dynamic trajectories.
Rerun existing controller, scene, fuzz, packet and Blender tests. Keep intentional
geometry corrections distinct from baseline/tuning changes.

## Completion criteria

- Every physical/tracked world entity and initial transform is attributable to
  the exported Blender source; profile-only calibration is explicitly identified.
- Additions, deletions, transforms and evaluated geometry edits affect the next
  simulation on both engines.
- Viewer-only objects do not accidentally become collision geometry.
- Export failure cannot start a stale/preset simulation.
- Live visualization and replay never modify authored initial conditions.
- CI verifies scene ownership, backend consistency and recorded replay behavior.

Implement in the order above. Establish the scene contract and migrate the
pool before wiring export into launch; simply invoking the current merge-based
importer would not meet these criteria.
