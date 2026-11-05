# Blender simulation viewer

The pool scene and propeller animation come from the `experimental` branch at
`ad1c601`. The scene is `auvs/stonefish_sim/data/pool_scene.blend`; its external
pool textures are included beside it. The HDR sky and BlueROV texture are packed
in the scene. The asset is migrated to the `AutonomySource` scene and
`Simulation` collection. Generated snapshots and preview files go under
`build/portable`.

The add-on uses the portable runtime with MuJoCo or Stonefish. A run is the
scene, the seed, and the tick budget, launched again when you press Play. There
is no dependency on the old `auvs/hydrus_mujoco` plugin or MJCF files.

**Blender now owns the simulation objects.** Before every add-on run the evaluated
authoring scene is evaluated, joined to embedded calibration, validated
by `simctl`, and transferred as a frozen binary scene. Evaluation errors prevent launch. The add-on never merges
the old preset's world objects or falls back to an earlier export. See the
[authoring contract](SCENE_AUTHORING.md) for object roles and calibration.

## Install

```sh
zig build sim
zig build
python3 tools/blender/package_addon.py
```

In Blender 4.5+, open Preferences → Add-ons → Install from Disk and choose
`build/portable/autonomy_sim.zip`. Enable **Autonomy Simulation**, then set its
**Repository** preference to this checkout. Disable **Hydrus MuJoCo Link** if
that experimental add-on is enabled, to avoid duplicate rotor handlers and UDP
listeners. The original Stonefish scene authoring add-on can remain enabled.

In the 3D View sidebar, open **Autonomy** and click **Load pool scene**, or open
`pool_scene.blend` directly. Loading the pool appends a scene and keeps the
current file's other scenes. Loading it again reuses that scene.

## Live viewing

Select **MuJoCo** or **Stonefish** and click **Run live**. This starts the current
`prequalify_robosub` mission at real time, sends poses to the local listener,
and saves the complete trace as a take. When it finishes the take is baked and
ready for timeline playback. **Cancel** stops the process; unsuccessful and
cancelled runs remain marked accordingly. **Record run** performs the same
mission as fast as possible and imports its result. Scenario randomization is
optional and uses the seed in the panel. The tick budget defaults to 21,600.

The `AutonomyPreview` scene receives live poses and take actions. **Edit source
scene** returns to the editable source; preview changes never become initial
conditions. **Add simulation obstacle** creates a collidable box in the source
collection with a stable ID. Move, resize or delete it with ordinary Blender
tools. Edits apply to the next run; the current run stays frozen. The panel shows
the source scene, initial-condition frame and snapshot hash.

To launch the mission from the terminal instead, press **Start link** in Blender:

```sh
python3 tools/blender/from_blender.py \
  auvs/stonefish_sim/data/pool_scene.blend \
  --scene AutonomySource --frame 1 --backend mujoco \
  --blender-link --realtime
```

The native runtime reads the saved `.blend` using the same evaluated collector
and an `ASIMSC02` socket handoff. Normal runs create no intermediate scene
file. The add-on sends current unsaved source data directly over an inherited
socket. `--backend stonefish` selects the other engine.

The pool embeds measured calibration, so its simulation definition can be loaded
without profile sidecars. **Embed calibration profiles** refreshes those values
from the repository; save the `.blend` afterwards. Rendering textures retain their
existing external-file packaging.

For an explicitly requested snapshot export use `--export-only --output scene.asim`.
`build/portable/sim/simctl snapshot source.blend frozen.asim` freezes a snapshot
for machines or workers that should run without Blender. A later run uses that
snapshot, the tick count, and the seed. It does not read a recording.

`AUTONOMY_SIM_BLENDER_PORT` (or CLI `--port`) overrides localhost port 9760. Presentation output
and real-time pacing are disabled by default so parallel fuzz workers stay
independent. Pacing changes wall-clock scheduling; simulation timestamps,
controller inputs and state hashes remain fixed-step. Dropped UDP packets and
an absent listener do not stop physics.

Blender snapshots use `AMJB` version 2: a little-endian `<4sHHd` header (magic,
version, pose count, simulation seconds), then `<I32s3d4d` poses (stable uint32
body ID, label, xyz, quaternion wxyz). ID 4294967295 denotes the camera. Names
can change without breaking bindings. Version 1 (`<32s3d4d`) remains supported
for explicitly selected legacy snapshots. The pool maps NED
to Blender as `(-x, y, -z)` and orientation as `M R M`, `M=diag(-1,1,-1)`.
Bodies are identified by their stable IDs; the camera has its own world
transform. Live propeller animation estimates effort from pose changes and is
for appearance only.

## Replay

A take in the sidebar remembers the seed, tick budget, and scene. **Play**
launches that run again. **Run live** does the same and follows the UDP pose
link for as long as the process runs. Progress is the simulation time in the
latest pose packet. Nothing is written except the process log.

**Onboard camera** switches the scene camera to `hydrus_camera`. **Detach**
removes actions that a take assigned, so the live link or a manual pose is not
overridden. The file's existing render engine is used.

The integration tests create `build/portable/blender_mujoco_replay.blend` and
`build/portable/blender_stonefish_replay.blend` as ready-to-open examples.

## Validation

`zig build sim-test` checks packet names and coordinates, camera transforms,
and identical state hashes with/without real-time output.
When `blender` is on PATH, it also runs isolated background Blender tests for
both backends: the Play operator, live UDP reception, onboard camera, textures
and all four vehicles' rotor rigs. Source ownership tests move/delete objects,
add and disable colliders, evaluate modifiers/parenting/instances and verify the
resulting engine models, observations and contacts. They reject invalid bindings
and stale vehicle calibration, and check source isolation and a second run
without source geometry.
Direct-loader tests also cover binary stdin, embedded calibration after relocation,
failed scene evaluation, malformed transfers and one evaluation per parallel batch.
CI installs the tested Blender version so these checks are required there.
No rendering is needed.

Blender MCP safe mode prevents installing/registering this add-on and executing
its networking code through MCP. Manual installation and normal Blender use do
not require MCP. Only the user can change `BLENDER_MCP_SAFE_MODE` in their MCP
server configuration.
