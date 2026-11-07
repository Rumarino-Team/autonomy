# Blender simulation viewer

The pool scene and propeller animation come from the `experimental` branch at
`ad1c601`. The scene is `auvs/stonefish_sim/data/pool_scene.blend`; its external
pool textures are included beside it. The HDR sky and BlueROV texture are packed
in the scene. The asset is migrated to the `AutonomySource` scene and
`Simulation` collection. Separate fixed robot scenes for Hydrus, Proteus,
BlueROV2 and Girona 500 are in
[`data/robots`](../../auvs/stonefish_sim/data/robots/README.md). Preview files go
under `build/portable`.

The add-on uses the portable runtime with MuJoCo or Stonefish. A run is the
scene, the seed, and the tick budget, launched again when you press Play. There
is no dependency on the old `auvs/hydrus_mujoco` plugin or MJCF files.

**Blender owns the world objects and placements.** Before every add-on run,
`simctl` reads and validates the saved file directly in C++. Registered robots
use compiled calibration; standalone robot files also require matching file and
configuration checksums. The reader constructs a frozen database without a
Python exporter handoff. Validation errors prevent launch. See the
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
the source scene, initial-condition frame and saved-source hash. Save the `.blend` before starting a run; unsaved edits are rejected.

To launch the mission from the terminal instead, press **Start link** in Blender:

```sh
python3 tools/blender/from_blender.py \
  auvs/stonefish_sim/data/pool_scene.blend \
  --scene AutonomySource --frame 1 --backend mujoco \
  --blender-link --realtime
```

The native C++ reader opens the saved `.blend` directly. It reads scene
collections, object transforms, mesh attributes, simulation properties and
embedded calibration from Blender datablocks. Launch passes the file path and
source scene name; it does not start background Blender or invoke the Python
scene exporter. Normal launches produce no intermediate scene document.
`--backend stonefish` selects the other engine.

Native loading supports little-endian 64-bit Blender 4.x files, static parent
transforms and standalone static collection instances. Apply mesh modifiers,
shape keys and constraints before saving physical objects. Bake/remove driven
or animated physical transforms; they require Blender evaluation and are rejected
by this reader. Visual-only objects can retain their presentation effects.

The pool embeds measured calibration, so its simulation definition can be loaded
without profile sidecars. **Embed calibration profiles** refreshes those values
from `sim/profiles/` into the `.blend`; save the file afterwards. Rendering textures
retain their existing external-file packaging. A later run reads that `.blend`,
the tick count, and the seed. It does not read a recording.

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
add and disable colliders, apply modifiers, read saved parenting/instances and verify the
resulting engine models, observations and contacts. They reject invalid bindings
and stale vehicle calibration, and check source isolation and a second run
without source geometry.
Direct-loader tests also cover binary stdin, embedded calibration after relocation,
invalid saved files, malformed internal transfers and successful native loading
with the Blender executable and Python exporter deliberately unavailable.
CI installs the tested Blender version so these checks are required there.
No rendering is needed.

Blender MCP safe mode prevents installing/registering this add-on and executing
its networking code through MCP. Manual installation and normal Blender use do
not require MCP. Only the user can change `BLENDER_MCP_SAFE_MODE` in their MCP
server configuration.

The runtime and bundled codec now use ASIMSC03 with shared mesh tables. Rebuild
the ABI-v3 plugins and Stonefish worker together; old ASIMSC02 recordings are
rejected and must be regenerated. This internal recording/worker format does
not reintroduce a Python scene-export step during launch. See
[the database layout](../../sim/DATABASE.md).
