# Autonomy: usage and configuration

Run underwater missions, develop the Zig controller, simulate with MuJoCo, and
run reproducible mission fuzz campaigns. The executable is a small host; mission
logic, `MissionContext`, controllers, and application configuration live in a
reloadable Zig library. The simulator core, AUV adapter, snapshot reader, and
MuJoCo backend are also implemented in Zig behind shared C interfaces.

This guide describes the current checkout. Old examples using a controller JSON
file, positional plugin/mission/config arguments, or `simctl` are not the current
application interface. MuJoCo is the supported and tested simulator here;
Stonefish sources are optional and outside this workflow.

## Contents

- [Quick start](#quick-start)
- [Build commands and options](#build-commands-and-options)
- [Application commands](#application-commands)
- [Configuration reference](#configuration-reference)
- [Controller gains and calibration](#controller-gains-and-calibration)
- [Timing and simulation speed](#timing-and-simulation-speed)
- [Application reload and compiler watch](#application-reload-and-compiler-watch)
- [Fuzzing and replay](#fuzzing-and-replay)
- [Scenes, robots, and sensors](#scenes-robots-and-sensors)
- [Blender](#blender)
- [Viewer, telemetry, and teleoperation](#viewer-telemetry-and-teleoperation)
- [Stationary plugin and hardware](#stationary-plugin-and-hardware)
- [Native C interfaces and examples](#native-c-interfaces-and-examples)
- [Tests and sanitizers](#tests-and-sanitizers)
- [Packaging](#packaging)
- [Troubleshooting](#troubleshooting)
- [Source and documentation map](#source-and-documentation-map)

## Quick start

Run commands from the repository root, for example `/home/cesar/tmp2/autonomy`.

Requirements for the simulator build:

- Zig **0.17.1**, CMake, Git, Python 3, and a working C/C++ compiler.
- Network access for the first build, which fetches the pinned MuJoCo sources.
- Linux is the validated runtime. Windows support has had cross-compilation
  coverage; that is not a claim of native Windows runtime validation.
- ZLS 0.17 is optional editor tooling. ROS, the ZED SDK, CUDA, and physical
  hardware are unnecessary for the MuJoCo workflow.

```sh
zig version
zig build -Dsimulator=true
./zig-out/bin/src_3 run
```

The first build command packages the simulator dependencies as well as the
application. Subsequent application/configuration edits usually need only:

```sh
zig build
./zig-out/bin/src_3 run
```

Defaults are Hydrus, the Robosub course, `prequalify_robosub`, 360 Hz physics,
60 Hz controller frames, a 60-second simulation budget, and no wall-clock pacing.
This is a **headless** run: console output is expected, not a 3D window. Use the
[viewer](#viewer-telemetry-and-teleoperation) or [Blender](#blender) for visualization.

A successful mission reports completion and a final line such as
`simulation steps=... hash=... min_depth=... max_depth=... status=0`.
Ticks and hashes depend on your gains, scene, configuration, and build; they are
not fixed success criteria across different configurations or compiler versions.

## Build commands and options

### Build steps

```sh
zig build                         # host, application library, metadata, default tools
zig build -Dsimulator=true        # additionally build/install core, adapter, MuJoCo
zig build simulator-core          # only the core and AUV adapter libraries
zig build simulator-mujoco        # backend; requires built MuJoCo headers/library
zig build test                    # Zig unit tests and shared C/Zig ABI checks
zig build run -- run              # build and launch the application
zig build teleop-sender           # optional SDL2 keyboard sender
zig build arduino-sender          # Proteus serial sender
zig build arduino                 # Arduino sketch through Arduino CLI
zig build fuzz-startup-test       # test fixture for worker startup failures
zig build uninstall               # remove installed artifacts for the chosen prefix
zig build --help                  # authoritative build-option/step listing
```

`simulator-core` does not build the MuJoCo engine. For a complete runnable
simulation, use `-Dsimulator=true` first. The default Linux build also installs
`arduino_sender`; SDL2 and ZED camera support are separate opt-in builds.

### Project build options

- `-Dsimulator=true|false`: build and package the native simulation dependencies;
  default `false`. This delivery explicitly configures Stonefish off.
- `-Doptimize=debug|safe|fast|small`: Zig optimization mode; default `debug`.
  `safe` is a useful release mode with runtime safety checks. Older examples
  spell these `Debug`, `ReleaseSafe`, `ReleaseFast`, and `ReleaseSmall`.
- `-Dtarget=<target>`: compilation target, such as `x86_64-windows-gnu`.
  `-Dcpu=<features>`, `-Dofmt=<format>`, and `-Ddynamic-linker=<path>` are also
  available through Zig's standard target options.
- `-Dmujoco-include=<directory>`: C header root for the Zig backend.
  Default `build/simulator/_deps/simulator_mujoco-src/include`.
- `-Dmujoco-library=<file>`: pinned MuJoCo shared library to link.
  Default `build/simulator/lib/libmujoco.so`.
- `-Dzed=true|false`: build the Proteus ZED hardware plugin; default `false`.
- `-Dcxx=<program>`: compiler used for that hardware plugin; default `g++`.
- `-Dzed-prefix=<directory>`: ZED SDK installation; default `/usr/local/zed`.
- `-Dcuda-prefix=<directory>`: CUDA installation; default `/usr/local/cuda`.
- `-Darduino-cli=<program>`: Arduino CLI executable; default `arduino-cli`.
- `-Darduino-fqbn=<board>`: Arduino board; default `arduino:avr:uno`.
- `-Dexternal-asan=true|false`: testing option for an externally preloaded ASan
  runtime to own signal handlers/stacks. It does not instrument Zig code.

Useful Zig build flags include `--prefix <directory>` for an install location,
`-j<N>` for build parallelism, `--summary all` for build details, and
`--cache-dir <directory>` for an isolated build cache. Build parallelism is
separate from the fuzzer's worker count.

For a prebuilt MuJoCo installation through CMake, see [native builds](#native-builds).
Do not combine headers and a native library from different MuJoCo revisions.

## Application commands

The executable accepts these operations:

```sh
./zig-out/bin/src_3                  # same as run
./zig-out/bin/src_3 run              # one configured mission
./zig-out/bin/src_3 fuzz             # configured finite fuzz campaign
./zig-out/bin/src_3 replay 10451216379200822465
./zig-out/bin/src_3 replay build/campaign-001/case-000000.bin
./zig-out/bin/src_3 inspect build/campaign-001/totals.bin
```

A decimal replay argument is a **case seed**, not a master seed or case index.
A nonnumeric replay argument is a binary case-file path. Replay runs one case
on one worker. `inspect` reads a binary file as human-readable text without
starting a physics world.

Operational options such as robot, scene, gains, seed, worker count, and pacing
are compiled from Zig. There are no `--robot`, `--cases`, `--seed`, `--workers`,
`--realtime`, or `--config` runtime flags in this interface, and no `-Dconfig`
build option. The old `<plugin> <mission> <controller.json>` launch is removed.
Use `zig build --help` for build help; it is separate from application operations.

Ctrl+C requests shutdown. Fuzz workers stop at controller-frame boundaries,
join, and record cancellation. Missions, failed cases, exhausted tick budgets,
startup failures, and output errors can produce a nonzero exit status.

## Configuration reference

Edit **[config/application.zig](config/application.zig)**. Ordinary `zig build`
always imports it. [src/Configuration.zig](src/Configuration.zig) defines and
validates all application/controller fields; it is the source of truth for types.

A complete basic example is:

```zig
const Types = @import("../src/Configuration.zig");
pub const config: Types.Application = .{
    .controller = @import("hydrus.zig").controller,
    .mission = .prequalify_robosub,
    .plugin = "libauv_simulator.so",
    .backend = "libsimulator_backend_mujoco.so",
    .robot = "hydrus",
    .scene = "robosub_course",
    .frame_ticks = 6,
    .max_ticks = 21600,
    .physics = .{ .dt_numerator = 1, .dt_denominator = 360, .hydro_period = 7 },
    .realtime = false,
    .telemetry = true,
    .telemetry_address = "127.0.0.1:5600",
    .blender = .{ .enabled = false, .port = 9760 },
    .verbose = true,
    .run_result_path = null,
    .reload = .{ .watch = true, .preserve_state = true },
    .fuzz = .{
        .seed = 1,
        .cases = 100,
        .workers = 8,
        .results = "build/campaign-001",
        .nominal = false,
        .profile = .{},
    },
};
```

Only `controller` is required; other fields have the defaults shown above,
except the type's default fuzz output directory is `fuzz-results`.
Your actual saved `config/application.zig` may override any default.

### Mission and libraries

- `controller`: compiled controller parameters. Presets are `hydrus.zig`,
  `proteus.zig`, and `stationary.zig` under `config/`.
- `mission`: `.prequalify_robosub` (default), `.prequalify_coco`, or `.teleop`.
  Robosub with Hydrus/MuJoCo has mission-completion coverage. Selecting another
  mission or robot does not imply that combination has calibrated mission tuning.
- `goal_dist_threshold`: waypoint distance tolerance in meters; default `1`,
  finite and positive.
- `plugin`: AUV interface library; default `libauv_simulator.so`. Alternatives
  include `libauv_stationary_sim.so` and the optional `libauv_proteus_hwd.so`.
- `backend`: physics plugin; default `libsimulator_backend_mujoco.so`.
- `robot`: `hydrus` (default), `proteus`, `bluerov`, or `girona500`.
- `scene`: `robosub_course` (default), `robot_only`, a `.blend` path, or an
  AUVDATA1 scene `.bin` path. File extensions select file loading.

Library filenames resolve relative to the installed libraries; absolute paths
are supported. Scene/result paths should preferably be absolute when launches
may use different working directories. Windows maps the bundled `.so` defaults
to the corresponding DLL names.

### Timing, output, and reload fields

- `physics.dt_numerator`, `physics.dt_denominator`: positive integers specifying
  the physics timestep as a rational number of seconds; defaults `1/360`.
- `physics.hydro_period`: hydrodynamic force-cache update interval in physics
  ticks; default `7`, positive.
- `frame_ticks`: physics steps per controller frame; default `6`, positive.
- `max_ticks`: physics-tick budget; default `21600`, positive and divisible by
  `frame_ticks`. Exhaustion stops the mission; it does not count as success.
- `realtime`: default `false`; `true` adds wall-clock pacing.
- `telemetry`: default `true`; publish binary UDP telemetry for the viewer.
- `telemetry_address`: UDP destination; default `127.0.0.1:5600`.
- `blender.enabled`: default `false`; publish live simulation poses to Blender.
- `blender.port`: UDP pose-link port; default `9760`, nonzero.
- `verbose`: default `true`; enable detailed controller-frame logging. Set it
  to `false` for quieter normal runs.
- `run_result_path`: default `null`; set a filename such as
  `"build/run-001.bin"` to write a binary single-run result. Choose a fresh
  filename; saved results are protected against accidental overwrite.
- `reload.watch`: default `true`; watch application/configuration sources while
  running. This is distinct from the compiler's `--watch` flag.
- `reload.preserve_state`: default `true`; restore compatible application state
  and retain the physics world during application-library replacement.

### Fuzzer fields

- `fuzz.seed`: master `u64` seed; default `1`.
- `fuzz.cases`: positive number of cases; default `100`.
- `fuzz.workers`: positive number of dedicated OS worker threads; default `8`.
  At most one worker per case is used.
- `fuzz.results`: output directory; default `"fuzz-results"`. Use a nonexistent
  or empty directory for each campaign/replay.
- `fuzz.nominal`: default `false`; `true` selects the no-variation profile.
- `fuzz.profile`: typed variation profile; defaults are listed in the
  [fuzzing reference](#variation-profile-reference).

## Controller gains and calibration

The normal Hydrus controller is in [config/hydrus.zig](config/hydrus.zig).
Change gains there, or derive a controller in `config/application.zig`:

```zig
.controller = blk: {
    var controller = @import("hydrus.zig").controller;
    controller.odometry.kp[2] = 12; // example: change the depth proportional gain
    break :blk controller;
},
```

This is a field fragment inside the application initializer, not an entire file.
For six-axis vectors the order is **X, Y, Z, rotation-X, rotation-Y, rotation-Z**.
Distances use meters and angles use radians. Translation errors can be in world
or body coordinates; attitude corrections use the controller's body conventions.

Controller options are:

- `teleop_port`: UDP command receiver; default `8080`, nonzero.
- `camera_forward`: nonzero body-frame camera viewing direction, required by
  the controller. Keep it consistent with the preset's calibrated camera.
- `rotation_limit`: maximum absolute body-axis rotations used to build look
  orientations. Zero holds an axis at its reference; defaults are unbounded.
- `tam`: thruster allocation rows, one six-axis row per active motor; 1–8 rows.
- `thruster_output_scale`: positive output multiplier; legacy default `5`.
  The calibrated Hydrus simulator preset uses its own value (`1`).
- `thruster_curve`: `.linear` (legacy default) or `.quadratic_rpm`.
- `odometry.rotation_gate`: attitude-error threshold above which horizontal
  translation is suppressed; default `pi/8` radians.
- `odometry.kp`, `odometry.ki`, `odometry.kd`: six proportional, integral,
  and derivative gains; required in a controller definition.
- `odometry.derivative_on_measurement`: default `false`; use measured motion
  for damping instead of differentiating changing goals.
- `odometry.translation_frame`: `.body` (legacy default) or `.world`.
- `odometry.integral_limit`: six bounds on integral contributions; unbounded
  by default.
- `odometry.output_limit`: six bounds on total PID outputs; unbounded by default.
- `odometry.feedforward_world_force`: known steady force in world axes, such as
  buoyancy compensation; default zero.
- `ibvs.center_deadband`: image-centering tolerance; required and nonnegative.
- `ibvs.target_height`: target image-height setting; required and positive.
- `ibvs.interaction`: six rows by three columns, mapping image error into body
  error; required.

Controller presets override these type defaults. The simulator receives
normalized shaft-speed commands in `[-1,1]`; motor order, mounts, signs,
allocation, and RPM conversion must agree. Robot presets are available for all
four vehicles, but a calibrated closed-loop simulator controller is supplied
for Hydrus. Do not assume the Hydrus allocator is valid for another vehicle.
See [controller/sensor conventions](auvs/simulator/SENSORS.md).

## Timing and simulation speed

With the defaults:

- Physics timestep: `1/360` second.
- Controller frame: `6/360` second, or 60 Hz.
- Budget: `21600/360 = 60` simulation seconds.
- Hydrodynamics: recomputed every 7 physics ticks and cached between updates.
- Pose tracking and pressure: every 6 ticks; IMU: every tick; camera: every 12
  ticks. Changing the physics timestep also changes these effective rates.

Set `.realtime = true` to aim for one simulation second per wall-clock second.
Set it to `false` to run as quickly as computation permits. Frame timestamps
always represent simulation time. A slower machine can fall behind real time.
There is currently no application option for an arbitrary 0.5x/2x speed factor.
Changing the physics timestep changes integration and sensor timing; use pacing
for wall-clock throttling.

## Application reload and compiler watch

### Reload a running mission

Leave this configuration enabled:

```zig
.reload = .{ .watch = true, .preserve_state = true },
```

Run `./zig-out/bin/src_3 run`, edit a controller gain or application source, and
save. The host watches `src/`, `config/`, `include/`, and its application/build
roots, invokes ordinary `zig build`, prepares a new application library, and
replaces it between controller frames. MuJoCo pauses during compilation and
its real-time clock is rebased afterward.

For interactive tuning, enable `.realtime = true` before launching so the
unpaced mission does not finish before you can edit it.

Compatible reloads preserve mission progress, active maneuvers/queues, targets,
PID history, held commands, physics tick, native solver state, and sensor caches.
Failed builds or rejected state migrations leave the previous generation active.
State transfer is a versioned binary in-memory operation; disk resume is deferred.

Set `.reload.preserve_state = false` for reloads that recreate the world from
initial conditions. Set `.reload.watch = false` to disable application reload.

Preserved reload rejects structural changes including mission selection, robot,
scene/builder, native plugin/backend, timing/budget/pacing, telemetry/Blender I/O,
teleop port, and motor count. Use a fresh-world reload or restart for those.
Host/host-ABI edits and simulator core/adapter/backend edits require rebuilding
and restarting the executable, even when the simulator implementation is Zig.
Fuzz campaigns keep one application generation until their workers finish.

### Keep the compiler running

For fast build feedback with Zig 0.17.1:

```sh
zig build --watch -fincremental --debounce 25
```

`--debounce 25` waits 25 ms to group nearby save events; it is configurable,
not part of the physics timestep. This command watches and rebuilds artifacts;
it does not launch a mission. Compiler watch and application reload are separate:
the host's current reload loop still invokes ordinary `zig build` itself.
An external compiler watcher does not replace that loop or guarantee the same
latency for a complete application reload.

For implementation details and state migrations, see
[application reload](docs/APPLICATION_RELOAD.md).

## Fuzzing and replay

A small campaign configuration is:

```zig
.fuzz = .{
    .seed = 1,
    .cases = 100,
    .workers = 8,
    .results = "build/campaign-001",
    .nominal = false,
    .profile = .{},
},
```

Build, run, then inspect:

```sh
zig build
./zig-out/bin/src_3 fuzz
./zig-out/bin/src_3 inspect build/campaign-001/totals.bin
./zig-out/bin/src_3 inspect build/campaign-001/case-000000.bin
python3 tools/data_binary.py build/campaign-001/summary.bin
```

Every worker owns its model, world, allocator, RNG, controller history, and
mission context. Cases are independent of worker scheduling. One failed case
does not stop the rest, but any failure makes the campaign exit nonzero.
A broad fault profile is intended to find failures, not guarantee every mission
succeeds. Fuzzing disables pacing, telemetry, Blender publishing, verbose frame
logs, and hot reload. `.teleop` cannot be fuzzed.

Output is binary AUVDATA1:

- `campaign.bin`: campaign configuration and provenance.
- `case-NNNNNN.bin`: case seed, resolved variations, outcome/error, mission stage,
  final tick/hash, depth bounds, and available final state.
- `summary.bin`: ordered case results.
- `totals.bin`: aggregate counts.

Set `fuzz.results` to a **new empty output location**, then rebuild before replay:

```sh
./zig-out/bin/src_3 replay 10451216379200822465
# Or replay recorded configuration/profile from a case file:
./zig-out/bin/src_3 replay build/campaign-001/case-000000.bin
```

Seed replay uses today's compiled profile/configuration. File replay restores
recorded configuration/profile but uses today's output directory. Reproducibility
requires the same profile/draw-order version, baseline assets, controller/mission
code, and simulator build. Hash equality is not promised across compiler versions.

### Variation profile reference

[SceneContext.FuzzProfile](src/SceneContext.zig) defines every profile field:

- `position`: X/Y/Z placement half-ranges in meters; `[0.2, 0.2, 0.1]`.
- `rotation`: local X/Y/Z rotation half-ranges in radians; `[1.05, 1.05, 1.15]`.
- `velocity`: per-axis world linear-velocity half-range; `0.2` m/s.
- `angular_velocity`: per-axis world angular-velocity half-range; `0.2` rad/s.
- `mass`: shared mass/dry-mass/inertia factor, `{ min = 0.97, max = 1.03 }`.
- `volume`: shared body/hull volume factor, `0.97–1.03`.
- `drag`: shared hull drag/skin coefficient factor per body, `0.7–1.4`.
- `thrust`: thrust/torque factor per motor, `0.8–1.2`.
- `rotor_inertia`: rotor inertia factor, `0.7–1.5`.
- `current_speed`: horizontal current speed, `0–0.15` m/s, uniform direction.
- `placements`: randomize scenery placement; `true`.
- `groups`: rigid placement groups. The course default groups IDs
  `2,10,11,12`, anchored on ID `2`, so gate pieces move together. Use explicit
  groups for a custom scene; `&.{}` removes groups. Members must be distinct,
  disjoint between groups, and include the anchor once.
- `actuator_fault`: append one motor fault when motors exist; `true`.
- `actuator_start_min`, `actuator_start_max`: inclusive start-tick range,
  `36–360`.
- `actuator_duration`: `180` physics ticks.
- `dead_probability`: probability of a dead rather than weak motor; `0.25`.
- `weak`: remaining motor-strength factor, `0.3–0.7`.
- `sensor_fault`: append one fault targeting available routed sensors; `true`.
- `sensor_start`, `sensor_end`: half-open tick interval `[360,720)`.
- `sensor_bias`: gyro-X/depth bias range, `-0.05–0.05` rad/s or meters,
  according to the selected fault kind.

Example profile fragment:

```zig
.profile = .{
    .position = .{ 0.1, 0.1, 0.05 },
    .rotation = .{ 0.05, 0.05, 0.15 },
    .actuator_fault = false,
    .sensor_fault = false,
},
```

Unspecified fields keep their defaults. To disable **all** variations, use
`.nominal = true`, not just zero position/rotation. Existing authored faults
remain present. Topology, geometry, motor ordering/signs, sensor mounts/rates,
controller settings, and mission logic stay fixed. Invalid cases fail without
silent resampling. Details: [fuzzing guide](auvs/simulator/FUZZING.md).

## Scenes, robots, and sensors

`robosub_course` contains the robot, a physical gate frame, a detectable opening,
a marker, and a floor. `robot_only` omits the course. These are initial scenes;
runtime body states and commands live in a separate world-owned runtime.

Choose the robot with `.robot`; choose its matching controller separately.
All four presets have collider, hull, thruster, and sensor coverage. Hydrus has
five colliders and eight active thrusters. Preset geometry/calibration is compiled
in [robot_data.zig](auvs/simulator/zig/robot_data.zig); saved Blender assets and
binary provenance live under `auvs/simulator/data/robots/`.

To author programmatic scenes, edit
[src/scenes/ApplicationScene.zig](src/scenes/ApplicationScene.zig) or use the shared
C construction API. Build a mutable scene, configure objects/sensors/faults,
validate, and finalize before creating a world. Frozen topology cannot be edited
in place. Clone to a mutable model or start a fresh world for changes.

Default ideal sensors are tracking, IMU, pressure, and camera detections. Tracking
and IMU mount transforms, positive-down pressure depth, scheduled samples,
frustum/range clipping, and explicit sensor routing are implemented. Observations
hold the latest sample between updates and reads do not advance simulation time.
Fault intervals are half-open and applied at sample times. RGB rendering,
occlusion, sensor noise, and latency are not modeled. See the
[sensor API and conventions](auvs/simulator/SENSORS.md).

World coordinates are NED and shared quaternions are xyzw. Camera optical axes
are +Z forward, +X right, +Y down. The AUV frame's historically named
`camera_pose` is the controller's body pose; simulator observations also expose
the actual optical camera pose.

## Blender

### Install the add-on and receive live poses

```sh
zig build -Dsimulator=true
python3 tools/blender/package_addon.py
```

Install `build/portable/autonomy_sim.zip` using Blender's **Install from Disk**
add-on action. Enable **Autonomy Simulation**, set its repository preference to
this checkout, and open the **Autonomy** panel in the 3D View sidebar. The add-on
specifies Blender 4.5 or later; the saved assets were tested with Blender 4.5.6.

Configure the application before launching:

```zig
.blender = .{ .enabled = true, .port = 9760 },
.realtime = true,
```

Use the add-on's pose receiver and **Run Live** controls, or start its receiver
and run `./zig-out/bin/src_3 run` from a terminal. Match the receiver port to the
compiled setting. Live updates drive disposable preview copies so the saved
source scene is not overwritten by simulation poses. Logs/take records are under
`build/portable/autonomy_takes/`.

Mission, scene, timing, seed, and backend settings for the current `src_3` launch
come from Zig configuration. Legacy controls/environment values visible in the
add-on do not override that configuration. Use CLI `fuzz`/`replay` for seeded
campaigns rather than assuming the add-on's take seed randomizes a normal run.

### Use a saved Blender scene

Set the application field to an absolute path and build:

```zig
.scene = "/absolute/path/to/pool_scene.blend",
```

Save edits before launching. The **current implementation launches background
Blender to export an AUVDATA1 binary snapshot**, then loads it with the Zig
snapshot reader. It does not parse `.blend` DNA directly and it does not generate
MuJoCo XML. It needs a `blender` executable on PATH, or an explicit executable:

```sh
AUTONOMY_BLENDER=/absolute/path/to/blender ./zig-out/bin/src_3 run
```

Saved scene geometry is resolved when constructing a fresh world. Saving a
`.blend` alone does not trigger the Zig source watcher or change a retained world.
For an explicit export, including a chosen source scene and frame:

```sh
blender --background --factory-startup --disable-autoexec \
  /absolute/path/to/pool_scene.blend \
  --python tools/blender/export_snapshot.py -- \
  --output /absolute/path/to/pool.scene.bin --scene AutonomySource --frame 1
```

`--output` is required; `--scene` and `--frame` are optional. Then configure
`.scene = "/absolute/path/to/pool.scene.bin"`; loading that file needs no Blender
process. [tools/blender/from_blender.py](tools/blender/from_blender.py) is a helper
that builds and launches the application using its existing Zig configuration.

The snapshot exporter uses custom properties: `sim_role="body"`, nonempty
`sim_id`, optional `sim_name`, `sim_motion` (`static`, `dynamic`, `kinematic`),
`sim_class`, `sim_tracked`, `sim_fuzz`, and collision/shape properties such as
`sim_collision` and `sim_shape`. Child colliders use `sim_role="collider"` and
`sim_body` pointing to the owner's `sim_id`. `sim_profile` identifies the vehicle
preset; that vehicle's pose comes from the scene and its physical calibration
comes from the compiled preset. Only the selected vehicle may be dynamic.
Start with `auvs/simulator/data/pool_scene.blend` and the supplied robot assets;
arbitrary unannotated Blender objects are not automatically simulator bodies.

## Viewer, telemetry, and teleoperation

The viewer displays binary telemetry on UDP 5600. Its telemetry-only build does
**not** require ZED/CUDA; CMake fetches raylib and needs its platform graphics/window
build dependencies. Run these in separate terminals after building:

```sh
cmake -S tools/viewer -B tools/viewer/build -DVIEWER_ENABLE_ZED=OFF
cmake --build tools/viewer/build
./tools/viewer/build/viewer 5600 none

# Application terminal; keep .telemetry=true in config/application.zig:
./zig-out/bin/src_3 run
```

Viewer arguments are `[telemetry_port] [video] [teleop_port]`. The default
telemetry port is `5600` and teleop port is `8080`; use `none` for no video.
Optional camera support uses `-DVIEWER_ENABLE_ZED=ON` and requires ZED SDK/CUDA.
The existing optional H264 stream is at `127.0.0.1:5610`.

To control the robot, choose `.mission = .teleop`, enable real-time pacing, rebuild,
and run. Either use the focused viewer window or the optional SDL2 sender:

```sh
zig build teleop-sender
./zig-out/bin/teleop_sender 8080
```

SDL2 development files and pkg-config are needed for that sender. Match its port
to `controller.teleop_port` and configure controller gains/geometry for the robot.
The simulator's tick budget still applies during teleop.

Hold W/S for surge, A/D for sway, and R/F for heave. Left/Right changes yaw about
world +Z; Up/Down changes pitch about body X; I/K changes roll about body +Y.
Releasing translation keys removes the translation command. The UDP packet is
28 bytes: three little-endian body-translation floats followed by world orientation
quaternion floats x, y, z, w. The sender's identity assumes a level camera along
body +Y, so account for the selected robot's camera/body convention.

## Stationary plugin and hardware

For the stationary plugin, change these application fields:

```zig
.plugin = "libauv_stationary_sim.so",
.controller = @import("stationary.zig").controller,
.mission = .teleop, // .prequalify_coco is another available mission
```

Use ordinary `zig build` and `./zig-out/bin/src_3 run`. This plugin does not create
a MuJoCo world. The `robot`, `scene`, and physics settings are simulator options;
they are not hardware configuration for this plugin.

For Proteus hardware use `libauv_proteus_hwd.so` with
`@import("proteus.zig").controller`, and build the SDK-dependent plugin:

```sh
zig build -Dzed=true -Dzed-prefix=/usr/local/zed -Dcuda-prefix=/usr/local/cuda
```

Its ZED/CUDA installation and physical serial/device setup are separate from the
MuJoCo workflow. `zig build arduino` uploads no firmware by itself: it invokes
Arduino CLI compilation for the selected FQBN. `./zig-out/bin/arduino_sender` is
an interactive serial sender using the hardware implementation's device settings.
Application configuration edits keep legacy hardware controller defaults unless
you explicitly select different values.

## Native C interfaces and examples

### Native builds

The same package can be built with CMake:

```sh
cmake -S . -B build/simulator \
  -DAUTONOMY_BUILD_SIMULATOR=ON -DSIMULATOR_ENABLE_STONEFISH=OFF \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build/simulator --target simulator_all --parallel 8
```

CMake invokes Zig for the core, adapter, and MuJoCo plugin, and builds the pinned
native engine and C/C++ examples/tests. Relevant CMake options are
`AUTONOMY_BUILD_SIMULATOR=ON`, `SIMULATOR_ENABLE_STONEFISH=OFF`,
`SIMULATOR_MUJOCO_PREFIX=<matching native install>`, `CMAKE_BUILD_TYPE`,
`SIMULATOR_SANITIZE`, and `SIMULATOR_THREAD_SANITIZE`. Existing Stonefish
options are legacy opt-in build paths, not part of the validated usage here.

```sh
build/simulator/auvs/simulator/simulator_c_example \
  "$PWD/build/simulator/auvs/simulator/libsimulator_backend_mujoco.so"
build/simulator/auvs/simulator/simulator_robot_example \
  "$PWD/build/simulator/auvs/simulator/libsimulator_backend_mujoco.so" hydrus
build/simulator/auvs/simulator/simulator_trace \
  "$PWD/build/simulator/auvs/simulator/libsimulator_backend_mujoco.so"
```

The robot example also accepts `proteus`, `bluerov`, and `girona500`.
These examples exercise the simulator directly, without the Zig autonomy mission;
a single final summary from the robot example is normal. `simulator_trace` emits
periodic state/rotor/hash data. `simulator_controller_config` emits binary Hydrus
controller calibration; inspect its redirected output with the binary inspector.

Shared headers in `auvs/simulator/` include:

- `simulation_types.h`: C model and runtime record layouts, settings, sensors,
  faults, states, and views.
- `sim.h`: lifecycle, construction, cloning, finalization, and runtime API.
- `sim_backend.h`: ABI-v5 physics plugin function table and opaque worlds.
- `auv_simulator.h`: separately versioned simulator-specific AUV extensions.
- `robots.h`, `scene_presets.h`, `arena_allocator.h`: presets, builders, allocators.
- `include/auv.h`: required AUV calls and shared frame layout.

Create a stable-address model, add records, finalize it, load a backend, create a
runtime, step/read/reset, then destroy runtime before backend and model. Views
borrow model/runtime memory and expire according to the API. Multiple independent
worlds can run on separate threads; calls to a single world remain serialized.
See [native simulator documentation](auvs/simulator/README.md) and
[examples](auvs/simulator/examples/).

### Environment compatibility for direct native AUV callers

The four-function `auv_init` compatibility path still accepts these variables:

- `AUTONOMY_SIM_ROBOT`: default `hydrus`.
- `AUTONOMY_SIM_SCENE`: default `robosub_course`.
- `AUTONOMY_SIM_BACKEND`: native library path; default adjacent MuJoCo.
- `AUTONOMY_SIM_FRAME_TICKS`: default `6`.
- `AUTONOMY_SIM_MAX_TICKS`: default `21600`, divisible by frame ticks.
- `AUTONOMY_SIM_REALTIME`: `0` (default) or `1`.
- `AUTONOMY_SIM_BLENDER`: enable the pose link when nonzero.
- `AUTONOMY_SIM_BLENDER_PORT`: default `9760`.

The Zig host uses explicit scene/options calls, so these do not override
`config/application.zig`. `AUTONOMY_BLENDER` is different: it selects the external
Blender executable used when resolving a `.blend` file.

## Tests and sanitizers

```sh
zig build test
cmake --build build/simulator --target simulator_all --parallel 8
ctest --test-dir build/simulator --output-on-failure -j4
ctest --test-dir build/simulator -R 'snapshot|codec|mujoco' --output-on-failure
```

The normal suite covers database/allocator ownership, ABI layouts, primitive and
mesh/compound scenes, forces, sensors, cold reset, deterministic missions,
parallel worlds, fuzz/replay, application reload, and binary input validation.
A broad campaign producing mission failures is distinct from a regression test
failing. Saved trajectory baselines are tied to the configuration/build used to
capture them; investigate changes before replacing expectations.

Use separate build directories for sanitizer modes; they cannot be enabled
together:

```sh
cmake -S . -B build/sim-asan \
  -DAUTONOMY_BUILD_SIMULATOR=ON -DSIMULATOR_ENABLE_STONEFISH=OFF \
  -DSIMULATOR_SANITIZE=ON -DCMAKE_CXX_COMPILER=clang++ \
  -DCMAKE_C_COMPILER=clang -DCMAKE_BUILD_TYPE=Debug
cmake --build build/sim-asan --target simulator_all --parallel 8
ctest --test-dir build/sim-asan --output-on-failure -j3

cmake -S . -B build/sim-tsan \
  -DAUTONOMY_BUILD_SIMULATOR=ON -DSIMULATOR_ENABLE_STONEFISH=OFF \
  -DSIMULATOR_THREAD_SANITIZE=ON -DCMAKE_CXX_COMPILER=clang++ \
  -DCMAKE_C_COMPILER=clang -DCMAKE_BUILD_TYPE=Debug
cmake --build build/sim-tsan --target simulator_all --parallel 8
ctest --test-dir build/sim-tsan -R concurrency --output-on-failure
```

ASan/UBSan options instrument native C/C++ clients and libraries and enable the
configured runtime checks. They do not imply full compiler instrumentation of
Zig code; Zig builds retain their own runtime safety checks. Blender/viewer
interactive checks and native Windows testing require their respective runtime
installations and are separate from the headless suite.

## Packaging

Build and install everything into a fresh directory:

```sh
zig build -Dsimulator=true -Doptimize=safe --prefix "$PWD/build/my-package"
./build/my-package/bin/src_3 run
```

On Linux, keep `bin`, `lib`, `include`, and `share` together. The important libraries
are `libautonomy_application.so`, `libsimulator_core.so`, `libauv_simulator.so`,
`libsimulator_backend_mujoco.so`, and the pinned `libmujoco.so.3.9.0` (plus its
symlink). The stationary plugin and native examples are packaged too. The core and
backend have no direct C++ runtime dependency; the native MuJoCo engine retains
its own implementation/dependencies.

Source-based live reload also needs the original source checkout and the Zig
compiler path embedded in the host. For a package deployed without source/build
tooling, compile with `.reload.watch = false`. A standalone installed binary
cannot rebuild a source tree that is absent.

To install just CMake's simulator component, without the application host/library:

```sh
cmake --install build/simulator --prefix "$PWD/build/native-package" --component simulator
```

There is no prebuilt ROS/colcon requirement for these packages. The validated
package contains MuJoCo artifacts and no Stonefish worker/backend.

## Troubleshooting

- **`pathFromRoot`, `@cImport`, or `dupeZ` compile errors on Zig 0.17:** those are
  older source APIs. This checkout has the compatibility migration; verify your
  branch/files, use `zig version`, and rebuild current sources. Adding `.*` does
  not restore a removed build API.
- **Library not found / missing simulator entry point:** run
  `zig build -Dsimulator=true`, keep matching libraries together, and check
  `plugin`/`backend` paths. ABI-v4 and stale mixed-version artifacts are rejected.
- **No visible simulation window:** a headless run is expected. Start the viewer
  or Blender receiver. `simulator_robot_example` normally prints one summary.
- **Too much output:** set `.verbose = false` and rebuild.
- **Run completes too quickly to watch:** set `.realtime = true` and enable the
  desired telemetry/Blender receiver.
- **Tick-budget exhaustion:** the mission did not finish within `max_ticks`.
  Check configuration, sensing, gains, and faults; increasing the budget is a
  configuration change, not proof the controller problem is resolved.
- **Results directory/file already exists:** use a new empty campaign directory
  or fresh result filename, then rebuild if its path is compiled configuration.
- **Reload rejected after changing a scene, timing, or I/O setting:** use
  `.reload.preserve_state = false` or restart for structural changes.
- **Saving `.blend` did not change physics:** save the authoring scene and start
  a fresh world; verify `scene` names that file rather than `robosub_course`.
- **Blender export fails:** check the Blender executable, saved source scene,
  custom properties, and exporter path. Preview-only geometry is not an authored
  simulator scene.
- **Fuzz run exits nonzero:** inspect `totals.bin` and individual case files.
  Fault-profile mission failures are recorded findings; startup/capacity/output
  errors are separate failures, and cancellation is reported separately.
- **Viewer receives nothing:** enable `.telemetry`, match the UDP address/port,
  and use `none` when no video source is intended. Fuzzing disables telemetry.
- **Another robot fails a mission:** matching motor count alone does not provide
  matching allocation, signs, camera direction, gains, or course calibration.

## Source and documentation map

- [Application configuration types](src/Configuration.zig) and
  [active configuration](config/application.zig).
- [Application reload, ownership, migration, and binary schemas](docs/APPLICATION_RELOAD.md).
- [Fuzzer ownership, ranges, and reproducibility](auvs/simulator/FUZZING.md).
- [Simulator model, custom allocation, presets, and native examples](auvs/simulator/README.md).
- [Sensor frames, sampling, routing, faults, and AUV adaptation](auvs/simulator/SENSORS.md).
- [Zig simulator implementation](auvs/simulator/zig/) and
  [shared binary codec](src/BinaryData.zig).
- [Blender exporter](tools/blender/autonomy_sim/exporter.py) and
  [add-on packager](tools/blender/package_addon.py).
- [Shared C AUV interface](include/auv.h) and [backend ABI](auvs/simulator/sim_backend.h).

Some deeper documents retain explicitly recorded historical measurements. For
current option names, use `zig build --help`, `Configuration.zig`, and
`SceneContext.FuzzProfile`; for current tuned values, read your saved preset files.
