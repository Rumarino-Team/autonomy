# Sensors and the Zig autonomy bridge

For the Zig application, configuration is now compiled from `config/application.zig`.
Use `zig build -Dsimulator=true` and `./zig-out/bin/src_3 run`; JSON configuration
and the old positional launch arguments are removed. See
[application reload and binary data](../../docs/APPLICATION_RELOAD.md) for current
commands, reload ownership, binary reports, and validation. Native C interfaces
described below remain available.


The simulator owns ideal sensor descriptions in the frozen C scene. Physics,
measurements, and the AUV adapter are separate stages:

`scene → physics state → scheduled sensor samples → AuvFrame → Zig control → commands`

## Run a mission

From `/home/cesar/tmp2/autonomy`:

```sh
zig build -Dsimulator=true

# MuJoCo is the default backend next to the installed AUV plugin.
./zig-out/bin/src_3 run

```

The default C-defined Robosub course has physical gate posts, a noncolliding
opening target, a marker, and a floor. The saved Hydrus optical camera faces body
-Y; the controller's viewing direction is derived from that mount. The course
is in front of it. Only visible targets enter the mocked camera detections.
The current mission passes the opening, circles the marker, and returns to its
circle starting position. This is a headless run, without rendered images or a
viewer. Normal Zig controller logs show progress; shutdown prints a text summary
with ticks, final diagnostic hash, depth bounds, and status. Set `run_result_path`
in the Zig configuration to save an AUVDATA1 binary result.

`-Dsimulator=true` builds/installs the core, AUV plugin, MuJoCo backend/native library,
headers, assets, and Hydrus controller config. It requires native Linux, Zig
0.17.1, and the simulator's existing native build dependencies. No ROS, camera SDK,
or hardware connection is used. The normal build and existing plugin choices
remain available.

## Scene records

`SensorRec` lives in `simulation_types.h` and contains a unique ID, kind, owning
object, local sensor-to-body pose, enabled flag, period in physics ticks, and
kind-specific settings. Sensors are plain records in one dedicated aligned,
capacity-limited buffer, not intrusive datablocks. Addresses stay fixed until
model destruction. `simulation_add_sensor` copies the record and returns a
borrowed stable pointer. `simulation_set_sensor_routing` explicitly chooses
tracking, IMU, pressure, and camera sources for observations. Routed sensors
must belong to the same body. Routes may be NULL.

Every robot preset supplies ideal pose tracking, IMU, and pressure sensors at the
body origin plus the existing calibrated camera mount/intrinsics. These three
added mounts are simulation defaults, not measured vehicle sensor locations.
Robot presets and model instances have independent sensor storage. Other sensors
can be attached to any scene object and read through the raw sample API.

Default sensor capacity is 16, configurable before initialization. Builders
reject zero periods, invalid poses/settings, duplicate IDs, foreign owners or
routes, and mutation after freeze. Cloning copies records and repairs owners,
routes, and fault targets. The robot sensor view selects its four owned records
inside the scene's collection. There is no PMR or individual record deletion.

For a new fixture, initialize a model and object, fill a `SensorRec` with
`sensor_default`, set its ID/owner/mount/settings, add it, select routes, and
finalize. C examples and tests demonstrate this lifecycle. Physical-only models
and worker projections may contain no sensors.

## Sampling and coordinates

Samples are initialized on reset at tick zero and refreshed after physics steps
whose tick is divisible by `period_ticks`. At the default 360 Hz, pose/pressure
use period 6, IMU period 1, and camera period 12. These are tick periods rather
than hard-coded frequencies: changing the physics timestep changes their rates.
Held samples keep their measurement tick and validity until the next update.
A disabled sensor publishes invalid samples at its scheduled updates.

`simulation_runtime_sensor_samples` returns a borrowed array of `SensorSample`:
ID/kind, measurement tick, validity, tracking axis masks, and a typed payload.
Camera payloads reference runtime-owned detection arrays. Views expire on the
next step, reset, or destruction. `simulation_runtime_observe` is a read of the
cached routed observation; reading neither samples nor observations advances
state or changes the diagnostic hash.

- World coordinates are NED; quaternions are xyzw. Mounts map sensor axes into
  body axes. Pose tracking measures the world pose of its mounted frame.
- IMU orientation is the world orientation of the sensor. Gyro and specific
  force are expressed in sensor axes. Specific force is COM acceleration plus
  `alpha × r + omega × (omega × r)`, minus world gravity, rotated into sensor
  axes. `r` runs from COM to the sensor. Reset assumes zero initial linear and
  angular acceleration; configured initial angular velocity still contributes
  centripetal acceleration. Subsequent derivatives use the last physics step.
- Pressure depth is the mounted point's world Z relative to the fixed water
  surface at Z=0, positive downward. It is not the camera depth.
- Cameras use optical +Z, +X right, +Y down. Intrinsics retain each preset's
  calibration. Near/far defaults are 0.05/50 m along optical Z. Object bounds
  include contained colliders, including noncolliding detection proxies.
  Bounds are clipped at near/far planes and image edges. Outputs are visible
  world-space 3D boxes plus projected pixel boxes, in scene-object order.
  The camera's own body is excluded. No occlusion, RGB rendering, learned
  perception, sensor noise, random dropouts, or latency is modeled in this phase.

Sensor faults reference a specific scene-owned sensor. Tracking, IMU, and camera
outages invalidate that sensor's samples; gyro bias adds to sensor-axis X and
pressure bias adds meters of depth. Fault intervals are `[start,end)` in physics
ticks and are evaluated at sampling time; changes become visible on that
sensor's next scheduled update. Actuator dead/weak faults retain their existing
per-step semantics. Camera and tracking failures are independent.

## AUV adaptation and commands

`AuvFrame` and the four required functions retain their layout/signatures. The
simulator publishes body-origin pose in the historically named `camera_pose`,
as expected by the controller. The separate simulator observation retains the
actual optical camera pose. Tracking/IMU mounts are removed when estimating body
pose; IMU vectors are rotated into body axes while retaining acceleration at
the actual sensor location.

Tracking is preferred for pose. A valid IMU supplies attitude when tracking
attitude is unavailable. A valid pressure sensor supplies body Z when tracking
Z is unavailable, correcting its mount offset only when full attitude is
available or the pressure mount is at the body origin. Unavailable axes stay
false. Raw pressure depth remains separate. Invalid IMU vectors are zero and
its quaternion is identity; `AuvFrame` has no separate gyro/accelerometer
validity bits, so consumers needing those must use the raw sensor API.

A frame contains at most 128 detections, selected in scene order; Zig's persistent
object caches are bounded too. The adapter rejects class IDs outside uint8.
Thruster slots/counts retain preset ordering. Commands are normalized shaft-speed
requests in [-1,1]; the adapter rejects invalid counts/non-finite/out-of-range
values and applies no extra scaling. Commands are held between controller frames.

Defaults and environment settings for direct native `auv_init` callers (the Zig
host uses compiled configuration and explicit scene/options calls instead):

- `AUTONOMY_SIM_ROBOT=hydrus`: also supports proteus, bluerov, girona500.
- `AUTONOMY_SIM_SCENE=robosub_course`: `robot_only` omits scenery.
- `AUTONOMY_SIM_BACKEND`: absolute backend .so path; defaults to adjacent MuJoCo.
- `AUTONOMY_SIM_FRAME_TICKS=6`: 60 Hz controller frames at default physics rate.
- `AUTONOMY_SIM_MAX_TICKS=21600`: positive and divisible by frame ticks.
- `AUTONOMY_SIM_REALTIME=0`: run as fast as possible; 1 adds steady-clock pacing.

Frame timestamps are nanoseconds of simulation time, independent of pacing.
`auv_simulation_status` is optional for hardware plugins: code 0 means running,
1 means tick budget exhausted, and 2 reports initialization/command/step errors.
The Zig loader queries it and terminates explicitly on simulator failures.
Each thread has its own bridge context; calls for that world stay serialized.
Deinitialization destroys the runtime before backend/model resources.

The Hydrus controller uses an SI wrench allocator derived from current mounts,
COM, rotor coefficients, shaft signs, and reaction torque. A quadratic mapping
converts bollard-thrust fractions to normalized RPM commands. PID limits,
measurement damping, world-frame translation, and buoyancy feedforward are
opt-in settings; legacy controller defaults remain available. Regenerate/check
its config with `simulator_controller_config` and the calibration test after
changing the preset. Other presets pass bridge tests; their mission tuning is
not supplied here.

Application-library reload can retain the native world and controller history
when `reload.preserve_state` is enabled. Reloads with preservation disabled create
a fresh world. Core, backend, or shared-layout changes require a process restart.
Model edits apply when building a new scene; saved Blender edits are not loaded
automatically. See the root [usage guide](../../README.md) for both reload modes.

The scene and backend ABI is now v5 and rejects v4. Rebuild/install all simulator
artifacts together. ASIMSC03 and worker protocol v2 remain physics-only: sensors,
routing, and their faults stay in the parent runtime, not the Stonefish worker.

MuJoCo-only parallel seeded mission fuzzing is documented in [FUZZING.md](FUZZING.md).
Stonefish is disabled by default in this delivery and is not part of fuzz validation.
