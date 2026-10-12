# Experimental integration

This branch combines the Motion API and cascaded Zig controllers from
`teleop_mission` with `simulator_improvements`: native Blender scenes and live
updates, Stonefish/RTX graphics, voice configuration editing, MuJoCo physics,
RL tooling, PID tuning, and keyboard teleoperation.

The C and Zig frame layout is ABI **3**; rebuild simulator and hardware plugins
with this branch before loading them. Both Stonefish and MuJoCo export geometry
in the reported body axes and fresh sensor timestamps. The Stonefish IMU is
converted into those body axes, so its default `body_from_imu` is identity.

Initialize the tracked dependencies with `git submodule update --init --recursive`.
Engine changes are preserved in [vendor_patches](vendor_patches/README.md) and
prepared automatically by the simulator CMake configure.

Match the controller file to the selected robot. `platform.json` currently
selects Proteus. Stonefish `hydrus.json`, `proteus.json`, `bluerov2.json`, and
`girona500.json` use backend geometry and nominal force models. Proteus and
Girona allow underactuated geometry; allocation cannot supply a missing force
or torque axis. Gains and thrust limits are initial values that need tuning.

For MuJoCo, select `AUV_MJCF` and use the matching `hydrus.json`, `proteus.json`,
or `bluerov.json` under `auvs/hydrus_mujoco`. The default `auv.json` selects
Hydrus control and preserves legacy tuner baseline parameters.

The voice demo in `demo/voice.py` still edits `platform.json` and the selected
Stonefish robot config. It now exposes active `control` gains and limits;
legacy `odometry` gains and the unused TAM are hidden when the new loops and
geometry allocation are configured. Its existing speech and Decisions service
credentials and Python dependencies are still required.

It also supports `--mission` for spoken Motion/Navigation commands over UDP.
Run the `teleop_command` mission, then `python demo/voice.py --mission --push-to-send`.
The handshake discovers typed command schemas; movement runs while new commands
remain receivable. Raw JSON commands require no OpenAI credentials. See the
[command set, configuration options and protocol](docs/teleop_commands.md).

## Build & Run: libauv_stationary_sim
```sh
zig build run -- \
    ./zig-out/lib/libauv_stationary_sim.so \
    prequalify \
    ./auvs/stationary_sim/auv.json
```

## Build & Run: teleop mission

Build the sender and simulator:

```sh
zig build teleop-sender
zig build
```

Run the teleop mission:

```sh
zig build run -- \
    ./zig-out/lib/libauv_stationary_sim.so \
    teleop \
    ./auvs/stationary_sim/auv.json
```

In another terminal, start the sender:

```sh
./zig-out/bin/teleop_sender 8080
```

## Build & Run: libauv_stonefish_sim

`auvs/stonefish_sim/platform.json` selects the vehicle with `robot`: `hydrus`, `proteus`, `bluerov2`, or `girona500`. Pass the matching controller file.

```sh
zig build -Dstonefish run -- \
    ./zig-out/lib/libauv_stonefish_sim.so  \
    prequalify \
    ./auvs/stonefish_sim/proteus.json
```


## Blender scene loading

The Stonefish simulator reads Blender scenes with CBlend and builds the
environment through Stonefish's C++ API. Build it with `zig build -Dstonefish`
and use the same vehicle settings. See [native Blender loader](auvs/stonefish_sim/blender_stonefish_cpp/README.md)
for CMake setup and validation checks.

## MuJoCo

Requires MuJoCo. `MUJOCO_PREFIX` is an install prefix (`include/` and `lib/`) or a MuJoCo source tree with `build/lib/libmujoco.so`. The window is Stonefish's ocean view, built with the in-tree Stonefish library and loaded from `libauv_mujoco_view.so` beside the plugin.

```sh
zig build -Dmujoco -DMUJOCO_PREFIX=/usr/local
./zig-out/bin/src_3 ./zig-out/lib/libauv_mujoco.so prequalify auvs/hydrus_mujoco/auv.json
```

`AUV_MJCF` selects the model. The default is `auvs/hydrus_mujoco/hydrus.xml`. `proteus.xml` and `bluerov.xml` use the same physics. The window loads the matching Stonefish robot: Hydrus from `open_space_env.scn`, Proteus from `open_space_proteus.scn`, and BlueROV2 from `pool_bluerov2.scn`. Set `HYDRUS_MUJOCO_HEADLESS=1` to skip it.

## Hydrus PID tuning (MuJoCo)

Build with `-Dmujoco` so `libauv_mujoco_rl.so` and `hydrus_tune_pid` are installed, then from the repo root:

```sh
zig build -Dmujoco -DMUJOCO_PREFIX=/usr/local
./zig-out/bin/hydrus_tune_pid --randomize
```

Options: `--episodes`, `--seconds`, `--generations`, `--population`, `--elite`, `--seed`, `--threads`, `--config`, `--xml`, `--randomize`. `AUV_MJCF` overrides the default MJCF.

The preserved tuner optimizes the legacy single pose PID and prints `odometry`
`kp` / `ki` / `kd` arrays. Its model and held-out baseline do not use the new Zig
velocity and angular-rate loops. The live controller on `experimental` uses
`control.velocity_*`, `control.rate_*`, `position_gain`, and `attitude_gain`;
pasting legacy tuning output into `odometry` does not tune those loops.
`rl/tune_pid.py` and `rl/pid_baseline.py` also retain their legacy model.
Source: `auvs/hydrus_mujoco/tune_pid.cpp`.

## Movement and facing

Mission moves require a facing policy. Positions are world XYZ in metres;
angles are Euler yaw in radians. The vehicle's forward axis is body **+Y**:
yaw 0 faces world +Y, and yaw -pi/2 faces world +X. Roll/pitch goals are kept.

```zig
// Capture and keep the heading while translating in XYZ.
try ctx.yieldUntilReachGoal(position, .{
    .facing = .hold_current,
    .position_tolerance = 0.2,
});

// Strafe to a position while continuously looking at an object.
try ctx.yieldUntilReachGoal(position, .{
    .facing = .{ .look_at_object = object.id },
    .translation = .simultaneous,
    .timeout = 60,
});

// Round corners without stopping at intermediate waypoints. Holding the
// heading captures it once for the entire route, not once per waypoint.
try ctx.yieldUntilFollowPath(&waypoints, .{
    .move = .{ .facing = .hold_current, .position_tolerance = 0.2 },
    .corner_radius = 0.6,
    .reference_speed = 0.5,
    .max_tracking_error = 0.5,
});
```

Other facing presets are `.face_goal` (face the final destination),
`.face_path` (face the route tangent), `.{ .look_at_point = world_point }`, and
`.{ .fixed_yaw = yaw_radians }`. Looking directly above/below, or arriving at
the facing target, preserves the last meaningful heading.

`.simultaneous` permits translation while rotating. `.align_first` pauses
horizontal translation and route advancement until the requested heading is
within `alignment_gate` (default 22.5 degrees), then blends translation in.
Yaw commands follow the shortest rotation with `yaw_rate_limit` (default
0.6 rad/s). This limits the **commanded** heading rate; actual speed and turn
response still depend on the vehicle, controller gains, and available thrust.

The reference advances at `reference_speed` and freezes when its distance from
the vehicle exceeds `max_tracking_error`. Intermediate corners use tangent
continuous quadratic curves. The final endpoint must be within
`position_tolerance` (otherwise the mission argument's distance threshold),
and yaw within `yaw_tolerance` (default 5 degrees), for `settle_time` (default
0.3 s). Set `require_yaw = false` for position-only completion. The final goal
and facing policy remain active after the call returns.

`look_at_object` refreshes the target from fresh world observations. Its last
position expires after `observation_timeout` (default 2 s), including time
spent in recovery. Fresh 2D detections can maintain image centering during a
pause; this does not approach the object or confirm world-position arrival.
World-space routes require measured position to start and to complete.

## Tracking recovery and persistent plans

`Motion.State` owns the active command, route geometry, progress, captured
heading, completion state, and timers. `Navigation` checks each sensor's
validity and timestamp; `MotionExecutor` selects supported controls;
`MotionController` combines their disjoint axes into one thruster command.
No active command means zero output. Discovery/search and object selection
are explicit mission actions.

```zig
try ctx.yieldUntilFollowPath(&waypoints, .{
    .move = .{
        .facing = .hold_current,
        .timeout = 120,
        .recovery = .{
            .strategy = .search_for_tracking,
            .max_duration = 15,
            .confirm_tracking = 0.3,
            .resume_ramp = 0.5,
            .max_rejoin_distance = 2.0,
            .search = .{ .sweep_angle = std.math.pi / 3.0, .sweep_rate = 0.3 },
        },
    },
});
```

Recovery strategies:

- `.pause_and_resume`: freeze horizontal route progress, maintain available
  heading/depth control, and await tracking. This is the default.
- `.search_for_tracking`: sweep yaw about the heading at tracking loss.
  `search.body_effort` optionally adds bounded body XY effort. Search movement
  does not advance the saved route.
- `.visual_target`: use an explicitly selected fresh image target through
  `recovery.visual`. A null `target_height` centers without approaching.
- `.estimate`: permit brief route execution with a predicted position. When
  prediction expires, maintain available heading/depth and await tracking.

`max_duration = null` uses `observation_timeout` for compatibility. Recovery
ends with `TrackingLost` if that deadline is reached. The overall move's
`timeout` includes recovery and tracking confirmation. Orientation and pressure
are independently checked; an unavailable axis receives no feedback command.
Pressure is aligned with world Z while measured pose is available. Heading
fallback uses aligned IMU orientation, never an invalid camera pose.

Recovered tracking must remain valid for `confirm_tracking`. A translation
jump larger than `max_rejoin_distance` returns `RejoinTooFar`; an unexpected
heading jump or changed `localization_epoch` returns `LocalizationChanged`.
The route rejoins only within `max_tracking_error` of its saved arc distance,
then ramps translation over `resume_ramp`. Captured heading is preserved.
An orbit cannot jump to an unrelated branch via global nearest-point search.

Failures stop thrusters and retain the plan for inspection/retry. Use
`try ctx.resumeMotion()` followed by `try ctx.yieldUntilMotionComplete()` to
retry with renewed deadlines. Use `ctx.holdPosition()` to explicitly cancel
and replace the plan with a hold. After a localization-frame reset, supply its
known transform using `ctx.reanchorLocalization(tracking_to_mission, epoch)`
before retrying; a transform is not inferred from an arbitrary pose jump.

### Search, visual and body commands

```zig
// Find a gate using fresh detections; position tracking is optional.
try ctx.yieldUntilSearch(.{
    .target_class = .gate,
    .duration = 30,
    .depth = 2.0, // Pressure depth, metres positive down.
});

// Center an object and approach to a normalized image height of 0.5.
// Image size expresses a visual standoff, not a distance in metres.
try ctx.yieldUntilVisualTarget(.{
    .object_id = object.id,
    .target_height = 0.5, // Omit for centering without approach.
    .max_effort = 0.2,
}, .{
    .facing = .{ .look_at_object = object.id },
    .timeout = 30,
});

// Apply body +Y effort for two seconds while maintaining captured heading.
try ctx.yieldUntilBodyMove(.{
    .effort = .{ 0, 0.15, 0 },
    .duration = 2,
}, .{
    .facing = .hold_current,
    .timeout = 10,
});
```

All commands have nonblocking `startFollowPath`, `startSearch`,
`startVisualTarget`, and `startBodyMove` variants. Update frames normally and
inspect `ctx.motion.?.mode`, `.completed`, `.failure`, and `.recovering`.
Body effort is a normalized body-force request, with +Y forward; it is not a
measured velocity or a direct motor command. `control.body_force_scale` converts
it to newtons. `max_effort` caps individual normalized motor outputs (default
0.2 for non-odometry movement); the allocator includes that cap in its solve.
Search and body commands
require orientation feedback to start. Only `.hold_current` and `.fixed_yaw`
are supported for body moves; absolute yaw requires prior world alignment.

A search completes after stable target detection, or after its duration when
no target was specified. Targeted searches return `SearchTimeout` when their
duration expires. Visual completion uses image centering/height tolerances
for `settle_time`; `align_vertical` selects image-driven Z instead of pressure
hold and cannot be combined with an explicit `depth`. Body duration counts
execution with valid heading feedback; final yaw settling follows MoveOptions.
For visual commands, `.align_first` centers the image target before applying
approach effort. Image centering requests a bounded body yaw rate; yaw-rate limits
apply to heading references generated from orientation feedback.
Recovery search uses sweep, body effort, depth and effort-cap settings;
its deadline is `recovery.max_duration`, and it ends when tracking recovers.

### Sensor interface and prediction

The optional `navigation` configuration provides `max_sensor_age` (0.25 s),
`body_from_camera` (camera pose in body axes), and `body_from_imu` (rotation of
IMU vectors into body axes). Identity defaults preserve body-pose simulator
conventions; configure mounting transforms for hardware. IMU/world alignment
and the pressure/world-Z offset are established from valid measured pose.

Prediction extrapolates recent measured velocity at constant speed. Its
configurable limits default to one second, 0.5 m estimated error, 0.5 m/s
error growth, and a 2 m/s maximum source velocity. The error envelope is a
heuristic and requires tuning; it is not a covariance or a guaranteed bound.
Prediction is an opt-in bridge for short outages, and cannot confirm arrival.
It requires fresh, aligned orientation and excludes excessive frame gaps.

Backend ABI version 3 adds per-sensor timestamps, IMU/gyro validity and a
localization epoch. The loader rejects older libraries: rebuild the Zig app
and the selected backend together. ZED uses actual SDK acquisition timestamps
and converts gyro degrees/s into radians/s. Stonefish uses its independent IMU
sensor and masks pose/world detections while tracking is unavailable.
SDK units and acquisition-time behavior follow the
[Stereolabs sensor-data documentation](https://docs.stereolabs.com/docs/tutorials/sensor-data).
Build the hardware library with `zig build -Dzed` when the SDK and CUDA are
available. Backend reinitialization reports a new localization epoch; a
successful library reload still restarts the mission context.

To configure a simulator outage beginning at 5 s and lasting 3 s:

```sh
AUV_SIM_TRACKING_LOSS_START=5 AUV_SIM_TRACKING_LOSS_DURATION=3 \
    ./zig-out/bin/src_3 ./build/libauv_stonefish_sim.so prequalify \
    ./auvs/stonefish_sim/auv.json
```

Without overrides, the existing simulated loss begins at 1000 s and persists.
A zero duration disables it. Rebuild with `zig build` and
`cmake --build build --target auv_stonefish_sim` before using the new interface.

Prequalification explicitly searches for objects and allows up to 15 s of
tracking recovery during crossings and orbits. It faces along the crossing direction through the gate,
then follows a closed, rounded route around the object while looking at it.
The route uses the nearest expanded corner, completes the orbit, and returns
along the entry side. Orbit geometry is planned from the object's initial
bounding box; facing follows its observed position. Generic routes do not
perform obstacle avoidance, so choose waypoints and corner trimming with
clearance in mind.

UDP teleop retains the six-float `[x, y, z, roll, pitch, yaw]` packet and its
Euler yaw convention. `ctx.setPoseGoal` uses the motion API's `.fixed_yaw`
policy with `.simultaneous` translation, so yaw changes while moving in XYZ.
Repeated pose packets preserve the yaw ramp and controller history. A pose
command replaces an active route; it cannot extend a tracking outage deadline.

Run `zig build test` for facing, route geometry, controller, failure handling,
and deterministic closed-loop motion regressions. The test plant has inertia
and drag; it is not a substitute for tuning in Stonefish or on the vehicle.


## Force, attitude, and thruster control

Motion execution requests one physical body wrench: `[Fx, Fy, Fz, Mx, My, Mz]`
in newtons and newton-metres. Position and depth error first request bounded
world velocity. Velocity feedback produces world force, which is combined
before all three components are rotated into body axes. Pressure depth remains
positive down; it does not directly drive body Z when the vehicle is tilted.

Attitude control uses the shortest quaternion rotation error to request body
angular rates. An inner rate PID uses fresh, mounting-corrected gyro feedback;
its derivative acts on measured rate and is filtered. Angular rate and reference
angular acceleration are bounded by `control.max_angular_rate` and
`control.max_angular_acceleration`. The motion's `yaw_rate_limit` also bounds
body yaw rate. These limit references, not guaranteed physical motion.

`Navigation.Snapshot` exposes `velocity_world`, `depth_rate`, and
`angular_velocity_body`. Camera-only backends fall back to successive fresh
quaternion observations for angular rate; gyro feedback takes precedence.
Without orientation/rate feedback, motor outputs are zero. Position/depth loops
wait for their respective velocity observations. Route progress and settling
pause when attitude/rate feedback is unavailable, while command deadlines
continue. Repeated or stale observations are not differentiated as new samples.

### Preserve heading and allow pitch for translation

Proteus has no direct body X (sideways) thrust. On a world position or path
command, release pitch so the controller can redirect vertical thrust sideways
while continuing to control depth, roll, and the requested heading:

```zig
try ctx.yieldUntilReachGoal(.{ 2, 3, 1.5 }, .{
    .facing = .{ .look_at_object = object.id },
    .attitude_policy = .allow_pitch,
    .max_pitch = std.math.pi / 6.0, // 30 degrees, default
    .pitch_rate_limit = 0.25, // commanded pitch change in rad/s, default
    .translation = .simultaneous,
});
```

`allow_pitch` gives the controller ownership of pitch. The facing preset still
owns yaw; for example, `look_at_object` looks toward the object, `hold_current`
keeps the captured heading, and `face_path` turns toward the route. In this
vehicle convention the nose is body +Y, so pitch about body Y redirects vertical
thrust without turning the nose away from the object horizontally.

The allocator checks that the geometry supports this policy. It works with a
missing body X force direction and independent attitude torque, or a fully
actuated layout (which needs no adaptive tilt). Other geometries, a nonlevel
roll goal, an explicit `attitude` quaternion, and body/visual commands reject
the policy. The prequalify orbit uses `allow_pitch`.

Depth demand has priority. Sideways force is capped at
`abs(vertical_force) * tan(max_pitch)`; with no vertical force demand there is
no sideways capacity in this mode. Tilt and thrust limits can slow a route.
The limits bound the attitude reference, not guaranteed physical angles.
This trades independent pitch control for sideways movement; it does not add
a sixth independent actuator. Allocation feedback still prevents integral
windup when thrust limits or the pitch bound restrict the requested motion.

The default `attitude_policy = .fixed` keeps the requested roll and pitch.
Use it when attitude must be preserved, accepting any movement limits of the
thruster layout.

### Fixed full orientation

Existing facing presets still work. To request a full attitude, use a normalized
body-to-world quaternion `[x,y,z,w]`, with `facing = .hold_current`:

```zig
const attitude = math.quaternionFromEuler(.{ 0, std.math.pi / 3, 0 });
try ctx.yieldUntilReachGoal(.{ 2, 3, 1.5 }, .{
    .facing = .hold_current,
    .attitude = attitude,
    .translation = .align_first,
    .attitude_tolerance = std.math.pi / 36,
    .timeout = 30,
});
```

With `attitude`, alignment and completion use quaternion angular distance.
`require_yaw` controls the orientation completion requirement for compatibility.
Settling also requires body angular speed below `angular_rate_tolerance`
(default 0.1 rad/s).
Fixed full attitude supports pause/resume and bounded-estimate recovery;
recovery yaw search and visual-target recovery are incompatible and rejected.
UDP Euler pose packets are converted to a quaternion inside the controller.
A fixed quaternion selects an endpoint; it cannot encode an intentional full
360-degree revolution or a sequence of timed attitude samples.

### Geometry allocation

`allocation` replaces the old `tam` mixer. Each thruster supplies a position
in body coordinates, a positive physical thrust direction, and a force model.
The effectiveness column is `[direction; (position - center_of_mass) cross
 direction + torque_per_force * direction]`. The allocator jointly minimizes
weighted force/torque error with effort regularization, subject to individual
forward/reverse thrust limits, the motion effort cap, and motor command slew.
It uses bounded coordinate descent with fixed storage and no per-frame allocation.
`require_full_rank` defaults to true; underactuated configurations must explicitly
set it false and can have nonzero residual even without actuator saturation.

Example configuration fragment for one thruster (include every motor in output
order, and replace these illustrative values with measured parameters):

```json
{
  "allocation": {
    "center_of_mass": [0, 0, 0],
    "weights": [1, 1, 2, 6, 6, 4],
    "thrusters": [{
      "position": [0.2, 0.4, 0],
      "direction": [0, 1, 0],
      "forward_force": 20,
      "reverse_force": 15,
      "command_sign": 1,
      "exponent": 2,
      "torque_per_force": 0,
      "command_rate": 1.5
    }]
  }
}
```

The force curve is `signed_force = full_force * sign(command * command_sign) *
abs(command)^exponent`, choosing the forward or reverse maximum by physical
force sign. Bounds are converted through this curve before solving. A linear
force-command actuator uses exponent 1; a nominal quadratic rotor-speed model
uses exponent 2. Geometry positions must reference the same body origin as pose
feedback, and `center_of_mass` is expressed in those same axes.

Stonefish exports optional `auv_get_thruster_geometry` metadata after init.
Its configuration enables `use_backend_geometry`; this reads the actual compound
body centre of mass and thruster frames in odometry body axes. The force models
remain explicit in JSON. Hydrus uses the scenario's nominal static-water model:
1025 kg/m^3, 0.18 m propeller, 1000 RPM, thrust coefficient 0.48, yielding
143.4672 N at full command; inverted motor polarity and propeller reaction torque
are included. This nominal model does not measure current generated thrust or
compensate rotor lag, inflow, or partial submergence. Update it when the scenario
or physical thrusters change.

`ctx.allocation_result` exposes requested and model-predicted achieved wrench,
residual, motor commands, and a residual flag (`saturated`). Residual includes
solver approximation and regularization, as well as limits or unavailable axes.
Allocation residual is transformed to the appropriate feedback frame and fed
back into the velocity/rate integrators to prevent windup. Emergency stops zero
motors immediately, bypassing slew limits. Configuration and backend reloads
rebuild the allocator and reset feedback history.

Configurations with only `tam` retain the legacy mixer and use uniform scaling
when motor limits are exceeded, with command-limiter feedback. They do not gain
physical geometry or thrust calibration. `control.legacy_wrench_scale` maps body
wrench signs/scales into that mixer's convention. The previous `odometry` PID
block is accepted for configuration compatibility; the new loops use `control`
gains, so existing gain values must be migrated deliberately. `non_odometry`
gains continue to set visual force feedback and image-centering rate gain.

The simulator's `control` gains are initial values for tuning, not validated
maneuver performance. Build the app with `zig build` and the simulator library
with `cmake --build build --target auv_stonefish_sim`. The optional geometry export
does not change ABI version 3 or the frame layout.
