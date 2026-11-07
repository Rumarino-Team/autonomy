# Seeded MuJoCo mission fuzzing

Build and run from `/home/cesar/tmp2/autonomy`:

```sh
zig build -Dsimulator=true
./zig-out/bin/src_3 \
  "$PWD/zig-out/lib/libauv_simulator.so" prequalify_robosub \
  "$PWD/zig-out/share/autonomy/simulator/config/hydrus.json" \
  --fuzz --seed 1 --cases 100 --workers 8 --results build/fuzz-seed-1
```

The current build, fuzz runner, package, and validation target MuJoCo only.
Stonefish is disabled by default and is not exercised by fuzzing. Its existing
sources remain available behind `SIMULATOR_ENABLE_STONEFISH`; the borrowed-scene
extension rejects other engines. The normal hardware/stationary plugin commands
remain supported. The legacy Zig `-Dstonefish` option explicitly enables its
existing CMake targets; it is not part of this delivery's validation.

Results directories must be fresh: an existing `campaign.json` is not overwritten.
The runner defaults to master seed 1, 100 cases, eight OS workers, Hydrus and the
Robosub course. Existing `AUTONOMY_SIM_ROBOT`, `AUTONOMY_SIM_SCENE`,
`AUTONOMY_SIM_BACKEND`, `AUTONOMY_SIM_MAX_TICKS`, and `AUTONOMY_SIM_FRAME_TICKS`
settings are captured before launching workers. The budget defaults to 21,600
physics ticks and frames to six ticks (360 Hz physics / 60 Hz controller).
Fuzz runs always disable pacing, watchers, telemetry, and verbose frame logs.
Teleop is rejected. Other robot presets require a matching controller config;
only Hydrus has supplied Robosub mission tuning.

Replay the **case seed**, rather than the campaign's master seed:

```sh
./zig-out/bin/src_3 \
  "$PWD/zig-out/lib/libauv_simulator.so" prequalify_robosub \
  "$PWD/zig-out/share/autonomy/simulator/config/hydrus.json" \
  --fuzz --replay-seed 10451216379200822465 --results build/fuzz-replay
```

Replay forces one case on one worker. `--profile nominal` selects the compiled
profile with no variations/faults, useful for checking the unmodified mission.
The default profile is selected by `--profile default`.

## Ownership and lifecycle

Each dedicated `std.Thread` constructs a `SceneContext` in place, including its
own frozen C baseline. Each case clones a mutable scene, constructs a private
Xoshiro256++ RNG and arena, applies variations, finalizes, and initializes that
thread's AUV adapter. A private `MissionContext` parses the coordinator's captured
controller configuration and owns its PID history, queues, and detection caches.
The mission loop is shared with the normal single-world launch.

Scene roots contain address-sensitive pointers: never assign or move them after
initialization. Model buffers are owned exclusively by their thread; they do not
all need physical TLS declarations. The adapter's active model/runtime/commands
are `thread_local`. No world or mutable RNG/controller storage is shared.
The coordinator shares immutable API/settings/configuration and uses an atomic
case counter and cancellation flag. Each result slot has only one writer.

`auv_simulator.h` defines extension v1, without changing AuvFrame or scene/backend
ABI v5. `auv_simulator_init_scene` borrows a frozen model until `auv_deinit`, and
copies/consumes its options during initialization. Options select the MuJoCo path,
frame/budget, pacing, and summary printing. `auv_simulator_read_result` copies
statistics and available final body state before destruction. All extension calls
operate on the calling OS thread. Failed initialization retains no borrowed model.

Destroy the batch mission context, adapter world/runtime/backend, cloned scene,
and case arena in that order. Baselines die when their worker exits. Libraries
are pinned until every worker joins. Ctrl+C stops assigning work and cancels
active missions cooperatively between frames; it does not forcibly kill a thread
inside an engine call. Partial thread startup also cancels and joins all workers.

## Generator contract and ranges

`SceneContext.zig` exposes `randomize(std.Random, FuzzProfile)` for mutable scenes.
Profiles are typed Zig values/functions; external JSON profiles are deferred.
Profile/generator version is 1. Case seeds use:

```
case_seed = splitmix64(master_seed +% index *% 0x9e3779b97f4a7c15)
```

`splitmix64` includes the standard additive step. Every case initializes an
explicit `std.Random.Xoshiro256` from that seed. Draw order is dynamic objects in
scene order, placement groups, marked standalone scenery, current, motor slots,
actuator fault, and sensor fault. Do not change it without versioning the profile.

Default variations relative to the baseline:

- Dynamic position X/Y ±0.2 m, Z ±0.1 m; local roll/pitch ±0.05 rad, yaw ±0.15 rad.
- World initial velocity ±0.2 m/s per component; angular velocity ±0.2 rad/s.
- Effective mass/dry mass/inertia share a body factor 0.97–1.03. Body and hull
  volumes share a separate factor 0.97–1.03. Hull drag/skin share 0.7–1.4 per body.
- Each motor's thrust/torque share 0.8–1.2; rotor inertia uses 0.7–1.5.
- Horizontal current speed 0–0.15 m/s with uniform direction.
- Gate opening and frame move as one world-space rigid group. Marker placement
  varies independently; floor stays fixed. Custom scenes can specify groups and
  mark independent static objects with `UnderwaterBody.fuzz`.
- One active motor is dead (25%) or weak (otherwise, magnitude 0.3–0.7), starting
  at an inclusive random tick 36–360 for 180 ticks.
- One available routed tracking/IMU/camera outage, gyro-X bias, or depth bias is
  selected uniformly. Interval is `[360,720)`; bias is ±0.05 rad/s or ±0.05 m.

Existing faults are retained. IDs, topology, geometry, mounts, motor signs,
sensor rates, and controller calibration remain fixed. Invalid ranges/groups,
capacity exhaustion, or invalid generated scenes fail that case without retries.
A fault-heavy profile is intended to discover failures; it does not guarantee
that every mission succeeds.

## Reports and reproducibility

Each `case-NNNNNN.json` includes seed/index, profile/version, resolved numeric
scene values, faults, mission stage, outcome/error/detail, tick, hash, depth
bounds, and available final robot state. Provenance includes robot asset SHA-256
and hashes of the adapter, core, backend, running Zig binary, and captured config.
JSON does not contain model pointers, intrusive links, or SIMD padding.

`summary.jsonl` is ordered by case index, independent of completion order;
`totals.json` counts passed/failed/cancelled cases. Individual failures do not stop
the batch. Failures, startup errors, output errors, or interruption yield a nonzero
exit. Unassigned cases on cancellation receive explicit cancelled records.

Seed replay requires the same compiled profile, baseline, controller config, and
simulator build. Compare provenance before interpreting a hash difference. It is
not a portable cross-version/cross-platform replay format. Resolved values are
recorded for diagnosis; importing modified JSON case files is not implemented.

## Validation commands

```sh
cmake -S . -B build/simulator -DAUTONOMY_BUILD_SIMULATOR=ON \
  -DSIMULATOR_ENABLE_STONEFISH=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build/simulator --target simulator_all -j8
ctest --test-dir build/simulator --output-on-failure
zig build test
```

The suite compares full cases with one and eight workers, seed replay, nominal
completion, budget/init failures, cancellation, and injected partial worker
startup. A native eight-thread harness repeatedly creates/destroys worlds and
checks exact per-case hashes, borrowed-model lifetime, freeze checks, and stable
allocation counts after warm-up. The header compiles independently in C11/C++23
and its extension layouts are checked from C/Zig.

ASan/UBSan/leak and ThreadSanitizer builds are separate. See `VALIDATION.md` for
commands, measured results, and the native-engine instrumentation qualification.
