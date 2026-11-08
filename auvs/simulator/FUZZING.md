# Seeded MuJoCo mission fuzzing

Configure `.fuzz` in `config/application.zig` and build normally:

```sh
zig build -Dsimulator=true
./zig-out/bin/src_3 fuzz
./zig-out/bin/src_3 replay 10451216379200822465
./zig-out/bin/src_3 replay build/my-campaign/case-000000.bin
./zig-out/bin/src_3 inspect build/my-campaign/totals.bin
```

Defaults: master seed 1, 100 cases, eight dedicated OS threads, Hydrus, Robosub,
21,600 physics ticks, six ticks per frame. Set a fresh `.fuzz.results` directory for
every campaign/replay. Operational settings come from compiled Zig. Teleop is
rejected. Fuzzing disables pacing, hot reload, telemetry, and verbose frame logging.

Each worker creates its own stable-address C baseline, mutable case clone, Zig
arena, RNG, simulator world, and MissionContext. Native adapter state remains
thread-local; no mutable controller or scene data crosses worlds. The coordinator
pins native libraries and the application generation until all workers join.
Cancellation is checked between frames; partial worker startup joins every started
thread. Normal and batch runs use the same frame-driven mission/controller code.

## Generator contract and ranges

`SceneContext.zig` exposes `randomize(std.Random, FuzzProfile)` for mutable scenes.
Profiles are typed Zig values configured through config/application.zig.
Profile/generator version is 1. Case seeds use:

```
case_seed = splitmix64(master_seed +% index *% 0x9e3779b97f4a7c15)
```

`splitmix64` includes the standard additive step. Every case initializes an
explicit `std.Random.Xoshiro256` from that seed. Draw order is dynamic objects in
scene order, placement groups, marked standalone scenery, current, motor slots,
actuator fault, and sensor fault. Do not change it without versioning the profile.

Default variations relative to the baseline:

- Dynamic position X/Y ±0.2 m, Z ±0.1 m; local roll/pitch ±1.05 rad, yaw ±1.15 rad.
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

## Binary results and replay

All results use AUVDATA1 binary data: `campaign.bin`, `case-NNNNNN.bin`, ordered
`summary.bin`, and `totals.bin`. Per-case records contain resolved scene values,
profile/configuration, seed, outcome/error/stage, tick/hash, depth bounds, and final
state. Provenance includes application, host, core, plugin, backend, and asset hashes.
The format serializes fields and IDs, excluding pointers and alignment padding.

Workers write distinct files and finalize them by rename. Existing campaign/output
names are not overwritten. Failures do not stop other cases; failures, cancellation,
startup, and output errors produce a nonzero exit. Cancellation records unassigned
cases too. Inspect results with `src_3 inspect` or `python tools/data_binary.py`.

Seed replay uses current configuration. File replay restores recorded configuration
and profile, keeping the current output directory. Matching trajectories/hashes
requires the same mission/controller code, native simulator build, and baseline
assets; reports retain provenance to diagnose differences. Changed broad profiles
can legitimately fail within the tick budget.

See [application reload and binary data](../../docs/APPLICATION_RELOAD.md) for the
binary schema, ownership, commands, compatibility changes, and validation. All
validation for this delivery is MuJoCo-only; Stonefish is not built or executed.
