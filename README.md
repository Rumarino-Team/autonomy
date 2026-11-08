# Autonomy mission runner

The executable is a small host that reloads the Zig application shared library.
MissionContext, missions, controllers, scene setup, configuration, and fuzzing live
in that library. Edit `config/application.zig` or application code; the running host
rebuilds it and optionally restores mission progress while keeping MuJoCo alive.

```sh
zig build -Dsimulator=true       # package MuJoCo and native simulator dependencies
zig build                      # normal build, including default Zig configuration
./zig-out/bin/src_3 run
```

Configuration is Zig source, with no JSON parsing or `-Dconfig` option. Defaults are
Hydrus, MuJoCo, Robosub prequalification, unpaced simulation, and state-preserving
reload. Set `.reload.preserve_state = false` for a fresh world on reload.
Structural scene/timing/backend changes require a fresh world. Host/ABI and native
plugin code changes require process restart. Fuzz campaigns pin their generation.

Configuration examples, ownership, state migration, binary formats, build/run
commands, and tests: [application reload guide](docs/APPLICATION_RELOAD.md).

## Seeded MuJoCo fuzzing

Set `.fuzz` in `config/application.zig`: master seed, cases, workers, profile, and a
fresh results directory. Default rotation variation remains ±1.05/1.05/1.15 radians.

```sh
./zig-out/bin/src_3 fuzz
./zig-out/bin/src_3 replay 10451216379200822465
./zig-out/bin/src_3 replay build/my-campaign/case-000000.bin
./zig-out/bin/src_3 inspect build/my-campaign/totals.bin
```

Reports are `campaign.bin`, per-case binary files, `summary.bin`, and `totals.bin`.
Failures are recorded and the campaign continues. Ctrl+C joins workers and records
cancellation. Details: [fuzzing](auvs/simulator/FUZZING.md).

## Stationary plugin, hardware, and teleop

In `config/application.zig`, choose `.plugin = "libauv_stationary_sim.so"`,
`.controller = @import("stationary.zig").controller`, and `.mission = .teleop`
(or `.prequalify_coco`). Then use ordinary `zig build` and `src_3 run`.
For Proteus hardware select `libauv_proteus_hwd.so` and `proteus.zig`; retain
`zig build -Dzed` for the ZED SDK/CUDA plugin.

Optional build steps remain `zig build arduino`, `zig build arduino-sender`,
`zig build teleop-sender`, and `zig build fuzz-startup-test`. `-Dexternal-asan=true`
lets preloaded ASan own signal stacks in instrumented-native tests. Native Windows
builds retain the existing compiler wrappers and DLL dependency search support.

Run the optional sender in another terminal: `./zig-out/bin/teleop_sender 8080`.

## Build & Run: viewer

The viewer needs the ZED SDK and CUDA. CMake fetches raylib.

```sh
cmake -S tools/viewer -B tools/viewer/build
cmake --build tools/viewer/build
./tools/viewer/build/viewer
```

It listens for binary telemetry on UDP 5600. The ZED H264 stream at `127.0.0.1:5610` is optional: pass `none` to follow simulation telemetry without a camera. With the window focused, it sends the teleop command to `127.0.0.1:8080`.

```sh
./tools/viewer/build/viewer [telemetry_port] [video] [teleop_port]
./tools/viewer/build/viewer 5600 none
```

Hold keys to command a body translation direction: W/S surge, A/D sway, R/F heave. Left/Right yaws about world +Z; Up/Down pitches about body X; I/K rolls about body +Y. Each packet is 28 bytes: three little-endian floats of body translation, then a world orientation quaternion `x, y, z, w`. Identity is level with the camera along body +Y. Releasing the translation keys leaves translation uncommanded. The orientation keys still apply.

Simulated missions stream the same telemetry format to `127.0.0.1:5600` when telemetry is enabled (disabled during fuzz runs).

## Blender and simulator core

Set `.scene` to a saved `.blend` or binary scene snapshot and
`.blender = .{ .enabled = true, .port = 9760 }` for the existing binary live pose link.
Enable `.realtime` for wall-clock pacing. Exported scene data is binary; embedded
calibration and saved assets are retained. The Blender add-on uses the compiled
application settings; it does not override configuration with JSON or environment
variables.

The C ABI core, custom arenas/pools, robot presets, and native examples are described
in [simulator documentation](auvs/simulator/README.md); the sensor interface is in
[sensor documentation](auvs/simulator/SENSORS.md). These native interfaces remain
available independently of the reloadable application.

This delivery builds and tests MuJoCo only. Existing Stonefish sources remain
behind the opt-in CMake option and are outside application-reload validation.
