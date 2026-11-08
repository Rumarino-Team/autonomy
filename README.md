## Build & Run: libauv_stationary_sim

```sh
zig build run -- \
    ./zig-out/lib/libauv_stationary_sim.so \
    prequalify_coco \
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

## Stonefish

Stonefish is a backend inside the data-oriented simulator, not a separate AUV plugin. Enable it with `-DSIMULATOR_ENABLE_STONEFISH=ON` when configuring `auvs/simulator`. The pool scene is `auvs/simulator/data/pool_scene.blend`.

## Build & Run: Proteus hardware (ZED 2i + Arduino)

```sh
zig build -Dzed
zig build -Dzed run -- \
    ./zig-out/lib/libauv_proteus_hwd.so \
    prequalify_coco \
    ./auvs/proteus_hwd/auv.json
```




## Data-oriented simulator core

The opt-in C ABI simulator, custom allocators, MuJoCo backend, and
programmatic examples and four robot presets are documented in
[auvs/simulator/README.md](auvs/simulator/README.md).
Build its dedicated target with `-DAUTONOMY_BUILD_SIMULATOR=ON` and
`cmake --build build/simulator --target simulator_all`. Blender import and
rendered images are deferred. The scene-defined sensor layer and Zig AUV bridge
are documented in [auvs/simulator/SENSORS.md](auvs/simulator/SENSORS.md).

MuJoCo-only seeded parallel campaigns and replay are documented in
[auvs/simulator/FUZZING.md](auvs/simulator/FUZZING.md).
