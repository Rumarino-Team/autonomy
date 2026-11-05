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

## Build & Run: libauv_stonefish_sim

```sh
zig build -Dstonefish run -- \
    ./zig-out/lib/libauv_stonefish_sim.so  \
    prequalify_robosub \
    ./auvs/stonefish_sim/auv.json
```

## Build & Run: Proteus hardware (ZED 2i + Arduino)

```sh
zig build -Dzed
zig build -Dzed run -- \
    ./zig-out/lib/libauv_proteus_hwd.so \
    prequalify_coco \
    ./auvs/proteus_hwd/auv.json
```




## Portable physics backends

Build the shared simulation runtime and both backends with `zig build sim`; run qualification with `zig build sim-test`. See [the simulation guide](sim/README.md) for backend selection, portable scenarios, custom force equations, deterministic replay, and physics/mission fuzzing.

## Blender simulation

The pool scene lives at `auvs/stonefish_sim/data/pool_scene.blend`. MuJoCo and Stonefish both use the same authoring scene; the add-on runs the `prequalify_robosub` mission and drives live preview in Blender. Full details are in the [Blender guide](tools/blender/README.md).

One-time setup from the repo root:

```sh
zig build sim
zig build
python3 tools/blender/package_addon.py
```

In Blender 4.5+, install `build/portable/autonomy_sim.zip` (Preferences → Add-ons → Install from Disk), enable **Autonomy Simulation**, and set **Repository** to this checkout. In the 3D View sidebar, open **Autonomy**, click **Load pool scene** (or open the `.blend` directly), choose **MuJoCo** or **Stonefish**, then **Run live** for real-time preview and a baked take. **Record run** runs the same mission as fast as possible. **Cancel** stops an in-flight run.

To start from a terminal while Blender listens (**Start link** in the panel):

```sh
python3 tools/blender/from_blender.py \
  auvs/stonefish_sim/data/pool_scene.blend \
  --scene AutonomySource --frame 1 --backend mujoco \
  --blender-link --realtime
```

Use `--backend stonefish` for the other engine. Refresh the checked-in pool snapshot at `sim/scenarios/pool.asim` when the authored scene changes.

### Live poses in Blender

The sim plugin streams body and camera transforms over **UDP** on `127.0.0.1` (default port **9760**). Blender must be listening first (**Run live** in the add-on, or **Start link** then launch from the terminal).

**Wrapper** (`from_blender.py` sets the env vars for `src_3`):

| Flag | Effect |
|------|--------|
| `--blender-link` | Send UDP pose packets each tick (`AUTONOMY_SIM_BLENDER=1`) |
| `--realtime` | Pace the loop to wall clock (`AUTONOMY_SIM_REALTIME=1`) |
| `--port N` | UDP port (default `9760`; must match the add-on **Port** preference) |

Example (live preview):

```sh
python3 tools/blender/from_blender.py \
  auvs/stonefish_sim/data/pool_scene.blend \
  --scene AutonomySource --frame 1 --backend mujoco \
  --blender-link --realtime --port 9760
```

**Direct** (same mission, after `zig build sim` and `zig build`):

```sh
export AUTONOMY_SIM_SCENARIO=auvs/stonefish_sim/data/pool_scene.blend
export AUTONOMY_SIM_BACKEND=$PWD/build/portable/sim/libsim_backend_mujoco.so
export AUTONOMY_SIM_BLENDER=1
export AUTONOMY_SIM_BLENDER_PORT=9760
export AUTONOMY_SIM_REALTIME=1   # omit for fastest headless run (no live pacing)
./zig-out/bin/src_3 ./build/portable/sim/libauv_sim.so \
  prequalify_robosub ./sim/scenarios/controller.json
```

Pose streaming does not require `--realtime`; **Run live** turns both on. **Record run** leaves them off and writes a trace only. Fuzz and CI runs keep Blender output disabled by default.
