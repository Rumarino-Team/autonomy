# Experimental integration

Initialize dependencies with `git submodule update --init --recursive`. Rebuild the simulator and the Zig app together; the frame layout is ABI 3.

Match the controller file to the robot. `auvs/stonefish_sim/platform.json` selects the Stonefish vehicle with `robot`: `hydrus`, `proteus`, `bluerov2`, or `girona500`. Use the matching file under `auvs/stonefish_sim/`. For MuJoCo, set `AUV_MJCF` and use the matching file under `auvs/hydrus_mujoco/`. `auv.json` is the Hydrus controller.

## Stationary simulator

```sh
zig build run -- \
    ./zig-out/lib/libauv_stationary_sim.so \
    prequalify \
    ./auvs/stationary_sim/auv.json
```

Teleop, then the sender in another terminal:

```sh
zig build teleop-sender
zig build run -- \
    ./zig-out/lib/libauv_stationary_sim.so \
    teleop \
    ./auvs/stationary_sim/auv.json
```

```sh
./zig-out/bin/teleop_sender 8080
```

## Stonefish

```sh
zig build -Dstonefish run -- \
    ./zig-out/lib/libauv_stonefish_sim.so \
    prequalify \
    ./auvs/stonefish_sim/proteus.json
```

Blender scenes load through the same `-Dstonefish` build. See [native Blender loader](auvs/stonefish_sim/blender_stonefish_cpp/README.md).

Simulated tracking loss starts at 1000 s and stays on. Override it, or set the duration to 0 to disable it:

```sh
AUV_SIM_TRACKING_LOSS_START=5 AUV_SIM_TRACKING_LOSS_DURATION=3 \
    ./zig-out/bin/src_3 ./zig-out/lib/libauv_stonefish_sim.so prequalify \
    ./auvs/stonefish_sim/proteus.json
```

## MuJoCo

`MUJOCO_PREFIX` is an install prefix (`include/` and `lib/`) or a source tree with `build/lib/libmujoco.so`. The default model is `auvs/hydrus_mujoco/proteus.xml`. `proteus.xml`, `hydrus.xml`, and `bluerov.xml` use the same physics. The view is Blender: the simulator sends poses over UDP to `127.0.0.1:9760` (`AUV_BLENDER_PORT` overrides the port). The pool comes from `auvs/stonefish_sim/data/pool_scene.blend` through `pool_scene.usd` and `pool_world.xml`. Regenerate those with `blender auvs/stonefish_sim/data/pool_scene.blend --background --python auvs/hydrus_mujoco/export_pool_usd.py`. Install `auvs/hydrus_mujoco/blender_addon`, open `pool_scene.blend`, and press Start link. Eevee and Cycles are whichever engine that file uses. Set `HYDRUS_MUJOCO_HEADLESS=1` to send nothing.

```sh
zig build -Dmujoco -DMUJOCO_PREFIX=/usr/local
AUV_MJCF=auvs/hydrus_mujoco/hydrus.xml \
    ./zig-out/bin/src_3 ./zig-out/lib/libauv_mujoco.so prequalify auvs/hydrus_mujoco/auv.json
```

### Recorded takes

`record` runs one seed on the simulator clock until the mission returns, and writes that run. The same seed on the same build gives the same recording. Samples are written at 24 fps.

```sh
AUV_MJCF=auvs/hydrus_mujoco/hydrus.xml \
    ./zig-out/bin/src_3 ./zig-out/lib/libauv_mujoco.so record auvs/hydrus_mujoco/auv.json \
    1 take_000.rec.jsonl
```

The recording header carries that seed. Poses in each sample are xyz plus quat xyzw in the Blender frame.

In Blender, the Hydrus sidebar has a Takes panel. Record run writes `//hydrus_takes/take_NNN.rec.jsonl` for the seed in the panel and bakes the finished file into keyframes on `hydrus`, `camera`, `Gate`, and `Marker`, a `path_take_NNN` curve, and timeline markers for the mission phases. Render Animation then renders the run; Onboard camera renders through `hydrus_camera`.

The add-on finds `zig-out/bin/src_3`, the plugin, `auv.json`, and `hydrus.xml` through the repository it sits in. Set them in the add-on preferences if it is installed elsewhere.

Hydrus, Proteus, BlueROV2, and Girona 500 propellers animate through the same add-on. Live poses estimate spin from translation and turning; imported recordings bake each stored thruster command into rotor keyframes, including reverse thrust and spin-down. `rotor_visual_rpm` on each vehicle root controls the presentation speed (210 RPM by default so blades remain visible at 24 fps). Restart Blender once after updating the add-on. This animation changes appearance only.

PID tuning (`libauv_mujoco_rl.so` and `hydrus_tune_pid` come from the same build):

```sh
./zig-out/bin/hydrus_tune_pid --randomize
```

Options: `--episodes`, `--seconds`, `--generations`, `--population`, `--elite`, `--seed`, `--threads`, `--config`, `--xml`, `--randomize`. `AUV_MJCF` overrides the default MJCF (`hydrus.xml` for the tuner).

## Voice commands

Run `teleop_command`, then the voice demo. Raw JSON commands need no speech credentials. Command schemas are in [docs/teleop_commands.md](docs/teleop_commands.md).

```sh
python demo/voice.py --mission --push-to-send
```

## Tests

```sh
zig build test
```
