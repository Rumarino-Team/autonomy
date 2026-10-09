## Build & Run: libauv_stationary_sim
```sh
zig build run -- \
    ./zig-out/lib/libauv_stationary_sim.so \
    prequalify \
    ./auvs/stationary_sim/auv.json
```

## Build & Run: libauv_stonefish_sim

`auvs/stonefish_sim/platform.json` selects the vehicle with `robot`: `hydrus`, `proteus`, `bluerov2`, or `girona500`. Pass the matching controller file.

```sh
zig build -Dstonefish run -- \
    ./zig-out/lib/libauv_stonefish_sim.so  \
    prequalify \
    ./auvs/stonefish_sim/hydrus.json
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

Prints tuned `kp` / `ki` / `kd` for pasting into `auv.json` (only `"tam"` is read from that file; gains are searched from scratch). Source: `auvs/hydrus_mujoco/tune_pid.cpp`.
