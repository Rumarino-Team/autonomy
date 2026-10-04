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


## MuJoCo

Requires MuJoCo. `MUJOCO_PREFIX` is an install prefix (`include/` and `lib/`) or a MuJoCo source tree with `build/lib/libmujoco.so`. The window is Stonefish's ocean view, built with the in-tree Stonefish library and loaded from `libauv_mujoco_view.so` beside the plugin.

```sh
zig build -Dmujoco -DMUJOCO_PREFIX=/usr/local
./zig-out/bin/src_3 ./zig-out/lib/libauv_mujoco.so prequalify auvs/hydrus_mujoco/auv.json
```

`AUV_MJCF` selects the model. The default is `auvs/hydrus_mujoco/hydrus.xml`. `proteus.xml` and `bluerov.xml` use the same physics. The window loads the matching Stonefish robot: Hydrus from `open_space_env.scn`, Proteus from `open_space_proteus.scn`, and BlueROV2 from `pool_bluerov2.scn`. Set `HYDRUS_MUJOCO_HEADLESS=1` to skip it.

## Hydrus RL

```sh
pip install -r rl/requirements.txt
cd rl
python train_ppo.py --name pose_ppo
```
