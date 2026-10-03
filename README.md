## Build & Run: libauv_stationary_sim
```sh
zig build run -- \
    ./zig-out/lib/libauv_stationary_sim.so \
    prequalify \
    ./auvs/stationary_sim/auv.json
```

## Build & Run: libauv_hydrus_sim

`auvs/hydrus_sim/platform.json` selects the vehicle with `robot`: `hydrus`, `proteus`, `bluerov2`, or `girona500`. Pass the matching controller file.

```sh
zig build -Dstonefish run -- \
    ./zig-out/lib/libauv_hydrus_sim.so  \
    prequalify \
    ./auvs/hydrus_sim/hydrus.json
```

## Hydrus MuJoCo

Requires MuJoCo and GLFW. `MUJOCO_PREFIX` is an install prefix (`include/` and `lib/`) or a MuJoCo source tree with `build/lib/libmujoco.so`.

```sh
zig build -Dmujoco -DMUJOCO_PREFIX=/usr/local
./zig-out/bin/src_3 ./zig-out/lib/libauv_hydrus_mujoco.so prequalify auvs/hydrus_mujoco/auv.json
```

Set `HYDRUS_MUJOCO_HEADLESS=1` to skip the GLFW viewer. In the viewer, left drag orbits, right drag pans, and scroll zooms.

The plugin reproduces the Stonefish Hydrus physics: shell-part mass with Stonefish added mass and inertia, buoyancy at the center of buoyancy, per-face form drag and skin friction recomputed at 50 Hz, and fluid-dynamics thrusters driven through the mechanical PI rotor model. The hull constants in `hydrus.xml` come from Stonefish 1.5 for `hydrus_auv.scn`. The physics lives in `hydrus_core.{h,cpp}`, shared by the plugin and the RL library below.

## Hydrus RL

`-Dmujoco` also builds `zig-out/lib/libhydrus_mujoco_rl.so`: many Hydrus vehicles with the same physics, stepped in parallel through a C API (`auvs/hydrus_mujoco/hydrus_batch.h`). `rl/` trains a PPO policy on that library. It gets a pose goal and outputs the 6-DOF TAM command at 60 Hz, using only the frame pose and timestamp.

```sh
pip install -r rl/requirements.txt
cd rl
python train_ppo.py --name pose_ppo
```
