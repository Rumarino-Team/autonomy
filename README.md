## Build
```sh
zig build
```

## Build & Run
```sh
zig build run -- \
    ./zig-out/lib/libauv_sleep_sim.so \
    prequalify \
    ./auvs/sleep_sim/auv.json
```

## Hydrus sim
Requires a Stonefish install. From the repo root:

```sh
zig build -Dhydrus -Dstonefish-prefix=/usr/local
./zig-out/bin/src_3 ./zig-out/lib/libauv_hydrus_sim.so prequalify auvs/hydrus_sim/auv.json
```

## Hydrus MuJoCo
Requires MuJoCo and GLFW. `mujoco-prefix` is an install prefix (`include/` and `lib/`) or a MuJoCo source tree with `build/lib/libmujoco.so`. From the repo root:

```sh
zig build -Dmujoco -Dmujoco-prefix=/usr/local
./zig-out/bin/src_3 ./zig-out/lib/libauv_hydrus_mujoco.so prequalify auvs/hydrus_mujoco/auv.json
```

Set `HYDRUS_MUJOCO_HEADLESS=1` to skip the GLFW viewer. In the viewer, left drag orbits, right drag pans, and scroll zooms.

The plugin reproduces the Stonefish Hydrus physics: shell-part mass with Stonefish added mass and inertia, buoyancy at the center of buoyancy, per-face form drag and skin friction recomputed at 50 Hz, and fluid-dynamics thrusters driven through the mechanical PI rotor model. The hull constants in `hydrus.xml` come from Stonefish 1.5 for `hydrus_auv.scn`. The physics lives in `hydrus_core.{h,cpp}`, shared by the plugin and the RL library below.

## Hydrus RL (low-level control)
`-Dmujoco` also builds `zig-out/lib/libhydrus_mujoco_rl.so`: many Hydrus vehicles with the same physics, stepped in parallel through a C API (`auvs/hydrus_mujoco/hydrus_batch.h`, about 22k policy steps/s on 16 threads). `rl/` trains a PPO policy that replaces the global-navigation PID: it gets a pose goal and outputs the 6-DOF TAM command at 60 Hz, using only the frame pose and timestamp.

```sh
pip install -r rl/requirements.txt
cd rl
python train_ppo.py --name pose_ppo               # ~45 min for 30M steps on 16 cores
python export_policy.py runs/pose_ppo             # writes auvs/hydrus_mujoco/policy.bin
python eval.py                                    # policy vs the host PID on random goals
```

Training randomizes mass, buoyancy, drag, thrust, rotor inertia and water current. Setting `"policy"` in the live config makes `src_3` use the policy (`src/policy.zig`) instead of the PID; removing it switches back on the next hot reload. The same file drives either simulator:

```sh
./zig-out/bin/src_3 ./zig-out/lib/libauv_hydrus_mujoco.so prequalify auvs/hydrus_mujoco/auv_policy.json
./zig-out/bin/src_3 ./zig-out/lib/libauv_hydrus_sim.so prequalify auvs/hydrus_mujoco/auv_policy.json
```

`rl/pose_env.py` `build_observation()` and `src/policy.zig` `observation()` must stay identical.
