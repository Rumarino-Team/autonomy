
## Build & Run: libauv_stationary_sim
```sh
zig build run -- \
    ./zig-out/lib/libauv_stationary_sim.so \
    prequalify \
    ./auvs/stationary_sim/auv.json
```

## Build & Run: libauv_stonefish_sim

```sh
zig build -Dstonefish run -- \
    ./zig-out/lib/libauv_stonefish_sim.so \
    prequalify \
    ./auvs/stonefish_sim/auv.json
```

## MuJoCo

`MUJOCO_PREFIX` is an install prefix (`include/` and `lib/`) or a source tree with `build/lib/libmujoco.so`. Default model: `auvs/hydrus_mujoco/proteus.xml` (override with `AUV_MJCF`). Set `HYDRUS_MUJOCO_HEADLESS=1` for no Blender UDP. `AUV_REALTIME_FACTOR` caps live wall-clock speed (default 4); use `0` for uncapped.

```sh
zig build -Dmujoco -DMUJOCO_PREFIX=/path/to/mujoco
AUV_MJCF=auvs/hydrus_mujoco/hydrus.xml HYDRUS_MUJOCO_HEADLESS=1 \
    ./zig-out/bin/src_3 ./zig-out/lib/libauv_mujoco.so prequalify auvs/hydrus_mujoco/auv.json
```

Parallel fuzz and record (headless, simulator clock):

```sh
HYDRUS_MUJOCO_HEADLESS=1 AUV_REALTIME_FACTOR=0 AUV_MJCF=auvs/hydrus_mujoco/hydrus.xml \
    ./zig-out/bin/src_3 ./zig-out/lib/libauv_mujoco.so fuzz auvs/hydrus_mujoco/auv.json 1 4

HYDRUS_MUJOCO_HEADLESS=1 AUV_REALTIME_FACTOR=0 AUV_MJCF=auvs/hydrus_mujoco/hydrus.xml \
    ./zig-out/bin/src_3 ./zig-out/lib/libauv_mujoco.so record auvs/hydrus_mujoco/auv.json 1 take.rec.jsonl
```

Pair each `AUV_MJCF` with the matching JSON (`auv.json`, `proteus.json`, `bluerov.json`).

## Tests

```sh
zig build test
```
