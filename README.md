
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

MuJoCo 3.x with mjSpec is required. `MUJOCO_PREFIX` is an install prefix (`include/` and `lib/`) or a source tree with `build/lib/libmujoco.so`.

Pool and vehicle **mesh geometry** come from `auvs/stonefish_sim/data/pool_scene.blend` at runtime (CBlend reader in `auvs/blender_scene_cpp/`). MJCF files under `auvs/hydrus_mujoco/` are mechanism skeletons only. Configure paths in [`auvs/hydrus_mujoco/platform.json`](auvs/hydrus_mujoco/platform.json) or with `AUV_BLEND`, `AUV_MJCF`, and `AUV_PLATFORM`. Saving the `.blend` triggers a debounced hot reload (200 ms).

Place `pool_scene.blend` next to the Stonefish data tree (same file as the experimental branch). Inspect a blend without running the sim:

```sh
cmake --build build --target mujoco_blend_inspect
./build/auvs/blender_scene_cpp/mujoco_blend_inspect \
    auvs/stonefish_sim/data/pool_scene.blend auvs/stonefish_sim/stonefish_config.yaml
```

Set `HYDRUS_MUJOCO_HEADLESS=1` for no Blender UDP. `AUV_REALTIME_FACTOR` caps live wall-clock speed (default 4); use `0` for uncapped.

```sh
zig build -Dmujoco -DMUJOCO_PREFIX=/path/to/mujoco
AUV_MJCF=auvs/hydrus_mujoco/hydrus.xml HYDRUS_MUJOCO_HEADLESS=1 \
    ./zig-out/bin/src_3 ./zig-out/lib/libauv_mujoco.so prequalify auvs/hydrus_mujoco/auv.json
```

Compose smoke test (from repo root, after `cmake` configure with `-DMUJOCO_PREFIX=...`):

```sh
cmake --build build --target mujoco_compose_smoke && ./build/mujoco_compose_smoke
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
