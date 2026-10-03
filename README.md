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
