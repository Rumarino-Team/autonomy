## Build & Run: libauv_stationary_sim
```sh
zig build run -- \
    ./zig-out/lib/libauv_stationary_sim.so \
    prequalify \
    ./auvs/stationary_sim/auv.json
```

## Build & Run: libauv_hydrus_sim

```sh
zig build -Dstonefish run -- \
    ./zig-out/lib/libauv_hydrus_sim.so  \
    prequalify \
    ./auvs/hydrus_sim/auv.json
```

## Build & Run: libauv_proteus_sim

```sh
zig build -Dstonefish run -- \
    ./zig-out/lib/libauv_proteus_sim.so \
    prequalify \
    ./auvs/proteus_sim/auv.json
```

## Build & Run: libauv_bluerov2_sim

```sh
zig build -Dstonefish run -- \
    ./zig-out/lib/libauv_bluerov2_sim.so \
    prequalify \
    ./auvs/bluerov2_sim/auv.json
```

## Build & Run: libauv_girona500_sim

```sh
zig build -Dstonefish run -- \
    ./zig-out/lib/libauv_girona500_sim.so \
    prequalify \
    ./auvs/girona500_sim/auv.json
```
