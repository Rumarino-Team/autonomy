## Build & Run: libauv_hydrus_sim


## Build & Run: libauv_stationary_sim
```sh
zig build run -- \
    ./zig-out/lib/libauv_sleep_sim.so \
    prequalify \
    ./auvs/sleep_sim/auv.json
```


```sh
zig build -Dstonefish run -- \
    ./zig-out/lib/libauv_hydrus_sim.so  \
    prequalify \
    ./auvs/hydrus_sim/auv.json
```
