# Autonomy

## Test and build

```sh
zig build test
zig build
```

## Stonefish mission

```sh
zig build -Dstonefish
zig build -Dstonefish run -- \
    ./zig-out/lib/libauv_stonefish_sim.so \
    prequalify_robosub \
    stonefish_sim
```

## Viewer

```sh
zig build viewer
./zig-out/bin/viewer \
    --shm autonomy_telemetry \
    --video none \
    --teleop-port 8080 \
    --profile stonefish_sim
```

```sh
./zig-out/bin/viewer \
    --udp 5600 \
    --video none \
    --teleop-port 8080 \
    --profile stonefish_sim
```

```sh
zig build viewer -Dviewer-zed
./zig-out/bin/viewer \
    --video 127.0.0.1:5610 \
    --shm autonomy_telemetry \
    --teleop-port 8080 \
    --profile stonefish_sim
```

## Teleop

```sh
zig build teleop-sender
./zig-out/bin/teleop_sender 8080
```

## Proteus hardware

```sh
zig build -Dzed
zig build -Dzed run -- \
    ./zig-out/lib/libauv_proteus_hwd.so \
    prequalify_coco \
    proteus_hwd
```
