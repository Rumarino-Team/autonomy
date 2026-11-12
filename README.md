# Autonomy

Zig-based AUV runtime, missions, telemetry, viewer, and simulator plugins.

## Dependencies

All commands need Zig `0.17+`, a C/C++ toolchain, libc, `pkg-config`, and
network access on the first build so Zig can fetch `raylib-zig`.

Ubuntu packages used by the base build and tests:

```sh
sudo apt install build-essential pkg-config \
  libsdl2-dev libgl1-mesa-dev libx11-dev libxcursor-dev \
  libxinerama-dev libxrandr-dev libxi-dev
```

## Build and test

```sh
zig build
zig build test
```

Useful build options: `-Dzed`, `-Dstonefish`, `-Dviewer-zed`, and
`-Dviewer-gstreamer`.

## Run missions

### Proteus/ZED

Requires the ZED SDK and CUDA Toolkit. The default locations are
`/usr/local/zed` and `/usr/local/cuda`; override them with
`-Dzed-prefix=...` and `-Dcuda-prefix=...`.

```sh
zig build -Dzed
zig build -Dzed run -- \
  ./zig-out/lib/libauv_proteus_hwd.so prequalify_coco proteus_hwd
```

### Stonefish

Requires CMake `3.20+`, OpenMP, and development libraries for SDL2, FreeType,
OpenGL, GLU, and Gomp. Stonefish itself is included under `vendor/stonefish`.

```sh
sudo apt install cmake libfreetype6-dev libglu1-mesa-dev
zig build -Dstonefish
zig build -Dstonefish run -- \
  ./zig-out/lib/libauv_stonefish_sim.so prequalify_robosub stonefish_sim
```

## Viewer

The default viewer requires a desktop OpenGL/X11 or Wayland environment and
the raylib dependencies used by `raylib-zig`.

```sh
zig build viewer
./zig-out/bin/viewer --video none
```

### ZED video

Requires the ZED SDK, CUDA Toolkit, and a C++ compiler. The defaults are the
same `/usr/local/zed` and `/usr/local/cuda` locations used above.

```sh
zig build viewer -Dviewer-zed
./zig-out/bin/viewer --video 127.0.0.1:5610
```

### GStreamer RTP video

Requires GStreamer development packages plus the RTP, H.264, video, and
appsink plugins. Software decoding uses `avdec_h264`; `nvv4l2decoder` is
optional hardware decoding on Jetson.

```sh
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-libav
zig build viewer -Dviewer-gstreamer
./zig-out/bin/viewer --video rtp://0.0.0.0:5612
```

## Teleop

Requires SDL2 development files and a graphical desktop session.

```sh
zig build teleop-sender
./zig-out/bin/teleop_sender 8080
```
