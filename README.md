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

The viewer and runtime use the compiled `viewer_profile` in `src/MissionConfig.zig`;
their telemetry transport and control port must match. The viewer accepts `--video`
and `--mission-control`. Use `--video none` to disable video. The default build
uses a video stub:

```sh
zig build viewer
./zig-out/bin/viewer --video none
```

### Standard RTP viewer (no ZED SDK on the viewer)

The GStreamer backend receives **side-by-side H.264 RTP**, payload type 96, over
UDP and displays the left half. It does not link the ZED SDK or CUDA. On a Jetson
it tries `nvv4l2decoder` with `nvvidconv`; otherwise it uses `avdec_h264` and
`videoconvert`. Decoded pixels are copied to CPU memory and uploaded to raylib;
this is hardware decoding, not a zero-copy display path.

Install GStreamer development packages `libgstreamer1.0-dev` and
`libgstreamer-plugins-base1.0-dev` on the build machine. Runtime plugins must
provide `udpsrc`, `rtpjitterbuffer`, `rtph264depay`, `h264parse`, `appsink`, and
the selected decoder/converter. On Ubuntu these are typically in the GStreamer
base, good, bad, and libav plugin packages; Jetson hardware plugins come from
JetPack. Check them with `gst-inspect-1.0`.

```sh
zig build viewer -Dviewer-gstreamer
./zig-out/bin/viewer --video rtp://0.0.0.0:5612
```

The address in `--video` is the **local listen address**, not the camera IP.
The default for a GStreamer viewer is `rtp://0.0.0.0:5612`. For a different
port, use the same port in the sender destination. The viewer can recover after
the sender stops, restarts, or changes resolution; it waits for a keyframe.

On the camera-side Jetson, ZED SDK **5.4 or newer**, GStreamer development
packages, and an SDK-supported H.264 hardware encoder are required. For a
viewer running on the same machine, build the ZED plugin with the optional
relay and send video to the local loopback address:

```sh
zig build -Dzed -Dzed-rtp
AUV_VIDEO_RTP_DEST=127.0.0.1:5612 ./zig-out/bin/src_3 \
    ./zig-out/lib/libauv_proteus_hwd.so prequalify_coco proteus_hwd
```

When the viewer runs on a different computer, replace `127.0.0.1` with that
computer's reachable IPv4 address (for example, find it with `hostname -I`
on the viewer computer). `0.0.0.0` is a listen address, not a destination.
The relay reuses packets from the ZED SDK's
H.264 encoder; it does not open the camera or encode a second time. Standard
RTP is opt-in, and the existing SDK stream remains available on port 5610. This
build fails explicitly with SDK versions older than 5.4; a raw-frame re-encode
fallback for older SDKs is not implemented. If the SDK encoder cannot start,
the relay reports the failure and the mission continues without standard video.

This path sends video only. Telemetry remains a separate connection configured
in `MissionConfig.zig`; detections are overlaid using the latest available
telemetry, not frame-synchronized metadata. For a software-only RTP integration
check, install the `x264enc` plugin and run `zig build test -Dviewer-gstreamer`.

### Legacy ZED SDK viewer

```sh
zig build viewer -Dviewer-zed
./zig-out/bin/viewer --video 127.0.0.1:5610
```

The ZED and GStreamer viewer backends are separate build options. The ZED backend
also supports SVO input. Select Operation, Mission, or Tuning with the sidebar
or keys `1`–`3`. Press `T` to switch control mode. The sidebar font buttons or
`-`/`+` adjust text size; `0` resets it. See the on-screen controls for movement,
mission commands, and tuning. PID edits send the entire live configuration, and
UDP send status does not confirm that the runtime received a command.

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
