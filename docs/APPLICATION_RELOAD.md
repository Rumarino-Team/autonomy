# Zig application reload and binary data

`src/main.zig` is a permanent executable host. `application.zig` builds the shared
application library containing configuration, MissionContext, mission definitions,
controllers, scene setup, and fuzzing. Ordinary `zig build` builds both. There is
no configuration build option or runtime configuration parser.

## Configure, build, run

Edit `config/application.zig` and its imported presets. The default is Hydrus,
MuJoCo, Robosub, 360 Hz physics, six physics ticks per controller frame, 21,600 ticks,
and no wall-clock pacing. The existing hydrodynamic cadence is seven physics ticks.

```zig
const Types = @import("../src/Configuration.zig");
pub const config: Types.Application = .{
    .controller = @import("hydrus.zig").controller,
    .mission = .prequalify_robosub,
    .realtime = false,
    .reload = .{ .watch = true, .preserve_state = true },
    .fuzz = .{ .seed = 1, .cases = 100, .workers = 8, .results = "build/my-campaign" },
};
```

```sh
zig build -Dsimulator=true       # first build/package of native MuJoCo dependencies
zig build                      # ordinary application/configuration compilation
./zig-out/bin/src_3 run
./zig-out/bin/src_3 fuzz
./zig-out/bin/src_3 replay 10451216379200822465
./zig-out/bin/src_3 replay build/my-campaign/case-000000.bin
./zig-out/bin/src_3 inspect build/my-campaign/totals.bin
```

Set a fresh `.fuzz.results` directory before each campaign/replay. Seed replay uses
current configuration; file replay uses the recorded configuration/profile with
today's output directory. Reproduction assumes the same scene assets, generator,
controller/mission implementation, and native simulator build. Reports retain both
resolved inputs and hashes so differences can be investigated.

Operational settings come from Zig, including plugin/backend/robot/scene selection,
controller values, sampling cadence, budgets, telemetry, Blender pose-link settings,
fuzzer ranges and faults, and reload policy. CLI arguments select operations only.
The native four-function AUV interface retains its environment-based compatibility
entry point for existing native callers; the Zig host uses explicit scene/options
calls. Select matching controller and robot geometry when changing presets.

Plugin/backend filenames resolve beside the installed native libraries. Absolute
paths are supported. Windows maps bundled `.so` defaults to installed DLL filenames.
Zig's application DLL resides beside the Windows executable; native CMake DLLs
reside in `lib`. Native Windows execution must be validated on Windows.

## Reload transaction and ownership

The host watches application/configuration/include sources recursively, checks at
100 ms intervals, and debounces saves. Builds use the ordinary build graph and Zig
cache. Candidate libraries get unique per-process generation filenames. A source
edit during compilation requires another build before activation.

MuJoCo pauses at a completed controller frame while compilation occurs. Hardware
plugins continue using the previous application on the owning main thread while a
host thread compiles. The final application replacement occurs between frames.

With `.reload.preserve_state = true`, the host captures named state fields into a
host-owned binary buffer. A candidate creates a fresh, inactive MissionContext,
validates structural settings, migrates the state, and reconstructs its containers.
Preparation cannot step physics, issue commands, or mutate retained native resources.
Only after preparation succeeds does the host destroy the old context with the old
library and unload it. It then activates the candidate and rebases MuJoCo's real-time
clock, avoiding catch-up after compilation.

The native model/world, solver history, rotor states, sensor caches, held commands,
and physics tick stay alive in the host-owned session. Mission stages, targets,
queued goals, detections, PID state, and timestamps transfer explicitly. State
contains object identifiers, never addresses into a previous DLL. Migration from
state schema 1 to 2 explicitly initializes the added diagnostic counter; unknown
schemas are rejected. This is an in-memory reload cache, not disk resume.

A failed build, invalid configuration, unsupported migration, or incompatible scene
change leaves the old application usable. Robot/scene/backend, physics timing,
command dimensions, and persistent I/O setting changes require a fresh world.
Set `.reload.preserve_state = false` to restart from initial conditions after an
edit. If native recreation fails after destruction, the host stops cleanly.

The executable retains native library handles, stable-address C scene roots,
telemetry/teleop sockets, signals, and cancellation. Each application generation has
its own allocator and synchronous I/O wrapper. It installs no signal handlers and
leaves no callbacks or threads behind. Fuzz workers join before their library can
unload. Campaigns pin their application/configuration until completion or cancellation.

Changes to host implementation, public host ABI, or shared host-side math/AUV
layouts require executable restart and are reported explicitly. Native C/C++ plugin
reload is outside this boundary. Development watching needs the source checkout
recorded at build time; packaged runs can execute without recompilation.

## Writing missions

`src/missions/prequalify.zig` defines the prequalification goals/classes and stage
names. `MissionRunner` advances operations one frame at a time. A frame update
returns to `main`, so no mission call stack survives a reload. Waiting for a
measurement and executing queued pass/circle maneuvers both have explicit state.

Keep persistent locals in the operation or MissionContext state. Add saved fields
and an explicit migration when changing the state contract. Do not store function
pointers, static strings, slices into code, or object-cache addresses in saved state.
Position/look goals are values; target objects use IDs. Controller history remains
available after compatible code/gain edits; new PID limits apply immediately.

The host/library API is `include/application.h`, independently checked from C11,
C++23, and Zig. Its ABI version is 1; simulator model/backend ABI remains 5.

## Binary persistence

Owned controller JSON inputs and JSON data reports are removed. Asset metadata is
authored in `config/robots.zig` and compiled to packaged binary metadata. Existing
saved `.blend` assets and embedded `ASIM_PROFILE_1` MessagePack remain unchanged.
Blender scene snapshots use binary interchange; the packaged background exporter
and native loader agree on the same field schema. Set `.scene` to a binary snapshot
or saved `.blend` path to use that scene. Scene edits require a fresh-world run.

Application data uses `AUVDATA1`: an eight-byte magic, little-endian u32 record kind,
u32 schema, u64 payload length, and u32 CRC32. Payload values have explicit tags for
null, boolean, signed/unsigned 64-bit integers, 64-bit floating point, byte strings,
sequences, records, and enum names. Lengths/counts are u32. Fields encode in declared
order; structs have named fields. Floats are widened from f32 exactly; SIMD padding,
pointers, intrusive links, and allocator internals are excluded. Schema changes
must be versioned. Limits are 64 MiB, one million elements, and 64 nesting levels.

Each campaign writes `campaign.bin`, `case-NNNNNN.bin`, `summary.bin`, and `totals.bin`.
Worker files are separate and finalized by rename; summaries use case-index order.
Existing result names are never overwritten. Incomplete files are rejected by
length/checksum validation. The native shutdown summary is human-readable text;
set `.run_result_path = "build/run.bin"` for an additional binary result.

Both inspectors produce readable text:

```sh
./zig-out/bin/src_3 inspect build/my-campaign/case-000000.bin
python tools/data_binary.py build/my-campaign/totals.bin
```

Old JSON inputs are rejected; old user reports are not deleted or auto-converted.
Telemetry's `AUV1` binary protocol and physics IPC remain unchanged.

## Validation

```sh
zig build test
cmake -S . -B build/simulator -DAUTONOMY_BUILD_SIMULATOR=ON -DSIMULATOR_ENABLE_STONEFISH=OFF
cmake --build build/simulator --target simulator_all --parallel 8
ctest --test-dir build/simulator --output-on-failure
zig build -Dtarget=x86_64-windows-gnu --prefix build/reload-windows
```

The current pre-refactor nominal Hydrus baseline is 14,718 ticks, hash
14467634289338063255, minimum depth 2 m and maximum depth 2.52970862 m. Tests compare
this baseline, preserve physics through repeated DLL unloading, cover state
migration/rejection and failed builds, compare serial/eight-worker binary cases,
and verify replay and independent C/C++/Zig ABI layouts. ASan/UBSan/leak and native
TSan checks use separate builds. No Stonefish build or execution is required.

### Recorded validation for this migration

The normal and ASan/UBSan/leak suites each passed all 26 CTests. The native
concurrency harness passed ThreadSanitizer. Eight broad-profile seeds were also run
through an isolated reconstruction of the original application and the new package:
outcomes, mission stages, ticks, hashes, depths, and final f32 states matched.

A packaged 100-case campaign with the current expanded rotation ranges produced
50 passes and 50 failures; these are mission findings, not a claimed qualification
of that broad profile. File replay reproduced the selected failed case at tick
21,600 with hash 3177940604208046160. The nominal scene still completes at tick
14,718 with hash 14467634289338063255.

A real Blender 4.5.6 background add-on test completed the same mission and received
60 paced live-pose updates while preserving the authoring-scene fingerprint. A
saved edited .blend also loaded through the binary exporter/native reader, and its
six-tick bounded run wrote a validated binary result. Native Windows execution was
not available; host/application DLL cross-compilation was verified.

Measured application builds with fresh local/global caches took 20.647 s; an
unchanged build took 0.050 s and a controller gain edit took 0.432 s. The host
executable hash stayed unchanged for the gain edit. These are measurements from
this machine during validation, not latency guarantees or native-engine build times.
