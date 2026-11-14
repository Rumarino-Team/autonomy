# Data-oriented simulator core

For the Zig application, configuration is now compiled from `config/application.zig`.
Use `zig build -Dsimulator=true` and `./zig-out/bin/src_3 run`; JSON configuration
and the old positional launch arguments are removed. See
[application reload and binary data](../../docs/APPLICATION_RELOAD.md) for current
commands, reload ownership, binary reports, and validation. Native C interfaces
described below remain available.


The simulator core, scene snapshot loader, AUV adapter, and MuJoCo backend are
implemented in `zig/`. They retain the shared C headers and ABI v5. The core and
adapter require libc; the backend additionally links MuJoCo through its C API.
MuJoCo itself remains the pinned native engine. The independent C++ binary codec
is used only by interoperability tests, not by either runtime library.

This is the programmatic-scene simulator in the isolated `tmp2/autonomy`
checkout, branch `codex/data-oriented-simulator-core`. `BASE_COMMIT.txt` records
its `main` base and MuJoCo revision. The implementation started from the four
untracked draft files in `/home/cesar/Projects/autonomy/auvs/simulator`.

## Build and run

From the repository root:

```sh
cmake -S . -B build/simulator \
  -DAUTONOMY_BUILD_SIMULATOR=ON -DSIMULATOR_ENABLE_STONEFISH=OFF \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build/simulator --target simulator_all -j8
ctest --test-dir build/simulator --output-on-failure

build/simulator/auvs/simulator/simulator_c_example \
  "$PWD/build/simulator/auvs/simulator/libsimulator_backend_mujoco.so"
```

The default build fetches the pinned MuJoCo revision. An existing matching
installation can be selected with `SIMULATOR_MUJOCO_PREFIX`. Python 3 and Zig 0.17.1
run validation; ROS/colcon and Stonefish dependencies are not needed. Existing
Stonefish targets are opt-in through `SIMULATOR_ENABLE_STONEFISH` and are outside
this delivery's fuzz implementation and validation.

Package the runtime and MuJoCo together:

```sh
cmake --install build/simulator --prefix "$PWD/build/package" --component simulator
build/package/bin/simulator_c_example \
  "$PWD/build/package/lib/libsimulator_backend_mujoco.so"
```

Headers are installed in `include/autonomy/simulator`. For downstream C++ code,
use a quote include path (`-iquote`) so the repository's `math.h` does not replace
the system header used by `<cmath>`.

## Shared model and ownership

`simulation_types.h` is the one C definition of all shared records. The runtime
and both adapters use it directly. `sim.h` contains the public model/runtime
functions. `sim_backend.h` contains the sole ABI-v5 engine function table.
All exported simulator functions have C linkage; private implementation helpers
use C++23. C11 clients require no C++ headers or containers.

- `Simulation` owns the database storage and immutable settings/model. It is a
  public C root with an opaque allocation-owner pointer.
- `ObjectBlock` contains an intrusive header, physical body values, and a
  collider pointer/count. Each object is one rigid body, including compounds.
- `Collider` contains local shape properties and a `MeshData` pointer/count view.
  Geometry has no registered datablock or independent ID.
- `Robot` embeds its registered object, five colliders, eight thrusters, and five
  hydrodynamic hulls. Counts select active slots. Sensors use scene-owned storage.
- `SimulationRuntime` owns commands, rotor state, force caches, observations, and
  reusable state/contact buffers. It borrows the model and loaded backend.
- `PhysicsWorld` is opaque engine-owned state. MuJoCo retains pointers into the
  frozen model. Stonefish's proxy transports a physics-only projection to an
  independently owned model in its worker process.

Roots must start zero-initialized and retain their addresses until destruction.
C assignment is not a clone: intrusive links and owners would still refer to the
original root. Use `simulation_clone`, then finalize the independent draft.

Typical lifecycle:

1. `simulation_init`, optionally with configured capacities/allocation callbacks.
2. Initialize values with `underwater_body_default`, `collider_default`, and
   `rotor_spec_default`; add objects, hardware, sensors, routes, faults, and selections.
3. Set settings and body properties while building, then `simulation_finalize`.
4. Open a backend, create a runtime, step it, and read state or observations.
5. Destroy the runtime before closing its backend and destroying the model.

Frozen data must not be edited through retained pointers, raw arena utilities,
or casts. Construction functions reject mutation after finalization. Calls on
one runtime/world must be serialized; separate runtimes can run concurrently.
A failed step may have changed module state; reset before continuing.

Initializers establish identity quaternions and physical defaults. An all-zero
pose is not a valid rotation. Shape dimensions are box half extents, sphere
radius in x, and cylinder radius in x/half height in z. Coordinates are NED,
quaternions xyzw, and state velocities are world-frame values at the center of
mass. Wrenches use world axes and act at COM; their ID is the external body ID.

## Robot presets

`robots.h` exposes four immutable templates defined in `zig/robot_data.zig`: `hydrus`
(five colliders, eight thrusters, five fluid hulls), `proteus` (one/six/one),
`bluerov` (one/eight/one), and `girona500` (one/five/one). Each includes its
physical body, local collision geometry, calibrated fluid surfaces, ordered
motor mounts/specifications, and one camera. Definitions use the shared C
`Robot`, `Collider`, `Thruster`, and `HullRec` records.

```c
#include "robots.h"

Simulation model = {0};
SimulationConfig config;
char error[512];
const Robot *robot = sim_robot_find("hydrus");
/* Check every SimResult in application code; see examples/robot.c. */
simulation_robot_config(robot, &config, error, sizeof error);
simulation_init(&model, &config, error, sizeof error);
simulation_add_robot(&model, robot, 1, NULL, error, sizeof error);
simulation_finalize(&model, error, sizeof error);
```

The capacity helper starts from defaults and increases geometry arenas enough
for that robot. Increase them further before initialization when adding mesh
scenery. `simulation_add_robot` copies geometry into the model's arenas, interns
shared meshes, and repairs ownership references. It leaves a mutable draft and
rolls back on failure. Only one fixed robot is allowed per model; other world
objects may precede or follow it. Commands remain runtime-owned. Never assign a
template directly into the model. Templates have static lifetime, empty list
membership, and NULL attachment owners.

The caller selects the external body ID and may supply a starting `MathPose`.
NULL uses the standalone Blender asset's saved pose: identity orientation at
NED depth 2 m. Preset motor order and local coordinate conventions match the
previous simulator. Run any preset with either engine:

```sh
build/simulator/auvs/simulator/simulator_robot_example \
  "$PWD/build/simulator/auvs/simulator/libsimulator_backend_mujoco.so" hydrus
build/simulator/auvs/simulator/simulator_robot_example \
  "$PWD/build/simulator/auvs/simulator/libsimulator_backend_mujoco.so" proteus
```

The saved `.blend` files and provenance manifest are in `data/robots` and are
installed under `share/autonomy/simulator/data/robots`. Collision transforms,
triangulation, and fluid surfaces were resolved from those files with the
previous native C++ Blender loader. Fixed geometry is compiled in
`robots/*_geometry.inc`; preset construction needs no file parsing or exporter.
Saved Blender edits do not automatically change these compiled definitions.
See `data/robots/README.md` for calibration qualifications and provenance.

## Allocation

There is no PMR. `arena_allocator.h` provides a C bump arena and generic typed
pool descriptor. Pools allocate sequentially, preserve addresses, and reclaim
storage in bulk. Unlinking a record does not reclaim a slot or update a database's
dense table; the low-level list utilities do not replace model construction APIs.

Separate aligned buffers hold vertices, triangles, hull vertices, collider
records, and each registered record type. Initial capacities are 100 elements
per geometry arena, 32 objects (including an embedded robot), 160 colliders,
16 faults, 16 module selections, 16 sensors, and 64 KiB step scratch. Configure
larger capacities before building; exhaustion returns `SIM_CAPACITY` and does
not relocate existing records. Total configured model storage is limited to
128 MiB. Allocation callbacks must return correctly aligned memory, remain alive
through destruction, and must not throw.

Geometry is copied and interned during construction, so borrowed input arrays
can be released immediately. This avoids storing duplicate drafts in a fixed
arena. Finalization validates/freezes the graph. Hashing and equality inspect
float components and triangle indices, ignore SIMD padding, reject non-finite
positions, and canonicalize signed zero. Clones preserve sharing internally.

Runtime and host adapter buffers use the model's allocation callbacks and reuse
capacity. Native adapter bindings use the same C pool mechanism. Engine-internal
allocations remain under native ownership. Contacts may grow beyond warm-up
capacity; returned runtime views expire on step/reset or destruction. Reading
cached observations does not invalidate views or advance the simulation.

## Force callbacks and observations

With no explicit module selections, the runtime runs gravity compensation,
hydrodynamics, and thrusters in that order. Explicit selections preserve their
order. Names are unique and nonempty physical contribution names are exclusive;
a gravity provider is required. Native Stonefish hydrodynamics replaces the
portable hydrodynamics module when selected through settings.

A `ForceRegistry` extends builtins with C callbacks. Descriptors are copied at
runtime creation. `create` receives registration user data and returns per-world
state; paired `destroy` releases it even when creation fails after returning a
partial state. It must accept NULL for that failure case. `reset` handles cold
reset, and `evaluate` receives the shared context and an indexed wrench buffer.
A module without `create`/`destroy` is stateless and receives NULL state. Callback
code/user-data lifetime is the caller's responsibility; callbacks must not throw.
Scratch allocations are valid only during the current step and reset on failure
as well as success.

The common force equations and hydrodynamic period (default seven ticks) come
from the working portable simulator. Rotor calibration/state remain double;
geometry, poses, vectors, and wrenches use the selected float representation.
Observations read scene-defined ideal sensors with independent sampling periods,
mounts, validity, and fault targets. Camera detections respect field of view and
range. See [SENSORS.md](SENSORS.md) for the C sensor API and Zig mission commands.

## Compatibility and current scope

Backend ABI v5 rejects v4. `PhysicsWorld` borrows a frozen model until destruction;
`read_state`/`read_contacts` write caller-owned buffers and report required counts.
Cold reset rebuilds Stonefish native state and clears MuJoCo solver history.

Internal IPC retains ASIMSC03's thirteen-section little-endian schema and worker
protocol v2. It transports settings, bodies, colliders, and shared mesh ranges;
runtime force modules/hardware/faults stay in the parent and are represented by
per-step wrenches. Float values widen to existing double wire fields. No pointers,
SIMD padding, allocator state, or raw C++ structs are serialized. IPC validation
checks lengths, counts, ranges, ownership, and references before world creation.

Blender loading and public replay tools are deferred. The AUV plugin now connects
all four presets to Zig; Hydrus has calibrated Robosub mission coverage.
Examples also support synthetic scenes and the four fixed vehicle presets. The
AUV ABI and physics-only worker transport remain compatible; scene/backend
layouts are ABI v5.
See `VALIDATION.md` for test coverage, measured float differences, and sanitizer
qualification.

Seeded parallel Zig mission campaigns and replay are documented in
[FUZZING.md](FUZZING.md). Each OS worker owns its scene and controller; MuJoCo
is the only supported fuzz backend in this delivery.
