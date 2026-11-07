# Portable simulation

The host owns scenarios, vehicle parameters, rotor state, force equations, sensor observations, fault injection, tick scheduling and tests. Engines own integration, contacts and solver state. Select an engine before a run using the same scenario; no live state transfer is attempted.

## Build

From the repository root, initialize the pinned engines and build:

```sh
git submodule update --init --recursive vendor/mujoco vendor/stonefish
zig build sim
zig build sim-test
zig build test
```

The optional simulation build needs CMake, a C/C++23 compiler, SDL2, OpenGL, GLU, Freetype, GLM, OpenSSL, zlib, zstd and reentrant Qhull development packages. The default Zig controller/hardware build keeps its existing dependencies.

Equivalent CMake commands:

```sh
cmake -S . -B build/portable -DAUTONOMY_BUILD_SIM=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build/portable --target sim_all --parallel 4
ctest --test-dir build/portable --output-on-failure
```

Existing **matching pinned** SDK installations can be supplied with `SIM_MUJOCO_PREFIX` and `SIM_STONEFISH_PREFIX`. Leave them empty to build the checked-in revisions. Older Stonefish SDKs with different C++ interfaces are incompatible. The runtime never uses the SDK types in its public ABI.

Install this component with `cmake --install build/portable --component simulation --prefix "$PWD/build/portable/package"`. It includes the engine libraries, adapters and worker.

The build produces `libautonomy_sim.so`, `libsim_backend_mujoco.so`, `libsim_backend_stonefish.so`, `libauv_sim.so`, `sim_stonefish_worker`, and `simctl` under `build/portable/sim`. Keep the Stonefish adapter and worker together. CMake installation places both in `lib`, with `simctl` in `bin`.

## Run and replay

```sh
build/portable/sim/simctl run \
  build/portable/sim/libsim_backend_mujoco.so \
  auvs/stonefish_sim/data/pool_scene.blend 720

build/portable/sim/simctl run \
  build/portable/sim/libsim_backend_mujoco.so \
  auvs/stonefish_sim/data/pool_scene.blend 720 7
```

Replace the library with `libsim_backend_stonefish.so` to use Stonefish. The command prints one JSON line with the backend, the number of steps begun, the final state hash, and the status. A failing run prints that same line, with `status` of `failed` and the error text, and writes `simctl: <error>` to stderr. Replay is the same command again. The scene, tick count, and seed are the inputs. Physics commands are calculated again from the seed. A mission calculates commands again in the controller.

`simctl resolve scene.blend <seed>` writes the drawn initial condition to stdout and does not simulate. `run ... - ... --resolved` reads that handoff from stdin, including an edited command sequence. A filesystem scene must be a `.blend`.

The qualification scene is `auvs/stonefish_sim/data/pool_scene.blend`. The core runtime does not read MJCF, Stonefish scenario files, or a second scene snapshot. Infinite legacy floor planes become finite floor boxes covering the bounded pool; their upper surfaces retain the authored floor depth.

## Current mission controllers

```sh
zig build sim
zig build
python3 tools/blender/from_blender.py \
  auvs/stonefish_sim/data/pool_scene.blend \
  --scene AutonomySource --frame 1 --backend mujoco
```

The bridge preserves `AuvFrame`, class IDs, the vehicle-origin pose convention, and per-axis validity flags. Its optional `auv_simulation_status` extension reports initialization failures, invalid controller commands and tick exhaustion. Mission runs freeze configuration and plugin reloads. A budget failure is a test result, not a successful mission.

The native C++ loader reads the saved `.blend` datablocks directly. It resolves
collection membership, parents, collision geometry, camera/thruster mounts and
embedded calibration into owned scene arrays. No Python scene exporter,
background Blender process or exported scene snapshot is used by a normal run.
The add-on passes the saved file path and source scene name to the mission
process and rejects unsaved changes. Apply modifiers/constraints/shape keys on
physical objects and remove animated/driven physical transforms before saving;
native loading reports unsupported evaluation requirements explicitly.
The default `AUTONOMY_SIM_SCENARIO` is
`auvs/stonefish_sim/data/pool_scene.blend`. Embedded calibration makes that file's
simulation definition independent of profile sidecars. See the
[authoring contract](../tools/blender/SCENE_AUTHORING.md) for loader settings and
the in-memory scene layout. `sim/profiles/hydrus.json` is only the calibration
source refreshed by **Embed calibration profiles**; save the `.blend` afterwards.

`controller.json` is tuned for the shared Hydrus model in the pool blend. The current Blender-authored course completes in about 44 simulated seconds on both engines with the 21,600-tick budget. Its six-axis PID outputs forces in N and moments in N m. The allocator uses the pseudoinverse of the actual thruster directions, COM lever arms, bollard thrust and propeller reaction torques; `quadratic_rpm` converts allocated thrust fractions into shaft-speed commands. Regenerate it with `python3 sim/controller_config.py` after deliberate thruster geometry edits or after changing that script's gains. These gains and this allocator are vehicle-specific.

The simulation profile uses `odometry.translation_frame: "world"`, so a horizontal waypoint cannot become a depth request when the hull tilts. `derivative_on_measurement: true` uses position velocity and body gyro for damping, avoiding derivative kicks when goals change. `integral_limit` bounds each integral contribution, `output_limit` bounds each PID output, and conditional integration prevents windup at those bounds. `feedforward_world_force` cancels the measured weight/buoyancy imbalance. Old controller files keep body-frame error derivatives, linear output and the scale of five unless these fields are set. The motor PI uses an implicit proportional update to avoid the alternating speed response of explicit Euler at the physics timestep.

`AUTONOMY_SIM_SEED` enables case randomization. Basic observations use the common geometry projection and body motion; they are synthetic observations, with no native camera renderer in the required test profile. Native viewer/camera plugins remain separate.

The [Blender viewer](../tools/blender/README.md) restores the experimental pool scene and the live pose link for either backend. Enable `AUTONOMY_SIM_BLENDER=1` for UDP output and `AUTONOMY_SIM_REALTIME=1` for wall-clock pacing, or use the add-on's Run live button. These settings are off during normal deterministic batch runs. The pose link is the view while a process is running.

## Fuzzing and failure reduction

Fuzz is a mission. It runs the RoboSub course once per seed, in up to 32 threads. `replay` runs one seed again.

```sh
AUTONOMY_SIM_BACKEND=build/portable/sim/libsim_backend_mujoco.so \
AUTONOMY_SIM_SCENARIO=auvs/stonefish_sim/data/pool_scene.blend \
AUTONOMY_SIM_MAX_TICKS=21600 \
  ./zig-out/bin/src_3 build/portable/sim/libauv_sim.so \
  fuzz sim/scenarios/controller.json 1 4

AUTONOMY_SIM_BACKEND=build/portable/sim/libsim_backend_mujoco.so \
AUTONOMY_SIM_SCENARIO=auvs/stonefish_sim/data/pool_scene.blend \
AUTONOMY_SIM_MAX_TICKS=21600 \
  ./zig-out/bin/src_3 build/portable/sim/libauv_sim.so \
  fuzz sim/scenarios/controller.json replay 7
```

`1` is the master seed and `4` is the case count. Each case seed is `splitmix64(master + index * 0x9E3779B97F4A7C15)`. The plugin draws that case with `auv_begin_fuzz` after the thread's world is loaded. A failing case logs its seed, the error, and the simulated time. The process exits nonzero when any case fails.

A physics failure can still be shrunk from a `failure.json` that names the scene, seed, tick count, and backend:

```sh
python3 sim/reduce.py build/fuzz/case/failure.json
```

The reducer finds the shortest failing prefix by rerunning `simctl run`, resolves the drawn scene once, regenerates the seeded command rows, removes unnecessary faults, and zeros command blocks while retaining the same failing condition. It writes `reduced-replay.json` with the replay command and the reduced scene inline. That handoff is run with `--resolved`, because deleted faults and zeroed commands are part of the saved initial condition. Mission failures are rerun with `fuzz ... replay <seed>`.

The PR workflow qualifies both engines on the pool blend, then runs a fixed seeded replay corpus. The nightly workflow expands the corpus. CTest also checks worker counts 1, 2 and 4 and verifies that deliberately failing cases reduce and reproduce. Mission integration tests require full course completion within 60 seconds, a depth band of 1.5–2.6 m on the result line, and the same result hash from a second run. Separate two-second cases require the expected tick-budget failure at step 720. Tests also check all six allocator axes, disabled collision shapes, and rotor speed steps/reversals at 90, 360 and 720 Hz. Mission tests run after `zig build`; CMake-only users see them marked skipped until that executable is built.

## ABI and model conventions

`include/sim_backend.h` is the versioned C boundary. Backends export one `sim_backend_get_api` entry point. It negotiates an exact ABI version and table size and returns capabilities, integrator support, world creation from a borrowed resolved scene view, cold reset, stepping, state and contact reads, and a diagnostic digest. Each step passes a wrench array and gets body state, contacts, and a hash back. Errors are explicit results and caller-owned strings; no engine pointers, STL containers, or exceptions cross the boundary.

`world_create` uses ABI v3: it borrows a C `SimSceneView` for the call and copies retained data into a private database. MuJoCo receives this view directly; Stonefish uses ASIMSC03 to reconstruct it in its worker. This handoff is built by C++, independently of the removed Blender exporter handoff. Keep a library loaded until its worlds are destroyed. Different worlds may run concurrently; serialize calls to one world. Buffer queries report the required count and never truncate silently.

All values use SI units and doubles. The world is NED, with water below `z=0`; quaternions are unit `xyzw`, rotating local vectors into their parent frame. `pose` identifies a body origin. `center_of_mass` supplies the principal inertial frame relative to that origin. Velocities and wrenches use world axes at the center of mass. Shapes are compound-local boxes, spheres, cylinders or **convex collision meshes**. Author concave collision environments as compounds; imported pool walls are boxes.

The scenario format is version 1. Required fields are `version` and `bodies`; each body specifies an integer `id`, `motion` (0 static, 1 dynamic, 2 kinematic), `shapes`, and dynamic mass/principal inertia. Shapes use `kind` 0 box, 1 sphere, 2 cylinder, 3 mesh. Box sizes are half extents; cylinders use radius `size.x` and half length `size.z`. Meshes supply `vertices` and triangle indices. `pose` defaults to identity.

The timestep defaults to the rational `[1,360]`. The host uses integer ticks and derives nanosecond timestamps by integer arithmetic. `integrator` accepts `default`, `euler`, `implicit`, `implicitfast`, and `rk4`; MuJoCo supports all of these, with `implicitfast` as its default. Stonefish exposes its default integration only. Unsupported choices fail before stepping. Shared external forces are sampled once per tick and held during internal stages, including RK4.

`dry_mass`, `volume`, `buoyancy_center`, `hull`, `current`, `thrusters`, and `hydro_period_ticks` describe the shared vehicle models. Effective inertial mass/principal inertia include the existing constant added-mass approximation. Gravity compensation acts on dry mass. This preserves the existing model and does not implement a general six-dimensional added-mass tensor.

Set `native_hydrodynamics: true` to opt into Stonefish's native fluid forces; shared fluid forces are then excluded. MuJoCo rejects that option. The two modes can have different surface-crossing and geometry approximations. Each native profile still passes deterministic replay qualification. The Stonefish adapter currently requires one contact material per compound body and uniform collision flags for native-fluid bodies; MuJoCo currently requires zero restitution. These limitations produce load errors.

Optional `assertions` configure positive finite `max_position_norm`, `max_speed`, and `max_angular_speed`. Defaults bound runaway states generously. Non-finite state, invalid quaternions and non-finite model forces always fail. Fault intervals use `[start_tick,end_tick)` and kinds `tracking`, `imu`, `gyro`, `depth`, `dead`, or `weak`.

## Add a force equation

Register a C++ function in `ForceRegistry`, then select its name in the scenario's ordered `force_modules`. The function receives owned, normalized data and writes body wrenches. Store per-world state in `context.state["module_name"]`; reset clears it. Functions must be reentrant and must not retain engine state or mutable globals.

```cpp
registry.add("linear_drag", [](sim::ForceContext& context,
                               sim::WrenchBuffer& output) {
    for (size_t i = 0; i < output.size(); ++i) {
        auto velocity = context.bodies[i].linear_velocity;
        sim::add_force(output[i], sim::operator*(velocity, -2.0),
                       context.bodies[i].center_of_mass,
                       context.bodies[i].center_of_mass);
    }
}, "hydrodynamics");
```

Select `["gravity_compensation","linear_drag","thrusters"]` and construct `World` with that registry. A nonempty contribution name declares exclusive ownership: multiple providers for the same contribution, including native hydrodynamics, are rejected. General additive disturbances may leave the contribution name empty. Model updates run once in declared order and are accumulated in stable body order.

## Blender authoring

The supported authoring path uses evaluated Blender data and a fresh entity
list. See the [authoring contract](../tools/blender/SCENE_AUTHORING.md) for roles,
collision proxies, stable IDs, calibration profiles, and source/preview isolation.
The `.blend` is the only authored scene. Each run evaluates it into the in-memory
scene the engines consume. Changes apply to the next run. Replay runs that same
file again with the same seed and tick count.

## Datablocks, pools and arenas

The simulator owns models through a noncopyable `Main`. Its single scene
registry contains pooled `ObjectBlock` records (plus one embedded fixed robot
object), each with contained colliders and shared immutable geometry buffers.
Colliders and geometry have no independent datablock identities. Intrusive
registration and dense pointer views support stable addresses and stepping. See [DATABASE.md](DATABASE.md) for ownership, builder usage,
allocator lifetimes, ABI-v3 descriptors, the exact ASIMSC03 section layout, and
worker protocol 2. Old ABI-v2 plugins and ASIMSC02 recordings must be rebuilt or
regenerated; no converter is provided. Saved Blender loading and authoring rules
are unchanged.

The database tests also check cloning, cross-database reference rejection,
allocation failure cleanup, C callers releasing borrowed descriptors, and
allocation stabilization after warm-up on both backends.

## Fixed robot files

Standalone, checksum-bound Hydrus, Proteus, BlueROV2 and Girona 500 scenes are in
[`auvs/stonefish_sim/data/robots`](../auvs/stonefish_sim/data/robots/README.md).
Their physical configurations are compiled into the native reader. Run
`simctl robots` to inspect the fixed registry. The asset README describes the
calibration sources, approximations, rebuild workflow and both-engine checks.
