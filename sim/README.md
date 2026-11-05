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

The optional simulation build needs CMake, a C/C++23 compiler, SDL2, OpenGL, GLU, Freetype and GLM development packages. The default Zig controller/hardware build keeps its existing dependencies.

Equivalent CMake commands:

```sh
cmake -S . -B build/portable -DAUTONOMY_BUILD_SIM=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build/portable --target sim_all --parallel 4
ctest --test-dir build/portable --output-on-failure
```

Existing **matching pinned** SDK installations can be supplied with `SIM_MUJOCO_PREFIX` and `SIM_STONEFISH_PREFIX`. Leave them empty to build the checked-in revisions. Older Stonefish SDKs with different C++ interfaces are incompatible. The runtime never uses the SDK types in its public ABI.

Install this component with `cmake --install build/portable --component simulation --prefix "$PWD/build/portable/package"`. It includes the engine libraries, adapters and worker.

The build produces `libautonomy_sim.so`, `libsim_backend_mujoco.so`, `libsim_backend_stonefish.so`, `libauv_sim.so`, `sim_stonefish_worker`, and `simctl` under `build/portable/sim`. Keep the Stonefish adapter and worker together. CMake installation places both in `lib`, with `simctl` in `bin`.

## Run, record and replay

```sh
build/portable/sim/simctl run \
  build/portable/sim/libsim_backend_mujoco.so \
  auvs/stonefish_sim/data/pool_scene.blend build/portable/take.jsonl 720

build/portable/sim/simctl replay \
  build/portable/sim/libsim_backend_mujoco.so \
  build/portable/take.jsonl build/portable/replay.jsonl
```

Replace the library with `libsim_backend_stonefish.so` to use Stonefish. An extra integer after the tick count randomizes the scenario and supplies seeded actuator inputs. Without it, the run uses the authored initial state and zero commands. Replay consumes recorded controller inputs and verifies every state digest.

JSONL traces contain the complete resolved scenario, engine and binary fingerprints, exact input commands, tick timestamps, body state, per-body wrenches and rotor diagnostics. Preserve the engine/adapter/runtime binaries for exact future replay. A different solver or build is compared numerically, rather than by digest.

Hydrus, Proteus and BlueROV scenarios include resolved pool/vehicle geometry and existing physical constants from `feat/mujoco-parallel` commit `c4b5103`. Their zero-command MuJoCo trajectories are regression checked against captured legacy results. The core runtime does not read MJCF or Stonefish scenario files. Infinite legacy floor planes become finite floor boxes covering the bounded pool; their upper surfaces retain the authored floor depth.

## Current mission controllers

```sh
zig build sim
zig build
python3 tools/blender/from_blender.py \
  auvs/stonefish_sim/data/pool_scene.blend \
  --scene AutonomySource --frame 1 --backend mujoco
```

The bridge preserves `AuvFrame`, class IDs, the vehicle-origin pose convention, and per-axis validity flags. Its optional `auv_simulation_status` extension reports initialization failures, invalid controller commands and tick exhaustion. Mission runs freeze configuration and plugin reloads. A budget failure is a test result, not a successful mission.

The native loader opens the saved `.blend` in background Blender, evaluates it,
and receives a versioned binary scene over a local socket. The add-on's Run live
and Record run buttons send current unsaved source data directly over an inherited
socket. Neither normal path generates or loads an intermediate scene JSON file.
The default `AUTONOMY_SIM_SCENARIO` is
`auvs/stonefish_sim/data/pool_scene.blend`. Embedded calibration makes that file's
simulation definition independent of profile sidecars. See the
[authoring contract](../tools/blender/SCENE_AUTHORING.md) for loader settings and
the binary format. Legacy JSON inputs remain supported explicitly; `zig build
sim-scene` refreshes the JSON qualification snapshot.

Use `simctl snapshot source.blend frozen.asim` to capture binary input once for
Blender-free simulation workers. `sim/fuzz.py --scenario source.blend` performs
that freezing once per batch automatically. JSONL remains the diagnostic trace
and replay format, with a complete resolved scene. The simulation loop uses typed
fault, bounds and force-module settings.

`controller.json` is tuned for the shared Hydrus model. The current Blender-authored course completes in about 44 simulated seconds on both engines with the 21,600-tick budget. Its six-axis PID outputs forces in N and moments in N m. The allocator uses the pseudoinverse of the actual thruster directions, COM lever arms, bollard thrust and propeller reaction torques; `quadratic_rpm` converts allocated thrust fractions into shaft-speed commands. Regenerate it with `python3 sim/controller_config.py --scenario sim/scenarios/pool.json` after exporting deliberate thruster geometry edits or changing that script's gains. These gains and this allocator are vehicle-specific; generate and tune separate configurations for other vehicles.

The simulation profile uses `odometry.translation_frame: "world"`, so a horizontal waypoint cannot become a depth request when the hull tilts. `derivative_on_measurement: true` uses position velocity and body gyro for damping, avoiding derivative kicks when goals change. `integral_limit` bounds each integral contribution, `output_limit` bounds each PID output, and conditional integration prevents windup at those bounds. `feedforward_world_force` cancels the measured weight/buoyancy imbalance. Old controller files keep body-frame error derivatives, linear output and the scale of five unless these fields are set. The motor PI uses an implicit proportional update to avoid the alternating speed response of explicit Euler at the physics timestep.

`AUTONOMY_SIM_SEED` enables case randomization. Basic observations use the common geometry projection and body motion; they are synthetic observations, with no native camera renderer in the required test profile. Native viewer/camera plugins remain separate.

The [Blender viewer](../tools/blender/README.md) restores the experimental pool scene, live pose link, take recording and timeline replay for either backend. Enable `AUTONOMY_SIM_BLENDER=1` for UDP output and `AUTONOMY_SIM_REALTIME=1` for wall-clock pacing, or use the add-on's Run live button. These settings are off during normal deterministic batch runs.

## Fuzzing and failure reduction

```sh
python3 sim/fuzz.py \
  --backend build/portable/sim/libsim_backend_stonefish.so \
  --scenario sim/scenarios/pool.json --cases 32 --workers 4 \
  --ticks 21600 --verify-replay --reduce-failures --output build/fuzz/stonefish
```

Use `--mode mission` to execute the current Zig controller instead. Optional arguments select `--controller`, `--mission-executable`, `--plugin`, and `--mission`. Tick exhaustion, invalid state/commands, crashes and infrastructure timeouts fail the case. The wall watchdog is independent of simulated time. Cases are sorted by index; changing worker count preserves the seeds and trajectories.

Failures retain a source scenario, resolved trace, inputs, stdout/stderr, platform and build identity, replay commands, and a summary. Successful traces are discarded unless `--keep-passed` is set. Use `--replay-seed N` for one generated case. Physics failures can be reduced automatically or with:

```sh
python3 sim/reduce.py build/fuzz/stonefish/case-000000-SEED/failure.json
```

The reducer finds the shortest failing prefix, removes unnecessary faults, and zeros command blocks while retaining the same failing condition. It emits a portable `reduced.json` and `reduced-replay.json`. Mission failures retain their seed and controller input trace; automatic command reduction currently targets physics runs.

The PR workflow qualifies both engines and all three vehicles, then runs a fixed seeded replay corpus. The nightly workflow expands the corpus. CTest also checks worker counts 1, 2 and 4 and verifies that deliberately failing cases reduce and reproduce. Mission integration tests require full course completion within 60 seconds, finite commands, bounded depth and exact input replay on each engine. Separate two-second cases require the expected tick-budget failure. Tests also check all six allocator axes, disabled collision shapes, and rotor speed steps/reversals at 90, 360 and 720 Hz. Mission tests run after `zig build`; CMake-only users see them marked skipped until that executable is built.

## ABI and model conventions

`include/sim_backend.h` is the versioned C boundary. Backends export one `sim_backend_get_api` entry point. It negotiates an exact ABI version/table size and returns capabilities, integrator support, model/world lifecycle functions, cold reset, stepping, state/contact reads and a diagnostic digest. Errors are explicit results and caller-owned strings; no engine pointers, STL containers or exceptions cross the boundary.

Inputs are borrowed for one call and deep-copied at model creation. Keep models alive until their worlds are destroyed, and keep a library loaded until its objects are destroyed. Different worlds may run concurrently; serialize calls to one world. Buffer queries report the required count and never truncate silently.

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
                               std::vector<SimWrench>& output) {
    for (size_t i = 0; i < output.size(); ++i) {
        auto velocity = context.bodies[i].linear_velocity;
        sim::add_force(output[i], sim::operator*(velocity, -2.0),
                       context.bodies[i].center_of_mass,
                       context.bodies[i].center_of_mass);
    }
}, "hydrodynamics");
```

Select `["gravity_compensation","linear_drag","thrusters"]` and construct `World` with that registry. A nonempty contribution name declares exclusive ownership: multiple providers for the same contribution, including native hydrodynamics, are rejected. General additive disturbances may leave the contribution name empty. Model updates run once in declared order and are accumulated in stable body order.

## Blender authoring and import

The supported authoring path uses evaluated Blender data and a fresh entity
list. See the [authoring contract](../tools/blender/SCENE_AUTHORING.md) for roles,
collision proxies, stable IDs, calibration profiles, and source/preview isolation.
Snapshots and traces record geometry, source identity and resolved profiles.
Changes apply to the next run, while replay uses the captured scene.

### Legacy stored-mesh importer

```sh
cmake -S . -B build/portable -DAUTONOMY_BUILD_SIM=ON -DSIM_BUILD_BLEND_IMPORTER=ON
cmake --build build/portable --target sim_import_blend --parallel 4
build/portable/sim/sim_import_blend \
  sim/scenarios/hydrus.json scene.blend scene-config.yaml resolved.json
```

This explicitly selected compatibility path reuses the pinned CBlend reader and merges into an old base scenario. It is not invoked by Run live, Record run, or the Blender-source CLI. `geometry_slot` binds a vehicle mesh to a scenario shape; the legacy Blender `mujoco_geom` property is accepted as an import alias.

The importer supports the existing stored Blender 4.x mesh layout, applied modifiers and the repository's coordinate mapping. It does not evaluate a live Blender scene. Reload by importing a new scenario and starting a new run.
