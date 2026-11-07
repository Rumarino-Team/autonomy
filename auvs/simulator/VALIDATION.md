# Validation

The new checkout starts at main commit
`450b0055d9222604beadd0fcff131477255c1e2f`. MuJoCo is pinned at
`237c17e48539b6c90bf90d3161547cbdcbfaa1e0`. This delivery builds and tests
MuJoCo only; Stonefish is disabled and was not built or executed for fuzzing.

## Automated coverage

All 21 MuJoCo/core/Zig CTests pass in Release and under the sanitizer configuration
described below. The suite covers strict C and C++ ABI layouts, two C translation
units, C and C++ clients on MuJoCo, intrusive lists, alignment, exhaustion,
initialization failures, stable addresses, geometry sharing, scaled geometry,
clone isolation, physics-only IPC round trips, malformed versions/lengths,
compound objects, six/eight active motors, force/torque, cold reset, independent
concurrent worlds, module state cleanup, scratch allocation, allocation reuse,
rotor settling at 90/360/720 Hz, hydrodynamic cache cadence, and mesh contacts.
Additional build targets compile each public header independently as strict C11
and C++23. Dynamic symbol inspection confirms the shared core exports C functions.
Zig tests include C/Zig layout checks for scene sensors, AuvFrame, status,
and borrowed-scene options/results. Public headers also compile in C11/C++23.

Sensor tests cover independent rates, held samples, read-only observations,
reset determinism, mounts, stationary specific force, angular acceleration and
centripetal lever-arm terms, pressure offsets, fault boundaries, camera frustum
and range clipping, self-exclusion, missing routes, and detection limits. Passive
sensors leave body trajectories unchanged. Capacity/failure, clone, reference,
and frozen-model tests include the unified sensor storage.

Bridge tests exercise all four presets on MuJoCo, invalid commands,
initialization failures, real-time pacing, budgets, repeated destruction, and
optional status handling. Lifecycle tests verify fresh-world plugin reload,
controller-history reset, SIGINT cleanup, and the legacy stationary plugin.
Calibration checks regenerate the Hydrus controller configuration and verify
six independent unit wrenches against the shared thruster geometry.

Robot coverage includes all four compiled presets on MuJoCo, source-model
fingerprints over every physical field and geometry component after float
rounding, saved Blender asset hashes, capacity errors without partial insertion,
existing scenery membership, pose/ID overrides, duplicate/frozen rejection,
arena ownership, clone reference repair, observations, active motors, and
deterministic repetition after cold reset. The installed C example also runs
the packaged MuJoCo backend.

Counting callbacks verify every injected model-allocation failure releases prior
buffers; runtime-allocation and module-creation failures clean up their partial
state. Host model/runtime/adapter buffer counts stabilize after warm-up. This does not
count native engine allocations. The native concurrency harness additionally
injects adapter-world allocation failures and verifies the borrowed model remains
valid and all simulator-owned allocations are released.

## Packaged mission and fuzz results

The packaged Zig controller completes nominal Hydrus `prequalify_robosub` in
13,488 physics ticks (37.467 simulated seconds), below the 21,600-tick budget.
Its hash is `2389847940724445617`, depth `1.96881521–2.03377461 m`.

A packaged campaign using master seed 1, 100 cases and eight OS workers completed
100/100 missions. Completion ticks ranged from 12,036 to 17,958; observed depth
across cases ranged from 1.70670438 to 2.30292296 m. These measurements qualify
this seed/profile/scene/configuration, not every possible randomized mission.
Reports are in `build/fuzz-qualification-100`.

The regression suite requires complete JSON case equality between one and eight
workers for the same eight seeds, plus matching replay of an individual case.
Seed `10451216379200822465` completes in 12,948 ticks, hash
`3909802629446991208`; nominal-profile cases retain the baseline mission hash.
Tests cover tick-budget and initialization failures, cancellation, and injected
failure of the second worker launch. Each case has an independent allocator,
scene, RNG, and MissionContext; library handles remain pinned until all joins.

The normal 21-test suite passed in 28.52 seconds. The full ASan/UBSan/leak suite
passed all 21 tests in 283.29 seconds; its full fuzz regression took 242.33 seconds.
Sanitizer test wall timeouts are larger, while simulation tick limits are unchanged.
The native eight-thread harness also passed ThreadSanitizer. Exact launch and
replay commands, default ranges, and report conventions are in
[FUZZING.md](FUZZING.md).

## Historical double-reference comparison

The following measurements were captured for the earlier ABI-v4 float migration,
including historical Stonefish results. No Stonefish validation was rerun for
the MuJoCo-only fuzz delivery.
The v5 sensor migration retains those physical preset fields and adds separate
passive-sensor trajectory tests; these historical numbers are not new mission
error bounds.

The earlier working implementation at `/home/cesar/tmp/autonomy` was used read-only
to build an oracle in this checkout's build directory. It runs the same synthetic
five-collider body with six motors, 360 ticks at 360 Hz, and a command reversal
halfway through. Ten samples compare each engine against its earlier double
implementation; this is not an assertion that the engines match one another.

Measured maximum absolute differences for this fixture:

- MuJoCo: position `6.0884e-9 m`, linear velocity `1.4394e-8 m/s`, quaternion
  component `2.4420e-8`, angular velocity `5.7785e-10 rad/s`, rotor speed
  `8.2353e-10 rad/s`, and thrust `6.8020e-8 N`.
- Stonefish: position `6.0884e-9 m`, linear velocity `1.4394e-8 m/s`, quaternion
  component `2.6828e-8`, angular velocity `5.7785e-10 rad/s`, rotor speed
  `8.2353e-10 rad/s`, and thrust `6.8020e-8 N`.

Raw reference and float traces, the oracle source, and JSON differences are in
`build/core/reference`. Float conversion intentionally changes state hashes.
These numbers qualify this short, contact-free fixture; longer trajectories and
contact-sensitive scenarios can diverge further. Contact tests separately check
IDs, finite forces/normals, and physical interaction.

The four migrated presets were additionally compared with the previous native
C++ loader and runtime, using each unchanged standalone Blender asset. Each
engine ran 360 ticks at 360 Hz with command `0.05` on every active motor. Ten
samples per run compare that engine's double reference with its new float model.
Across all four robots, maximum position differences were `2.272e-7 m` for
MuJoCo and `2.640e-7 m` for Stonefish; linear velocity differences were
`2.368e-7 m/s` and `2.267e-7 m/s`, respectively. Maximum quaternion component
difference was below `3e-8`; angular velocity difference was below
`4.652e-8 rad/s`. These are short runs without scenery contacts, not bounds on
all future trajectories. Raw traces, reference build sources, and per-robot
measurements are in `build/robots-reference`.

## Sanitizers

Address/undefined sanitizers are enabled with `SIMULATOR_SANITIZE=ON`. All 21 configured
tests also pass with leak detection enabled. Validation uses Clang 18 because
this pinned MuJoCo version's ASan header has an attribute placement that GCC
rejects. Simulator core, MuJoCo adapter, AUV bridge, native tests, and examples are
instrumented; the native MuJoCo library and Zig executable are not.

The Clang build uses a shared ASan runtime in the host and instrumented plugins.
CTest supplies preloads to native tests and child Zig processes, leaving the
Python harnesses uninstrumented. Run the configured sanitizer suite with:

```sh
cmake -S . -B build/sanitize-clang -DAUTONOMY_BUILD_SIMULATOR=ON \
  -DSIMULATOR_ENABLE_STONEFISH=OFF -DSIMULATOR_SANITIZE=ON \
  -DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ \
  -DSIMULATOR_MUJOCO_PREFIX="$PWD/build/simulator" \
  -DSIMULATOR_MUJOCO_INCLUDE="$PWD/build/simulator/_deps/simulator_mujoco-src/include" \
  -DSIMULATOR_MUJOCO_LIB="$PWD/build/simulator/lib/libmujoco.so"
cmake --build build/sanitize-clang --target simulator_all -j8
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1 \
ctest --test-dir build/sanitize-clang --output-on-failure
```

The sanitizer host uses Zig's `-Dexternal-asan=true` option so ASan owns the
alternate signal stack. Zig 0.16's byte-aligned TLS signal stack otherwise fails
ASan's stack alignment checks when C++ exceptions cross its host stack. The
normal build retains Zig's usual signal handling.

MuJoCo retains small global registration allocations whose roots disappear on
library unload. Sanitizer tests preload the pinned native MuJoCo library to
keep those roots visible to LeakSanitizer. Backend plugins, worlds, runtime
buffers, and model buffers still undergo normal creation/destruction. No leak
suppressions are used. This qualifies simulator-owned storage and does not claim
that native engine global teardown is leak-free.


For the native ThreadSanitizer harness (separate from ASan):

```sh
cmake -S . -B build/fuzz-tsan -DAUTONOMY_BUILD_SIMULATOR=ON \
  -DSIMULATOR_ENABLE_STONEFISH=OFF -DSIMULATOR_THREAD_SANITIZE=ON \
  -DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ \
  -DSIMULATOR_MUJOCO_PREFIX="$PWD/build/simulator" \
  -DSIMULATOR_MUJOCO_INCLUDE="$PWD/build/simulator/_deps/simulator_mujoco-src/include" \
  -DSIMULATOR_MUJOCO_LIB="$PWD/build/simulator/lib/libmujoco.so"
cmake --build build/fuzz-tsan \
  --target simulator_concurrency_tests simulator_backend_mujoco -j8
ctest --test-dir build/fuzz-tsan -R simulator_mujoco_concurrency --output-on-failure
```

The harness pins engine libraries across all serial references and concurrent
worlds, matching the Zig coordinator. In an earlier harness that repeatedly
unloaded the engine, TSan reported initialization of MuJoCo's immutable static
indentation string inside an uninstrumented library. Keeping the actual intended
library lifetime resolves that diagnostic without suppressions. The TSan run
qualifies native simulator-owned storage; it does not instrument Zig or claim
coverage of every native-engine internal operation.
