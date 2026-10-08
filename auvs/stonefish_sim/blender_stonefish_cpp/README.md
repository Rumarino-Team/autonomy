# Native Blender scene loader (experimental)

This alternative reads `.blend` files with CBlend and registers the environment
and static obstacles directly with `sf::SimulationManager`. It uses the same
addon properties and `../blender_stonefish/config.yaml` as the Python importer.
The native importer uses no XML scenario or Python runtime.

Both paths are available: `libauv_stonefish_sim.so` uses Python and
`libauv_stonefish_cpp_sim.so` uses the native scene loader. The Python package,
addon and original plugin are retained. Robot definitions in `../robot.cpp`
still use their existing C++ helper through its C wrapper; that helper is shared
by both plugins. Blender scene construction calls the Stonefish C++ API directly.

`blend_scene.cpp` reads mesh data, tags and transforms; `build_scene.cpp` creates
materials, looks, ocean, current, atmosphere and static obstacles. Vertices, faces
and UVs are passed to Stonefish in memory. Identical geometry is built once and
copied per obstacle. `inspect` can still write OBJ files for comparison.

## Try the built plugin

Run from the `platform_json_parsing` checkout:

```sh
cd /home/cesar/autonomy
./zig-out/bin/src_3 ./build/blend-cpp/libauv_stonefish_cpp_sim.so prequalify ./auvs/stonefish_sim/hydrus.json
```

The same `platform.json` selects the scene, vehicle and rendering settings.
The existing file watcher rebuilds the native scene on edits. Unsupported blend
data is checked before destroying the running world.

## Build

The new Zig option builds and installs the alternative plugin:

```sh
zig build -Dstonefish-cpp run -- ./zig-out/lib/libauv_stonefish_cpp_sim.so prequalify ./auvs/stonefish_sim/hydrus.json
```

Dependencies: a C++23 compiler for the simulator, CMake 3.20+, yaml-cpp, zlib
and zstd development packages. CMake fetches CBlend and its header dependencies
at pinned revisions. Configuring the retained Python target still requires
Python development headers, but the native plugin does not link `libpython`.

The tested build on this machine uses the existing GCC 14 SDK and upstream
Stonefish install, keeping the usual `build/` cache intact:

```sh
cmake -S . -B build/blend-cpp \
  -DAUTONOMY_BLEND_CPP=ON \
  -DAUTONOMY_STONEFISH_PREFIX="$PWD/build/gcc14-up/stonefish" \
  -DCMAKE_CXX_COMPILER="$PWD/build/_sdk/gcc14/bin/x86_64-conda-linux-gnu-g++" \
  -DCMAKE_CXX_FLAGS="--sysroot=/ -isystem$PWD/vendor/nlohmann_json/single_include" \
  -Dyaml-cpp_DIR=/usr/lib/x86_64-linux-gnu/cmake/yaml-cpp \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build/blend-cpp --target auv_stonefish_cpp_sim stonefish_blend_inspect stonefish_blend_smoke stonefish_blend_plugin_smoke -j4
```

On another machine, use your normal compiler and omit the machine-specific
compiler, YAML path and Stonefish prefix overrides. Without a prefix, the
existing ExternalProject builds Stonefish as usual.

## Validate

```sh
python3 auvs/stonefish_sim/blender_stonefish_cpp/tests/compare_legacy.py \
  build/blend-cpp/auvs/stonefish_sim/blender_stonefish_cpp/stonefish_blend_inspect
build/blend-cpp/stonefish_blend_smoke auvs/stonefish_sim/data auvs/stonefish_sim/blender_stonefish/config.yaml
build/blend-cpp/stonefish_blend_plugin_smoke ./build/blend-cpp/libauv_stonefish_cpp_sim.so
```

Python is used only to compare the native results with the retained importer.
That check covers all 20 objects, tags, transforms, vertices, polygons, UVs,
normals and winding; mirrored scale; gzip/zstd compression; and error cases.
The simulation check creates the pool and Hydrus robot, advances physics and
rebuilds the scene. The plugin check loads the actual shared library through
the AUV API and checks 36 frames and camera metadata. The simulation checks
require a display and OpenGL 4.3 because Hydrus includes a camera.

## Supported data

Validated with the checked-in Blender 4.5 pool. The reader accepts 64-bit
little-endian Blender 4.x files using `position`, `.corner_vert`,
`poly_offset_indices` and optional UV attributes, including gzip and zstd
compression. Blender 5.x and other mesh layouts are rejected explicitly.

The same `stonefish` property, `SF` prefix and eight pool fallback names select
objects. Custom `stonefish_name`, `material`, `look`, `cls` and `convex` properties
are read. Scale is baked into vertices and the current pool coordinate mapping
is preserved: position `(-x, y, -z)`, rotation `(-roll, pitch, -yaw)`.

Objects must be unparented, use XYZ Euler rotation and have applied modifiers.
The reader uses stored meshes and does not evaluate Geometry Nodes,
constraints, animations, instanced collections or Blender shader graphs.
Textures and simulation properties continue to come from the YAML settings.

CBlend is MIT licensed, pinned to
[`a80b576`](https://github.com/SK83RJOSH/cblend/tree/a80b57666faf001a04ee00972a4a84871894099e).
Its original multidimensional-array parser shifts later DNA field offsets.
`cblend_process_field.inc` fixes that function in a build-local copy. The
downloaded source is untouched; the pool geometry regression catches this bug.
