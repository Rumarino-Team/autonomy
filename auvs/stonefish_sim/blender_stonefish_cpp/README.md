# Native Blender scene loader (experimental)

The Stonefish simulator reads `.blend` files with CBlend and registers the
environment and static obstacles directly with `sf::SimulationManager`. It uses
the scene metadata in `../stonefish_config.yaml` and the properties stamped by
the Blender add-on. Robot definitions and Blender scene construction call the
Stonefish C++ API directly.

`blend_scene.cpp` reads mesh data, tags and transforms; `build_scene.cpp` creates
materials, looks, ocean, current, atmosphere and static obstacles. Vertices, faces
and UVs are passed to Stonefish in memory. Identical geometry is built once and
copied per obstacle. `inspect` can still write OBJ files for comparison.

## Try the built plugin

Run from the repository checkout:

```sh
cd /home/cesar/autonomy
./zig-out/bin/src_3 ./build/libauv_stonefish_sim.so prequalify ./auvs/stonefish_sim/hydrus.json
```

The same `platform.json` selects the scene, vehicle and rendering settings.
The file watcher applies supported edits to the running world. Unsupported
data is rejected while the previous scene keeps running.

## Editing while the simulator runs

Save the `.blend`, `platform.json`, controller JSON (for example `proteus.json`),
`stonefish_config.yaml`, or images under the scene's `textures/` directory.
Scene saves are debounced for 200 ms and applied between physics steps on the
thread that owns the graphics context. Texture subdirectories are watched too.
Robot pose/velocity, simulation time, mission progress, sensor handles and
controller state survive a scene update.

Supported live scene edits:

- Move, rotate, scale, add, delete, or replace static Blender meshes and UVs.
- Change object `material`, `look`, `cls`, and `convex` properties.
- Update YAML colors, roughness, metalness, reflectivity, textures, normal maps,
  static physical materials, and friction pairs.
- Update sun position, ocean enable/disable, water density/type/temperature,
  wave height, particles, current velocity, and NED reference coordinates.
- Change timing, tracking settings, selected view camera, or Blender scene/data
  paths in `platform.json`.
- Update PID/controller JSON through the existing controller watcher.

Set a unique **string** custom property `stonefish_id` on a Blender object if
you want its identity (including the ID delivered to the controller) preserved
when you rename it. Without this property, its Stonefish name is its identity;
renaming is handled as deletion and addition. Duplicating an object with this
property requires giving the copy a new ID. Names starting with `__hot_reload_`
are reserved. Disabling all Blender meshes is allowed; the robot stays alive.

Changed meshes rebuild only the affected static collision bodies and OpenGL
objects. Converted mesh cache entries no longer used by the scene are freed.
RTX invalidates its copied scene resources and resets DLSS/motion history on
the next rendered frame, keeping its pipeline and camera buffers alive. A large
edit can briefly stall a frame while resources are rebuilt.

Invalid YAML/JSON, unsupported Blender data, missing/unreadable textures,
duplicate IDs, and names conflicting with robot/sensor entities are rejected
before existing static bodies are changed. Repair and save again to retry.
Deleting a referenced image keeps the current scene until it is restored or
the reference is removed from YAML.

Switching the robot or console/graphical mode still needs an explicit manual
restart; the watcher rejects those changes instead of resetting the mission.
Robot mass/inertia, joints, sensors and thruster construction are not rebuilt
by scene edits. Robot physical material copies retain their construction-time
values; live physical material updates apply to the static scene and friction
table. Rebuilding/reloading the simulation plugin itself also still restarts
the mission. Unsaved Blender edits are not streamed, and Blender shader nodes
remain outside this importer; use YAML for the simulation's render materials.

## Graphics controls in platform.json

The `graphics` section is applied on startup and reloaded on save. Invalid
values, misspelled graphics keys, and unknown look names reject the update.
JSON sun, ocean and look overrides take precedence over YAML, including when
YAML or Blender is subsequently saved. Set those individual overrides to `null`
or remove them to use the YAML values again. Texture paths remain in YAML.

| JSON field under `graphics` | Values / effect |
| --- | --- |
| `sun.azimuth_deg`, `sun.elevation_deg` | Sun direction in degrees; azimuth −360..360, elevation −90..90 |
| `ocean.render_enabled` | Show/hide the water visually; keeps water physics active |
| `ocean.jerlov` | 0..1, water clarity/attenuation/scattering |
| `ocean.particles` | Show suspended particles |
| `ocean.wave_height` | 0..2, Stonefish wave parameter; also affects water physics |
| `viewer_exposure_ev` | −20..20 exposure compensation for the main viewer |
| `camera_exposure_ev` | −20..20 exposure compensation for color sensors and their preview |
| `raster.anti_aliasing` | Toggle FXAA |
| `raster.ambient_occlusion` | Toggle raster ambient occlusion |
| `raster.screen_space_reflections` | Toggle raster screen space reflections |
| `rtx.enabled` | Switch color sensors between OpenGL and real-time OptiX; requires RTX build |
| `rtx.samples_per_pixel` | Integer 1..4096; more samples reduce path tracing noise |
| `rtx.max_bounces` | Integer 0..64; path tracing bounce limit |
| `rtx.dlss` | Enable/disable DLSS upscaling |
| `rtx.dlss_quality` | `quality`, `balanced`, `performance`, `ultra`, or `dlaa` |
| `rtx.debug_albedo` | Display the camera's material colors for debugging |
| `looks.<yaml-look-name>.color` | Three RGB components, each 0..1 |
| `looks.<yaml-look-name>.roughness`, `.metalness`, `.reflectivity` | Each 0..1 |

Example: change `graphics.rtx.enabled` to `true`, `samples_per_pixel` to `8`,
and `max_bounces` to `3` in the existing RTX block. To reduce glare on pool tiles,
set `graphics.looks.pool_tile.roughness` to `0.7`. `null` keeps the YAML value.
Exposure +1 EV doubles brightness and −1 EV halves it.

Raster effects control the main viewer and raster cameras. OptiX sensor shading
uses its own materials/path tracing settings. The RTX block selects the
real-time renderer; offline spectral settings continue to use their separate
configuration. Explicit JSON RTX settings override startup environment flags;
with no RTX block, existing environment configuration is preserved at startup.
Removing a previously active JSON RTX block disables that override's renderer.
Unavailable RTX/DLSS falls back to the available render path and logs a message.

Changing DLSS quality or enabling/disabling it recreates camera features and
resets temporal history at a render boundary. Changing samples/bounces resets
history without restarting the simulation. Window/camera dimensions and the
fixed raster shadow/atmosphere quality settings are not runtime controls here.

## Build

The Zig Stonefish option builds and installs the CBlend powered plugin:

```sh
zig build -Dstonefish run -- ./zig-out/lib/libauv_stonefish_sim.so prequalify ./auvs/stonefish_sim/hydrus.json
```

Dependencies: a C++23 compiler for the simulator, CMake 3.20+, yaml-cpp, zlib
and zstd development packages. CMake fetches CBlend and its header dependencies
at pinned revisions. The native plugin does not link `libpython`.

The tested build on this machine uses the existing GCC 14 SDK and upstream
Stonefish install, keeping the usual `build/` cache intact:

```sh
cmake -S . -B build \
  -DAUTONOMY_BLEND_CPP=ON \
  -DAUTONOMY_STONEFISH_PREFIX="$PWD/build/gcc14-up/stonefish" \
  -DCMAKE_CXX_COMPILER="$PWD/build/_sdk/gcc14/bin/x86_64-conda-linux-gnu-g++" \
  -DCMAKE_CXX_FLAGS="--sysroot=/ -isystem$PWD/vendor/nlohmann_json/single_include" \
  -Dyaml-cpp_DIR=/usr/lib/x86_64-linux-gnu/cmake/yaml-cpp \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build --target auv_stonefish_sim stonefish_blend_inspect stonefish_blend_smoke stonefish_blend_plugin_smoke -j4
```

On another machine, use your normal compiler and omit the machine-specific
compiler, YAML path and Stonefish prefix overrides. Without a prefix, the
existing ExternalProject builds Stonefish as usual.

## Validate

```sh
build/stonefish_blend_smoke auvs/stonefish_sim/data auvs/stonefish_sim/stonefish_config.yaml
build/stonefish_blend_plugin_smoke ./build/libauv_stonefish_sim.so
```

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
objects. Custom `stonefish_name`, `stonefish_id`, `material`, `look`, `cls` and `convex` properties
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
