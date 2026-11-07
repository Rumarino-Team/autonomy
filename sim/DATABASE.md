# Simulator database and ownership

The portable runtime uses one `sim::Main` per resolved model. It follows Blender's
intrusive-datablock organization while retaining the simulator's physical fields,
NED coordinates, xyzw quaternions, body IDs, and deterministic evaluation order.
It does not implement Blender's dependency graph or load engine XML.

Double-precision math storage is defined in `include/math.h`: `SimVec3` and
`SimQuat` alias `MathVector3d` and `MathQuaterniond`. `MathVector6d` pairs linear
and angular vectors (force and torque in hydrodynamics). These types use natural
double alignment, preserving the simulator ABI and encoded formats; the existing
SIMD-aligned float math types used by controllers are unchanged.

## Objects, colliders and shared geometry

`Main.objects` is the single scene-entity registry. Each `ObjectBlock` begins
with a `DataBlock`, which begins with `Link { next, prev }`. The header records
kind (`Object`), external body ID, database owner, owning list, and dense index.
Compile-time assertions enforce standard-layout/first-member assumptions.
`Main.find(DataKind::Object, id)` and `body_index(id)` resolve the same external
IDs used by engines, controllers, observations and Blender publishing.

The registry has an intrusive list and a dense pointer table. World objects are
pool-allocated; table growth moves pointers without relocating objects. The one
fixed robot embeds its ObjectBlock in `Robot`; the registry borrows that record
and never destroys it as a pooled allocation. Fault and module selections remain
separate run settings. Backend bindings remain private to their engine world.

An object owns an ordered `span<const Collider>`. A collider contains its local
pose, primitive/mesh description, contact coefficients, and a geometry view.
It has no datablock header, UID, intrusive links or list membership. Its typed
parent pointer supports ownership checks. Ordinary objects' contiguous collider
arrays live in the model arena. The fixed robot uses its embedded collider
array (up to five); its motors, fluid surfaces and camera also retain fixed
storage. Hydrus's five colliders still describe one rigid object.

Mesh geometry is a pair of immutable pointer/count spans over positions and
triangles. Finalization hashes normalized geometry bytes and confirms exact
array equality before sharing buffers. Different baked geometry/scaling remains
separate. There are no Mesh datablocks or reference counts: all finalized geometry lives
until Main destruction. A small ordered table of buffer views supports transport
and ownership validation; it does not give geometry database identities.

`BodyRec` and `ShapeRec` retain existing physical field definitions for codec and
backend compatibility. A BodyRec's shape range is a checked transport cache;
actual ownership is the ObjectBlock's collider span. Shape geometry ranges are
used only during import/encoding. Access geometry through
`Main.positions(collider)`, `faces(collider)`, or `positions(hull)`.

## Construction, freezing and cloning

Use a builder to add complete objects with their ordered colliders:

```cpp
sim::MainBuilder builder;
auto &draft = builder.draft();
sim::BodyRec physics; // set ID, mass, inertia, initial pose, etc.
const sim::Collider collider{}; // default primitive box
// Mesh importers also fill draft geometry arrays and collider ranges.
draft.add_object(physics, {&collider, 1});
auto model = builder.finish();
sim::World world(std::move(model), backend);
```

`add_object` copies collider descriptions and registers one stable object.
`edit_collider(object_index, local_index)` permits edits only while building.
Finalization validates physical values/ownership, interns mesh buffers, builds
lookups, copies hull geometry, updates transport counts, releases staging
buffers, and freezes topology. Runtime receives a const Main. Low-level mutable
block access is intended for builders and invariant tests, not live edits.

A Main cannot be copied or moved. Moving its unique_ptr transfers ownership
without relocating records. `Main.clone()` copies into an independent mutable
draft, repairs collider parents, and copies required geometry; finalization
restores sharing within the clone without retaining source pointers. Seeded
randomization uses this path. Frozen draft vertex arrays are empty, with no
retained capacity. Saved Blender topology edits take effect on the next run.

`World.observe()` returns a view whose object span is valid until the next
`observe()` on that world or World destruction. Copy it when retaining an
observation across those calls.

## Memory resources and destruction

Geometry uses `ArenaBuffer<T>` in `sim/arena.hpp`, with no PMR resource or
container underneath it. Each Main has independent typed buffers for draft
vertices, triangles and hull positions, and separate typed buffers for their
finalized data. Each buffer owns one contiguous virtual-memory reservation,
a used-element cursor and an element limit. Appending advances the cursor;
earlier addresses never move. `allocate(n)` returns a contiguous range for the
caller to fill, and `append(span)` copies a range into the arena.

The POSIX implementation reserves pages with `mmap(PROT_NONE)` and enables just
the required prefix with `mprotect`. Physical pages are supplied on demand.
Draft reservations cover u32 geometry indices so repeated input geometry can
exceed the final scene size before interning. Final geometry buffers are bounded
by the existing 128 MiB scene limit; binary validation also enforces the combined
scene limit. `reset()` reuses the reservation; `release()` unmaps it. Destruction
releases all remaining mappings. Finalization releases draft geometry buffers;
shared mesh and hull spans reference only the final typed arenas.

Remaining simulator-owned records and containers use local PMR resources: a pool
for records/reusable containers, a monotonic model arena for collider arrays,
commands and descriptor views, and a scratch arena for temporary metadata. The
allocation helpers construct/destroy records and unwind failed construction.
Scratch is released only after temporary users have been destroyed. Geometry
arrays never allocate through these resources.

States, contacts, commands, force-module state, observations and worker buffers
reuse PMR storage. Fixed workloads stabilize after warm-up; capacity may grow
when required. Calls on one world are serialized. Separate worlds own independent
resources and can run concurrently.

Records and containers are destroyed before their resources. Engine resources
are destroyed before source model buffers. Stonefish cold reset tears down the
old application/manager before replacement. Native engine allocations retain
their engine-specific ownership rules; simulator binding wrappers use pools.

Typed-arena tests check alignment, stable addresses, capacity/overflow failures,
reset/release/reuse, lazy commitment, independent buffers and OS unmapping.
Tests inject counting PMR resources to check remaining container reuse and cleanup. Model-arena failures
and initial pool failure are injected independently. A standalone reproduction
on this machine's libstdc++ crashes inside `unsynchronized_pool_resource` when a
later internal bookkeeping allocation fails; the tests do not qualify that
standard-library failure path.

## Backend ABI 3

`SimBackendApi.world_create` borrows a `const SimSceneView *` for the call.
The C view contains model settings, body/shape tables, and mesh pointer/count
pairs. Shapes reference body and mesh table indices; `UINT32_MAX` means no mesh.
No STL type, intrusive link, resource, or engine pointer crosses this interface.
Each adapter copies retained input before returning. Overwriting or freeing the
caller's descriptors afterwards cannot affect the created world.

Each adapter reconstructs objects with contained colliders from the flat C
view. MuJoCo builds `mjSpec` directly and stores body/geometry IDs in pooled
bindings that reference objects and contained colliders. The Stonefish proxy serializes the view to
its worker, which reconstructs a private Main and uses pooled bindings containing
Stonefish/Bullet pointers. Engine states and contacts still use caller-owned C
output buffers. ABI-v2 plugins are rejected; rebuild the runtime and plugins as
one distribution.

## ASIMSC03 and worker protocol 2

ASIMSC03 is a resolved-model recording/IPC format, not a Python exporter handoff.
Normal launches read the saved `.blend` in C++. Profile MessagePack encoding is
unchanged. No code serializes a Main, native pointers, or C++ struct padding.

The prefix is eight magic bytes `ASIMSC03` and a little-endian u64 payload length,
limited to 128 MiB. The payload starts with u32 section count 13 and u32 directory
schema 1. Each directory entry is u32 kind, u32 count, u64 payload-relative offset,
and u64 byte length. Sections are contiguous, uniquely ordered and checked before
record allocation. Their kinds are:

1. Settings/provenance, 408 bytes per record (exactly one).
2. Bodies, 328 bytes per record.
3. Shapes, 168 bytes per record.
4. Shared mesh positions, 24 bytes per xyz double triple.
5. Shared mesh triangles, 12 bytes per u32 index triple.
6. Thrusters, 120 bytes per record.
7. Hydrodynamic hulls, 184 bytes per record.
8. Hull positions, 24 bytes per xyz double triple.
9. Faults, 32 bytes per record.
10. Module names, 32 bytes per zero-padded name.
11. Command samples, 8 bytes per double.
12. Mesh ranges, four u32 fields: position begin/count, triangle begin/count.
13. Shape-to-mesh references, one u32 mesh index or `UINT32_MAX` per shape.

The scalar field visitors in `scene_codec.cpp` and Python's explicit struct
formats define these widths; C++ sizeof/offsetof is not the wire schema. Meshes
appear once in table order. Body, shape, thruster, hull, fault, module and command
orders retain their physical meaning. Decoding constructs records first, resolves
references into contained colliders, interns buffers during finalization, and
checks the resulting graph. C++ and Python
produce the same canonical bytes, including shared geometry.

Worker operation magics end in `02`. The length framing remains u32 little-endian.
Headers and every state/wrench/contact field are encoded explicitly; body states
occupy 132 bytes, wrenches 52 bytes, and contacts 64 bytes, excluding C padding.
Incoming/outgoing buffers are retained and reused across steps.

ASIMSC02 recordings and worker-v1 messages are rejected. There is no converter:
regenerate recordings and rebuild both plugins and the worker together. Blender
preview AMJB packets and the controller's C interface keep their existing formats.
