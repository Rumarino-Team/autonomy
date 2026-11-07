#include "backends/wire.hpp"
#include "runtime.hpp"
#include <cstring>
#include <iostream>
#include <limits>
using namespace sim;
static_assert(std::is_same_v<SimVec3, MathVector3d>);
static_assert(std::is_same_v<SimQuat, MathQuaterniond>);
static_assert(std::is_standard_layout_v<MathVector6d> &&
              std::is_trivially_copyable_v<MathVector6d>);
static_assert(sizeof(MathVector6d) == 6 * sizeof(double));
namespace {
void expect(bool b, const char *message) {
  if (!b)
    throw std::runtime_error(message);
}
template <class F> void rejects(F f) {
  bool caught = false;
  try {
    f();
  } catch (const std::invalid_argument &) {
    caught = true;
  }
  expect(caught, "invalid database input was accepted");
}
struct Counting : std::pmr::memory_resource {
  size_t calls = 0, bytes = 0, peak = 0, fail_at = SIZE_MAX;
  void *do_allocate(size_t n, size_t a) override {
    if (calls++ >= fail_at)
      throw std::bad_alloc();
    auto *p = std::pmr::new_delete_resource()->allocate(n, a);
    bytes += n;
    peak = std::max(peak, bytes);
    return p;
  }
  void do_deallocate(void *p, size_t n, size_t a) override {
    bytes -= n;
    std::pmr::new_delete_resource()->deallocate(p, n, a);
  }
  bool do_is_equal(const std::pmr::memory_resource &r) const noexcept override {
    return this == &r;
  }
};
struct alignas(64) Probe {
  static inline int alive = 0;
  Probe() { ++alive; }
  ~Probe() { --alive; }
  std::byte data[64]{};
};
void populate(Main &main, bool separate = false) {
  main.header.gravity = {0, 0, 0};
  const SimVec3 positions[] = {{-1, -1, -1}, {1, -1, -1}, {1, 1, -1},
                               {-1, 1, -1},  {-1, -1, 1}, {1, -1, 1},
                               {1, 1, 1},    {-1, 1, 1}};
  const SimTriangle triangles[] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7},
                                   {0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5},
                                   {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
  for (uint32_t i = 0; i < 2; ++i) {
    BodyRec b;
    b.id = 7 + i;
    b.motion = SIM_DYNAMIC;
    b.mass = b.dry_mass = 2;
    b.initial_pose.position = {double(i) * 4, 0, 2};
    b.shape_begin = i;
    b.shape_count = 1;
    copy_text(b.name, sizeof(b.name), "cube");
    ShapeRec s;
    s.kind = SIM_MESH;
    s.vertex_begin = main.vertices.size();
    s.vertex_count = 8;
    s.triangle_begin = main.triangles.size();
    s.triangle_count = 12;
    for (auto p : positions) {
      if (separate && i)
        p.x *= 2;
      main.vertices.push_back(p);
    }
    main.triangles.append(triangles);
    const Collider collider{s};
    main.add_object(b, {&collider, 1});
  }
  ThrusterRec thruster;
  main.robot.add_thruster(thruster);
  FaultRec fault;
  fault.kind = FaultKind::Weak;
  fault.start = 1;
  fault.end = 2;
  fault.magnitude = .5;
  main.run.faults.push_back(fault);
  HullRec hull;
  hull.vertex_count = 3;
  main.robot.add_hull(hull);
  main.hull_vertices.append(std::span(positions).first(3));
}
uint64_t little(const std::vector<uint8_t> &b, size_t at, size_t n) {
  uint64_t v = 0;
  for (size_t i = 0; i < n; ++i)
    v |= uint64_t(b.at(at + i)) << (i * 8);
  return v;
}
void links() {
  ListBase list, other;
  DataBlock a, b, c;
  append(list, a);
  append(list, b);
  append(list, c);
  expect(list.first == &a.link && list.last == &c.link &&
             c.link.prev == &b.link,
         "list ordering");
  rejects([&] { append(other, a); });
  rejects([&] { unlink(other, a); });
  unlink(list, b);
  expect(a.link.next == &c.link && c.link.prev == &a.link, "unlink middle");
  unlink(list, a);
  unlink(list, c);
  expect(!list.first && !list.last, "empty list endpoints");
}
void shared_math() {
  // These increments would disappear if a shared vector stored float values.
  const double precise = 1.0 + 0x1p-40;
  MathVector6d wrench{{precise, 2, 3}, {4, 5, -precise}};
  SimVec3 force = wrench.linear;
  expect(force.x == precise && force.x != 1.0 && wrench.angular.z == -precise,
         "shared math lost double precision");
  expect(dot(force, {1, 0, 0}) == precise,
         "shared vector arithmetic lost precision");
}
void typed_arenas() {
  static_assert(!std::is_copy_constructible_v<ArenaBuffer<SimVec3>>);
  ArenaBuffer<SimVec3> vertices(1024), hulls(16);
  ArenaBuffer<SimTriangle> triangles(16);
  expect(vertices.empty() && !vertices.data() && !vertices.reserved_bytes(),
         "empty arena reserved storage eagerly");
  vertices.append({});
  expect(!vertices.data(), "empty append reserved an arena");
  const SimVec3 initial[] = {{1, 2, 3}, {4, 5, 6}};
  auto first = vertices.append(initial);
  hulls.append(initial);
  triangles.push_back({0, 1, 0});
  auto *address = first.data();
  expect(address != hulls.data() &&
             reinterpret_cast<void *>(address) != triangles.data(),
         "typed arenas share storage");
  for (size_t i = 2; i < 1024; ++i)
    vertices.push_back({double(i), 0, 0});
  expect(address == vertices.data() && first[1].z == 6 &&
             vertices[1023].x == 1023,
         "arena append moved or corrupted earlier ranges");
  auto expect_limit = [&](auto f) {
    bool caught = false;
    try {
      f();
    } catch (const std::length_error &) {
      caught = true;
    }
    expect(caught, "arena accepted an overflowing request");
  };
  expect_limit([&] { vertices.allocate(1); });
  expect(vertices.size() == 1024 && address == vertices.data(),
         "failed arena allocation changed the cursor");
  const auto writable = vertices.writable_bytes();
  vertices.reset();
  expect(vertices.empty() && vertices.data() == address &&
             vertices.writable_bytes() == writable,
         "arena reset discarded reusable storage");
  auto reused = vertices.append(initial);
  vertices.append(reused);
  expect(vertices.size() == 4 && vertices[3].z == 6,
         "arena could not append its own earlier range");
  vertices.release();
  expect(vertices.empty() && !vertices.data() && !vertices.capacity() &&
             !vertices.reserved_bytes() && !vertices.writable_bytes(),
         "arena release retained storage");
  // No intervening allocation: verify that release actually unmapped the page.
  unsigned char residency = 0;
  errno = 0;
  expect(mincore(address, 1, &residency) == -1 && errno == ENOMEM,
         "arena release did not return its pages to the OS");
  vertices.append(initial);
  expect(vertices[0].x == 1, "released arena could not be reused");
  struct alignas(64) Aligned {
    uint64_t values[8];
  };
  ArenaBuffer<Aligned> aligned(16);
  auto slot = aligned.allocate(1);
  expect(uintptr_t(slot.data()) % 64 == 0, "typed arena alignment");
  slot[0].values[0] = 17;
  expect_limit([&] { ArenaBuffer<SimVec3> huge(SIZE_MAX); });
  ArenaBuffer<SimVec3> zero(0);
  expect_limit([&] { zero.allocate(1); });
  // Draft indices may cover more than 128 MiB of repeated input geometry.
  // Address reservation must not make unused pages writable or initialize them.
  ArenaBuffer<SimVec3> large(UINT32_MAX);
  large.push_back({7, 8, 9});
  expect(large.capacity() == UINT32_MAX && large.size() == 1 &&
             large.writable_bytes() <= size_t(sysconf(_SC_PAGESIZE)) * 2,
         "large arena eagerly committed its capacity");
  void *destroyed = nullptr;
  {
    ArenaBuffer<SimTriangle> temporary(16);
    temporary.push_back({0, 1, 2});
    destroyed = temporary.data();
  }
  errno = 0;
  expect(mincore(destroyed, 1, &residency) == -1 && errno == ENOMEM,
         "arena destructor leaked its mapping");
}
void allocation() {
  Counting upstream;
  {
    Memory mem(&upstream);
    auto *p = mem.make<Probe>();
    expect(uintptr_t(p) % 64 == 0 && Probe::alive == 1,
           "pool alignment/construction");
    mem.destroy(p);
    expect(Probe::alive == 0, "pool destructor");
    for (int i = 0; i < 20; ++i) {
      auto *q = mem.make<Probe>();
      mem.destroy(q);
    }
    auto calls = upstream.calls;
    for (int i = 0; i < 1000; ++i) {
      auto *q = mem.make<Probe>();
      mem.destroy(q);
    }
    expect(upstream.calls == calls, "pool failed to reuse storage");
    for (int i = 0; i < 20; ++i) {
      {
        std::pmr::vector<double> temp(100, 0, &mem.scratch);
      }
      mem.scratch.release();
    }
  }
  expect(upstream.bytes == 0, "allocator resources leaked");
}
void models() {
  Counting upstream;
  {
    MainBuilder builder(&upstream);
    auto &draft = builder.draft();
    populate(draft);
    auto *body = &draft.objects.block(0);
    // Table growth moves only pointers, never the pooled nodes.
    for (int i = 0; i < 1000; ++i) {
      NameRec n{};
      copy_text(n.text, sizeof(n.text), ("module" + std::to_string(i)).c_str());
      draft.run.modules.push_back(n);
    }
    expect(body == &draft.objects.block(0), "record address moved");
    draft.run.modules.clear();
    NameRec n{};
    copy_text(n.text, sizeof(n.text), "gravity_compensation");
    draft.run.modules.push_back(n);
    auto main = builder.finish();
    expect(main->frozen() && main->geometry().size() == 1,
           "equal meshes weren't shared");
    expect(main->vertices.empty() && main->vertices.capacity() == 0 &&
               main->triangles.capacity() == 0,
           "draft geometry retained");
    expect(main->objects.block(0).colliders[0].geometry.positions.data() ==
               main->objects.block(1).colliders[0].geometry.positions.data(),
           "mesh buffer sharing");
    expect(main->positions(main->objects.block(0).colliders[0]).data() ==
               main->positions(main->objects.block(1).colliders[0]).data(),
           "mesh array sharing");
    expect(main->objects.block(0).colliders[0].object == body &&
               main->robot.thrusters()[0].body == body->id.index &&
               main->robot.hulls()[0].body == body->id.index &&
               main->robot.camera.body == body->id.index,
           "typed ownership");
    expect(main->run.faults.block(0).thruster == &main->robot.thrusters()[0],
           "typed fault reference");
    expect(main->find(DataKind::Object, 7) == &body->id &&
               main->body_index(8) == 1,
           "database lookup");
    auto clone = main->clone();
    clone->objects[0].initial_pose.position.x = 12;
    clone->finalize();
    expect(
        clone->objects.block(0).colliders[0].geometry.positions.data() ==
                clone->objects.block(1)
                    .colliders[0]
                    .geometry.positions.data() &&
            clone->objects.block(0).colliders[0].geometry.positions.data() !=
                main->objects.block(0).colliders[0].geometry.positions.data(),
        "clone alias repair");
    expect(main->objects[0].initial_pose.position.x == 0,
           "clone mutated source");
    const Collider primitive{};
    rejects([&] { main->add_object(BodyRec{}, {&primitive, 1}); });
    rejects([&] { main->edit_collider(0, 0); });
    rejects([&] { main->objects.clear(); });
    rejects([&] { main->robot.add_thruster(ThrusterRec{}); });
    rejects([&] { main->robot.add_hull(HullRec{}); });
    rejects([&] {
      checked_block<ModuleBlock>(body->id, DataKind::Module, main.get());
    });
    rejects([&] {
      checked_block<ObjectBlock>(body->id, DataKind::Object, clone.get());
    });
    auto &collider =
        const_cast<Collider &>(main->objects.block(0).colliders[0]);
    auto *original = collider.object;
    collider.object = &clone->objects.block(0);
    rejects([&] { main->validate(); });
    collider.object = original;
    auto geometry = collider.geometry;
    collider.geometry = clone->objects.block(0).colliders[0].geometry;
    rejects([&] { main->validate(); });
    collider.geometry = geometry;
    auto bytes = main->binary();
    auto restored = Main::read(bytes.data(), bytes.size());
    expect(restored->binary() == bytes && restored->geometry().size() == 1,
           "database wire roundtrip");
    auto view = main->view();
    auto imported = Main::from_view(view);
    expect(imported->geometry().size() == 1 &&
               imported->positions(imported->objects.block(0).colliders[0])
                       .data() != view.meshes[0].positions,
           "borrowed view wasn't copied");
    main.reset();
    imported->validate();
    expect(imported->objects[0].id == 7, "view retained caller storage");
    auto bad = bytes;
    bad[7] = '2';
    rejects([&] { Main::read(bad.data(), bad.size()); });
    bad = bytes;
    bad.pop_back();
    rejects([&] { Main::read(bad.data(), bad.size()); });
    bad = bytes;
    bad[16 + 8 + 8] = 0;
    rejects([&] { Main::read(bad.data(), bad.size()); });
    bad = bytes;
    size_t refs = 16 + little(bytes, 16 + 8 + 12 * 24 + 8, 8);
    bad[refs] = 255;
    rejects([&] { Main::read(bad.data(), bad.size()); });
    auto malformed = imported->view();
    malformed.mesh_count = UINT32_MAX;
    rejects([&] { Main::from_view(malformed); });
  }
  expect(upstream.bytes == 0, "Main leaked upstream allocations");
  MainBuilder distinct;
  populate(distinct.draft(), true);
  expect(distinct.finish()->geometry().size() == 2,
         "different scales shared a mesh");
  // Inject failures into the model arena independently of the standard pool
  // implementation. Also exercise failure of the initial pool reservation.
  {
    Counting c;
    c.fail_at = 0;
    try {
      Main m(&c);
    } catch (const std::bad_alloc &) {
    }
    expect(c.bytes == 0, "initial pool failure leaked");
  }
  size_t total = 0;
  {
    Counting pool, arena;
    {
      Main m(&pool, &arena);
      populate(m);
      m.finalize();
      m.view();
    }
    total = arena.calls;
    expect(pool.bytes == 0 && arena.bytes == 0, "successful build leaked");
  }
  expect(total > 0, "arena failure test did not allocate");
  for (size_t stop = 0; stop < total; ++stop) {
    Counting pool, arena;
    arena.fail_at = stop;
    try {
      Main m(&pool, &arena);
      populate(m);
      m.finalize();
      m.view();
    } catch (const std::bad_alloc &) {
    }
    expect(pool.bytes == 0 && arena.bytes == 0, "failed arena build leaked");
  }
}
// Qualify containment as well as sharing: a rigid object can have several
// colliders, and object-table growth must move neither records nor their parts.
void object_storage() {
  Counting upstream;
  {
    MainBuilder builder(&upstream);
    auto &draft = builder.draft();
    BodyRec body;
    copy_text(body.name, sizeof(body.name), "primitive");
    std::array<Collider, 2> parts{};
    parts[0].data.pose.position.x = -.5;
    parts[1].data.pose.position.x = .5;
    body.id = 1000;
    auto *first = &draft.add_object(body, parts);
    auto *storage = first->colliders.data();
    for (uint32_t i = 1; i < 128; ++i) {
      body.id = 1000 + i;
      draft.add_object(body, parts);
    }
    expect(first == &draft.objects.block(0) &&
               storage == first->colliders.data(),
           "object or contained collider moved during growth");
    auto main = builder.finish();
    expect(main->geometry().empty() && main->collider_count() == 256,
           "primitive objects allocated mesh geometry");
    size_t forward = 0, backward = 0;
    for (auto *p = main->objects.list.first; p; p = p->next) {
      expect(datablock(p).kind == DataKind::Object, "scene registry kind");
      ++forward;
    }
    for (auto *p = main->objects.list.last; p; p = p->prev)
      ++backward;
    expect(forward == 128 && backward == 128, "object list traversal");
    auto view = main->view();
    expect(view.body_count == 128 && view.shape_count == 256 &&
               view.mesh_count == 0 && view.bodies[1].shape_begin == 2 &&
               view.shapes[3].body == 1,
           "compound object transport ranges");
    auto decoded = Main::from_view(view);
    expect(decoded->objects.block(0).colliders.size() == 2 &&
               decoded->objects.block(0).colliders[0].data.pose.position.x ==
                   -.5 &&
               decoded->objects.block(0).colliders[1].data.pose.position.x ==
                   .5,
           "compound primitive properties lost");
    auto bytes = main->binary();
    expect(Main::read(bytes.data(), bytes.size())->binary() == bytes,
           "compound primitive encoding changed");
    auto &object = main->objects.block(0);
    auto owned = object.colliders;
    object.colliders = main->objects.block(1).colliders;
    rejects([&] { main->validate(); });
    object.colliders = owned;
  }
  expect(upstream.bytes == 0, "contained object storage leaked");
  MainBuilder mesh_builder;
  auto &draft = mesh_builder.draft();
  populate(draft);
  auto body = draft.objects[0];
  body.id = 9;
  body.initial_pose.position.x = 8;
  std::array<Collider, 2> parts{};
  parts[0].data = draft.objects.block(0).colliders[0].data;
  parts[1].data = parts[0].data;
  parts[1].data.pose.position.x = 3;
  draft.add_object(body, parts);
  auto main = mesh_builder.finish();
  expect(main->geometry().size() == 1 && main->collider_count() == 4 &&
             main->objects.block(2).colliders.size() == 2,
         "compound objects did not share geometry");
  const auto &c = main->objects.block(2).colliders[1];
  expect(
      c.geometry.positions.data() ==
              main->objects.block(0).colliders[0].geometry.positions.data() &&
          point(main->objects[2].initial_pose, c.data.pose.position).x == 11,
      "shared geometry changed independent transforms");
  auto copy = main->clone();
  copy->finalize();
  expect(copy->binary() == main->binary() && copy->geometry().size() == 1,
         "compound clone lost geometry sharing");
  // Engine view ownership checks must reject overlapping and orphaned parts.
  auto view = main->view();
  std::vector<SimBodyDesc> bodies(view.bodies, view.bodies + view.body_count);
  view.bodies = bodies.data();
  bodies[1].shape_begin = bodies[0].shape_begin;
  rejects([&] { Main::from_view(view); });
  view = main->view();
  std::vector<SimShapeDesc> shapes(view.shapes, view.shapes + view.shape_count);
  shapes.back().body = 0;
  view.shapes = shapes.data();
  rejects([&] { Main::from_view(view); });
}
void transport() {
  Counting upstream;
  {
    Memory mem(&upstream);
    WireBuffer encoded(&mem.pool);
    WorkerRequest request(&mem.pool);
    WorkerReply reply(&mem.pool);
    SimWrench w{7, {1, 2, 3}, {4, 5, 6}};
    worker_step(encoded, &w, 1);
    expect(encoded.size() == 24 + 52, "worker wrench contains padding");
    read_worker_request(encoded, request);
    expect(request.forces[0].torque.z == 6, "worker wrench roundtrip");
    SimBodyState state{
        7, {{1, 2, 3}, {0, 0, 0, 1}}, {4, 5, 6}, {7, 8, 9}, {10, 11, 12}};
    SimContact c{7, 8, {1, 2, 3}, {0, 0, 1}, 4};
    for (int i = 0; i < 20; ++i) {
      worker_reply(encoded, {&state, 1}, {&c, 1}, 42);
      read_worker_reply(encoded, reply);
    }
    auto calls = upstream.calls;
    for (int i = 0; i < 1000; ++i) {
      worker_reply(encoded, {&state, 1}, {&c, 1}, 42);
      read_worker_reply(encoded, reply);
    }
    expect(upstream.calls == calls,
           "worker transport allocations didn't stabilize");
    expect(reply.states[0].angular_velocity.z == 12 &&
               reply.contacts[0].normal_force == 4 && reply.hash == 42,
           "worker reply roundtrip");
    encoded.pop_back();
    rejects([&] { read_worker_reply(encoded, reply); });
    worker_reset(encoded, &state, 1);
    read_worker_request(encoded, request);
    expect(request.op == WorkerOp::Reset && request.states[0].id == 7,
           "worker reset roundtrip");
    encoded[7] = '1';
    rejects([&] { read_worker_request(encoded, request); });
  }
  expect(upstream.bytes == 0, "worker transport leaked");
}
void runtime(const char *library) {
  MainBuilder b;
  populate(b.draft());
  auto source = b.finish();
  auto backend = std::make_shared<Backend>(library);
  Counting counter;
  {
    World w(*source, backend, ForceRegistry{}, &counter);
    std::vector<double> commands(w.commands.size(), 0);
    for (int i = 0; i < 100; ++i) {
      w.step(commands);
      w.observe();
    }
    auto calls = counter.calls;
    for (int i = 0; i < 100; ++i) {
      w.step(commands);
      w.observe();
    }
    expect(counter.calls == calls,
           "world runtime upstream allocations didn't stabilize");
    for (int i = 0; i < 8; ++i) {
      w.reset();
      w.step(commands);
    }
  }
  expect(counter.bytes == 0, "world runtime leaked");
}
} // namespace
int main(int argc, char **argv) {
  try {
    shared_math();
    typed_arenas();
    links();
    allocation();
    models();
    object_storage();
    transport();
    if (argc > 1)
      runtime(argv[1]);
    std::cout << "database ownership, pools, arenas, cloning, wire, failures "
                 "and runtime reuse passed\n";
  } catch (const std::exception &e) {
    std::cerr << "database: " << e.what() << '\n';
    return 1;
  }
}
