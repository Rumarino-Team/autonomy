#include "internal.hpp"
#include "robots.h"
using namespace core;

struct SimulationStorage {
    SimulationConfig config;
    Buffer<ObjectBlock> objects;
    Buffer<FaultBlock> faults;
    Buffer<ModuleBlock> modules;
    Buffer<MathVector3f> vertices, hulls;
    Buffer<SimTriangle> triangles;
    Buffer<Collider> colliders;
    Buffer<SensorRec> sensors;
    Buffer<const ObjectBlock *> object_table;
    Buffer<const FaultBlock *> fault_table;
    Buffer<const ModuleBlock *> module_table;
    Buffer<MeshData> geometry;
    Buffer<uint64_t> hashes;

    explicit SimulationStorage(SimulationConfig c)
        : config(c),
          objects(c.allocator),
          faults(c.allocator),
          modules(c.allocator),
          vertices(c.allocator),
          hulls(c.allocator),
          triangles(c.allocator),
          colliders(c.allocator),
          sensors(c.allocator),
          object_table(c.allocator),
          fault_table(c.allocator),
          module_table(c.allocator),
          geometry(c.allocator),
          hashes(c.allocator) {
    }
};

static_assert(
    offsetof(IntrusiveDataNode, link) == 0 && offsetof(ObjectBlock, id) == 0 &&
    offsetof(FaultBlock, id) == 0 && offsetof(ModuleBlock, id) == 0
);

namespace {
constexpr uint32_t initialized = 0x53494d35;

void draft(Simulation *s) {
    require(s && s->initialized == initialized && s->storage, "uninitialized simulation");
    require(!s->frozen, "simulation is frozen");
}

void text(char *dst, size_t n, const char *s) {
    require(s && std::strlen(s) < n, "name too long");
    std::snprintf(dst, n, "%s", s);
}

void node(
    IntrusiveDataNode &n,
    DataKind kind,
    uint32_t uid,
    Simulation *s,
    ListBase &list,
    uint32_t i,
    bool pooled = true
) {
    n = {};
    n.kind = kind;
    n.uid = uid;
    n.owner = s;
    n.index = i;
    n.pooled = pooled;
    check(sim_list_append(&list, &n), "invalid membership");
}

uint64_t geometry_hash(const MeshData &m) {
    Hash h;
    h.scalar(m.position_count);
    h.scalar(m.triangle_count);
    for (uint32_t i = 0; i < m.position_count; ++i)
        h.vec(m.positions[i]);
    for (uint32_t i = 0; i < m.triangle_count; ++i) {
        h.scalar(m.triangles[i].a);
        h.scalar(m.triangles[i].b);
        h.scalar(m.triangles[i].c);
    }
    return h.value;
}

bool equal(const MeshData &a, const MeshData &b) {
    if (a.position_count != b.position_count || a.triangle_count != b.triangle_count)
        return false;
    for (uint32_t i = 0; i < a.position_count; ++i)
        for (unsigned k = 0; k < 3; ++k)
            if (a.positions[i].buf[k] != b.positions[i].buf[k])
                return false;
    for (uint32_t i = 0; i < a.triangle_count; ++i) {
        auto x = a.triangles[i], y = b.triangles[i];
        if (x.a != y.a || x.b != y.b || x.c != y.c)
            return false;
    }
    return true;
}

MeshData intern(Simulation &s, const MeshData &m) {
    auto &st = *s.storage;
    auto hash = geometry_hash(m);
    for (uint32_t i = 0; i < s.geometry_count; ++i)
        if (st.hashes[i] == hash && equal(st.geometry[i], m))
            return st.geometry[i];
    require(s.geometry_count < st.geometry.cap, "geometry table exhausted", SIM_CAPACITY);
    auto *p = static_cast<MathVector3f *>(arena_alloc(
        &s.vertices, bytes(m.position_count, sizeof(MathVector3f)), alignof(MathVector3f)
    ));
    auto *t = static_cast<SimTriangle *>(arena_alloc(
        &s.triangles, bytes(m.triangle_count, sizeof(SimTriangle)), alignof(SimTriangle)
    ));
    require(p && t, "geometry arena exhausted", SIM_CAPACITY);
    for (uint32_t i = 0; i < m.position_count; ++i) {
        auto a = m.positions[i];
        p[i] = v(
            a.buf[0] == 0 ? 0 : a.buf[0], a.buf[1] == 0 ? 0 : a.buf[1], a.buf[2] == 0 ? 0 : a.buf[2]
        );
    }
    std::copy_n(m.triangles, m.triangle_count, t);
    MeshData result{p, m.position_count, t, m.triangle_count};
    st.geometry[s.geometry_count] = result;
    st.hashes[s.geometry_count++] = hash;
    return result;
}

void shape_valid(const Collider &c) {
    const auto &g = c.data;
    require(g.kind <= SIM_MESH && g.collisions <= 1, "invalid collider kind/flags");
    valid_pose(g.pose);
    require(
        finite(g.size) && finite(g.drag) && finite(g.skin) && std::isfinite(g.friction) &&
            g.friction >= 0 && std::isfinite(g.restitution) && g.restitution >= 0 &&
            g.restitution <= 1,
        "invalid collider values"
    );
    if (g.kind == SIM_MESH) {
        auto &m = c.geometry;
        require(
            m.positions && m.triangles && m.position_count >= 4 && m.position_count <= 1000000 &&
                m.triangle_count && m.triangle_count <= 2000000,
            "invalid mesh arrays"
        );
        for (uint32_t i = 0; i < m.position_count; ++i)
            require(finite(m.positions[i]), "invalid mesh position");
        for (uint32_t i = 0; i < m.triangle_count; ++i) {
            auto t = m.triangles[i];
            require(
                t.a < m.position_count && t.b < m.position_count && t.c < m.position_count,
                "invalid triangle index"
            );
        }
    } else {
        require(
            !c.geometry.position_count && !c.geometry.triangle_count && !c.geometry.positions &&
                !c.geometry.triangles,
            "primitive has mesh data"
        );
        require(
            g.size.buf[0] > 0 && (g.kind != SIM_BOX || (g.size.buf[1] > 0 && g.size.buf[2] > 0)) &&
                (g.kind != SIM_CYLINDER || g.size.buf[2] > 0),
            "invalid primitive size"
        );
    }
}

void body_valid(const UnderwaterBody &b) {
    require(
        b.id != UINT32_MAX && b.motion_type <= SIM_KINEMATIC &&
            std::memchr(b.name, 0, sizeof b.name),
        "invalid body ID/type/name"
    );
    valid_pose(b.initial_pose);
    valid_pose(b.center_of_mass);
    require(
        finite(b.initial_velocity) && finite(b.initial_angular_velocity) && finite(b.inertia) &&
            finite(b.buoyancy_center) && std::isfinite(b.mass) && b.mass >= 0 &&
            std::isfinite(b.dry_mass) && b.dry_mass >= 0 && std::isfinite(b.volume) &&
            b.volume >= 0,
        "invalid body values"
    );
    if (b.motion_type == SIM_DYNAMIC)
        require(
            b.mass > 0 && b.inertia.buf[0] > 0 && b.inertia.buf[1] > 0 && b.inertia.buf[2] > 0,
            "dynamic mass/inertia must be positive"
        );
}

bool sensor_member(const Simulation &s, const SensorRec *p) {
    for (uint32_t i = 0; i < s.sensor_count; ++i)
        if (p == s.sensors + i)
            return true;
    return false;
}

void sensor_valid(const Simulation &s, const SensorRec &sensor) {
    require(member(s, sensor.object), "foreign sensor owner");
    require(
        sensor.kind <= SIM_SENSOR_CAMERA && sensor.enabled <= 1 && sensor.period_ticks,
        "invalid sensor kind, enabled flag, or sampling period"
    );
    valid_pose(sensor.pose);
    if (sensor.kind == SIM_SENSOR_POSE)
        require(
            sensor.settings.tracking.position_axes <= 7 &&
                sensor.settings.tracking.attitude_axes <= 7,
            "invalid tracking axis mask"
        );
    if (sensor.kind == SIM_SENSOR_CAMERA) {
        const auto &c = sensor.settings.camera;
        require(
            c.width && c.width <= 65536 && c.height && c.height <= 65536 &&
                std::isfinite(c.horizontal_fov) && c.horizontal_fov > 0 && c.horizontal_fov < 180 &&
                std::isfinite(c.near_range) && std::isfinite(c.far_range) && c.near_range > 0 &&
                c.far_range > c.near_range,
            "invalid camera settings"
        );
    }
}

void routing_valid(const Simulation &s, const SensorRouting &r) {
    const SensorRec *refs[] = {r.tracking, r.imu, r.pressure, r.camera};
    for (uint32_t i = 0; i < 4; ++i)
        if (refs[i])
            require(sensor_member(s, refs[i]) && refs[i]->kind == i, "invalid sensor routing");
    const ObjectBlock *owner = nullptr;
    for (auto *p : refs)
        if (p) {
            require(!owner || owner == p->object, "routed sensors must share one body");
            owner = p->object;
        }
}

void fault_valid(const Simulation &s, const FaultBlock &f) {
    require(
        f.kind <= SIM_FAULT_CAMERA && f.start < f.end && std::isfinite(f.magnitude), "invalid fault"
    );
    if (f.kind == SIM_FAULT_DEAD || f.kind == SIM_FAULT_WEAK) {
        require(
            !f.sensor && f.thruster < s.robot.thruster_count && f.magnitude >= 0 &&
                f.magnitude <= 1,
            "invalid actuator fault"
        );
    } else {
        require(sensor_member(s, f.sensor), "foreign sensor fault target");
        auto kind = f.kind == SIM_FAULT_TRACKING ? SIM_SENSOR_POSE
                    : f.kind == SIM_FAULT_DEPTH  ? SIM_SENSOR_PRESSURE
                    : f.kind == SIM_FAULT_CAMERA ? SIM_SENSOR_CAMERA
                                                 : SIM_SENSOR_IMU;
        require(f.sensor->kind == uint32_t(kind), "sensor fault kind mismatch");
    }
}
} // namespace

extern "C" {
void simulation_config_default(SimulationConfig *c) {
    if (c)
        *c = {32, 16, 16, 160, 100, 100, 100, 16, 65536, {}};
}

void simulation_settings_default(SimulationSettings *s) {
    if (s)
        *s = {1, 360, SIM_INTEGRATOR_DEFAULT, 0, 7, v(0, 0, 9.81), v(), 1025, {10000, 1000, 1000}};
}

void underwater_body_default(UnderwaterBody *b) {
    if (b) {
        *b = {};
        b->inertia = v(1, 1, 1);
        b->center_of_mass.quat = q();
        b->initial_pose.quat = q();
    }
}

void collider_default(Collider *c) {
    if (c) {
        *c = {};
        c->data.collisions = 1;
        c->data.pose.quat = q();
        c->data.size = v(1, 1, 1);
        c->data.friction = .5;
    }
}

void rotor_spec_default(RotorSpec *r) {
    if (r)
        *r = {1000, .18, .48, .05, .00146468, 1, 10, -1};
}

SimResult sim_list_append(ListBase *l, IntrusiveDataNode *n) {
    if (!l || !n || n->list || n->link.next || n->link.prev)
        return SIM_INVALID;
    if (bool(l->first) != bool(l->last))
        return SIM_INVALID;
    n->list = l;
    n->link.prev = l->last;
    if (l->last)
        l->last->next = &n->link;
    else
        l->first = &n->link;
    l->last = &n->link;
    return SIM_OK;
}

SimResult sim_list_unlink(ListBase *l, IntrusiveDataNode *n) {
    if (!l || !n || n->list != l)
        return SIM_INVALID;
    if (n->link.prev)
        n->link.prev->next = n->link.next;
    else
        l->first = n->link.next;
    if (n->link.next)
        n->link.next->prev = n->link.prev;
    else
        l->last = n->link.prev;
    n->link = {};
    n->list = nullptr;
    return SIM_OK;
}

SimResult sim_node_cast(const IntrusiveDataNode *n, DataKind k, const void *o, const void **out) {
    if (out)
        *out = nullptr;
    if (!out || !n || n->kind != k || n->owner != o)
        return SIM_INVALID;
    *out = n;
    return SIM_OK;
}

SimResult simulation_init(Simulation *s, const SimulationConfig *input, char *e, uint32_t n) {
    return guard(e, n, [&] {
        require(s && !s->initialized, "root must be zero initialized");
        SimulationConfig c;
        simulation_config_default(&c);
        if (input)
            c = *input;
        c.allocator = allocator(c.allocator);
        require(
            c.object_capacity && c.object_capacity <= 4096 &&
                c.collider_capacity <= 4096u * 1024u && c.module_capacity <= 4096 &&
                c.fault_capacity <= 1000000 && c.sensor_capacity <= 64 &&
                c.scratch_bytes <= SIM_MAX_SCENE_BYTES,
            "invalid model capacities"
        );
        *s = {};
        s->struct_size = sizeof(*s);
        s->abi_version = SIM_MODEL_ABI_VERSION;
        simulation_settings_default(&s->settings);
        try {
            auto *p = c.allocator.allocate(
                c.allocator.user, sizeof(SimulationStorage), alignof(SimulationStorage)
            );
            if (!p)
                throw std::bad_alloc();
            if (uintptr_t(p) % alignof(SimulationStorage)) {
                c.allocator.deallocate(
                    c.allocator.user, p, sizeof(SimulationStorage), alignof(SimulationStorage)
                );
                throw Error(SIM_INVALID, "misaligned storage");
            }
            s->storage = new (p) SimulationStorage(c);
            s->initialized = initialized;
            auto &t = *s->storage;
            size_t total = 0;
            auto reserve = [&](auto &b, size_t count) {
                auto sz = bytes(count, sizeof(*b.data()));
                require(
                    sz <= SIM_MAX_SCENE_BYTES - total, "model capacity exceeds limit", SIM_CAPACITY
                );
                total += sz;
                b.reserve(count);
            };
            reserve(t.objects, c.object_capacity);
            reserve(t.faults, c.fault_capacity);
            reserve(t.modules, c.module_capacity);
            reserve(t.vertices, c.vertex_capacity);
            reserve(t.triangles, c.triangle_capacity);
            reserve(t.hulls, c.hull_vertex_capacity);
            reserve(t.colliders, c.collider_capacity);
            reserve(t.sensors, c.sensor_capacity);
            reserve(t.object_table, c.object_capacity);
            reserve(t.fault_table, c.fault_capacity);
            reserve(t.module_table, c.module_capacity);
            reserve(t.geometry, c.collider_capacity);
            reserve(t.hashes, c.collider_capacity);
            check(
                pool_init(
                    &s->object_pool,
                    t.objects.p,
                    bytes(t.objects.cap, sizeof(ObjectBlock)),
                    sizeof(ObjectBlock),
                    alignof(ObjectBlock),
                    c.object_capacity
                )
                    ? SIM_OK
                    : SIM_INVALID,
                "object pool init"
            );
            check(
                pool_init(
                    &s->fault_pool,
                    t.faults.p,
                    bytes(t.faults.cap, sizeof(FaultBlock)),
                    sizeof(FaultBlock),
                    alignof(FaultBlock),
                    c.fault_capacity
                )
                    ? SIM_OK
                    : SIM_INVALID,
                "fault pool init"
            );
            check(
                pool_init(
                    &s->module_pool,
                    t.modules.p,
                    bytes(t.modules.cap, sizeof(ModuleBlock)),
                    sizeof(ModuleBlock),
                    alignof(ModuleBlock),
                    c.module_capacity
                )
                    ? SIM_OK
                    : SIM_INVALID,
                "module pool init"
            );
            arena_init(&s->vertices, t.vertices.p, bytes(t.vertices.cap, sizeof(MathVector3f)));
            arena_init(&s->triangles, t.triangles.p, bytes(t.triangles.cap, sizeof(SimTriangle)));
            arena_init(&s->hull_vertices, t.hulls.p, bytes(t.hulls.cap, sizeof(MathVector3f)));
            s->object_table = t.object_table.p;
            s->fault_table = t.fault_table.p;
            s->module_table = t.module_table.p;
            s->geometry = t.geometry.p;
            s->sensors = t.sensors.p;
        } catch (...) {
            simulation_destroy(s);
            throw;
        }
    });
}

void simulation_destroy(Simulation *s) {
    if (!s)
        return;
    if (s->storage) {
        auto a = s->storage->config.allocator;
        auto *p = s->storage;
        p->~SimulationStorage();
        a.deallocate(a.user, p, sizeof(SimulationStorage), alignof(SimulationStorage));
    }
    *s = {};
}

SimResult simulation_get_allocator(const Simulation *s, SimAllocator *out) {
    if (!s || !out || s->initialized != initialized || !s->storage)
        return SIM_INVALID;
    *out = s->storage->config.allocator;
    return SIM_OK;
}

SimResult simulation_add_object(
    Simulation *s,
    const UnderwaterBody *b,
    const Collider *c,
    uint32_t count,
    uint32_t fixed,
    ObjectBlock **out,
    char *e,
    uint32_t n
) {
    if (out)
        *out = nullptr;
    return guard(e, n, [&] {
        draft(s);
        require(b && c && out && count && count <= 1024 && fixed <= 1, "invalid object arguments");
        body_valid(*b);
        for (uint32_t i = 0; i < s->object_count; ++i)
            require(s->object_table[i]->body.id != b->id, "duplicate body ID");
        auto &t = *s->storage;
        require(s->object_count < t.config.object_capacity, "object table exhausted", SIM_CAPACITY);
        require(
            !fixed || (!s->robot.registered && count <= SIM_MAX_ROBOT_COLLIDERS),
            "invalid fixed robot"
        );
        require(
            fixed || count <= t.colliders.cap - t.colliders.n,
            "collider arena exhausted",
            SIM_CAPACITY
        );
        for (uint32_t i = 0; i < count; ++i) {
            require(!c[i].object || c[i].object->id.owner == s, "foreign collider owner");
            shape_valid(c[i]);
        }
        auto vo = s->vertices.offset, to = s->triangles.offset;
        uint32_t gc = s->geometry_count;
        std::array<MeshData, 1024> meshes{};
        try {
            for (uint32_t i = 0; i < count; ++i)
                if (c[i].data.kind == SIM_MESH)
                    meshes[i] = intern(*s, c[i].geometry);
        } catch (...) {
            s->vertices.offset = vo;
            s->triangles.offset = to;
            s->geometry_count = gc;
            throw;
        }
        auto *object =
            fixed ? &s->robot.object : static_cast<ObjectBlock *>(pool_alloc(&s->object_pool));
        require(object, "object pool exhausted", SIM_CAPACITY);
        auto *colliders = fixed ? s->robot.colliders : t.colliders.p + t.colliders.n;
        if (!fixed)
            t.colliders.n += count;
        *object = {};
        object->body = *b;
        object->colliders = colliders;
        object->collider_count = count;
        for (uint32_t i = 0; i < count; ++i) {
            colliders[i] = {c[i].data, meshes[i], object};
        }
        node(object->id, SIM_DATA_OBJECT, b->id, s, s->objects, s->object_count, !fixed);
        t.object_table[s->object_count++] = object;
        if (fixed)
            s->robot.registered = 1;
        *out = object;
    });
}

SimResult simulation_add_thruster(Simulation *s, const Thruster *t, char *e, uint32_t n) {
    return guard(e, n, [&] {
        draft(s);
        require(t && member(*s, t->object), "foreign thruster object");
        require(s->robot.thruster_count < SIM_MAX_THRUSTERS, "thruster capacity", SIM_CAPACITY);
        require(
            finite(t->position) && finite(t->direction) && std::abs(norm(t->direction) - 1) < 1e-4,
            "invalid thruster direction"
        );
        const auto &r = t->spec;
        require(
            std::isfinite(r.max_rpm) && r.max_rpm > 0 && std::isfinite(r.diameter) &&
                r.diameter > 0 && std::isfinite(r.inertia) && r.inertia > 0 &&
                std::isfinite(r.thrust) && std::isfinite(r.torque) && std::isfinite(r.kp) &&
                r.kp >= 0 && std::isfinite(r.ki) && r.ki >= 0 &&
                (r.command_sign == 1 || r.command_sign == -1),
            "invalid rotor specification"
        );
        s->robot.thrusters[s->robot.thruster_count++] = *t;
    });
}

SimResult simulation_add_robot(
    Simulation *s,
    const Robot *definition,
    uint32_t body_id,
    const MathPose *pose,
    char *e,
    uint32_t n
) {
    return guard(e, n, [&] {
        draft(s);
        SimulationConfig needed{};
        char message[256];
        check(simulation_robot_config(definition, &needed, message, sizeof message), message);
        require(
            !s->robot.registered && !s->robot.thruster_count && !s->robot.hull_count,
            "robot hardware already configured"
        );
        const auto &r = *definition;
        require(
            r.sensor_count <= s->storage->sensors.cap - s->sensor_count,
            "sensor capacity",
            SIM_CAPACITY
        );

        // All attachments are fixed, validated preset values. Check the entire
        // surface allocation before inserting anything into the database.
        size_t hull_bytes = 0;
        for (uint32_t i = 0; i < r.hull_count; ++i)
            hull_bytes += bytes(r.hulls[i].position_count, sizeof(MathVector3f));
        require(
            hull_bytes <= s->hull_vertices.capacity - s->hull_vertices.offset,
            "robot hull arena exhausted",
            SIM_CAPACITY
        );
        auto body = r.object.body;
        body.id = body_id;
        if (pose)
            body.initial_pose = *pose;

        // Only cursors and newly appended entries change. Keep a checkpoint so
        // an invalid pose, exhausted geometry arena, or failed attachment never
        // leaves half a robot in the caller's draft. Existing records stay put.
        const Robot previous = s->robot;
        const size_t vertices = s->vertices.offset, triangles = s->triangles.offset;
        const size_t hulls = s->hull_vertices.offset;
        const uint32_t geometry = s->geometry_count, objects = s->object_count;
        const uint32_t sensor_count = s->sensor_count;
        const SensorRouting previous_routing = s->routing;
        try {
            ObjectBlock *object = nullptr;
            check(
                simulation_add_object(
                    s,
                    &body,
                    r.colliders,
                    r.object.collider_count,
                    1,
                    &object,
                    message,
                    sizeof message
                ),
                message
            );
            for (uint32_t i = 0; i < r.thruster_count; ++i) {
                auto thruster = r.thrusters[i];
                thruster.object = object;
                check(simulation_add_thruster(s, &thruster, message, sizeof message), message);
            }
            for (uint32_t i = 0; i < r.hull_count; ++i) {
                auto hull = r.hulls[i];
                hull.object = object;
                check(simulation_add_hull(s, &hull, message, sizeof message), message);
            }
            for (uint32_t i = 0; i < r.sensor_count; ++i) {
                auto sensor = r.sensors[i];
                sensor.object = object;
                const SensorRec *inserted = nullptr;
                check(
                    simulation_add_sensor(s, &sensor, &inserted, message, sizeof message), message
                );
            }
            s->robot.sensors = s->sensors + sensor_count;
            s->robot.sensor_count = r.sensor_count;
            SensorRouting routing{};
            for (uint32_t i = 0; i < r.sensor_count; ++i) {
                auto *sensor = &s->robot.sensors[i];
                switch (sensor->kind) {
                case SIM_SENSOR_POSE:
                    routing.tracking = sensor;
                    break;
                case SIM_SENSOR_IMU:
                    routing.imu = sensor;
                    break;
                case SIM_SENSOR_PRESSURE:
                    routing.pressure = sensor;
                    break;
                case SIM_SENSOR_CAMERA:
                    routing.camera = sensor;
                    break;
                }
            }
            check(simulation_set_sensor_routing(s, &routing, message, sizeof message), message);
            std::memcpy(s->robot.name, r.name, sizeof r.name);
            std::memcpy(s->robot.asset_file, r.asset_file, sizeof r.asset_file);
            std::memcpy(s->robot.asset_hash, r.asset_hash, sizeof r.asset_hash);
        } catch (...) {
            if (s->object_count != objects) {
                sim_list_unlink(&s->objects, &s->robot.object.id);
                s->storage->object_table[objects] = nullptr;
            }
            s->robot = previous;
            s->sensor_count = sensor_count;
            s->routing = previous_routing;
            s->object_count = objects;
            s->geometry_count = geometry;
            s->vertices.offset = vertices;
            s->triangles.offset = triangles;
            s->hull_vertices.offset = hulls;
            throw;
        }
    });
}

SimResult simulation_add_hull(Simulation *s, const HullRec *h, char *e, uint32_t n) {
    return guard(e, n, [&] {
        draft(s);
        require(h && member(*s, h->object), "foreign hull object");
        require(s->robot.hull_count < SIM_MAX_HULLS, "hull capacity", SIM_CAPACITY);
        require(
            h->positions && h->position_count && h->position_count % 3 == 0,
            "invalid hull triangle soup"
        );
        valid_pose(h->pose);
        valid_pose(MathPose{v(), h->coefficient_frame});
        require(
            finite(h->drag) && finite(h->skin) && finite(h->centroid) && std::isfinite(h->volume) &&
                h->volume >= 0,
            "invalid hull values"
        );
        for (uint32_t i = 0; i < h->position_count; ++i)
            require(finite(h->positions[i]), "invalid hull vertex");
        auto *p = static_cast<MathVector3f *>(arena_alloc(
            &s->hull_vertices, bytes(h->position_count, sizeof(MathVector3f)), alignof(MathVector3f)
        ));
        require(p, "hull arena exhausted", SIM_CAPACITY);
        std::copy_n(h->positions, h->position_count, p);
        auto &dest = s->robot.hulls[s->robot.hull_count++];
        dest = *h;
        dest.positions = p;
    });
}

void sensor_default(SensorRec *sensor, SensorKind kind) {
    if (!sensor)
        return;
    *sensor = {};
    sensor->kind = kind;
    sensor->pose.quat = q();
    sensor->enabled = 1;
    sensor->period_ticks = kind == SIM_SENSOR_IMU ? 1 : kind == SIM_SENSOR_CAMERA ? 12 : 6;
    if (kind == SIM_SENSOR_POSE)
        sensor->settings.tracking = {7, 7};
    if (kind == SIM_SENSOR_CAMERA)
        sensor->settings.camera = {800, 600, 60, .05, 50};
}

SimResult simulation_add_sensor(
    Simulation *s, const SensorRec *sensor, const SensorRec **out, char *e, uint32_t n
) {
    if (out)
        *out = nullptr;
    return guard(e, n, [&] {
        draft(s);
        require(sensor && out, "invalid sensor arguments");
        sensor_valid(*s, *sensor);
        for (uint32_t i = 0; i < s->sensor_count; ++i)
            require(s->sensors[i].id != sensor->id, "duplicate sensor ID");
        require(s->sensor_count < s->storage->sensors.cap, "sensor capacity", SIM_CAPACITY);
        auto *p = &s->storage->sensors[s->sensor_count++];
        *p = *sensor;
        *out = p;
    });
}

SimResult
simulation_set_sensor_routing(Simulation *s, const SensorRouting *routing, char *e, uint32_t n) {
    return guard(e, n, [&] {
        draft(s);
        require(routing, "null routing");
        routing_valid(*s, *routing);
        s->routing = *routing;
    });
}

SimResult simulation_add_fault(Simulation *s, const FaultBlock *f, char *e, uint32_t n) {
    return guard(e, n, [&] {
        draft(s);
        require(f, "null fault");
        fault_valid(*s, *f);
        auto *p = static_cast<FaultBlock *>(pool_alloc(&s->fault_pool));
        require(p, "fault capacity", SIM_CAPACITY);
        p->kind = f->kind;
        p->thruster = f->thruster;
        p->sensor = f->sensor;
        p->start = f->start;
        p->end = f->end;
        p->magnitude = f->magnitude;
        node(p->id, SIM_DATA_FAULT, s->fault_count, s, s->faults, s->fault_count);
        s->storage->fault_table[s->fault_count++] = p;
    });
}

SimResult simulation_add_module(Simulation *s, const char *name, char *e, uint32_t n) {
    return guard(e, n, [&] {
        draft(s);
        require(name && *name && std::strlen(name) < 64, "invalid module name");
        for (uint32_t i = 0; i < s->module_count; ++i)
            require(std::strcmp(name, s->module_table[i]->name) != 0, "duplicate module");
        auto *p = static_cast<ModuleBlock *>(pool_alloc(&s->module_pool));
        require(p, "module capacity", SIM_CAPACITY);
        text(p->name, sizeof p->name, name);
        node(p->id, SIM_DATA_MODULE, s->module_count, s, s->modules, s->module_count);
        s->storage->module_table[s->module_count++] = p;
    });
}

SimResult simulation_validate(const Simulation *s, char *e, uint32_t n) {
    return guard(e, n, [&] {
        require(
            s && s->struct_size == sizeof(*s) && s->abi_version == SIM_MODEL_ABI_VERSION &&
                s->initialized == initialized && s->storage,
            "invalid model version/root"
        );
        const auto &c = s->storage->config;
        require(
            s->object_table == s->storage->object_table.p &&
                s->fault_table == s->storage->fault_table.p &&
                s->module_table == s->storage->module_table.p &&
                s->geometry == s->storage->geometry.p,
            "foreign model tables"
        );
        require(
            s->vertices.data == reinterpret_cast<const uint8_t *>(s->storage->vertices.p) &&
                s->vertices.offset <= s->vertices.capacity &&
                s->triangles.data == reinterpret_cast<const uint8_t *>(s->storage->triangles.p) &&
                s->triangles.offset <= s->triangles.capacity,
            "invalid geometry arenas"
        );
        require(
            s->object_count && s->object_count <= c.object_capacity &&
                s->fault_count <= c.fault_capacity && s->module_count <= c.module_capacity &&
                s->geometry_count <= c.collider_capacity && s->robot.thruster_count <= 8 &&
                s->robot.hull_count <= 5 && s->sensor_count <= c.sensor_capacity,
            "invalid model counts"
        );
        auto &d = s->settings;
        require(
            d.dt_numerator && d.dt_denominator && d.integrator <= SIM_INTEGRATOR_RK4 &&
                d.hydro_period && d.native_hydrodynamics <= 1 && finite(d.gravity) &&
                finite(d.current) && std::isfinite(d.water_density) && d.water_density > 0 &&
                std::isfinite(d.bounds.position) && d.bounds.position > 0 &&
                std::isfinite(d.bounds.speed) && d.bounds.speed > 0 &&
                std::isfinite(d.bounds.angular_speed) && d.bounds.angular_speed > 0,
            "invalid settings"
        );
        auto list = [&](const ListBase &l, auto table, uint32_t count, uint32_t kind) {
            auto *p = l.first;
            Link *prev = nullptr;
            for (uint32_t i = 0; i < count; ++i) {
                require(p && table && table[i] && p == &table[i]->id.link, "list/table mismatch");
                auto &id = table[i]->id;
                require(
                    id.owner == s && id.kind == kind && id.list == &l && id.index == i &&
                        p->prev == prev,
                    "invalid intrusive owner/type/index"
                );
                prev = p;
                p = p->next;
            }
            require(!p && prev == l.last, "list count mismatch");
        };
        list(s->objects, s->object_table, s->object_count, SIM_DATA_OBJECT);
        list(s->faults, s->fault_table, s->fault_count, SIM_DATA_FAULT);
        list(s->modules, s->module_table, s->module_count, SIM_DATA_MODULE);
        for (uint32_t i = 0; i < s->object_count; ++i) {
            auto &o = *s->object_table[i];
            body_valid(o.body);
            require(o.id.uid == o.body.id && o.id.pooled <= 1, "invalid object identity");
            require(
                o.colliders && o.collider_count && o.collider_count <= 1024,
                "invalid collider array"
            );
            for (uint32_t j = 0; j < i; ++j)
                require(s->object_table[j]->body.id != o.body.id, "duplicate body ID");
            for (uint32_t k = 0; k < o.collider_count; ++k) {
                auto &g = o.colliders[k];
                require(g.object == &o, "foreign collider");
                shape_valid(g);
                if (g.data.kind == SIM_MESH) {
                    bool found = false;
                    for (uint32_t m = 0; m < s->geometry_count; ++m)
                        found |= g.geometry.positions == s->geometry[m].positions &&
                                 g.geometry.triangles == s->geometry[m].triangles &&
                                 g.geometry.position_count == s->geometry[m].position_count &&
                                 g.geometry.triangle_count == s->geometry[m].triangle_count;
                    require(found, "unregistered geometry");
                }
            }
        }
        require(
            !s->robot.registered ||
                (member(*s, &s->robot.object) && s->robot.object.colliders == s->robot.colliders &&
                 s->robot.object.collider_count <= 5),
            "invalid embedded robot"
        );
        for (uint32_t i = 0; i < s->robot.thruster_count; ++i) {
            const auto &t = s->robot.thrusters[i];
            const auto &r = t.spec;
            require(
                member(*s, t.object) && finite(t.position) && finite(t.direction) &&
                    std::abs(norm(t.direction) - 1) < 1e-4,
                "invalid thruster"
            );
            require(
                std::isfinite(r.max_rpm) && r.max_rpm > 0 && std::isfinite(r.diameter) &&
                    r.diameter > 0 && std::isfinite(r.inertia) && r.inertia > 0 &&
                    std::isfinite(r.thrust) && std::isfinite(r.torque) && std::isfinite(r.kp) &&
                    r.kp >= 0 && std::isfinite(r.ki) && r.ki >= 0 &&
                    (r.command_sign == 1 || r.command_sign == -1),
                "invalid rotor"
            );
        }
        for (uint32_t i = 0; i < s->robot.hull_count; ++i) {
            const auto &h = s->robot.hulls[i];
            require(
                member(*s, h.object) && h.positions && h.position_count &&
                    h.position_count % 3 == 0,
                "invalid hull"
            );
            auto begin = reinterpret_cast<uintptr_t>(s->hull_vertices.data),
                 address = reinterpret_cast<uintptr_t>(h.positions);
            require(
                address >= begin && address - begin <= s->hull_vertices.offset &&
                    bytes(h.position_count, sizeof(MathVector3f)) <=
                        s->hull_vertices.offset - (address - begin),
                "foreign hull geometry"
            );
            valid_pose(h.pose);
            valid_pose(MathPose{v(), h.coefficient_frame});
            require(
                finite(h.drag) && finite(h.skin) && finite(h.centroid) && std::isfinite(h.volume) &&
                    h.volume >= 0,
                "invalid hull values"
            );
            for (uint32_t j = 0; j < h.position_count; ++j)
                require(finite(h.positions[j]), "invalid hull vertex");
        }
        require(s->sensors == s->storage->sensors.p, "foreign sensor table");
        for (uint32_t i = 0; i < s->sensor_count; ++i) {
            sensor_valid(*s, s->sensors[i]);
            for (uint32_t j = 0; j < i; ++j)
                require(s->sensors[i].id != s->sensors[j].id, "duplicate sensor ID");
        }
        if (s->robot.sensor_count) {
            require(s->robot.sensor_count <= s->sensor_count, "invalid robot sensors");
            bool found = false;
            for (uint32_t i = 0; i + s->robot.sensor_count <= s->sensor_count; ++i)
                if (s->robot.sensors == s->sensors + i)
                    found = true;
            require(found, "foreign robot sensor view");
        }
        routing_valid(*s, s->routing);
        for (uint32_t i = 0; i < s->fault_count; ++i)
            fault_valid(*s, *s->fault_table[i]);
        for (uint32_t i = 0; i < s->module_count; ++i) {
            require(
                s->module_table[i]->name[0] && std::memchr(s->module_table[i]->name, 0, 64),
                "invalid force module name"
            );
            for (uint32_t j = 0; j < i; ++j)
                require(
                    std::strcmp(s->module_table[i]->name, s->module_table[j]->name) != 0,
                    "duplicate module selection"
                );
        }
    });
}

SimResult
simulation_draft_object(Simulation *s, uint32_t body_id, ObjectBlock **out, char *e, uint32_t n) {
    if (out)
        *out = nullptr;
    return guard(e, n, [&] {
        require(out, "missing mutable object output");
        draft(s);
        for (uint32_t i = 0; i < s->object_count; ++i) {
            if (s->object_table[i]->body.id == body_id) {
                auto *object = const_cast<ObjectBlock *>(s->object_table[i]);
                require(
                    object->id.owner == s && object->id.kind == SIM_DATA_OBJECT, "foreign object"
                );
                *out = object;
                return;
            }
        }
        throw Error(SIM_INVALID, "unknown body ID");
    });
}

SimResult simulation_finalize(Simulation *s, char *e, uint32_t n) {
    return guard(e, n, [&] {
        draft(s);
        char message[256];
        check(simulation_validate(s, message, sizeof message), message);
        s->frozen = 1;
    });
}

SimResult simulation_clone(const Simulation *source, Simulation *out, char *e, uint32_t n) {
    return guard(e, n, [&] {
        require(out && !out->initialized, "clone destination must be zero initialized");
        char msg[256];
        check(simulation_validate(source, msg, sizeof msg), msg);
        check(simulation_init(out, &source->storage->config, msg, sizeof msg), msg);
        try {
            out->settings = source->settings;
            for (uint32_t i = 0; i < source->object_count; ++i) {
                auto &o = *source->object_table[i];
                std::array<Collider, 1024> c{};
                for (uint32_t j = 0; j < o.collider_count; ++j) {
                    c[j] = o.colliders[j];
                    c[j].object = nullptr;
                }
                ObjectBlock *obj;
                check(
                    simulation_add_object(
                        out,
                        &o.body,
                        c.data(),
                        o.collider_count,
                        &o == &source->robot.object,
                        &obj,
                        msg,
                        sizeof msg
                    ),
                    msg
                );
            }
            auto fix = [&](const ObjectBlock *p) { return out->object_table[p->id.index]; };
            for (uint32_t i = 0; i < source->robot.thruster_count; ++i) {
                auto t = source->robot.thrusters[i];
                t.object = fix(t.object);
                check(simulation_add_thruster(out, &t, msg, sizeof msg), msg);
            }
            for (uint32_t i = 0; i < source->robot.hull_count; ++i) {
                auto h = source->robot.hulls[i];
                h.object = fix(h.object);
                check(simulation_add_hull(out, &h, msg, sizeof msg), msg);
            }
            for (uint32_t i = 0; i < source->sensor_count; ++i) {
                auto sensor = source->sensors[i];
                sensor.object = fix(sensor.object);
                const SensorRec *inserted;
                check(simulation_add_sensor(out, &sensor, &inserted, msg, sizeof msg), msg);
            }
            auto fix_sensor = [&](const SensorRec *sensor) -> const SensorRec * {
                if (!sensor)
                    return nullptr;
                for (uint32_t i = 0; i < source->sensor_count; ++i)
                    if (sensor == source->sensors + i)
                        return out->sensors + i;
                throw Error(SIM_INVALID, "foreign sensor reference");
            };
            if (source->robot.sensor_count) {
                out->robot.sensors = fix_sensor(source->robot.sensors);
                out->robot.sensor_count = source->robot.sensor_count;
            }
            SensorRouting routes{
                fix_sensor(source->routing.tracking),
                fix_sensor(source->routing.imu),
                fix_sensor(source->routing.pressure),
                fix_sensor(source->routing.camera)
            };
            check(simulation_set_sensor_routing(out, &routes, msg, sizeof msg), msg);
            for (uint32_t i = 0; i < source->fault_count; ++i) {
                auto fault = *source->fault_table[i];
                fault.sensor = fix_sensor(fault.sensor);
                check(simulation_add_fault(out, &fault, msg, sizeof msg), msg);
            }
            for (uint32_t i = 0; i < source->module_count; ++i)
                check(
                    simulation_add_module(out, source->module_table[i]->name, msg, sizeof msg), msg
                );
            std::memcpy(out->robot.name, source->robot.name, 64);
            std::memcpy(out->robot.asset_file, source->robot.asset_file, 64);
            std::memcpy(out->robot.asset_hash, source->robot.asset_hash, 32);
        } catch (...) {
            simulation_destroy(out);
            throw;
        }
    });
}
}

namespace core {
const SimulationConfig &model_config(const Simulation &s) {
    require(s.storage, "uninitialized model");
    return s.storage->config;
}
} // namespace core
