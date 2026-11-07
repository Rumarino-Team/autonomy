#include "../internal.hpp"
#include "../examples/sensors.h"
#include <iostream>
using namespace core;

namespace {
struct Scene {
    Simulation m{};

    ~Scene() {
        simulation_destroy(&m);
    }
};

struct Runtime {
    SimulationRuntime *p = nullptr;

    ~Runtime() {
        simulation_runtime_destroy(p);
    }
};

struct Count {
    size_t allocations = 0, live = 0, fail = SIZE_MAX;

    static void *alloc(void *u, size_t n, size_t a) {
        auto &c = *static_cast<Count *>(u);
        if (c.allocations++ == c.fail)
            return nullptr;
        auto *p = default_alloc(nullptr, n, a);
        if (p)
            ++c.live;
        return p;
    }

    static void release(void *u, void *p, size_t n, size_t a) {
        if (p) {
            --static_cast<Count *>(u)->live;
            default_free(nullptr, p, n, a);
        }
    }

    SimAllocator allocator() {
        return {this, alloc, release};
    }
};

ObjectBlock *
body(Simulation &m, uint32_t id, uint32_t motion, const Collider *c = nullptr, uint32_t count = 1) {
    UnderwaterBody b;
    underwater_body_default(&b);
    b.id = id;
    b.motion_type = motion;
    b.mass = b.dry_mass = motion == SIM_DYNAMIC ? 2 : 0;
    b.initial_pose.pos = v(0, 0, 2);
    std::snprintf(b.name, 64, "body%u", id);
    Collider box;
    if (!c) {
        collider_default(&box);
        box.data.collisions = 0;
        box.data.size = v(.1, .1, .1);
        c = &box;
    }
    ObjectBlock *out;
    char e[512];
    check(simulation_add_object(&m, &b, c, count, 0, &out, e, sizeof e), e);
    return out;
}

void init(Scene &s, Count *counter = nullptr) {
    SimulationConfig cfg;
    simulation_config_default(&cfg);
    if (counter)
        cfg.allocator = counter->allocator();
    char e[256];
    check(simulation_init(&s.m, &cfg, e, sizeof e), e);
    s.m.settings.gravity = v();
}

void finish(Scene &s) {
    char e[256];
    check(simulation_finalize(&s.m, e, sizeof e), e);
}

void create(
    Runtime &r, const Scene &s, const PhysicsBackend &b, const ForceRegistry *reg = nullptr
) {
    char e[512];
    check(simulation_runtime_create(&s.m, &b, reg, &r.p, e, sizeof e), e);
}

void step(Runtime &r, const double *c = nullptr, uint32_t count = 0) {
    char e[512];
    check(simulation_runtime_step(r.p, c, count, e, sizeof e), e);
}

struct Calls {
    unsigned creates = 0, resets = 0, destroys = 0, evaluations = 0;
    bool fail = false;
};

struct State {
    Calls *c;
    unsigned tick;
};

SimResult create_module(const Simulation *, void *user, void **out, char *, uint32_t) {
    auto *c = static_cast<Calls *>(user);
    ++c->creates;
    *out = new State{c, 0};
    return c->fail ? SIM_BACKEND_ERROR : SIM_OK;
}

void destroy_module(void *p) {
    auto *s = static_cast<State *>(p);
    if (s) {
        ++s->c->destroys;
        delete s;
    }
}

SimResult reset_module(void *p, char *, uint32_t) {
    auto *s = static_cast<State *>(p);
    s->tick = 0;
    ++s->c->resets;
    return SIM_OK;
}

SimResult eval_module(const SimForceContext *c, void *p, SimWrench *, uint32_t, char *, uint32_t) {
    auto *s = static_cast<State *>(p);
    if (c->tick != s->tick++)
        return SIM_INVALID;
    auto *scratch = arena_alloc(c->scratch, 64, 32);
    if (!scratch || uintptr_t(scratch) % 32)
        return SIM_CAPACITY;
    ++s->c->evaluations;
    return SIM_OK;
}

void lifecycle(const PhysicsBackend &backend) {
    Count counter;
    {
        Scene scene;
        init(scene, &counter);
        body(scene.m, 3, SIM_DYNAMIC);
        char e[512];
        check(simulation_add_module(&scene.m, "gravity_compensation", e, sizeof e), e);
        check(simulation_add_module(&scene.m, "stateful", e, sizeof e), e);
        finish(scene);
        Calls calls;
        ForceModule module{};
        std::strcpy(module.name, "stateful");
        module.user = &calls;
        module.create = create_module;
        module.reset = reset_module;
        module.evaluate = eval_module;
        module.destroy = destroy_module;
        ForceRegistry registry{&module, 1};
        {
            Runtime runtime;
            create(runtime, scene, backend, &registry);
            for (unsigned i = 0; i < 16; ++i)
                step(runtime);
            size_t warm = counter.allocations;
            for (unsigned i = 0; i < 200; ++i)
                step(runtime);
            require(counter.allocations == warm, "steady state allocated simulator storage");
            check(simulation_runtime_reset(runtime.p, nullptr, 0, e, sizeof e), e);
            step(runtime);
            require(
                calls.creates == 1 && calls.resets == 2 && calls.evaluations == 217,
                "module lifecycle"
            );
        }
        require(calls.destroys == 1, "module state leak");
        calls.fail = true;
        Runtime failure;
        require(
            simulation_runtime_create(&scene.m, &backend, &registry, &failure.p, e, sizeof e) ==
                    SIM_BACKEND_ERROR &&
                !failure.p,
            "failed module creation result"
        );
        require(calls.creates == 2 && calls.destroys == 2, "failed module creation cleanup");
        size_t before = counter.live;
        counter.fail = counter.allocations;
        require(
            simulation_runtime_create(&scene.m, &backend, nullptr, &failure.p, e, sizeof e) ==
                    SIM_NO_MEMORY &&
                !failure.p,
            "runtime allocation failure"
        );
        require(counter.live == before, "runtime failure leaked buffers");
        counter.fail = SIZE_MAX;
    }
    require(counter.live == 0, "counting allocator leak");
}

void rotor(const PhysicsBackend &backend) {
    for (uint32_t frequency : {90u, 360u, 720u}) {
        Scene scene;
        init(scene);
        scene.m.settings.dt_denominator = frequency;
        auto *object = body(scene.m, 4, SIM_STATIC);
        Thruster t{};
        t.object = object;
        t.direction = v(1, 0, 0);
        rotor_spec_default(&t.spec);
        char e[256];
        check(simulation_add_thruster(&scene.m, &t, e, sizeof e), e);
        finish(scene);
        Runtime runtime;
        create(runtime, scene, backend);
        double target = 1000. / 60 * 2 * 3.141592653589793 * .5;
        for (int phase = 0; phase < 3; ++phase) {
            double command = phase == 0 ? .5 : phase == 1 ? -.5 : 0;
            for (uint32_t k = 0; k < frequency * 2; ++k) {
                step(runtime, &command, 1);
                const RotorState *rotors;
                uint32_t n;
                check(simulation_runtime_rotors(runtime.p, &rotors, &n), "rotor view");
                require(
                    n == 1 && std::isfinite(rotors[0].omega) &&
                        std::abs(rotors[0].omega) < 1.25 * target,
                    "rotor stability"
                );
            }
            const RotorState *rotors;
            uint32_t n;
            simulation_runtime_rotors(runtime.p, &rotors, &n);
            double goal = phase == 0 ? -target : phase == 1 ? target : 0;
            require(std::abs(rotors[0].omega - goal) < .01, "rotor did not settle");
        }
    }
}

void hydro(const PhysicsBackend &backend) {
    Scene scene;
    init(scene);
    scene.m.settings.gravity = v(0, 0, 9.81);
    auto *object = body(scene.m, 8, SIM_DYNAMIC);
    object->body.initial_pose.pos = v(0, 0, .001);
    object->body.initial_velocity = v(0, 0, -100);
    object->body.volume = .001;
    MathVector3f surface[] = {v(), v(.1, 0, 0), v(0, .1, 0)};
    HullRec hull{};
    hull.object = object;
    hull.pose.quat = q();
    hull.positions = surface;
    hull.position_count = 3;
    hull.coefficient_frame = q();
    hull.volume = .001;
    char e[256];
    check(simulation_add_hull(&scene.m, &hull, e, sizeof e), e);
    finish(scene);
    Runtime runtime;
    create(runtime, scene, backend);
    double first = 0;
    for (unsigned tick = 0; tick < 8; ++tick) {
        step(runtime);
        const SimWrench *w;
        uint32_t n;
        simulation_runtime_wrenches(runtime.p, &w, &n);
        if (tick == 0) {
            first = w[0].force.buf[2];
            require(std::abs(first + 10.05525) < 1e-4, "buoyancy force");
        } else if (tick < 7)
            require(w[0].force.buf[2] == first, "fluid cache was not held");
        else
            require(std::abs(w[0].force.buf[2]) < 1e-6, "fluid cache did not refresh at period");
    }
}

void faults(const PhysicsBackend &backend) {
    Scene scene;
    init(scene);
    auto *object = body(scene.m, 33, SIM_DYNAMIC);
    object->body.tracked = 1;
    Thruster t{};
    t.object = object;
    t.direction = v(1, 0, 0);
    rotor_spec_default(&t.spec);
    char e[256];
    check(simulation_add_thruster(&scene.m, &t, e, sizeof e), e);
    check(example_sensors(&scene.m, object, e, sizeof e), e);
    FaultBlock f{};
    f.kind = SIM_FAULT_DEAD;
    f.start = 0;
    f.end = 1;
    check(simulation_add_fault(&scene.m, &f, e, sizeof e), e);
    f.start = 1;
    f.end = 2;
    f.kind = SIM_FAULT_WEAK;
    f.magnitude = .5;
    check(simulation_add_fault(&scene.m, &f, e, sizeof e), e);
    for (uint32_t kind : {SIM_FAULT_TRACKING, SIM_FAULT_IMU, SIM_FAULT_GYRO, SIM_FAULT_DEPTH}) {
        f.kind = kind;
        f.sensor = kind == SIM_FAULT_TRACKING ? scene.m.routing.tracking
                   : kind == SIM_FAULT_DEPTH  ? scene.m.routing.pressure
                                              : scene.m.routing.imu;
        f.magnitude = kind == SIM_FAULT_DEPTH ? .5 : .25;
        check(simulation_add_fault(&scene.m, &f, e, sizeof e), e);
    }
    finish(scene);
    Runtime runtime;
    create(runtime, scene, backend);
    double command = 1;
    step(runtime, &command, 1);
    const RotorState *rotors;
    uint32_t count;
    simulation_runtime_rotors(runtime.p, &rotors, &count);
    require(rotors[0].omega == 0, "dead actuator fault");
    Observation observation;
    check(simulation_runtime_observe(runtime.p, &observation, e, sizeof e), e);
    require(
        !observation.position_ok && !observation.imu_ok && !observation.object_count &&
            std::abs(observation.gyro.buf[0]) < 1e-6 && std::abs(observation.depth - 2.5) < 1e-6,
        "sensor faults"
    );
    step(runtime, &command, 1);
    simulation_runtime_rotors(runtime.p, &rotors, &count);
    double dt = 1. / 360, target = -1000. / 60 * 2 * 3.141592653589793 * .5;
    require(
        std::abs(rotors[0].omega - dt * target / (.00146468 + dt)) < 1e-8, "weak actuator command"
    );
    check(simulation_runtime_observe(runtime.p, &observation, e, sizeof e), e);
    require(
        observation.position_ok && observation.imu_ok && observation.object_count == 0,
        "sensor fault end boundary"
    );
}

void contacts(const PhysicsBackend &backend) {
    Scene scene;
    init(scene);
    scene.m.settings.gravity = v(0, 0, 9.81);
    MathVector3f vertices[] = {v(-.1, -.1, -.1), v(.1, -.1, -.1), v(0, .1, -.1), v(0, 0, .1)};
    SimTriangle triangles[] = {{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}};
    Collider mesh;
    collider_default(&mesh);
    mesh.data.kind = SIM_MESH;
    mesh.geometry = {vertices, 4, triangles, 4};
    auto *object = body(scene.m, 11, SIM_DYNAMIC, &mesh);
    object->body.initial_pose.pos = v();
    Collider floor;
    collider_default(&floor);
    floor.data.size = v(5, 5, .25);
    auto *ground = body(scene.m, 12, SIM_STATIC, &floor);
    ground->body.initial_pose.pos = v(0, 0, 1.5);
    finish(scene);
    Runtime runtime;
    create(runtime, scene, backend);
    bool contact = false;
    for (unsigned k = 0; k < 360; ++k) {
        step(runtime);
        const SimContact *c;
        uint32_t count;
        simulation_runtime_contacts(runtime.p, &c, &count);
        for (uint32_t j = 0; j < count; ++j) {
            require(
                (c[j].a == 11 && c[j].b == 12) || (c[j].a == 12 && c[j].b == 11), "contact IDs"
            );
            require(norm(c[j].normal) > .5 && std::isfinite(c[j].normal_force), "contact values");
            contact = true;
        }
    }
    require(contact, "mesh did not contact floor");
}
} // namespace

int main(int argc, char **argv) {
    try {
        require(argc == 2, "backend path");
        PhysicsBackend backend{};
        char e[512];
        check(physics_backend_open(&backend, argv[1], e, sizeof e), e);

        struct Close {
            PhysicsBackend *b;

            ~Close() {
                physics_backend_close(b);
            }
        } close{&backend};

        lifecycle(backend);
        rotor(backend);
        hydro(backend);
        contacts(backend);
        faults(backend);
        std::cout << backend.api.name
                  << " module cleanup, allocation reuse/failure, rotor settling, "
                     "hydro cadence, and mesh contacts passed\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
