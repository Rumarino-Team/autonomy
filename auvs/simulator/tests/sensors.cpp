#include "../runtime_internal.hpp"
#include "../auv_frame.hpp"
#include "../examples/sensors.h"
#include <iostream>
using namespace core;

/* A prescribed-kinematics backend isolates sensor mathematics from engine
 * solver differences. Each world owns its states; no shared mutable fixtures. */
struct PhysicsWorld {
    const Simulation *model;
    Buffer<SimBodyState> states;

    explicit PhysicsWorld(const Simulation *m)
        : model(m),
          states(model_allocator(*m)) {
        states.resize(m->object_count);
    }
};

namespace {
char error[512];

void ok(SimResult result) {
    check(result, error);
}

void near(double a, double b, double tolerance = 2e-5) {
    require(std::abs(a - b) < tolerance, "analytic sensor value");
}

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

PhysicsBackend backend() {
    PhysicsBackend b{};
    b.library = reinterpret_cast<void *>(1); // In-process test table; never dlclosed.
    auto &api = b.api;
    api.struct_size = sizeof api;
    api.abi_version = PHYSICS_BACKEND_ABI_VERSION;
    api.capabilities = SIM_CAP_THREADS;
    api.integrators = 1;
    api.reset = [](PhysicsWorld *w, const SimBodyState *input, uint32_t, char *, uint32_t) {
        for (uint32_t i = 0; i < w->model->object_count; ++i) {
            auto &b = w->model->object_table[i]->body;
            w->states[i] = input ? input[i]
                                 : SimBodyState{
                                       b.id,
                                       b.initial_pose,
                                       point(b.initial_pose, b.center_of_mass.pos),
                                       b.initial_velocity,
                                       b.initial_angular_velocity
                                   };
        }
        return SimResult(SIM_OK);
    };
    api.world_create = [](const Simulation *m, PhysicsWorld **out, char *e, uint32_t n) {
        if (out)
            *out = nullptr;
        return guard(e, n, [&] {
            require(out, "null world output");
            *out = new PhysicsWorld(m);
        });
    };
    api.world_destroy = [](PhysicsWorld *w) { delete w; };
    api.step = [](PhysicsWorld *w, const SimWrench *, uint32_t, char *, uint32_t) {
        const double dt =
            double(w->model->settings.dt_numerator) / w->model->settings.dt_denominator;
        for (uint32_t i = 0; i < w->model->object_count; ++i) {
            const auto &b = w->model->object_table[i]->body;
            w->states[i].pose.pos = w->states[i].pose.pos + w->states[i].linear_velocity * dt;
            w->states[i].center_of_mass = point(w->states[i].pose, b.center_of_mass.pos);
            if (b.fuzz)
                w->states[i].angular_velocity.buf[2] += float(dt);
        }
        return SimResult(SIM_OK);
    };
    api.read_state = [](const PhysicsWorld *w, SimBodyState *out, uint32_t cap, uint32_t *count) {
        return copy_out(w->states, out, cap, count);
    };
    api.read_contacts = [](const PhysicsWorld *, SimContact *, uint32_t, uint32_t *count) {
        *count = 0;
        return SimResult(SIM_OK);
    };
    api.state_hash = [](const PhysicsWorld *w) {
        Hash h;
        for (const auto &s : w->states)
            h.state(s);
        return h.value;
    };
    return b;
}

ObjectBlock *
add(Simulation &m,
    uint32_t id,
    MathVector3f position = v(),
    bool tracked = false,
    MathVector3f size = v(.5, .5, .5)) {
    UnderwaterBody body;
    underwater_body_default(&body);
    body.id = id;
    body.initial_pose.pos = position;
    body.tracked = tracked;
    Collider c;
    collider_default(&c);
    c.data.size = size;
    ObjectBlock *object;
    ok(simulation_add_object(&m, &body, &c, 1, 0, &object, error, sizeof error));
    return object;
}

void create(Runtime &r, Scene &s, PhysicsBackend &b) {
    ok(simulation_finalize(&s.m, error, sizeof error));
    ok(simulation_runtime_create(&s.m, &b, nullptr, &r.p, error, sizeof error));
}

Observation observe(Runtime &r) {
    Observation o;
    ok(simulation_runtime_observe(r.p, &o, error, sizeof error));
    return o;
}

void steps(Runtime &r, unsigned n) {
    while (n--)
        ok(simulation_runtime_step(r.p, nullptr, 0, error, sizeof error));
}

void sampling_and_faults(PhysicsBackend &b) {
    Scene scene;
    ok(simulation_init(&scene.m, nullptr, error, sizeof error));
    auto *object = add(scene.m, 1, v(0, 0, 2));
    object->body.initial_velocity = v(1, 0, 0);
    SensorRouting routing{};
    for (uint32_t i = 0; i < 4; ++i) {
        SensorRec s;
        sensor_default(&s, i);
        s.id = i + 1;
        s.object = object;
        const SensorRec *out;
        ok(simulation_add_sensor(&scene.m, &s, &out, error, sizeof error));
        if (i == 0)
            routing.tracking = out;
        if (i == 1)
            routing.imu = out;
        if (i == 2)
            routing.pressure = out;
        if (i == 3)
            routing.camera = out;
    }
    ok(simulation_set_sensor_routing(&scene.m, &routing, error, sizeof error));
    for (uint32_t kind : {SIM_FAULT_TRACKING, SIM_FAULT_IMU, SIM_FAULT_CAMERA}) {
        FaultBlock f{};
        f.kind = kind;
        f.sensor = kind == SIM_FAULT_TRACKING ? routing.tracking
                   : kind == SIM_FAULT_IMU    ? routing.imu
                                              : routing.camera;
        f.start = kind == SIM_FAULT_IMU ? 7 : kind == SIM_FAULT_CAMERA ? 12 : 6;
        f.end = kind == SIM_FAULT_IMU ? 9 : kind == SIM_FAULT_CAMERA ? 24 : 12;
        ok(simulation_add_fault(&scene.m, &f, error, sizeof error));
    }
    Scene clone;
    ok(simulation_clone(&scene.m, &clone.m, error, sizeof error));
    require(
        clone.m.routing.imu == clone.m.sensors + 1 && clone.m.sensors != scene.m.sensors &&
            clone.m.fault_table[0]->sensor == clone.m.routing.tracking,
        "clone sensor references"
    );
    Runtime r;
    create(r, scene, b);
    const SensorSample *samples;
    uint32_t count;
    ok(simulation_runtime_sensor_samples(r.p, &samples, &count));
    require(count == 4, "four samples");
    require(samples[0].tick == 0 && samples[3].tick == 0, "reset samples");
    auto hash = simulation_runtime_hash(r.p);
    auto first = observe(r);
    auto again = observe(r);
    require(
        hash == simulation_runtime_hash(r.p) && first.body.pos.buf[0] == again.body.pos.buf[0],
        "read has no side effects"
    );
    steps(r, 5);
    require(
        samples[0].tick == 0 && samples[1].tick == 5 && samples[3].tick == 0, "independent periods"
    );
    near(observe(r).body.pos.buf[0], 0);
    steps(r, 1);
    require(samples[0].tick == 6 && samples[0].valid == 0, "tracking failure at sample");
    auto o = observe(r);
    require(o.position_axes == 4 && o.attitude_axes == 7 && o.depth_ok, "IMU/depth fallback");
    steps(r, 1);
    o = observe(r);
    require(!o.attitude_axes && !o.imu_ok && norm(o.gyro) == 0, "IMU outage");
    steps(r, 2);
    require(observe(r).imu_ok, "IMU recovery");
    steps(r, 3);
    require(
        observe(r).position_axes == 7 && !samples[3].valid, "tracking recovery and camera fault"
    );
    steps(r, 12);
    require(samples[3].valid == SIM_SAMPLE_CAMERA, "camera recovery");
    hash = simulation_runtime_hash(r.p);
    ok(simulation_runtime_reset(r.p, nullptr, 0, error, sizeof error));
    steps(r, 24);
    require(hash == simulation_runtime_hash(r.p), "cold reset sampling deterministic");
}

void mounts(PhysicsBackend &b) {
    Scene scene;
    ok(simulation_init(&scene.m, nullptr, error, sizeof error));
    auto *object = add(scene.m, 1, v(0, 0, 2));
    object->body.center_of_mass.pos = v(.25, 0, 0);
    object->body.initial_angular_velocity = v(0, 0, 2);
    object->body.fuzz = 1;
    const SensorRec *sensors[4];
    for (uint32_t i = 0; i < 4; ++i) {
        SensorRec s;
        sensor_default(&s, i);
        s.id = i;
        s.object = object;
        s.period_ticks = 1;
        if (i == 0)
            s.pose.pos = v(.25, 0, 0);
        if (i == 1) {
            s.pose.pos = v(1.25, 0, 0);
            s.pose.quat = q(std::sqrt(.5), 0, 0, std::sqrt(.5));
        }
        if (i == 2)
            s.pose.pos = v(0, 0, .25);
        ok(simulation_add_sensor(&scene.m, &s, &sensors[i], error, sizeof error));
    }
    SensorRouting route{sensors[0], sensors[1], sensors[2], sensors[3]};
    ok(simulation_set_sensor_routing(&scene.m, &route, error, sizeof error));
    FaultBlock f{};
    f.kind = SIM_FAULT_TRACKING;
    f.sensor = sensors[0];
    f.start = 1;
    f.end = 10;
    ok(simulation_add_fault(&scene.m, &f, error, sizeof error));
    Runtime r;
    create(r, scene, b);
    const SensorSample *samples;
    uint32_t count;
    simulation_runtime_sensor_samples(r.p, &samples, &count);
    near(samples[1].value.imu.gyro.buf[1], 2);
    near(samples[1].value.imu.accel.buf[0], -4);
    near(samples[1].value.imu.accel.buf[1], -9.81);
    auto o = observe(r);
    near(o.body.pos.buf[0], 0);
    near(o.gyro.buf[2], 2);
    near(o.accel.buf[2], -9.81);
    near(o.depth, 2.25);
    steps(r, 1);
    o = observe(r);
    near(o.body.pos.buf[2], 2);
    const double omega = 2 + 1. / 360;
    near(samples[1].value.imu.accel.buf[0], -omega * omega);
    near(samples[1].value.imu.accel.buf[2], -1, 2e-4); // alpha x arm, transformed by mount
    near(o.gyro.buf[2], omega);
    require(o.position_axes == 4, "mounted pressure fallback");
}

void vision(PhysicsBackend &b) {
    Scene scene;
    ok(simulation_init(&scene.m, nullptr, error, sizeof error));
    auto *camera = add(scene.m, 1, v(), true);
    add(scene.m, 2, v(0, 0, 3), true);
    add(scene.m, 3, v(0, 0, -3), true);
    add(scene.m, 4, v(0, 0, 20), true);
    add(scene.m, 5, v(50, 0, 3), true);
    add(scene.m, 6, v(0, 0, .05), true, v(.1, .1, .1));
    add(scene.m, 7, v(3, 0, 3), true);
    SensorRec s;
    sensor_default(&s, SIM_SENSOR_CAMERA);
    s.object = camera;
    s.settings.camera.horizontal_fov = 90;
    s.settings.camera.far_range = 10;
    const SensorRec *out;
    ok(simulation_add_sensor(&scene.m, &s, &out, error, sizeof error));
    SensorRouting routing{};
    routing.camera = out;
    ok(simulation_set_sensor_routing(&scene.m, &routing, error, sizeof error));
    Runtime r;
    create(r, scene, b);
    auto o = observe(r);
    require(
        o.object_count == 3 && o.objects[0].id == 2 && o.objects[1].id == 6 && o.objects[2].id == 7,
        "frustum culling, range, self exclusion, stable order"
    );
    require(o.objects[0].x0 == 320 && o.objects[0].x1 == 480, "projection");
    require(
        o.objects[1].x0 == 0 && o.objects[1].y0 == 0 && o.objects[1].x1 == 799 &&
            o.objects[1].y1 == 599,
        "near plane clipping"
    );
    require(o.objects[2].x1 == 799, "image clipping");
    require(!o.position_axes && !o.attitude_axes && !o.depth_ok, "absent sensors");
}

void limits_and_failure(PhysicsBackend &b) {
    Scene scene;
    SimulationConfig c;
    simulation_config_default(&c);
    c.sensor_capacity = 1;
    c.object_capacity = 160;
    c.collider_capacity = 160;
    ok(simulation_init(&scene.m, &c, error, sizeof error));
    auto *object = add(scene.m, 1);
    SensorRec sensor;
    sensor_default(&sensor, SIM_SENSOR_CAMERA);
    sensor.object = object;
    sensor.id = 1;
    const SensorRec *out;
    sensor.period_ticks = 0;
    require(
        simulation_add_sensor(&scene.m, &sensor, &out, error, sizeof error) == SIM_INVALID,
        "zero period"
    );
    sensor.period_ticks = 1;
    ok(simulation_add_sensor(&scene.m, &sensor, &out, error, sizeof error));
    require(
        simulation_add_sensor(&scene.m, &sensor, &out, error, sizeof error) == SIM_INVALID,
        "duplicate ID"
    );
    sensor.id = 2;
    require(
        simulation_add_sensor(&scene.m, &sensor, &out, error, sizeof error) == SIM_CAPACITY &&
            scene.m.sensor_count == 1,
        "sensor exhaustion"
    );
    SensorRouting routing{};
    routing.camera = scene.m.sensors;
    ok(simulation_set_sensor_routing(&scene.m, &routing, error, sizeof error));
    Scene foreign;
    ok(simulation_init(&foreign.m, nullptr, error, sizeof error));
    auto *other = add(foreign.m, 2);
    sensor.object = other;
    require(
        simulation_add_sensor(&scene.m, &sensor, &out, error, sizeof error) == SIM_INVALID,
        "foreign owner"
    );
    FaultBlock fault{};
    fault.kind = SIM_FAULT_CAMERA;
    fault.sensor = &sensor;
    fault.end = 10;
    require(
        simulation_add_fault(&scene.m, &fault, error, sizeof error) == SIM_INVALID, "foreign fault"
    );
    for (uint32_t i = 0; i < 140; ++i)
        add(scene.m, i + 2, v(0, 0, 3 + float(i) * .01), true);
    Runtime r;
    create(r, scene, b);
    require(
        simulation_add_sensor(&scene.m, &sensor, &out, error, sizeof error) == SIM_INVALID,
        "frozen sensors"
    );
    auto o = observe(r);
    require(o.object_count == 140, "rich view not limited by AUV capacity");
    AuvFrame frame;
    fill_auv_frame(&frame, o, 123);
    require(
        frame.objects_len == 128 && frame.objects2d_len == 128 && frame.timestamp == 123,
        "AUV bounds"
    );
    Observation empty{};
    empty.body.quat = empty.camera.quat = empty.imu = q();
    fill_auv_frame(&frame, empty, 124);
    require(
        !frame.objects_len && !frame.pressure_depth_ok && !frame.position_ok.x,
        "no stale AUV fields"
    );
    ObjectObservation invalid{};
    invalid.cls = 256;
    o.objects = &invalid;
    o.object_count = 1;
    bool rejected = false;
    try {
        fill_auv_frame(&frame, o, 125);
    } catch (const Error &) {
        rejected = true;
    }
    require(rejected, "class overflow rejected");
}

void passive(PhysicsBackend &b) {
    Scene a, c;
    ok(simulation_init(&a.m, nullptr, error, sizeof error));
    ok(simulation_init(&c.m, nullptr, error, sizeof error));
    auto *one = add(a.m, 1), *two = add(c.m, 1);
    one->body.initial_velocity = two->body.initial_velocity = v(1, 2, 0);
    ok(example_sensors(&c.m, two, error, sizeof error));
    Runtime x, y;
    create(x, a, b);
    create(y, c, b);
    for (unsigned i = 0; i < 50; ++i) {
        steps(x, 1);
        steps(y, 1);
        const SimBodyState *sx, *sy;
        uint32_t n;
        simulation_runtime_states(x.p, &sx, &n);
        simulation_runtime_states(y.p, &sy, &n);
        for (unsigned k = 0; k < 3; ++k)
            require(sx[0].pose.pos.buf[k] == sy[0].pose.pos.buf[k], "sensors are passive");
    }
}
} // namespace

int main() {
    try {
        auto b = backend();
        sampling_and_faults(b);
        mounts(b);
        vision(b);
        limits_and_failure(b);
        passive(b);
        std::cout << "sensor sampling, mounted IMU/depth, visibility, AUV conversion, ownership "
                     "and passive trajectories passed\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
