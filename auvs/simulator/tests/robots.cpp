#include "../robots.h"
#include "../internal.hpp"
#include <iostream>
using namespace core;

namespace {
#include "robot_reference.h"

// Match the source model's semantic fingerprint, including every geometry
// component and calibration value. Owner pointers and SIMD padding are absent.
uint64_t fingerprint(const Robot &r) {
    Hash h;
    auto text = [&](const char *s) {
        h.scalar(uint32_t(std::strlen(s)));
        h.raw(s, std::strlen(s));
    };
    auto pose = [&](MathPose p) {
        h.vec(p.pos);
        h.quat(p.quat);
    };
    text(r.name);
    text(r.asset_file);
    h.raw(r.asset_hash, 32);
    const auto &b = r.object.body;
    for (uint32_t value : {b.id, b.cls, b.motion_type, b.tracked, b.fuzz})
        h.scalar(value);
    text(b.name);
    h.scalar(b.mass);
    h.vec(b.inertia);
    pose(b.center_of_mass);
    pose(b.initial_pose);
    h.vec(b.initial_velocity);
    h.vec(b.initial_angular_velocity);
    h.scalar(b.dry_mass);
    h.scalar(b.volume);
    h.vec(b.buoyancy_center);
    h.scalar(r.object.collider_count);
    for (uint32_t i = 0; i < r.object.collider_count; ++i) {
        const auto &c = r.colliders[i];
        const auto &s = c.data;
        h.scalar(s.kind);
        h.scalar(s.collisions);
        pose(s.pose);
        h.vec(s.size);
        h.vec(s.drag);
        h.vec(s.skin);
        h.scalar(s.friction);
        h.scalar(s.restitution);
        h.scalar(c.geometry.position_count);
        h.scalar(c.geometry.triangle_count);
        for (uint32_t j = 0; j < c.geometry.position_count; ++j)
            h.vec(c.geometry.positions[j]);
        for (uint32_t j = 0; j < c.geometry.triangle_count; ++j) {
            auto t = c.geometry.triangles[j];
            h.scalar(t.a);
            h.scalar(t.b);
            h.scalar(t.c);
        }
    }
    h.scalar(r.thruster_count);
    for (uint32_t i = 0; i < r.thruster_count; ++i) {
        const auto &t = r.thrusters[i];
        h.vec(t.position);
        h.vec(t.direction);
        const auto &s = t.spec;
        for (double x :
             {s.max_rpm, s.diameter, s.thrust, s.torque, s.inertia, s.kp, s.ki, s.command_sign})
            h.scalar(x);
    }
    h.scalar(r.hull_count);
    for (uint32_t i = 0; i < r.hull_count; ++i) {
        const auto &s = r.hulls[i];
        pose(s.pose);
        h.vec(s.drag);
        h.vec(s.skin);
        h.vec(s.centroid);
        h.quat(s.coefficient_frame);
        h.scalar(s.volume);
        h.scalar(s.position_count);
        for (uint32_t j = 0; j < s.position_count; ++j)
            h.vec(s.positions[j]);
    }
    h.scalar(uint32_t(1)); // Preserve the original camera-only calibration fingerprint.
    for (uint32_t i = 0; i < r.sensor_count; ++i) {
        if (r.sensors[i].kind != SIM_SENSOR_CAMERA)
            continue;
        const auto &c = r.sensors[i].settings.camera;
        h.scalar(c.width);
        h.scalar(c.height);
        h.scalar(c.horizontal_fov);
        pose(r.sensors[i].pose);
    }
    return h.value;
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

void database(const Robot *r, uint32_t expected_thrusters) {
    char e[512];
    SimulationConfig c;
    check(simulation_robot_config(r, &c, e, sizeof e), e);
    Scene scene;
    auto &m = scene.m;
    check(simulation_init(&m, &c, e, sizeof e), e);
    require(
        !r->registered && !r->object.id.owner && !r->object.id.list,
        "template has no database membership"
    );
    auto pose = r->object.body.initial_pose;
    pose.pos = v(1, 2, 3);
    // Exhaustion must not insert an object or change any allocation cursors.
    const size_t hull_capacity = m.hull_vertices.capacity;
    m.hull_vertices.capacity = 0;
    require(
        simulation_add_robot(&m, r, 31, &pose, e, sizeof e) == SIM_CAPACITY, "hull capacity error"
    );
    m.hull_vertices.capacity = hull_capacity;
    require(
        !m.object_count && !m.objects.first && !m.robot.registered && !m.vertices.offset &&
            !m.triangles.offset && !m.geometry_count,
        "failed robot leaves empty draft"
    );
    const size_t triangle_capacity = m.triangles.capacity;
    m.triangles.capacity = 0;
    require(
        simulation_add_robot(&m, r, 31, &pose, e, sizeof e) == SIM_CAPACITY, "mesh capacity error"
    );
    m.triangles.capacity = triangle_capacity;
    require(
        !m.object_count && !m.vertices.offset && !m.triangles.offset && !m.geometry_count,
        "geometry failure rolls back"
    );
    auto invalid = pose;
    invalid.quat = {};
    require(
        simulation_add_robot(&m, r, 31, &invalid, e, sizeof e) == SIM_INVALID, "invalid robot pose"
    );
    // A preexisting object retains its list membership and dense table order.
    UnderwaterBody scenery;
    underwater_body_default(&scenery);
    scenery.id = 42;
    Collider box;
    collider_default(&box);
    ObjectBlock *first = nullptr;
    check(simulation_add_object(&m, &scenery, &box, 1, 0, &first, e, sizeof e), e);
    SensorRec scenery_sensor;
    sensor_default(&scenery_sensor, SIM_SENSOR_IMU);
    scenery_sensor.id = 999;
    scenery_sensor.object = first;
    const SensorRec *scenery_measurement;
    check(simulation_add_sensor(&m, &scenery_sensor, &scenery_measurement, e, sizeof e), e);
    require(
        simulation_add_robot(&m, r, 42, &pose, e, sizeof e) == SIM_INVALID, "duplicate external ID"
    );
    require(
        m.object_count == 1 && m.objects.last == &first->id.link && !first->id.link.next &&
            m.object_table[0] == first,
        "failure preserves scenery"
    );
    check(simulation_add_robot(&m, r, 31, &pose, e, sizeof e), e);
    require(
        m.robot.registered && m.object_count == 2 && m.object_table[1] == &m.robot.object &&
            m.robot.object.body.id == 31 && m.robot.thruster_count == expected_thrusters &&
            m.robot.object.body.initial_pose.pos.buf[2] == 3,
        "robot registration and placement"
    );
    require(
        std::strcmp(m.robot.name, r->name) == 0 &&
            std::memcmp(m.robot.asset_hash, r->asset_hash, 32) == 0 &&
            m.robot.object.body.mass == r->object.body.mass,
        "robot metadata and calibration"
    );
    for (uint32_t i = 0; i < m.robot.object.collider_count; ++i) {
        auto &a = m.robot.colliders[i];
        auto &b = r->colliders[i];
        require(
            a.object == &m.robot.object && a.geometry.positions != b.geometry.positions &&
                a.geometry.triangles != b.geometry.triangles &&
                a.geometry.position_count == b.geometry.position_count &&
                a.geometry.triangle_count == b.geometry.triangle_count,
            "model owns copied geometry"
        );
    }
    for (uint32_t i = 0; i < m.robot.thruster_count; ++i)
        require(m.robot.thrusters[i].object == &m.robot.object, "thruster ownership");
    for (uint32_t i = 0; i < m.robot.hull_count; ++i)
        require(
            m.robot.hulls[i].object == &m.robot.object &&
                m.robot.hulls[i].positions != r->hulls[i].positions,
            "hull ownership"
        );
    require(
        m.robot.sensors[0].object == &m.robot.object && m.robot.sensors != r->sensors,
        "camera ownership"
    );
    const auto offset = m.vertices.offset;
    require(
        simulation_add_robot(&m, r, 32, nullptr, e, sizeof e) == SIM_INVALID &&
            m.object_count == 2 && m.vertices.offset == offset,
        "duplicate robot rejected"
    );
    check(simulation_finalize(&m, e, sizeof e), e);
    require(
        simulation_add_robot(&m, r, 33, nullptr, e, sizeof e) == SIM_INVALID,
        "frozen topology rejected"
    );
    Scene copy;
    check(simulation_clone(&m, &copy.m, e, sizeof e), e);
    check(simulation_finalize(&copy.m, e, sizeof e), e);
    require(
        copy.m.robot.thrusters[0].object == &copy.m.robot.object &&
            copy.m.robot.colliders[0].geometry.positions !=
                m.robot.colliders[0].geometry.positions &&
            copy.m.robot.hulls[0].positions != m.robot.hulls[0].positions &&
            copy.m.robot.sensors[0].object == &copy.m.robot.object,
        "clone repairs robot ownership"
    );
}

void run(const Robot *r, const PhysicsBackend &backend) {
    Scene scene;
    char e[512];
    SimulationConfig c;
    check(simulation_robot_config(r, &c, e, sizeof e), e);
    check(simulation_init(&scene.m, &c, e, sizeof e), e);
    check(simulation_add_robot(&scene.m, r, 77, nullptr, e, sizeof e), e);
    check(simulation_finalize(&scene.m, e, sizeof e), e);
    Runtime runtime;
    check(simulation_runtime_create(&scene.m, &backend, nullptr, &runtime.p, e, sizeof e), e);
    double commands[8];
    std::fill_n(commands, 8, .05);
    uint64_t hash = 0;
    for (unsigned repetition = 0; repetition < 2; ++repetition) {
        for (unsigned i = 0; i < 180; ++i)
            check(simulation_runtime_step(runtime.p, commands, r->thruster_count, e, sizeof e), e);
        const SimBodyState *states;
        uint32_t count;
        check(simulation_runtime_states(runtime.p, &states, &count), "states");
        require(
            count == 1 && states[0].id == 77 && finite(states[0].linear_velocity) &&
                finite(states[0].angular_velocity),
            "finite robot state"
        );
        valid_pose(states[0].pose);
        const RotorState *rotors;
        check(simulation_runtime_rotors(runtime.p, &rotors, &count), "rotors");
        require(
            count == r->thruster_count && std::isfinite(rotors[0].omega) &&
                std::abs(rotors[0].omega) > 0,
            "active robot motors"
        );
        Observation observation;
        check(simulation_runtime_observe(runtime.p, &observation, e, sizeof e), e);
        require(std::isfinite(observation.depth), "robot camera observation");
        auto current = simulation_runtime_hash(runtime.p);
        if (repetition)
            require(current == hash, "robot cold reset deterministic");
        hash = current;
        check(simulation_runtime_reset(runtime.p, nullptr, 0, e, sizeof e), e);
    }
    std::cout << r->name << " / " << backend.api.name << " hash=" << hash << '\n';
}
} // namespace

int main(int argc, char **argv) {
    try {
        const char *names[] = {"hydrus", "proteus", "bluerov", "girona500"};
        const uint32_t motors[] = {8, 6, 8, 5};
        require(
            sim_robot_count() == 4 && !sim_robot_at(4) && !sim_robot_at(UINT32_MAX) &&
                !sim_robot_find(nullptr) && !sim_robot_find("unknown"),
            "preset lookup"
        );
        Robot foreign{};
        SimulationConfig c;
        char e[512];
        require(
            simulation_robot_config(&foreign, &c, e, sizeof e) == SIM_INVALID &&
                simulation_robot_config(nullptr, &c, e, sizeof e) == SIM_INVALID &&
                simulation_robot_config(sim_robot_at(0), nullptr, e, sizeof e) == SIM_INVALID,
            "invalid preset arguments"
        );
        PhysicsBackend backend{};

        struct Close {
            PhysicsBackend *b;

            ~Close() {
                physics_backend_close(b);
            }
        } close{&backend};

        if (argc == 2)
            check(physics_backend_open(&backend, argv[1], e, sizeof e), e);
        for (uint32_t i = 0; i < 4; ++i) {
            const auto *r = sim_robot_at(i);
            require(r == sim_robot_find(names[i]), "preset order");
            require(fingerprint(*r) == reference_fingerprints[i], "source calibration fingerprint");
            database(r, motors[i]);
            if (argc == 2)
                run(r, backend);
        }
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
