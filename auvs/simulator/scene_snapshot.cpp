#include "scene_presets.h"
#include "internal.hpp"
#include "data_binary.hpp"
#include <fstream>
#include <string>
#include <utility>
#include <vector>

namespace {
using namespace core;

using Value = autonomy::data::Value;

const Value &need(const Value &object, const char *key) {
    const Value *value = object.field(key);
    require(value, "missing snapshot field");
    return *value;
}

double number_of(const Value &value) {
    require(value.kind == Value::Number, "expected number");
    require(std::isfinite(value.number), "non-finite snapshot number");
    return double(value.number);
}

bool flag_of(const Value &value) {
    if (value.kind == Value::Bool)
        return value.boolean;
    if (value.kind == Value::Number)
        return number_of(value) != 0;
    require(false, "expected boolean");
    return false;
}

MathVector3f vector_of(const Value &value) {
    require(value.kind == Value::Array && value.items.size() == 3, "expected a 3-vector");
    return v(number_of(value.items[0]), number_of(value.items[1]), number_of(value.items[2]));
}

MathQuaternionf quaternion_of(const Value &value) {
    require(value.kind == Value::Array && value.items.size() == 4, "expected an xyzw quaternion");
    MathQuaternionf quaternion{};
    for (int i = 0; i < 4; ++i)
        quaternion.buf[i] = float(number_of(value.items[size_t(i)]));
    return quaternion;
}

MathPose pose_of(const Value &value) {
    require(value.kind == Value::Object, "expected a pose");
    return {vector_of(need(value, "position")), quaternion_of(need(value, "orientation"))};
}

struct StoredMesh {
    std::vector<MathVector3f> positions;
    std::vector<SimTriangle> triangles;
};

struct StoredShape {
    Collider collider{};
    StoredMesh mesh;
};

struct StoredBody {
    UnderwaterBody body{};
    std::vector<StoredShape> shapes;
    bool vehicle = false;
};

StoredShape shape_of(const Value &value) {
    require(value.kind == Value::Object, "expected a shape");
    StoredShape stored;
    collider_default(&stored.collider);
    double kind_number = number_of(need(value, "kind"));
    require(
        kind_number == std::floor(kind_number) && kind_number >= 0 && kind_number <= 3,
        "unknown shape"
    );
    stored.collider.data.kind = uint32_t(kind_number);
    stored.collider.data.pose = pose_of(need(value, "pose"));
    stored.collider.data.collisions = flag_of(need(value, "collisions"));
    if (const Value *friction = value.field("friction"))
        stored.collider.data.friction = number_of(*friction);
    if (const Value *restitution = value.field("restitution"))
        stored.collider.data.restitution = number_of(*restitution);
    if (stored.collider.data.kind == SIM_MESH) {
        const Value &vertices = need(value, "vertices");
        const Value &triangles = need(value, "triangles");
        require(
            vertices.kind == Value::Array && triangles.kind == Value::Array, "mesh arrays required"
        );
        stored.mesh.positions.reserve(vertices.items.size());
        for (const Value &point : vertices.items)
            stored.mesh.positions.push_back(vector_of(point));
        stored.mesh.triangles.reserve(triangles.items.size());
        for (const Value &face : triangles.items) {
            require(
                face.kind == Value::Array && face.items.size() == 3, "triangle needs 3 indices"
            );
            SimTriangle triangle{};
            for (const auto &index : face.items) {
                double n = number_of(index);
                require(n >= 0 && n <= UINT32_MAX && n == std::floor(n), "invalid triangle index");
            }
            triangle.a = uint32_t(number_of(face.items[0]));
            triangle.b = uint32_t(number_of(face.items[1]));
            triangle.c = uint32_t(number_of(face.items[2]));
            require(
                triangle.a < stored.mesh.positions.size() &&
                    triangle.b < stored.mesh.positions.size() &&
                    triangle.c < stored.mesh.positions.size(),
                "triangle index out of range"
            );
            stored.mesh.triangles.push_back(triangle);
        }
        stored.collider.geometry.positions = stored.mesh.positions.data();
        stored.collider.geometry.position_count = uint32_t(stored.mesh.positions.size());
        stored.collider.geometry.triangles = stored.mesh.triangles.data();
        stored.collider.geometry.triangle_count = uint32_t(stored.mesh.triangles.size());
    } else {
        MathVector3f size = vector_of(need(value, "size"));
        stored.collider.data.size = size;
    }
    return stored;
}

StoredBody body_of(const Value &value, const char *robot_name) {
    require(value.kind == Value::Object, "expected a body");
    StoredBody stored;
    underwater_body_default(&stored.body);
    double id = number_of(need(value, "id"));
    require(id >= 0 && id < double(UINT32_MAX) && id == std::floor(id), "invalid body id");
    stored.body.id = uint32_t(id);
    const Value &name = need(value, "name");
    require(
        name.kind == Value::String && !name.text.empty() &&
            name.text.size() < sizeof stored.body.name,
        "invalid body name"
    );
    std::snprintf(stored.body.name, sizeof stored.body.name, "%s", name.text.c_str());
    double cls = number_of(need(value, "class"));
    require(cls >= 0 && cls <= 255 && cls == std::floor(cls), "class must fit uint8");
    stored.body.cls = uint32_t(cls);
    double motion = number_of(need(value, "motion"));
    require(motion == std::floor(motion) && motion >= 0 && motion <= 2, "invalid motion");
    stored.body.motion_type = uint32_t(motion);
    stored.body.tracked = flag_of(need(value, "tracked"));
    stored.body.fuzz = flag_of(need(value, "fuzz"));
    stored.body.initial_pose = pose_of(need(value, "pose"));
    const Value *profile = value.field("profile");
    stored.vehicle = name.text == robot_name ||
                     (profile && profile->kind == Value::String && profile->text == robot_name);
    if (!stored.vehicle) {
        require(stored.body.motion_type != SIM_DYNAMIC, "only the vehicle may be dynamic");
        const Value &shapes = need(value, "shapes");
        require(
            shapes.kind == Value::Array && !shapes.items.empty(), "scenery body has no geometry"
        );
        for (const Value &shape : shapes.items)
            stored.shapes.push_back(shape_of(shape));
    }
    return stored;
}

} // namespace

extern "C" SimResult simulation_scene_load(
    Simulation *simulation, const char *path, const Robot *robot, char *error, uint32_t capacity
) {
    return core::guard(error, capacity, [&] {
        core::require(simulation && path && robot, "missing snapshot arguments");
        Value document = autonomy::data::read(path, 6);
        core::require(document.kind == Value::Object, "snapshot must be an object");
        core::require(number_of(need(document, "version")) == 1, "unsupported snapshot version");
        const Value &bodies = need(document, "bodies");
        core::require(
            bodies.kind == Value::Array && !bodies.items.empty(), "snapshot has no bodies"
        );

        require(bodies.items.size() <= 100000, "too many snapshot bodies");
        std::vector<StoredBody> stored;
        stored.reserve(bodies.items.size());
        bool found_vehicle = false;
        MathPose start{};
        uint32_t scenery = 0, colliders = 0, vertices = 0, triangles = 0;
        for (const Value &body : bodies.items) {
            stored.push_back(body_of(body, robot->name));
            StoredBody &item = stored.back();
            if (item.vehicle) {
                core::require(!found_vehicle, "snapshot has two vehicles");
                found_vehicle = true;
                start = item.body.initial_pose;
                continue;
            }
            core::require(item.body.id != 1, "body id 1 is reserved for the vehicle");
            ++scenery;
            for (const StoredShape &shape : item.shapes) {
                require(
                    colliders < UINT32_MAX &&
                        vertices <= UINT32_MAX - shape.collider.geometry.position_count &&
                        triangles <= UINT32_MAX - shape.collider.geometry.triangle_count,
                    "snapshot capacity overflow"
                );
                ++colliders;
                vertices += shape.collider.geometry.position_count;
                triangles += shape.collider.geometry.triangle_count;
            }
        }

        SimulationConfig config;
        char message[256];
        core::check(simulation_robot_config(robot, &config, message, sizeof message), message);
        if (config.object_capacity < scenery + 2)
            config.object_capacity = scenery + 2;
        require(
            config.collider_capacity <= UINT32_MAX - colliders &&
                config.vertex_capacity <= UINT32_MAX - vertices &&
                config.triangle_capacity <= UINT32_MAX - triangles,
            "snapshot capacity overflow"
        );
        config.collider_capacity += colliders;
        config.vertex_capacity += vertices;
        config.triangle_capacity += triangles;
        core::check(simulation_init(simulation, &config, message, sizeof message), message);
        bool owned = true;
        try {
            core::check(
                simulation_add_robot(
                    simulation, robot, 1, found_vehicle ? &start : nullptr, message, sizeof message
                ),
                message
            );
            for (StoredBody &item : stored) {
                if (item.vehicle)
                    continue;
                std::vector<Collider> shapes;
                shapes.reserve(item.shapes.size());
                for (StoredShape &shape : item.shapes) {
                    shape.collider.geometry.positions = shape.mesh.positions.data();
                    shape.collider.geometry.triangles = shape.mesh.triangles.data();
                    shapes.push_back(shape.collider);
                }
                ObjectBlock *object = nullptr;
                core::check(
                    simulation_add_object(
                        simulation,
                        &item.body,
                        shapes.data(),
                        uint32_t(shapes.size()),
                        0,
                        &object,
                        message,
                        sizeof message
                    ),
                    message
                );
            }
            core::check(simulation_finalize(simulation, message, sizeof message), message);
            owned = false;
        } catch (...) {
            if (owned)
                simulation_destroy(simulation);
            throw;
        }
    });
}
