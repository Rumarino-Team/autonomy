#include "scene_presets.h"
#include "internal.hpp"
#include <fstream>
#include <string>
#include <utility>
#include <vector>

namespace {
using namespace core;

struct Json {
    enum Kind { Null, Bool, Number, String, Array, Object };
    Kind kind = Null;
    bool boolean = false;
    double number = 0;
    std::string text;
    std::vector<Json> items;
    std::vector<std::pair<std::string, Json>> fields;

    const Json *field(const char *key) const {
        for (const auto &entry : fields)
            if (entry.first == key)
                return &entry.second;
        return nullptr;
    }
};

struct Parser {
    const std::string &in;
    size_t at = 0;

    explicit Parser(const std::string &text)
        : in(text) {
    }

    void skip() {
        while (at < in.size() && (in[at] == ' ' || in[at] == '\n' || in[at] == '\r' || in[at] == '\t'))
            ++at;
    }

    char peek() {
        skip();
        require(at < in.size(), "truncated scene snapshot");
        return in[at];
    }

    char take() {
        char c = peek();
        ++at;
        return c;
    }

    Json parse() {
        char c = peek();
        if (c == '{')
            return object();
        if (c == '[')
            return array();
        if (c == '"')
            return string();
        if (c == 't' || c == 'f')
            return boolean();
        if (c == 'n')
            return null();
        return number();
    }

    Json object() {
        require(take() == '{', "expected object");
        Json value;
        value.kind = Json::Object;
        if (peek() == '}') {
            ++at;
            return value;
        }
        while (true) {
            Json key = string();
            require(take() == ':', "expected key separator");
            value.fields.emplace_back(std::move(key.text), parse());
            char next = take();
            if (next == '}')
                return value;
            require(next == ',', "expected object separator");
        }
    }

    Json array() {
        require(take() == '[', "expected array");
        Json value;
        value.kind = Json::Array;
        if (peek() == ']') {
            ++at;
            return value;
        }
        while (true) {
            value.items.push_back(parse());
            char next = take();
            if (next == ']')
                return value;
            require(next == ',', "expected array separator");
        }
    }

    Json string() {
        require(take() == '"', "expected string");
        Json value;
        value.kind = Json::String;
        while (at < in.size()) {
            char c = in[at++];
            if (c == '"')
                return value;
            if (c != '\\') {
                value.text.push_back(c);
                continue;
            }
            require(at < in.size(), "truncated string escape");
            char escaped = in[at++];
            if (escaped == '"' || escaped == '\\' || escaped == '/')
                value.text.push_back(escaped);
            else if (escaped == 'n')
                value.text.push_back('\n');
            else if (escaped == 't')
                value.text.push_back('\t');
            else if (escaped == 'r')
                value.text.push_back('\r');
            else if (escaped == 'b')
                value.text.push_back('\b');
            else if (escaped == 'f')
                value.text.push_back('\f');
            else if (escaped == 'u') {
                require(at + 4 <= in.size(), "truncated unicode escape");
                int code = 0;
                for (int k = 0; k < 4; ++k) {
                    char hex = in[at++];
                    code <<= 4;
                    if (hex >= '0' && hex <= '9')
                        code += hex - '0';
                    else if (hex >= 'a' && hex <= 'f')
                        code += hex - 'a' + 10;
                    else if (hex >= 'A' && hex <= 'F')
                        code += hex - 'A' + 10;
                    else
                        require(false, "invalid unicode escape");
                }
                if (code < 0x80)
                    value.text.push_back(char(code));
                else if (code < 0x800) {
                    value.text.push_back(char(0xC0 | (code >> 6)));
                    value.text.push_back(char(0x80 | (code & 0x3F)));
                } else {
                    value.text.push_back(char(0xE0 | (code >> 12)));
                    value.text.push_back(char(0x80 | ((code >> 6) & 0x3F)));
                    value.text.push_back(char(0x80 | (code & 0x3F)));
                }
            } else
                require(false, "invalid string escape");
        }
        require(false, "unterminated string");
        return value;
    }

    Json boolean() {
        Json value;
        value.kind = Json::Bool;
        if (in.compare(at, 4, "true") == 0) {
            value.boolean = true;
            at += 4;
            return value;
        }
        require(in.compare(at, 5, "false") == 0, "expected boolean");
        at += 5;
        return value;
    }

    Json null() {
        require(in.compare(at, 4, "null") == 0, "expected null");
        at += 4;
        Json value;
        value.kind = Json::Null;
        return value;
    }

    Json number() {
        skip();
        size_t start = at;
        require(at < in.size(), "expected number");
        if (in[at] == '-')
            ++at;
        require(at < in.size() && in[at] >= '0' && in[at] <= '9', "expected number");
        while (at < in.size() && in[at] >= '0' && in[at] <= '9')
            ++at;
        if (at < in.size() && in[at] == '.') {
            ++at;
            require(at < in.size() && in[at] >= '0' && in[at] <= '9', "expected fraction");
            while (at < in.size() && in[at] >= '0' && in[at] <= '9')
                ++at;
        }
        if (at < in.size() && (in[at] == 'e' || in[at] == 'E')) {
            ++at;
            if (at < in.size() && (in[at] == '+' || in[at] == '-'))
                ++at;
            require(at < in.size() && in[at] >= '0' && in[at] <= '9', "expected exponent");
            while (at < in.size() && in[at] >= '0' && in[at] <= '9')
                ++at;
        }
        Json value;
        value.kind = Json::Number;
        value.number = std::strtod(in.c_str() + start, nullptr);
        require(std::isfinite(value.number), "non-finite number");
        return value;
    }
};

const Json &need(const Json &object, const char *key) {
    const Json *value = object.field(key);
    require(value, "missing snapshot field");
    return *value;
}

double number_of(const Json &value) {
    require(value.kind == Json::Number, "expected number");
    return value.number;
}

bool flag_of(const Json &value) {
    if (value.kind == Json::Bool)
        return value.boolean;
    if (value.kind == Json::Number)
        return value.number != 0;
    require(false, "expected boolean");
    return false;
}

MathVector3f vector_of(const Json &value) {
    require(value.kind == Json::Array && value.items.size() == 3, "expected a 3-vector");
    return v(number_of(value.items[0]), number_of(value.items[1]), number_of(value.items[2]));
}

MathQuaternionf quaternion_of(const Json &value) {
    require(value.kind == Json::Array && value.items.size() == 4, "expected an xyzw quaternion");
    MathQuaternionf quaternion{};
    for (int i = 0; i < 4; ++i)
        quaternion.buf[i] = float(number_of(value.items[size_t(i)]));
    return quaternion;
}

MathPose pose_of(const Json &value) {
    require(value.kind == Json::Object, "expected a pose");
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

StoredShape shape_of(const Json &value) {
    require(value.kind == Json::Object, "expected a shape");
    StoredShape stored;
    collider_default(&stored.collider);
    double kind_number = number_of(need(value, "kind"));
    require(kind_number == std::floor(kind_number) && kind_number >= 0 && kind_number <= 3, "unknown shape");
    stored.collider.data.kind = uint32_t(kind_number);
    stored.collider.data.pose = pose_of(need(value, "pose"));
    stored.collider.data.collisions = flag_of(need(value, "collisions"));
    if (const Json *friction = value.field("friction"))
        stored.collider.data.friction = number_of(*friction);
    if (const Json *restitution = value.field("restitution"))
        stored.collider.data.restitution = number_of(*restitution);
    if (stored.collider.data.kind == SIM_MESH) {
        const Json &vertices = need(value, "vertices");
        const Json &triangles = need(value, "triangles");
        require(vertices.kind == Json::Array && triangles.kind == Json::Array, "mesh arrays required");
        stored.mesh.positions.reserve(vertices.items.size());
        for (const Json &point : vertices.items)
            stored.mesh.positions.push_back(vector_of(point));
        stored.mesh.triangles.reserve(triangles.items.size());
        for (const Json &face : triangles.items) {
            require(face.kind == Json::Array && face.items.size() == 3, "triangle needs 3 indices");
            SimTriangle triangle{};
            triangle.a = uint32_t(number_of(face.items[0]));
            triangle.b = uint32_t(number_of(face.items[1]));
            triangle.c = uint32_t(number_of(face.items[2]));
            require(
                triangle.a < stored.mesh.positions.size() && triangle.b < stored.mesh.positions.size() &&
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

StoredBody body_of(const Json &value, const char *robot_name) {
    require(value.kind == Json::Object, "expected a body");
    StoredBody stored;
    underwater_body_default(&stored.body);
    double id = number_of(need(value, "id"));
    require(id >= 0 && id < double(UINT32_MAX) && id == std::floor(id), "invalid body id");
    stored.body.id = uint32_t(id);
    const Json &name = need(value, "name");
    require(name.kind == Json::String && !name.text.empty() && name.text.size() < sizeof stored.body.name, "invalid body name");
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
    const Json *profile = value.field("profile");
    stored.vehicle = name.text == robot_name || (profile && profile->kind == Json::String && profile->text == robot_name);
    if (!stored.vehicle) {
        require(stored.body.motion_type != SIM_DYNAMIC, "only the vehicle may be dynamic");
        const Json &shapes = need(value, "shapes");
        require(shapes.kind == Json::Array && !shapes.items.empty(), "scenery body has no geometry");
        for (const Json &shape : shapes.items)
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
        std::ifstream file(path, std::ios::binary);
        core::require(bool(file), "cannot read scene snapshot");
        std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        Parser parser(text);
        Json document = parser.parse();
        parser.skip();
        core::require(parser.at == parser.in.size(), "trailing snapshot data");
        core::require(document.kind == Json::Object, "snapshot must be an object");
        core::require(number_of(need(document, "version")) == 1, "unsupported snapshot version");
        const Json &bodies = need(document, "bodies");
        core::require(bodies.kind == Json::Array && !bodies.items.empty(), "snapshot has no bodies");

        std::vector<StoredBody> stored;
        stored.reserve(bodies.items.size());
        bool found_vehicle = false;
        MathPose start{};
        uint32_t scenery = 0, colliders = 0, vertices = 0, triangles = 0;
        for (const Json &body : bodies.items) {
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
        config.collider_capacity += colliders;
        config.vertex_capacity += vertices;
        config.triangle_capacity += triangles;
        core::check(simulation_init(simulation, &config, message, sizeof message), message);
        bool owned = true;
        try {
            core::check(
                simulation_add_robot(simulation, robot, 1, found_vehicle ? &start : nullptr, message, sizeof message),
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
                        simulation, &item.body, shapes.data(), uint32_t(shapes.size()), 0, &object, message,
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
