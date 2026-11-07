#include "wire.hpp"
#include <cfloat>

namespace core {
namespace {
struct Writer {
    WireBuffer bytes;

    template <class T> void put(T x) {
        if constexpr (std::is_same_v<T, double>)
            put(std::bit_cast<uint64_t>(x));
        else
            for (size_t i = 0; i < sizeof(T); ++i)
                bytes.push_back(uint8_t(x >> (8 * i)));
    }

    void zero(size_t n) {
        for (size_t i = 0; i < n; ++i)
            put(uint8_t(0));
    }

    void vec(MathVector3f a) {
        for (float f : a.buf)
            put(double(f));
    }

    void quat(MathQuaternionf a) {
        for (float f : a.buf)
            put(double(f));
    }

    void pose(MathPose a) {
        vec(a.pos);
        quat(a.quat);
    }

    void text(const char *p, size_t n) {
        for (size_t i = 0; i < n; ++i)
            put(uint8_t(p[i]));
    }
};

struct Reader {
    std::span<const uint8_t> b;
    size_t at = 0;

    template <class T> T get() {
        if constexpr (std::is_same_v<T, double>)
            return std::bit_cast<double>(get<uint64_t>());
        else {
            require(at <= b.size() && sizeof(T) <= b.size() - at, "truncated scene");
            T r = 0;
            for (size_t i = 0; i < sizeof(T); ++i)
                r |= T(b[at++]) << (8 * i);
            return r;
        }
    }

    void skip(size_t n) {
        require(at <= b.size() && n <= b.size() - at, "truncated scene");
        at += n;
    }

    MathVector3f vec() {
        double x = get<double>(), y = get<double>(), z = get<double>();
        require(
            std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::abs(x) <= FLT_MAX &&
                std::abs(y) <= FLT_MAX && std::abs(z) <= FLT_MAX,
            "invalid float transport vector"
        );
        return v(x, y, z);
    }

    MathQuaternionf quat() {
        double x = get<double>(), y = get<double>(), z = get<double>(), w = get<double>();
        require(
            std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(w) &&
                std::abs(x) <= FLT_MAX && std::abs(y) <= FLT_MAX && std::abs(z) <= FLT_MAX &&
                std::abs(w) <= FLT_MAX,
            "invalid float transport quaternion"
        );
        return q(x, y, z, w);
    }

    MathPose pose() {
        auto p = vec();
        return {p, quat()};
    }

    void text(char *p, size_t n) {
        require(at <= b.size() && n <= b.size() - at, "truncated string");
        std::memcpy(p, b.data() + at, n);
        at += n;
    }
};

constexpr size_t widths[] = {408, 328, 168, 24, 12, 120, 184, 24, 32, 32, 8, 16, 4};

struct WireBody {
    UnderwaterBody body;
    uint32_t begin, count;
};

struct WireShape {
    ShapeRec shape;
    uint32_t vb, vc, tb, tc;
};
} // namespace

void encode_scene(const Simulation &m, WireBuffer &out) {
    validate_model(&m);
    Writer s[13];
    uint32_t counts[13]{};
    counts[0] = 1;
    counts[1] = m.object_count;
    uint32_t vertices = 0, triangles = 0, shapes = 0;
    for (uint32_t i = 0; i < m.geometry_count; ++i) {
        auto &g = m.geometry[i];
        s[11].put(vertices);
        s[11].put(g.position_count);
        s[11].put(triangles);
        s[11].put(g.triangle_count);
        for (uint32_t j = 0; j < g.position_count; ++j)
            s[3].vec(g.positions[j]);
        for (uint32_t j = 0; j < g.triangle_count; ++j) {
            auto t = g.triangles[j];
            s[4].put(t.a);
            s[4].put(t.b);
            s[4].put(t.c);
        }
        vertices += g.position_count;
        triangles += g.triangle_count;
    }
    for (uint32_t i = 0; i < m.object_count; ++i) {
        auto &o = *m.object_table[i];
        auto &b = o.body;
        auto &w = s[1];
        w.put(b.id);
        w.text(b.name, 64);
        w.put(b.cls);
        w.put(b.motion_type);
        w.put(b.tracked);
        w.put(b.fuzz);
        w.put(uint32_t(0));
        w.put(b.mass);
        w.vec(b.inertia);
        w.pose(b.center_of_mass);
        w.pose(b.initial_pose);
        w.vec(b.initial_velocity);
        w.vec(b.initial_angular_velocity);
        w.put(b.dry_mass);
        w.put(b.volume);
        w.vec(b.buoyancy_center);
        w.put(shapes);
        w.put(o.collider_count);
        for (uint32_t j = 0; j < o.collider_count; ++j) {
            auto &c = o.colliders[j];
            auto &g = c.data;
            auto &a = s[2];
            a.put(g.kind);
            a.put(g.collisions);
            a.pose(g.pose);
            a.vec(g.size);
            a.vec(g.drag);
            a.vec(g.skin);
            a.put(g.friction);
            a.put(g.restitution);
            uint32_t mesh = UINT32_MAX, vb = vertices, tb = triangles, vc = 0, tc = 0;
            if (g.kind == SIM_MESH) {
                vb = tb = 0;
                for (uint32_t k = 0; k < m.geometry_count; ++k) {
                    if (m.geometry[k].positions == c.geometry.positions &&
                        m.geometry[k].triangles == c.geometry.triangles) {
                        mesh = k;
                        vc = c.geometry.position_count;
                        tc = c.geometry.triangle_count;
                        break;
                    }
                    vb += m.geometry[k].position_count;
                    tb += m.geometry[k].triangle_count;
                }
                require(mesh != UINT32_MAX, "mesh outside model");
            }
            a.put(vb);
            a.put(vc);
            a.put(tb);
            a.put(tc);
            s[12].put(mesh);
            ++shapes;
        }
    }
    counts[2] = counts[12] = shapes;
    counts[3] = vertices;
    counts[4] = triangles;
    counts[11] = m.geometry_count;
    // Preserve the established 408-byte header and all thirteen section schemas.
    auto &h = s[0];
    h.put(uint32_t(1));
    h.put(m.settings.dt_numerator);
    h.put(m.settings.dt_denominator);
    h.put(m.settings.integrator);
    h.put(m.settings.native_hydrodynamics);
    h.put(m.settings.hydro_period);
    for (unsigned i = 1; i <= 10; ++i)
        h.put(i <= 4 ? counts[i] : uint32_t(0));
    h.put(uint32_t(0));
    h.put(uint32_t(0));
    h.put(uint64_t(0));
    h.put(uint32_t(0));
    h.put(uint32_t(0));
    h.put(m.settings.water_density);
    h.vec(m.settings.gravity);
    h.vec(m.settings.current);
    h.put(uint32_t(0));
    h.put(uint32_t(800));
    h.put(uint32_t(600));
    h.put(uint32_t(0));
    h.put(double(60));
    h.pose(MathPose{v(0, .3, 0), q()});
    h.put(m.settings.bounds.position);
    h.put(m.settings.bounds.speed);
    h.put(m.settings.bounds.angular_speed);
    h.zero(160);
    Writer payload;
    payload.put(uint32_t(13));
    payload.put(uint32_t(1));
    uint64_t offset = 8 + 13 * 24;
    for (uint32_t i = 0; i < 13; ++i) {
        require(s[i].bytes.size() == size_t(counts[i]) * widths[i], "scene schema width mismatch");
        payload.put(i + 1);
        payload.put(counts[i]);
        payload.put(offset);
        payload.put(uint64_t(s[i].bytes.size()));
        offset += s[i].bytes.size();
    }
    require(offset <= SIM_MAX_SCENE_BYTES, "scene too large");
    for (auto &a : s)
        payload.bytes.insert(payload.bytes.end(), a.bytes.begin(), a.bytes.end());
    Writer result;
    result.text(scene_magic, 8);
    result.put(uint64_t(payload.bytes.size()));
    result.bytes.insert(result.bytes.end(), payload.bytes.begin(), payload.bytes.end());
    out.clear();
    out.insert(out.end(), result.bytes.begin(), result.bytes.end());
}

std::unique_ptr<ModelOwner> decode_scene(std::span<const uint8_t> raw) {
    require(raw.size() >= 16 && !std::memcmp(raw.data(), scene_magic, 8), "ASIMSC03 required");
    Reader prefix{raw.subspan(8)};
    auto length = prefix.get<uint64_t>();
    require(length <= SIM_MAX_SCENE_BYTES && length == raw.size() - 16, "invalid scene length");
    Reader directory{raw.subspan(16)};
    require(
        directory.get<uint32_t>() == 13 && directory.get<uint32_t>() == 1, "unsupported directory"
    );
    Reader r[13]{};
    uint32_t counts[13];
    uint64_t end = 8 + 13 * 24;
    for (uint32_t i = 0; i < 13; ++i) {
        auto kind = directory.get<uint32_t>(), count = directory.get<uint32_t>();
        auto off = directory.get<uint64_t>(), n = directory.get<uint64_t>();
        require(
            kind == i + 1 && off == end && n == uint64_t(count) * widths[i] && off <= length &&
                n <= length - off,
            "invalid section directory"
        );
        r[i] = {raw.subspan(16 + off, n)};
        counts[i] = count;
        end += n;
    }
    require(
        end == length && counts[0] == 1 && counts[1] && counts[1] <= 4096 &&
            counts[2] <= 4096u * 1024u && counts[11] <= counts[2],
        "invalid section counts"
    );
    require(
        !counts[5] && !counts[6] && !counts[7] && !counts[8] && !counts[9] && !counts[10],
        "worker expects physics-only scene"
    );
    auto owner = std::make_unique<ModelOwner>();
    auto &m = owner->value;
    SimulationConfig config;
    simulation_config_default(&config);
    config.object_capacity = counts[1];
    config.collider_capacity = counts[2];
    config.vertex_capacity = counts[3];
    config.triangle_capacity = counts[4];
    char msg[256];
    check(simulation_init(&m, &config, msg, sizeof msg), msg);
    auto &h = r[0];
    require(h.get<uint32_t>() == 1, "invalid model schema");
    m.settings.dt_numerator = h.get<uint32_t>();
    m.settings.dt_denominator = h.get<uint32_t>();
    m.settings.integrator = h.get<uint32_t>();
    m.settings.native_hydrodynamics = h.get<uint32_t>();
    m.settings.hydro_period = h.get<uint32_t>();
    for (unsigned i = 1; i <= 10; ++i)
        require(h.get<uint32_t>() == counts[i], "header/table count mismatch");
    require(h.get<uint32_t>() == 0 && h.get<uint32_t>() == 0, "unexpected command schema");
    h.skip(16);
    m.settings.water_density = h.get<double>();
    m.settings.gravity = h.vec();
    m.settings.current = h.vec();
    h.skip(80);
    m.settings.bounds = {h.get<double>(), h.get<double>(), h.get<double>()};
    h.skip(160);
    require(h.at == 408, "invalid header width");
    Buffer<WireBody> bodies;
    bodies.resize(counts[1]);
    for (auto &x : bodies) {
        auto &b = x.body;
        auto &a = r[1];
        b.id = a.get<uint32_t>();
        a.text(b.name, 64);
        b.cls = a.get<uint32_t>();
        b.motion_type = a.get<uint32_t>();
        b.tracked = a.get<uint32_t>();
        b.fuzz = a.get<uint32_t>();
        a.skip(4);
        b.mass = a.get<double>();
        b.inertia = a.vec();
        b.center_of_mass = a.pose();
        b.initial_pose = a.pose();
        b.initial_velocity = a.vec();
        b.initial_angular_velocity = a.vec();
        b.dry_mass = a.get<double>();
        b.volume = a.get<double>();
        b.buoyancy_center = a.vec();
        x.begin = a.get<uint32_t>();
        x.count = a.get<uint32_t>();
    }
    Buffer<WireShape> shapes;
    shapes.resize(counts[2]);
    for (auto &x : shapes) {
        auto &a = r[2];
        auto &g = x.shape;
        g.kind = a.get<uint32_t>();
        g.collisions = a.get<uint32_t>();
        g.pose = a.pose();
        g.size = a.vec();
        g.drag = a.vec();
        g.skin = a.vec();
        g.friction = a.get<double>();
        g.restitution = a.get<double>();
        x.vb = a.get<uint32_t>();
        x.vc = a.get<uint32_t>();
        x.tb = a.get<uint32_t>();
        x.tc = a.get<uint32_t>();
    }
    Buffer<MathVector3f> positions;
    positions.resize(counts[3]);
    for (auto &p : positions)
        p = r[3].vec();
    Buffer<SimTriangle> triangles;
    triangles.resize(counts[4]);
    for (auto &t : triangles) {
        t.a = r[4].get<uint32_t>();
        t.b = r[4].get<uint32_t>();
        t.c = r[4].get<uint32_t>();
    }
    Buffer<std::array<uint32_t, 4>> ranges;
    ranges.resize(counts[11]);
    for (auto &range : ranges) {
        for (auto &x : range)
            x = r[11].get<uint32_t>();
        require(
            uint64_t(range[0]) + range[1] <= positions.size() &&
                uint64_t(range[2]) + range[3] <= triangles.size(),
            "invalid mesh range"
        );
    }
    Buffer<uint32_t> refs;
    refs.resize(counts[12]);
    require(counts[12] == counts[2], "invalid mesh reference count");
    for (auto &x : refs)
        x = r[12].get<uint32_t>();
    Buffer<uint8_t> claimed;
    claimed.resize(shapes.size());
    for (auto &x : bodies) {
        require(
            x.count && x.count <= 1024 && uint64_t(x.begin) + x.count <= shapes.size(),
            "invalid collider ownership range"
        );
        std::array<Collider, 1024> cs{};
        for (uint32_t j = 0; j < x.count; ++j) {
            auto k = x.begin + j;
            require(!claimed[k], "duplicate collider ownership");
            claimed[k] = 1;
            auto &g = shapes[k];
            cs[j].data = g.shape;
            if (g.shape.kind == SIM_MESH) {
                require(refs[k] < ranges.size(), "unknown mesh reference");
                require(
                    ranges[refs[k]] == std::array<uint32_t, 4>{g.vb, g.vc, g.tb, g.tc},
                    "mesh reference mismatch"
                );
                cs[j].geometry = {positions.p + g.vb, g.vc, triangles.p + g.tb, g.tc};
            } else
                require(refs[k] == UINT32_MAX && !g.vc && !g.tc, "primitive mesh reference");
        }
        ObjectBlock *object;
        check(
            simulation_add_object(&m, &x.body, cs.data(), x.count, 0, &object, msg, sizeof msg), msg
        );
    }
    for (auto c : claimed)
        require(c, "unowned collider");
    check(simulation_finalize(&m, msg, sizeof msg), msg);
    return owner;
}
} // namespace core
