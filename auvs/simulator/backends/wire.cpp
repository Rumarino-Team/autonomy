#include "wire.hpp"
#include <cstring>

namespace core {
namespace {
// Protocol v2 writes every scalar, including IDs, explicitly. Native struct
// padding and endianness cannot leak into worker messages. Buffers are reused.
struct Encode {
    WireBuffer &out;

    template <class T> void scalar(T value) {
        if constexpr (std::is_same_v<T, double>)
            scalar(std::bit_cast<uint64_t>(value));
        else
            for (size_t i = 0; i < sizeof(T); ++i)
                out.push_back(uint8_t(value >> (i * 8)));
    }

    void vec(MathVector3f v) {
        scalar(double(v.buf[0]));
        scalar(double(v.buf[1]));
        scalar(double(v.buf[2]));
    }

    void quat(MathQuaternionf q) {
        scalar(double(q.buf[0]));
        scalar(double(q.buf[1]));
        scalar(double(q.buf[2]));
        scalar(double(q.buf[3]));
    }

    void pose(MathPose p) {
        vec(p.pos);
        quat(p.quat);
    }

    void state(SimBodyState s) {
        scalar(s.id);
        pose(s.pose);
        vec(s.center_of_mass);
        vec(s.linear_velocity);
        vec(s.angular_velocity);
    }

    void force(SimWrench w) {
        scalar(w.id);
        vec(w.force);
        vec(w.torque);
    }

    void contact(SimContact c) {
        scalar(c.a);
        scalar(c.b);
        vec(c.position);
        vec(c.normal);
        scalar(c.normal_force);
    }

    void head(const char *magic, uint32_t count, uint32_t extra, uint64_t hash) {
        out.clear();
        out.insert(out.end(), magic, magic + 8);
        scalar(count);
        scalar(extra);
        scalar(hash);
    }
};

struct Decode {
    std::span<const uint8_t> bytes;
    size_t at = 0;

    template <class T> T scalar() {
        if constexpr (std::is_same_v<T, double>)
            return std::bit_cast<double>(scalar<uint64_t>());
        else {
            require(bytes.size() - at >= sizeof(T), "truncated worker field");
            T v = 0;
            for (size_t i = 0; i < sizeof(T); ++i)
                v |= T(bytes[at++]) << (i * 8);
            return v;
        }
    }

    MathVector3f vec() {
        double x = scalar<double>(), y = scalar<double>(), z = scalar<double>();
        return core::v(x, y, z);
    }

    MathQuaternionf quat() {
        double x = scalar<double>(), y = scalar<double>(), z = scalar<double>(),
               w = scalar<double>();
        return core::q(x, y, z, w);
    }

    MathPose pose() {
        auto p = vec();
        return {p, quat()};
    }

    SimBodyState state() {
        auto id = scalar<uint32_t>();
        auto p = pose();
        auto c = vec();
        auto v = vec();
        return {id, p, c, v, vec()};
    }

    SimWrench force() {
        auto id = scalar<uint32_t>();
        auto f = vec();
        return {id, f, vec()};
    }

    SimContact contact() {
        auto a = scalar<uint32_t>(), b = scalar<uint32_t>();
        auto p = vec();
        auto n = vec();
        return {a, b, p, n, scalar<double>()};
    }

    WorkerHead head() {
        require(bytes.size() >= 24, "truncated worker message");
        WorkerHead h;
        std::memcpy(h.magic, bytes.data(), 8);
        at = 8;
        h.count = scalar<uint32_t>();
        h.extra = scalar<uint32_t>();
        h.hash = scalar<uint64_t>();
        return h;
    }
};

constexpr size_t state_bytes = 132, force_bytes = 52, contact_bytes = 64;
} // namespace

void send_frame(int fd, const void *data, size_t n) {
    require(n && n <= SIM_MAX_SCENE_BYTES, "worker message too large");
    uint32_t size = n;
    unsigned char bytes[4] = {
        uint8_t(size), uint8_t(size >> 8), uint8_t(size >> 16), uint8_t(size >> 24)
    };
    transfer(fd, bytes, 4, true);
    transfer(fd, const_cast<void *>(data), n, true);
}

void receive_frame(int fd, WireBuffer &out) {
    unsigned char bytes[4];
    transfer(fd, bytes, 4, false);
    uint32_t n = uint32_t(bytes[0]) | (uint32_t(bytes[1]) << 8) | (uint32_t(bytes[2]) << 16) |
                 (uint32_t(bytes[3]) << 24);
    require(n && n <= SIM_MAX_SCENE_BYTES, "invalid worker message size");
    out.resize(n);
    transfer(fd, out.data(), n, false);
}

void worker_step(WireBuffer &out, const SimWrench *forces, uint32_t count) {
    require(
        (!count || forces) && uint64_t(count) * force_bytes + 24 <= SIM_MAX_SCENE_BYTES,
        "invalid step message size"
    );
    Encode e{out};
    e.head(worker_step_magic, count, 0, 0);
    for (uint32_t i = 0; i < count; ++i)
        e.force(forces[i]);
}

void worker_reset(WireBuffer &out, const SimBodyState *states, uint32_t count) {
    require(
        (!count || states) && uint64_t(count) * state_bytes + 24 <= SIM_MAX_SCENE_BYTES,
        "invalid reset message size"
    );
    Encode e{out};
    e.head(worker_reset_magic, count, 0, 0);
    for (uint32_t i = 0; i < count; ++i)
        e.state(states[i]);
}

void worker_reply(
    WireBuffer &out,
    std::span<const SimBodyState> states,
    std::span<const SimContact> contacts,
    uint64_t hash
) {
    require(
        states.size() <= UINT32_MAX && contacts.size() <= UINT32_MAX &&
            24 + states.size() * state_bytes + contacts.size() * contact_bytes <=
                SIM_MAX_SCENE_BYTES,
        "worker reply too large"
    );
    Encode e{out};
    e.head(worker_reply_magic, states.size(), contacts.size(), hash);
    for (auto &s : states)
        e.state(s);
    for (auto &c : contacts)
        e.contact(c);
}

void worker_error(WireBuffer &out, const char *message) {
    require(message, "missing worker error");
    auto n = std::strlen(message);
    require(n <= SIM_MAX_SCENE_BYTES - 24, "worker error too large");
    Encode e{out};
    e.head(worker_error_magic, n, 0, 0);
    out.insert(out.end(), message, message + n);
}

void read_worker_request(std::span<const uint8_t> frame, WorkerRequest &out) {
    Decode d{frame};
    auto h = d.head();
    require(h.extra == 0 && h.hash == 0, "invalid worker request header");
    if (!std::memcmp(h.magic, worker_step_magic, 8)) {
        require(uint64_t(h.count) * force_bytes == frame.size() - 24, "invalid worker force array");
        out.op = WorkerOp::Step;
        out.forces.resize(h.count);
        out.states.clear();
        for (auto &f : out.forces)
            f = d.force();
    } else {
        require(
            !std::memcmp(h.magic, worker_reset_magic, 8),
            "unsupported worker protocol; version 2 is required"
        );
        require(uint64_t(h.count) * state_bytes == frame.size() - 24, "invalid worker state array");
        out.op = WorkerOp::Reset;
        out.states.resize(h.count);
        out.forces.clear();
        for (auto &s : out.states)
            s = d.state();
    }
}

void read_worker_reply(std::span<const uint8_t> frame, WorkerReply &out) {
    Decode d{frame};
    auto h = d.head();
    if (!std::memcmp(h.magic, worker_error_magic, 8)) {
        require(
            h.extra == 0 && h.hash == 0 && h.count == frame.size() - 24, "invalid worker error"
        );
        throw std::runtime_error(
            std::string(reinterpret_cast<const char *>(frame.data() + 24), h.count)
        );
    }
    require(
        !std::memcmp(h.magic, worker_reply_magic, 8),
        "unsupported worker reply; version 2 is required"
    );
    require(
        uint64_t(h.count) * state_bytes + uint64_t(h.extra) * contact_bytes == frame.size() - 24,
        "invalid worker reply arrays"
    );
    out.states.resize(h.count);
    out.contacts.resize(h.extra);
    out.hash = h.hash;
    for (auto &s : out.states)
        s = d.state();
    for (auto &c : out.contacts)
        c = d.contact();
}
} // namespace core
