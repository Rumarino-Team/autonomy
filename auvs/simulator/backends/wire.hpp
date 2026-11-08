#pragma once
#include "../internal.hpp"

namespace core {
inline constexpr char worker_step_magic[8] = {'A', 'S', 'I', 'M', 'S', 'T', '0', '2'},
                      worker_reset_magic[8] = {'A', 'S', 'I', 'M', 'R', 'S', '0', '2'},
                      worker_reply_magic[8] = {'A', 'S', 'I', 'M', 'R', 'P', '0', '2'},
                      worker_error_magic[8] = {'A', 'S', 'I', 'M', 'E', 'R', '0', '2'};
inline constexpr char scene_magic[8] = {'A', 'S', 'I', 'M', 'S', 'C', '0', '3'};

struct WorkerHead {
    char magic[8]{};
    uint32_t count = 0, extra = 0;
    uint64_t hash = 0;
};

using WireBuffer = Buffer<uint8_t>;

struct WorkerReply {
    explicit WorkerReply(SimAllocator a = {})
        : states(a),
          contacts(a) {
    }

    Buffer<SimBodyState> states;
    Buffer<SimContact> contacts;
    uint64_t hash = 0;
};
enum class WorkerOp {
    Step,
    Reset
};

struct WorkerRequest {
    WorkerOp op = WorkerOp::Step;
    Buffer<SimWrench> forces;
    Buffer<SimBodyState> states;
};

void send_frame(int, const void *, size_t);
void receive_frame(int, WireBuffer &);
void worker_step(WireBuffer &, const SimWrench *, uint32_t);
void worker_reset(WireBuffer &, const SimBodyState *, uint32_t);
void worker_reply(
    WireBuffer &, std::span<const SimBodyState>, std::span<const SimContact>, uint64_t
);
void worker_error(WireBuffer &, const char *);
void read_worker_request(std::span<const uint8_t>, WorkerRequest &);
void read_worker_reply(std::span<const uint8_t>, WorkerReply &);
/* Physics-only ASIMSC03 projection; no live pointers or SIMD padding on the
 * wire. */
void encode_scene(const Simulation &, WireBuffer &);

struct ModelOwner {
    Simulation value{};

    ~ModelOwner() {
        simulation_destroy(&value);
    }

    ModelOwner() = default;
    ModelOwner(const ModelOwner &) = delete;
};

std::unique_ptr<ModelOwner> decode_scene(std::span<const uint8_t>);
} // namespace core
