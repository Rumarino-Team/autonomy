#pragma once
#include "image.hpp"
#include <cerrno>
#include <sys/socket.h>
#include <system_error>
#include <unistd.h>
#include <vector>
namespace sim {
inline constexpr char worker_step_magic[8] = {'A', 'S', 'I', 'M',
                                              'S', 'T', '0', '2'};
inline constexpr char worker_reset_magic[8] = {'A', 'S', 'I', 'M',
                                               'R', 'S', '0', '2'};
inline constexpr char worker_reply_magic[8] = {'A', 'S', 'I', 'M',
                                               'R', 'P', '0', '2'};
inline constexpr char worker_error_magic[8] = {'A', 'S', 'I', 'M',
                                               'E', 'R', '0', '2'};
struct WorkerHead {
  char magic[8]{};
  uint32_t count = 0;
  uint32_t extra = 0;
  uint64_t hash = 0;
};
static_assert(sizeof(WorkerHead) == 24);
using WireBuffer = std::pmr::vector<uint8_t>;
struct WorkerReply {
  std::pmr::vector<SimBodyState> states;
  std::pmr::vector<SimContact> contacts;
  uint64_t hash = 0;
  explicit WorkerReply(std::pmr::memory_resource *r) : states(r), contacts(r) {}
};
enum class WorkerOp { Step, Reset };
struct WorkerRequest {
  WorkerOp op = WorkerOp::Step;
  std::pmr::vector<SimWrench> forces;
  std::pmr::vector<SimBodyState> states;
  explicit WorkerRequest(std::pmr::memory_resource *r) : forces(r), states(r) {}
};

inline void transfer(int fd, void *buffer, size_t n, bool send) {
  auto p = static_cast<char *>(buffer);
  while (n) {
    ssize_t k = send ? ::send(fd, p, n, MSG_NOSIGNAL) : ::recv(fd, p, n, 0);
    if (k < 0 && errno == EINTR)
      continue;
    if (k <= 0)
      throw std::runtime_error("Stonefish worker disconnected or timed out");
    p += k;
    n -= size_t(k);
  }
}
void send_frame(int fd, const void *data, size_t n);
void receive_frame(int fd, WireBuffer &out);
void worker_step(WireBuffer &out, const SimWrench *forces, uint32_t count);
void worker_reset(WireBuffer &out, const SimBodyState *states, uint32_t count);
void worker_reply(WireBuffer &out, std::span<const SimBodyState> states,
                  std::span<const SimContact> contacts, uint64_t hash);
void worker_error(WireBuffer &out, const char *message);
void read_worker_request(std::span<const uint8_t> frame, WorkerRequest &out);
void read_worker_reply(std::span<const uint8_t> frame, WorkerReply &out);
} // namespace sim
