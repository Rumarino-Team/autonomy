#pragma once
#include "image.hpp"
#include <cerrno>
#include <sys/socket.h>
#include <system_error>
#include <unistd.h>
#include <vector>
namespace sim {
inline constexpr char worker_step_magic[8] = {'A', 'S', 'I', 'M', 'S', 'T', '0', '1'};
inline constexpr char worker_reset_magic[8] = {'A', 'S', 'I', 'M', 'R', 'S', '0', '1'};
inline constexpr char worker_reply_magic[8] = {'A', 'S', 'I', 'M', 'R', 'P', '0', '1'};
inline constexpr char worker_error_magic[8] = {'A', 'S', 'I', 'M', 'E', 'R', '0', '1'};
struct WorkerHead {
  char magic[8]{};
  uint32_t count = 0;
  uint32_t extra = 0;
  uint64_t hash = 0;
};
static_assert(sizeof(WorkerHead) == 24);
struct WorkerReply {
  std::vector<SimBodyState> states;
  std::vector<SimContact> contacts;
  uint64_t hash = 0;
};
enum class WorkerOp { Step, Reset };
struct WorkerRequest {
  WorkerOp op = WorkerOp::Step;
  std::vector<SimWrench> forces;
  std::vector<SimBodyState> states;
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
std::vector<uint8_t> receive_frame(int fd);
std::vector<uint8_t> worker_step(const SimWrench *forces, uint32_t count);
std::vector<uint8_t> worker_reset(const SimBodyState *states, uint32_t count);
std::vector<uint8_t> worker_reply(const std::vector<SimBodyState> &states,
                                  const std::vector<SimContact> &contacts,
                                  uint64_t hash);
std::vector<uint8_t> worker_error(const char *message);
WorkerRequest read_worker_request(const std::vector<uint8_t> &frame);
WorkerReply read_worker_reply(const std::vector<uint8_t> &frame);
} // namespace sim
