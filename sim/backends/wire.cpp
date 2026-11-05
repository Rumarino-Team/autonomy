#include "wire.hpp"
#include <cstdint>
#include <cstring>
namespace sim {
namespace {
std::vector<uint8_t> pack(const char magic[8], uint32_t count, uint32_t extra,
                          uint64_t hash, const void *first, size_t first_bytes,
                          const void *second, size_t second_bytes) {
  uint64_t total = sizeof(WorkerHead) + uint64_t(first_bytes) + second_bytes;
  require(total && total <= 128 * 1024 * 1024, "worker message too large");
  WorkerHead head{};
  std::memcpy(head.magic, magic, 8);
  head.count = count;
  head.extra = extra;
  head.hash = hash;
  std::vector<uint8_t> out(static_cast<size_t>(total));
  std::memcpy(out.data(), &head, sizeof(head));
  auto p = out.data() + sizeof(head);
  if (first_bytes) {
    std::memcpy(p, first, first_bytes);
    p += first_bytes;
  }
  if (second_bytes)
    std::memcpy(p, second, second_bytes);
  return out;
}
template <class T>
std::vector<T> take(const uint8_t *data, uint32_t count, size_t bytes) {
  require(uint64_t(count) * sizeof(T) == bytes, "invalid worker array");
  std::vector<T> out(count);
  if (count)
    std::memcpy(out.data(), data, bytes);
  return out;
}
} // namespace
void send_frame(int fd, const void *data, size_t n) {
  require(n && n <= 128 * 1024 * 1024, "worker message too large");
  uint32_t size = n;
  unsigned char bytes[4] = {static_cast<unsigned char>(size),
                            static_cast<unsigned char>(size >> 8),
                            static_cast<unsigned char>(size >> 16),
                            static_cast<unsigned char>(size >> 24)};
  transfer(fd, bytes, 4, true);
  transfer(fd, const_cast<void *>(data), n, true);
}
std::vector<uint8_t> receive_frame(int fd) {
  unsigned char bytes[4];
  transfer(fd, bytes, 4, false);
  uint32_t n = uint32_t(bytes[0]) | (uint32_t(bytes[1]) << 8) |
               (uint32_t(bytes[2]) << 16) | (uint32_t(bytes[3]) << 24);
  require(n && n <= 128 * 1024 * 1024, "invalid worker message size");
  std::vector<uint8_t> out(n);
  transfer(fd, out.data(), out.size(), false);
  return out;
}
std::vector<uint8_t> worker_step(const SimWrench *forces, uint32_t count) {
  require(!count || forces, "missing step wrenches");
  return pack(worker_step_magic, count, 0, 0, forces, count * sizeof(SimWrench),
              nullptr, 0);
}
std::vector<uint8_t> worker_reset(const SimBodyState *states, uint32_t count) {
  require(!count || states, "missing reset states");
  return pack(worker_reset_magic, count, 0, 0, states,
              count * sizeof(SimBodyState), nullptr, 0);
}
std::vector<uint8_t> worker_reply(const std::vector<SimBodyState> &states,
                                  const std::vector<SimContact> &contacts,
                                  uint64_t hash) {
  require(states.size() <= UINT32_MAX && contacts.size() <= UINT32_MAX,
          "worker reply too large");
  return pack(worker_reply_magic, uint32_t(states.size()),
              uint32_t(contacts.size()), hash, states.data(),
              states.size() * sizeof(SimBodyState), contacts.data(),
              contacts.size() * sizeof(SimContact));
}
std::vector<uint8_t> worker_error(const char *message) {
  require(message, "missing worker error");
  auto n = std::strlen(message);
  require(n <= UINT32_MAX, "worker error too large");
  return pack(worker_error_magic, uint32_t(n), 0, 0, message, n, nullptr, 0);
}
WorkerRequest read_worker_request(const std::vector<uint8_t> &frame) {
  require(frame.size() >= sizeof(WorkerHead), "truncated worker message");
  WorkerHead head{};
  std::memcpy(&head, frame.data(), sizeof(head));
  auto body = frame.data() + sizeof(head);
  auto bytes = frame.size() - sizeof(head);
  WorkerRequest request;
  if (std::memcmp(head.magic, worker_step_magic, 8) == 0) {
    require(head.extra == 0 && head.hash == 0, "invalid step message");
    request.op = WorkerOp::Step;
    request.forces = take<SimWrench>(body, head.count, bytes);
    return request;
  }
  require(std::memcmp(head.magic, worker_reset_magic, 8) == 0,
          "unknown worker command");
  require(head.extra == 0 && head.hash == 0, "invalid reset message");
  request.op = WorkerOp::Reset;
  request.states = take<SimBodyState>(body, head.count, bytes);
  return request;
}
WorkerReply read_worker_reply(const std::vector<uint8_t> &frame) {
  require(frame.size() >= sizeof(WorkerHead), "truncated worker reply");
  WorkerHead head{};
  std::memcpy(&head, frame.data(), sizeof(head));
  auto body = frame.data() + sizeof(head);
  auto bytes = frame.size() - sizeof(head);
  if (std::memcmp(head.magic, worker_error_magic, 8) == 0) {
    require(head.extra == 0 && head.hash == 0 && head.count == bytes,
            "invalid worker error");
    throw std::runtime_error(std::string(reinterpret_cast<const char *>(body), bytes));
  }
  require(std::memcmp(head.magic, worker_reply_magic, 8) == 0,
          "invalid worker reply");
  uint64_t state_bytes = uint64_t(head.count) * sizeof(SimBodyState);
  uint64_t contact_bytes = uint64_t(head.extra) * sizeof(SimContact);
  require(state_bytes + contact_bytes == bytes, "truncated worker reply");
  WorkerReply reply;
  reply.hash = head.hash;
  reply.states = take<SimBodyState>(body, head.count, size_t(state_bytes));
  reply.contacts = take<SimContact>(body + state_bytes, head.extra, size_t(contact_bytes));
  return reply;
}
} // namespace sim
