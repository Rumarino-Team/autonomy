#pragma once
#include "scene.hpp"
#include <cerrno>
#include <sys/socket.h>
#include <system_error>
#include <unistd.h>
namespace sim {
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
inline void send_message(int fd, const Json &j) {
  auto s = j.dump();
  require(s.size() <= 128 * 1024 * 1024, "worker message too large");
  uint32_t size = s.size();
  unsigned char bytes[4] = {static_cast<unsigned char>(size),
                            static_cast<unsigned char>(size >> 8),
                            static_cast<unsigned char>(size >> 16),
                            static_cast<unsigned char>(size >> 24)};
  transfer(fd, bytes, 4, true);
  transfer(fd, s.data(), s.size(), true);
}
inline Json receive_message(int fd) {
  unsigned char bytes[4];
  transfer(fd, bytes, 4, false);
  uint32_t n = uint32_t(bytes[0]) | (uint32_t(bytes[1]) << 8) |
               (uint32_t(bytes[2]) << 16) | (uint32_t(bytes[3]) << 24);
  require(n && n <= 128 * 1024 * 1024, "invalid worker message size");
  std::string s(n, '\0');
  transfer(fd, s.data(), s.size(), false);
  return Json::parse(s);
}
} // namespace sim
