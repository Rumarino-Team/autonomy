// Binary input remains available for backend IPC, resolved replay and tests.
// Saved .blend files are read directly in blend/native_scene.cpp.
#include "image.hpp"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>
namespace sim {
namespace {
constexpr char magic_bytes[] = {'A', 'S', 'I', 'M', 'S', 'C', '0', '3'};
void read_exact(int fd, void *data, size_t count,
                std::chrono::steady_clock::time_point deadline) {
  auto out = static_cast<char *>(data);
  while (count) {
    auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                         deadline - std::chrono::steady_clock::now())
                         .count();
    require(remaining > 0, "Blender scene transfer timed out");
    pollfd p{fd, POLLIN, 0};
    int ready = ::poll(&p, 1, int(remaining));
    if (ready < 0 && errno == EINTR)
      continue;
    require(ready > 0, "Blender scene transfer timed out or failed");
    ssize_t n = ::read(fd, out, count);
    if (n < 0 && errno == EINTR)
      continue;
    require(n > 0, "truncated binary scene transfer");
    out += n;
    count -= n;
  }
}
} // namespace
MainPtr Main::load_fd(int fd) {
  require(fd >= 0, "invalid scene descriptor");
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(120);
  unsigned char header[16];
  read_exact(fd, header, sizeof(header), deadline);
  require(std::memcmp(header, magic_bytes, 8) == 0,
          "unsupported scene format; ASIMSC03 is required (regenerate old recordings)");
  uint64_t count = 0;
  for (int i = 0; i < 8; ++i)
    count |= uint64_t(header[8 + i]) << (8 * i);
  require(count && count <= max_scene_bytes, "invalid binary scene size");
  struct stat status {};
  if (::fstat(fd, &status) == 0 && S_ISREG(status.st_mode))
    require(uint64_t(status.st_size) == 16 + count,
            "binary scene file size does not match its header");
  std::vector<uint8_t> file(16 + count);
  std::memcpy(file.data(), header, 16);
  read_exact(fd, file.data() + 16, count, deadline);
  return Main::read(file.data(), file.size());
}
} // namespace sim
