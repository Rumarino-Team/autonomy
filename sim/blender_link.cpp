#include "blender_link.hpp"
#include <arpa/inet.h>
#include <bit>
#include <sys/socket.h>
#include <unistd.h>
namespace sim {
BlenderPublisher::BlenderPublisher(uint16_t port) : port_(port) {
  socket_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  require(socket_ >= 0, "cannot open Blender UDP output");
}
BlenderPublisher::~BlenderPublisher() {
  if (socket_ >= 0)
    ::close(socket_);
}
void BlenderPublisher::publish(const World &w, const Observation &observation) {
  const double time = w.timestamp_ns() * 1e-9;
  if (time + 1e-9 < next_time_)
    return;
  const bool stable_ids = w.scene.from_blender();
  std::vector<unsigned char> packet{'A', 'M', 'J', 'B', static_cast<unsigned char>(stable_ids ? 2 : 1), 0, 0, 0};
  auto number = [&](double x) {
    auto bits = std::bit_cast<uint64_t>(x);
    for (int i = 0; i < 8; ++i)
      packet.push_back(static_cast<unsigned char>(bits >> (8 * i)));
  };
  number(time);
  uint16_t count = 0;
  auto pose = [&](uint32_t id, const std::string &name, SimPose p) {
    if (name.empty() || (!stable_ids && name.size() > 31) || count == 64)
      return;
    ++count;
    if (stable_ids)
      for (int i = 0; i < 4; ++i)
        packet.push_back(static_cast<unsigned char>(id >> (8 * i)));
    auto offset = packet.size();
    packet.resize(offset + 32, 0);
    std::copy_n(name.begin(), std::min<size_t>(name.size(),31), packet.begin() + offset);
    // The original pool uses M=diag(-1,1,-1). R_blender = M R_ned M.
    number(-p.position.x);
    number(p.position.y);
    number(-p.position.z);
    number(p.orientation.w);
    number(-p.orientation.x);
    number(p.orientation.y);
    number(-p.orientation.z);
  };
  for (size_t i = 0; i < w.bodies.size(); ++i) {
    const auto &body = w.scene.bodies[i];
    if (body.motion == SIM_DYNAMIC || body.tracked || body.fuzz)
      pose(body.id, body.name, w.bodies[i].pose);
  }
  pose(UINT32_MAX, "camera", observation.camera);
  packet[6] = count & 255;
  packet[7] = count >> 8;
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port_);
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  // Dropped presentation frames or an absent listener never stop physics.
  ::sendto(socket_, packet.data(), packet.size(), MSG_DONTWAIT,
           reinterpret_cast<sockaddr *>(&address), sizeof(address));
  next_time_ = time + 1.0 / 30;
}
} // namespace sim
