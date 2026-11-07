#pragma once
#include "runtime.hpp"
namespace sim {
// Optional presentation output. Never feeds wall time or Blender state back
// into the fixed-step world. Wire-compatible with the experimental AMJB link.
class BlenderPublisher {
  int socket_ = -1;
  uint16_t port_;
  double next_time_ = 0;
  Memory memory_;
  std::pmr::vector<unsigned char> packet_{&memory_.pool};

public:
  explicit BlenderPublisher(uint16_t port);
  ~BlenderPublisher();
  BlenderPublisher(const BlenderPublisher &) = delete;
  BlenderPublisher &operator=(const BlenderPublisher &) = delete;
  void publish(const World &world, const Observation &observation);
};
} // namespace sim
