#pragma once
#include "image.hpp"
#include <string_view>

namespace sim {
struct RobotShapeDefinition {
  std::string_view id;
  uint32_t kind = SIM_MESH;
  bool collisions = true;
  std::span<const SimVec3> points{};
};
struct RobotHullDefinition {
  HullRec physics;
  // Explicit calibrated triangle positions, or a collider index from which to
  // derive the fixed envelope surface. Geometry arrays have static lifetime.
  std::span<const SimVec3> positions{};
  uint32_t collider = UINT32_MAX;
};
// Fixed storage for the four supported vehicles. Counts select the populated
// slots (Hydrus 5/8/5, Proteus 1/6/1, BlueROV 1/8/1, Girona 1/5/1). These are
// values, not optional modules, factories or separately registered datablocks.
struct RobotDefinition {
  std::string_view name, asset, description;
  BodyRec body;
  std::array<RobotShapeDefinition, Robot::max_shapes> shapes;
  std::array<RobotHullDefinition, Robot::max_hulls> hulls;
  std::array<ThrusterRec, Robot::max_thrusters> thrusters;
  CameraRec camera;
  size_t shape_count, hull_count, thruster_count;
};
extern const RobotDefinition hydrus;
extern const RobotDefinition proteus;
extern const RobotDefinition bluerov;
extern const RobotDefinition girona500;
} // namespace sim
