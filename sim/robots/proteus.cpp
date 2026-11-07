#include "definition.hpp"
namespace sim {
namespace {
#include "proteus_geometry.inc"
} // namespace

// Complete fixed vehicle: physical body, collision references, fluid surfaces,
// motors and the single camera. Edit these values as one configuration.
const RobotDefinition proteus{
  .name = "proteus",
  .asset = "robots/proteus.blend",
  .description = "Physical parameters from the earlier simulator preset. Fixed convex collision/hydrodynamic envelope of the saved visual body; not a newly measured vehicle calibration.",
  .body = {
    .mass = 63.83969252,
    .inertia = {0.64460719, 0.67566181, 1.12846747},
    .center_of_mass = {{0.0, 0.0, -0.00486738}, {0.001358089995564168, 0.010049569967175813, 0.001107049996384122, 0.9999479667339418}},
    .dry_mass = 4.29466999,
    .volume = 0.0038491,
    .buoyancy_center = {0.0, 0.0, -0.0056495},
  },
  .shapes = {{
    {.id = "proteus/hull/0", .kind = 3, .collisions = true, .points = collider_0_points},
  }},
  .hulls = {{
    {.physics = {
       .pose = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}},
       .drag = {0.5, 1.0, 0.5}, .skin = {0.05, 0.1, 0.05},
       .centroid = {0.0, 0.0, -0.0056495},
       .coefficient_frame = {0.0, 0.0, 0.0, 1.0},
       .volume = 0.0038491,
     },
     .collider = 0},
  }},
  .thrusters = {{
    {.position = {0.20500004291534424, 0.0, 0.13000011444091797},
     .direction = {8.972070495364297e-05, 0.9999999959750975, -1.1784302913357267e-16},
     .spec = {.max_rpm = 1000.0, .diameter = 0.18,
              .thrust = 0.48, .torque = 0.05,
              .inertia = 0.00019288, .kp = 1.0,
              .ki = 10.0, .command_sign = -1.0}},
    {.position = {-0.20500004291534424, 0.0, 0.13000011444091797},
     .direction = {8.972070495364297e-05, 0.9999999959750975, -1.1784302913357267e-16},
     .spec = {.max_rpm = 1000.0, .diameter = 0.18,
              .thrust = 0.48, .torque = 0.05,
              .inertia = 0.00019288, .kp = 1.0,
              .ki = 10.0, .command_sign = -1.0}},
    {.position = {0.16499996185302734, 0.26499998569488525, -0.0},
     .direction = {0.00020317170536215766, 0.0, -0.9999999793606288},
     .spec = {.max_rpm = 1000.0, .diameter = 0.18,
              .thrust = 0.48, .torque = 0.05,
              .inertia = 0.00019288, .kp = 1.0,
              .ki = 10.0, .command_sign = -1.0}},
    {.position = {-0.16499996185302734, 0.26499998569488525, -0.0},
     .direction = {0.00020317170536215766, 0.0, -0.9999999793606288},
     .spec = {.max_rpm = 1000.0, .diameter = 0.18,
              .thrust = 0.48, .torque = 0.05,
              .inertia = 0.00019288, .kp = 1.0,
              .ki = 10.0, .command_sign = -1.0}},
    {.position = {0.16499996185302734, -0.26499998569488525, -0.0},
     .direction = {0.00020317170536215766, 0.0, -0.9999999793606288},
     .spec = {.max_rpm = 1000.0, .diameter = 0.18,
              .thrust = 0.48, .torque = 0.05,
              .inertia = 0.00019288, .kp = 1.0,
              .ki = 10.0, .command_sign = -1.0}},
    {.position = {-0.16499996185302734, -0.26499998569488525, -0.0},
     .direction = {0.00020317170536215766, 0.0, -0.9999999793606288},
     .spec = {.max_rpm = 1000.0, .diameter = 0.18,
              .thrust = 0.48, .torque = 0.05,
              .inertia = 0.00019288, .kp = 1.0,
              .ki = 10.0, .command_sign = -1.0}},
  }},
  .camera = {.width = 800, .height = 600,
             .horizontal_fov = 60.000003546768944,
             .pose = {{4.470348358154297e-08, 0.3000001311302185, -1.1920928955078125e-07}, {0.4782053232192993, 0.5407803058624268, 0.45771893858909607, -0.5190085768699646}}},
  .shape_count = 1,
  .hull_count = 1,
  .thruster_count = 6,
};
} // namespace sim
