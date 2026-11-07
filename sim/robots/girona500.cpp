#include "definition.hpp"
namespace sim {
namespace {
#include "girona500_geometry.inc"
} // namespace

// Complete fixed vehicle: physical body, collision references, fluid surfaces,
// motors and the single camera. Edit these values as one configuration.
const RobotDefinition girona500{
  .name = "girona500",
  .asset = "robots/girona500.blend",
  .description = "Mass, augmented inertia, COM and displacement extracted from the pinned Stonefish Girona500 model. Fixed convex envelope; default portable rotor inertia, not a newly measured vehicle calibration.",
  .body = {
    .mass = 865.8697498294008,
    .inertia = {97.20027671486048, 129.70958836715837, 138.28526970223857},
    .center_of_mass = {{-0.0732347554422949, 0.0002664892857737885, 0.05430864068198341}, {-0.7018863880125085, 0.08991465957591266, 0.08994423021514532, 0.70084298367545}},
    .dry_mass = 143.14603282315827,
    .volume = 0.14706076542512625,
    .buoyancy_center = {-0.06531234519691499, 0.0002589629590271144, -0.17474323190136395},
  },
  .shapes = {{
    {.id = "girona500/hull/0", .kind = 3, .collisions = true, .points = collider_0_points},
  }},
  .hulls = {{
    {.physics = {
       .pose = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}},
       .drag = {1.0, 1.0, 1.0}, .skin = {0.01, 0.01, 0.01},
       .centroid = {-0.06531234519691499, 0.0002589629590271144, -0.17474323190136395},
       .coefficient_frame = {0.0, 0.0, 0.0, 1.0},
       .volume = 0.14706076542512625,
     },
     .collider = 0},
  }},
  .thrusters = {{
    {.position = {-0.3296999931335449, -0.2587001323699951, -0.021000027656555176},
     .direction = {-0.9999999999999887, 1.5099580252620062e-07, 2.1262273748361194e-20},
     .spec = {.max_rpm = 1000.0, .diameter = 0.18,
              .thrust = 0.48, .torque = 0.05,
              .inertia = 0.00146468, .kp = 1.0,
              .ki = 0.5, .command_sign = -1.0}},
    {.position = {-0.3296999931335449, 0.258699893951416, -0.021000027656555176},
     .direction = {-0.9999999999999887, 0.0, -1.5099580252808492e-07},
     .spec = {.max_rpm = 1000.0, .diameter = 0.18,
              .thrust = 0.48, .torque = 0.05,
              .inertia = 0.00146468, .kp = 1.0,
              .ki = 0.5, .command_sign = -1.0}},
    {.position = {-0.06270003318786621, 0.030699968338012695, -0.021000027656555176},
     .direction = {-0.00020329249977890513, 0.9999999793111215, -7.065136054099844e-06},
     .spec = {.max_rpm = 1000.0, .diameter = 0.18,
              .thrust = 0.48, .torque = 0.05,
              .inertia = 0.00146468, .kp = 1.0,
              .ki = 0.5, .command_sign = -1.0}},
    {.position = {-0.5827000141143799, 0.0, -0.31369996070861816},
     .direction = {-1.7826095952071902e-08, 0.00020360946423775438, -0.9999999792715926},
     .spec = {.max_rpm = 1000.0, .diameter = 0.18,
              .thrust = 0.48, .torque = 0.05,
              .inertia = 0.00146468, .kp = 1.0,
              .ki = 0.5, .command_sign = -1.0}},
    {.position = {0.5346998572349548, 0.0, -0.31369996070861816},
     .direction = {-6.582346056908384e-08, -0.00020381805341247657, -0.9999999792290981},
     .spec = {.max_rpm = 1000.0, .diameter = 0.18,
              .thrust = 0.48, .torque = 0.05,
              .inertia = 0.00146468, .kp = 1.0,
              .ki = 0.5, .command_sign = -1.0}},
  }},
  .camera = {.width = 800, .height = 600,
             .horizontal_fov = 60.000003546768944,
             .pose = {{0.5, -0.11999999731779099, 0.5}, {0.4999491572380066, 0.5000509023666382, 0.4999490976333618, -0.5000508427619934}}},
  .shape_count = 1,
  .hull_count = 1,
  .thruster_count = 5,
};
} // namespace sim
