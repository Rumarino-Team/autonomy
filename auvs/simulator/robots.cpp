#include "robots.h"
#include "internal.hpp"

// Saved standalone Blender assets resolved by the existing native C++ loader.
// Coordinates are NED, quaternions xyzw. Float components are rounded once at
// compile time; scalar calibration stays double. These are immutable templates,
// not registered objects. simulation_add_robot supplies ownership and arenas.
namespace {
using namespace core;
#include "robots/hydrus_geometry.inc"
constexpr SensorRec hydrus_sensors[] = {
    {.id = 1,
     .kind = SIM_SENSOR_POSE,
     .object = nullptr,
     .pose = {{{0, 0, 0}}, {{0, 0, 0, 1}}},
     .enabled = 1,
     .period_ticks = 6,
     .settings = {.tracking = {7, 7}}},
    {.id = 2,
     .kind = SIM_SENSOR_IMU,
     .object = nullptr,
     .pose = {{{0, 0, 0}}, {{0, 0, 0, 1}}},
     .enabled = 1,
     .period_ticks = 1,
     .settings = {}},
    {.id = 3,
     .kind = SIM_SENSOR_PRESSURE,
     .object = nullptr,
     .pose = {{{0, 0, 0}}, {{0, 0, 0, 1}}},
     .enabled = 1,
     .period_ticks = 6,
     .settings = {}},
    {.id = 4,
     .kind = SIM_SENSOR_CAMERA,
     .object = nullptr,
     .pose =
         {{{0.0f, 0.299999237f, -1.1920929e-07f}},
          {{-0.500398934f, 0.499602586f, -0.499600798f, -0.500397086f}}},
     .enabled = 1,
     .period_ticks = 12,
     .settings = {.camera = {800, 600, 60.000003546768944, .05, 50}}}
};
extern const Robot hydrus;
// Existing calibrated Hydrus model; fixed standalone asset.
constinit const Robot hydrus{
    .name = "hydrus",
    .asset_file = "robots/hydrus.blend",
    .asset_hash = {0x02, 0x85, 0xfe, 0xf5, 0x5d, 0x4d, 0x78, 0x4c, 0x14, 0x26, 0x77,
                   0x14, 0x53, 0x81, 0xf5, 0xc2, 0x4a, 0x06, 0x53, 0xf1, 0x87, 0x61,
                   0xb0, 0xc5, 0x4b, 0x1e, 0x90, 0x36, 0xe0, 0xaa, 0xc4, 0x22},
    .object =
        {.id = {},
         .body =
             {
                 .id = 1,
                 .name = "hydrus",
                 .cls = 0,
                 .motion_type = SIM_DYNAMIC,
                 .tracked = 0,
                 .fuzz = 0,
                 .mass = 154.627734,
                 .inertia = {{1.30448198f, 2.26130795f, 21.1694889f}},
                 .center_of_mass =
                     {{{-0.000117529999f, -0.000194890003f, -0.00638695993f}},
                      {{-6.94999981e-05f, 4.19999997e-05f, 3.99999999e-06f, 1.0f}}},
                 .initial_pose = {{{0.0f, 0.0f, 2.0f}}, {{0.0f, 0.0f, 0.0f, 1.0f}}},
                 .initial_velocity = {{0.0f, 0.0f, 0.0f}},
                 .initial_angular_velocity = {{0.0f, 0.0f, 0.0f}},
                 .dry_mass = 8.679731,
                 .volume = 0.00735533,
                 .buoyancy_center = {{-0.000146220002f, -0.000241710004f, 0.00931505021f}},
             },
         .colliders = hydrus.colliders,
         .collider_count = 5},
    .colliders =
        {
            {.data =
                 {
                     .kind = SIM_MESH,
                     .collisions = 1,
                     .pose =
                         {{{0.0f, 0.0f, -0.0774999857f}},
                          {{3.09085735e-08f, 0.707105577f, -0.707108021f, -3.0908673e-08f}}},
                     .size = {{1.0f, 1.0f, 1.0f}},
                     .drag = {{0.0f, 0.0f, 0.0f}},
                     .skin = {{0.0f, 0.0f, 0.0f}},
                     .friction = 1.0,
                     .restitution = 0.0,
                 },
             .geometry = {hydrus_collider_0_positions, 96, hydrus_collider_0_triangles, 188},
             .object = nullptr},
            {.data =
                 {
                     .kind = SIM_MESH,
                     .collisions = 1,
                     .pose =
                         {{{0.0f, 0.0f, -0.0800000429f}},
                          {{3.09085699e-08f, 0.707105577f, -0.707108021f, -3.0908673e-08f}}},
                     .size = {{1.0f, 1.0f, 1.0f}},
                     .drag = {{0.0f, 0.0f, 0.0f}},
                     .skin = {{0.0f, 0.0f, 0.0f}},
                     .friction = 1.0,
                     .restitution = 0.0,
                 },
             .geometry = {hydrus_collider_1_positions, 8, hydrus_collider_1_triangles, 12},
             .object = nullptr},
            {.data =
                 {
                     .kind = SIM_MESH,
                     .collisions = 1,
                     .pose =
                         {{{0.0f, -0.375f, -0.0800000429f}},
                          {{3.09085735e-08f, 0.707105577f, -0.707108021f, -3.0908673e-08f}}},
                     .size = {{1.0f, 1.0f, 1.0f}},
                     .drag = {{0.0f, 0.0f, 0.0f}},
                     .skin = {{0.0f, 0.0f, 0.0f}},
                     .friction = 1.0,
                     .restitution = 0.0,
                 },
             .geometry = {hydrus_collider_2_positions, 86, hydrus_collider_2_triangles, 68},
             .object = nullptr},
            {.data =
                 {
                     .kind = SIM_MESH,
                     .collisions = 1,
                     .pose =
                         {{{0.228f, -0.375f, -0.0800000429f}},
                          {{3.09085735e-08f, 0.707105577f, -0.707108021f, -3.0908673e-08f}}},
                     .size = {{1.0f, 1.0f, 1.0f}},
                     .drag = {{0.0f, 0.0f, 0.0f}},
                     .skin = {{0.0f, 0.0f, 0.0f}},
                     .friction = 1.0,
                     .restitution = 0.0,
                 },
             .geometry = {hydrus_collider_3_positions, 86, hydrus_collider_3_triangles, 68},
             .object = nullptr},
            {.data =
                 {
                     .kind = SIM_MESH,
                     .collisions = 1,
                     .pose =
                         {{{0.0f, 0.0f, 0.0699999332f}},
                          {{3.09085735e-08f, 0.707105577f, -0.707108021f, -3.0908673e-08f}}},
                     .size = {{1.0f, 1.0f, 1.0f}},
                     .drag = {{0.0f, 0.0f, 0.0f}},
                     .skin = {{0.0f, 0.0f, 0.0f}},
                     .friction = 1.0,
                     .restitution = 0.0,
                 },
             .geometry = {hydrus_collider_4_positions, 8, hydrus_collider_4_triangles, 12},
             .object = nullptr},
        },
    .thrusters =
        {
            {.object = nullptr,
             .position = {{0.197999805f, 0.406999201f, -2.38418579e-07f}},
             .direction = {{-0.382680595f, 0.923880696f, 1.50995817e-07f}},
             .spec =
                 {.max_rpm = 1000.0,
                  .diameter = 0.18,
                  .thrust = 0.48,
                  .torque = 0.05,
                  .inertia = 0.00146468,
                  .kp = 1.0,
                  .ki = 10.0,
                  .command_sign = 1.0}},
            {.object = nullptr,
             .position = {{-0.198000118f, 0.406998813f, -2.38418579e-07f}},
             .direction = {{0.382687777f, 0.923877716f, 6.99211977e-09f}},
             .spec =
                 {.max_rpm = 1000.0,
                  .diameter = 0.18,
                  .thrust = 0.48,
                  .torque = 0.05,
                  .inertia = 0.00146468,
                  .kp = 1.0,
                  .ki = 10.0,
                  .command_sign = 1.0}},
            {.object = nullptr,
             .position = {{0.19799988f, -0.40800038f, -1.1920929e-07f}},
             .direction = {{-0.382681191f, -0.923880458f, -1.50995803e-07f}},
             .spec =
                 {.max_rpm = 1000.0,
                  .diameter = 0.18,
                  .thrust = 0.48,
                  .torque = 0.05,
                  .inertia = 0.00146468,
                  .kp = 1.0,
                  .ki = 10.0,
                  .command_sign = 1.0}},
            {.object = nullptr,
             .position = {{-0.198000103f, -0.408000737f, 0.0f}},
             .direction = {{0.382687777f, -0.923877716f, 4.23655644e-09f}},
             .spec =
                 {.max_rpm = 1000.0,
                  .diameter = 0.18,
                  .thrust = 0.48,
                  .torque = 0.05,
                  .inertia = 0.00146468,
                  .kp = 1.0,
                  .ki = 10.0,
                  .command_sign = 1.0}},
            {.object = nullptr,
             .position = {{0.210999817f, 0.168999419f, 0.0f}},
             .direction = {{0.000203171701f, -1.5506588e-08f, -1.0f}},
             .spec =
                 {.max_rpm = 1000.0,
                  .diameter = 0.18,
                  .thrust = 0.48,
                  .torque = 0.05,
                  .inertia = 0.00146468,
                  .kp = 1.0,
                  .ki = 10.0,
                  .command_sign = 1.0}},
            {.object = nullptr,
             .position = {{-0.21100013f, 0.168999508f, 0.0f}},
             .direction = {{0.000203171701f, -1.5506588e-08f, -1.0f}},
             .spec =
                 {.max_rpm = 1000.0,
                  .diameter = 0.18,
                  .thrust = 0.48,
                  .torque = 0.05,
                  .inertia = 0.00146468,
                  .kp = 1.0,
                  .ki = 10.0,
                  .command_sign = 1.0}},
            {.object = nullptr,
             .position = {{0.210999921f, -0.169001371f, -3.57627869e-07f}},
             .direction = {{0.000203171701f, -1.5506588e-08f, -1.0f}},
             .spec =
                 {.max_rpm = 1000.0,
                  .diameter = 0.18,
                  .thrust = 0.48,
                  .torque = 0.05,
                  .inertia = 0.00146468,
                  .kp = 1.0,
                  .ki = 10.0,
                  .command_sign = 1.0}},
            {.object = nullptr,
             .position = {{-0.2110001f, -0.169001296f, -2.38418579e-07f}},
             .direction = {{0.000203171701f, -1.5506588e-08f, -1.0f}},
             .spec =
                 {.max_rpm = 1000.0,
                  .diameter = 0.18,
                  .thrust = 0.48,
                  .torque = 0.05,
                  .inertia = 0.00146468,
                  .kp = 1.0,
                  .ki = 10.0,
                  .command_sign = 1.0}},
        },
    .hulls =
        {
            {
                .object = nullptr,
                .pose =
                    {{{-0.000117529999f, -0.000194890003f, -0.00638695993f}},
                     {{1.80222372e-07f, 1.03068473e-07f, 1.26724956e-07f, 1.0f}}},
                .positions = hydrus_hull_0_positions,
                .position_count = 6144,
                .drag = {{0.5f, 0.999998152f, 0.500003695f}},
                .skin = {{0.0500000007f, 0.0999998152f, 0.0500003658f}},
                .centroid = {{0.000117882526f, 0.000194839769f, -0.0711132064f}},
                .coefficient_frame = {{1.80222372e-07f, 1.03068473e-07f, 1.26724956e-07f, 1.0f}},
                .volume = 0.00215788,
            },
            {
                .object = nullptr,
                .pose = {{{-0.000117529999f, -0.000194890003f, -0.00638695993f}}, {{1.80222372e-07f, 1.03068473e-07f, 1.26724956e-07f, 1.0f}}},
                .positions = hydrus_hull_1_positions,
                .position_count = 2304,
                .drag = {{0.550948977f, 0.999998212f, 0.489041358f}},
                .skin = {{0.0550948977f, 0.0999998227f, 0.0489041358f}},
                .centroid = {{0.00011788361f, 0.00019484025f, -0.0736132041f}},
                .coefficient_frame = {{1.80222372e-07f, 1.03068473e-07f, 1.26724956e-07f, 1.0f}},
                .volume = 0.00060725,
            },
            {
                .object = nullptr,
                .pose = {{{-0.000117529999f, -0.000194890003f, -0.00638695993f}}, {{1.80222372e-07f, 1.03068473e-07f, 1.26724956e-07f, 1.0f}}},
                .positions = hydrus_hull_2_positions,
                .position_count = 2304,
                .drag = {{1.0f, 0.99002403f, 0.887631536f}},
                .skin = {{0.100000001f, 0.099002406f, 0.088763155f}},
                .centroid = {{0.000117883697f, 0.000194840104f, 0.0763867944f}},
                .coefficient_frame = {{1.80222372e-07f, 1.03068473e-07f, 1.26724956e-07f, 1.0f}},
                .volume = 0.001,
            },
            {
                .object = nullptr,
                .pose = {{{-0.000117529999f, -0.000194890003f, -0.00638695993f}}, {{1.80222372e-07f, 1.03068473e-07f, 1.26724956e-07f, 1.0f}}},
                .positions = hydrus_hull_3_positions,
                .position_count = 504,
                .drag = {{1.0f, 0.0166304614f, 0.0814493448f}},
                .skin = {{0.100000001f, 0.00166304619f, 0.00814493466f}},
                .centroid = {{-0.114182115f, -0.000301005668f, 0.066085659f}},
                .coefficient_frame = {{1.80222372e-07f, 1.03068473e-07f, 1.26724956e-07f, 1.0f}},
                .volume = 0.0017951,
            },
            {
                .object = nullptr,
                .pose = {{{-0.000117529999f, -0.000194890003f, -0.00638695993f}}, {{1.80222372e-07f, 1.03068473e-07f, 1.26724956e-07f, 1.0f}}},
                .positions = hydrus_hull_4_positions,
                .position_count = 504,
                .drag = {{1.0f, 0.0166304614f, 0.0814493448f}},
                .skin = {{0.100000001f, 0.00166304619f, 0.00814493466f}},
                .centroid = {{0.113817886f, -0.000301002205f, 0.066085659f}},
                .coefficient_frame = {{1.80222372e-07f, 1.03068473e-07f, 1.26724956e-07f, 1.0f}},
                .volume = 0.0017951,
            },
        },
    .registered = 0,
    .thruster_count = 8,
    .hull_count = 5,
    .sensors = hydrus_sensors,
    .sensor_count = 4
};

#include "robots/proteus_geometry.inc"
constexpr SensorRec proteus_sensors[] = {
    {.id = 1,
     .kind = SIM_SENSOR_POSE,
     .object = nullptr,
     .pose = {{{0, 0, 0}}, {{0, 0, 0, 1}}},
     .enabled = 1,
     .period_ticks = 6,
     .settings = {.tracking = {7, 7}}},
    {.id = 2,
     .kind = SIM_SENSOR_IMU,
     .object = nullptr,
     .pose = {{{0, 0, 0}}, {{0, 0, 0, 1}}},
     .enabled = 1,
     .period_ticks = 1,
     .settings = {}},
    {.id = 3,
     .kind = SIM_SENSOR_PRESSURE,
     .object = nullptr,
     .pose = {{{0, 0, 0}}, {{0, 0, 0, 1}}},
     .enabled = 1,
     .period_ticks = 6,
     .settings = {}},
    {.id = 4,
     .kind = SIM_SENSOR_CAMERA,
     .object = nullptr,
     .pose =
         {{{4.47034836e-08f, 0.300000131f, -1.1920929e-07f}},
          {{0.478205323f, 0.540780306f, 0.457718939f, -0.519008577f}}},
     .enabled = 1,
     .period_ticks = 12,
     .settings = {.camera = {800, 600, 60.000003546768944, .05, 50}}}
};
extern const Robot proteus;
// Physical parameters from the earlier simulator preset. Fixed convex collision/hydrodynamic
// envelope of the saved visual body; not a newly measured vehicle calibration.
constinit const Robot proteus{
    .name = "proteus",
    .asset_file = "robots/proteus.blend",
    .asset_hash = {0xdf, 0xd8, 0xa8, 0xfd, 0xc9, 0x38, 0xc4, 0x91, 0x25, 0xa2, 0x41,
                   0x18, 0xcf, 0x9a, 0x98, 0x7f, 0xb6, 0xed, 0x36, 0xb5, 0x45, 0x62,
                   0x7f, 0xe7, 0xf7, 0x59, 0x5f, 0x8e, 0xe6, 0x9b, 0xe8, 0x7d},
    .object =
        {.id = {},
         .body =
             {
                 .id = 1,
                 .name = "proteus",
                 .cls = 0,
                 .motion_type = SIM_DYNAMIC,
                 .tracked = 0,
                 .fuzz = 0,
                 .mass = 63.83969252,
                 .inertia = {{0.644607186f, 0.675661802f, 1.12846744f}},
                 .center_of_mass =
                     {{{0.0f, 0.0f, -0.00486738002f}},
                      {{0.00135808997f, 0.0100495704f, 0.00110704999f, 0.999947965f}}},
                 .initial_pose = {{{0.0f, 0.0f, 2.0f}}, {{0.0f, 0.0f, 0.0f, 1.0f}}},
                 .initial_velocity = {{0.0f, 0.0f, 0.0f}},
                 .initial_angular_velocity = {{0.0f, 0.0f, 0.0f}},
                 .dry_mass = 4.29466999,
                 .volume = 0.0038491,
                 .buoyancy_center = {{0.0f, 0.0f, -0.00564950006f}},
             },
         .colliders = proteus.colliders,
         .collider_count = 1},
    .colliders =
        {
            {.data =
                 {
                     .kind = SIM_MESH,
                     .collisions = 1,
                     .pose = {{{0.0f, 0.0f, 0.0f}}, {{0.0f, 0.0f, 0.0f, 1.0f}}},
                     .size = {{1.0f, 1.0f, 1.0f}},
                     .drag = {{0.0f, 0.0f, 0.0f}},
                     .skin = {{0.0f, 0.0f, 0.0f}},
                     .friction = 1.0,
                     .restitution = 0.0,
                 },
             .geometry = {proteus_collider_0_positions, 285, proteus_collider_0_triangles, 566},
             .object = nullptr},
        },
    .thrusters =
        {
            {.object = nullptr,
             .position = {{0.205000043f, 0.0f, 0.130000114f}},
             .direction = {{8.97207065e-05f, 1.0f, -1.17843035e-16f}},
             .spec =
                 {.max_rpm = 1000.0,
                  .diameter = 0.18,
                  .thrust = 0.48,
                  .torque = 0.05,
                  .inertia = 0.00019288,
                  .kp = 1.0,
                  .ki = 10.0,
                  .command_sign = -1.0}},
            {.object = nullptr,
             .position = {{-0.205000043f, 0.0f, 0.130000114f}},
             .direction = {{8.97207065e-05f, 1.0f, -1.17843035e-16f}},
             .spec =
                 {.max_rpm = 1000.0,
                  .diameter = 0.18,
                  .thrust = 0.48,
                  .torque = 0.05,
                  .inertia = 0.00019288,
                  .kp = 1.0,
                  .ki = 10.0,
                  .command_sign = -1.0}},
            {.object = nullptr,
             .position = {{0.164999962f, 0.264999986f, 0.0f}},
             .direction = {{0.000203171701f, 0.0f, -1.0f}},
             .spec =
                 {.max_rpm = 1000.0,
                  .diameter = 0.18,
                  .thrust = 0.48,
                  .torque = 0.05,
                  .inertia = 0.00019288,
                  .kp = 1.0,
                  .ki = 10.0,
                  .command_sign = -1.0}},
            {.object = nullptr,
             .position = {{-0.164999962f, 0.264999986f, 0.0f}},
             .direction = {{0.000203171701f, 0.0f, -1.0f}},
             .spec =
                 {.max_rpm = 1000.0,
                  .diameter = 0.18,
                  .thrust = 0.48,
                  .torque = 0.05,
                  .inertia = 0.00019288,
                  .kp = 1.0,
                  .ki = 10.0,
                  .command_sign = -1.0}},
            {.object = nullptr,
             .position = {{0.164999962f, -0.264999986f, 0.0f}},
             .direction = {{0.000203171701f, 0.0f, -1.0f}},
             .spec =
                 {.max_rpm = 1000.0,
                  .diameter = 0.18,
                  .thrust = 0.48,
                  .torque = 0.05,
                  .inertia = 0.00019288,
                  .kp = 1.0,
                  .ki = 10.0,
                  .command_sign = -1.0}},
            {.object = nullptr,
             .position = {{-0.164999962f, -0.264999986f, 0.0f}},
             .direction = {{0.000203171701f, 0.0f, -1.0f}},
             .spec =
                 {.max_rpm = 1000.0,
                  .diameter = 0.18,
                  .thrust = 0.48,
                  .torque = 0.05,
                  .inertia = 0.00019288,
                  .kp = 1.0,
                  .ki = 10.0,
                  .command_sign = -1.0}},
        },
    .hulls =
        {
            {
                .object = nullptr,
                .pose = {{{0.0f, 0.0f, 0.0f}}, {{0.0f, 0.0f, 0.0f, 1.0f}}},
                .positions = proteus_hull_0_positions,
                .position_count = 1698,
                .drag = {{0.5f, 1.0f, 0.5f}},
                .skin = {{0.0500000007f, 0.100000001f, 0.0500000007f}},
                .centroid = {{0.0f, 0.0f, -0.00564950006f}},
                .coefficient_frame = {{0.0f, 0.0f, 0.0f, 1.0f}},
                .volume = 0.0038491,
            },
        },
    .registered = 0,
    .thruster_count = 6,
    .hull_count = 1,
    .sensors = proteus_sensors,
    .sensor_count = 4
};

#include "robots/bluerov_geometry.inc"
constexpr SensorRec bluerov_sensors[] = {
    {.id = 1,
     .kind = SIM_SENSOR_POSE,
     .object = nullptr,
     .pose = {{{0, 0, 0}}, {{0, 0, 0, 1}}},
     .enabled = 1,
     .period_ticks = 6,
     .settings = {.tracking = {7, 7}}},
    {.id = 2,
     .kind = SIM_SENSOR_IMU,
     .object = nullptr,
     .pose = {{{0, 0, 0}}, {{0, 0, 0, 1}}},
     .enabled = 1,
     .period_ticks = 1,
     .settings = {}},
    {.id = 3,
     .kind = SIM_SENSOR_PRESSURE,
     .object = nullptr,
     .pose = {{{0, 0, 0}}, {{0, 0, 0, 1}}},
     .enabled = 1,
     .period_ticks = 6,
     .settings = {}},
    {.id = 4,
     .kind = SIM_SENSOR_CAMERA,
     .object = nullptr,
     .pose =
         {{{0.0724999979f, 0.159999996f, 0.150000095f}},
          {{-0.500050843f, 0.500050902f, 0.499949098f, 0.499949157f}}},
     .enabled = 1,
     .period_ticks = 12,
     .settings = {.camera = {800, 600, 74.99999836640009, .05, 50}}}
};
extern const Robot bluerov;
// Physical parameters from the earlier simulator preset. Fixed convex collision/hydrodynamic
// envelope of the saved visual body; not a newly measured vehicle calibration.
constinit const Robot bluerov{
    .name = "bluerov",
    .asset_file = "robots/bluerov.blend",
    .asset_hash = {0xf8, 0x87, 0x5d, 0xd0, 0xc8, 0x2d, 0xae, 0x5f, 0xc7, 0x20, 0xc0,
                   0x60, 0x54, 0x6f, 0xe9, 0x8b, 0xb5, 0x73, 0xfc, 0xb5, 0x38, 0xab,
                   0x74, 0x6d, 0xfd, 0xeb, 0x55, 0x24, 0x81, 0xef, 0x40, 0xda},
    .object =
        {.id = {},
         .body =
             {
                 .id = 1,
                 .name = "bluerov",
                 .cls = 0,
                 .motion_type = SIM_DYNAMIC,
                 .tracked = 0,
                 .fuzz = 0,
                 .mass = 163.78488689,
                 .inertia = {{0.481810004f, 0.601817787f, 3.4223485f}},
                 .center_of_mass =
                     {{{9.24499982e-05f, -0.00547896978f, 0.0769923925f}},
                      {{-0.700413406f, -0.0966697931f, -0.0966806784f, 0.700520456f}}},
                 .initial_pose = {{{0.0f, 0.0f, 2.0f}}, {{0.0f, 0.0f, 0.0f, 1.0f}}},
                 .initial_velocity = {{0.0f, 0.0f, 0.0f}},
                 .initial_angular_velocity = {{0.0f, 0.0f, 0.0f}},
                 .dry_mass = 11.18880611,
                 .volume = 0.01092,
                 .buoyancy_center = {{-0.00499999989f, 0.0f, 0.0f}},
             },
         .colliders = bluerov.colliders,
         .collider_count = 1},
    .colliders =
        {
            {.data =
                 {
                     .kind = SIM_MESH,
                     .collisions = 1,
                     .pose = {{{0.0f, 0.0f, 0.0f}}, {{0.0f, 0.0f, 0.0f, 1.0f}}},
                     .size = {{1.0f, 1.0f, 1.0f}},
                     .drag = {{0.0f, 0.0f, 0.0f}},
                     .skin = {{0.0f, 0.0f, 0.0f}},
                     .friction = 1.0,
                     .restitution = 0.0,
                 },
             .geometry = {bluerov_collider_0_positions, 422, bluerov_collider_0_triangles, 838},
             .object = nullptr},
        },
    .thrusters =
        {
            {.object = nullptr,
             .position = {{-0.100000024f, 0.135500014f, 0.0724999905f}},
             .direction = {{-0.707106709f, -0.707106829f, -1.50995788e-07f}},
             .spec =
                 {.max_rpm = 4000.0,
                  .diameter = 0.076,
                  .thrust = 0.334,
                  .torque = 0.032,
                  .inertia = 0.2,
                  .kp = 1.0,
                  .ki = 0.0,
                  .command_sign = -1.0}},
            {.object = nullptr,
             .position = {{0.100000024f, 0.135500014f, 0.0724999905f}},
             .direction = {{0.707106829f, -0.707106709f, 2.92992339e-15f}},
             .spec =
                 {.max_rpm = 4000.0,
                  .diameter = 0.076,
                  .thrust = 0.334,
                  .torque = 0.032,
                  .inertia = 0.2,
                  .kp = 1.0,
                  .ki = 0.0,
                  .command_sign = -1.0}},
            {.object = nullptr,
             .position = {{-0.100000024f, -0.147500038f, 0.0724999905f}},
             .direction = {{-0.707106709f, 0.707106829f, 1.50995788e-07f}},
             .spec =
                 {.max_rpm = 4000.0,
                  .diameter = 0.076,
                  .thrust = 0.334,
                  .torque = 0.032,
                  .inertia = 0.2,
                  .kp = 1.0,
                  .ki = 0.0,
                  .command_sign = -1.0}},
            {.object = nullptr,
             .position = {{0.100000024f, -0.147500038f, 0.0724999905f}},
             .direction = {{0.707106829f, 0.707106709f, -2.92992339e-15f}},
             .spec =
                 {.max_rpm = 4000.0,
                  .diameter = 0.076,
                  .thrust = 0.334,
                  .torque = 0.032,
                  .inertia = 0.2,
                  .kp = 1.0,
                  .ki = 0.0,
                  .command_sign = -1.0}},
            {.object = nullptr,
             .position = {{-0.218000054f, 0.120000005f, 0.0f}},
             .direction = {{-1.34358856e-07f, 0.0f, 1.0f}},
             .spec =
                 {.max_rpm = 4000.0,
                  .diameter = 0.076,
                  .thrust = 0.334,
                  .torque = 0.032,
                  .inertia = 0.2,
                  .kp = 1.0,
                  .ki = 0.0,
                  .command_sign = -1.0}},
            {.object = nullptr,
             .position = {{0.218000054f, 0.120000005f, 0.0f}},
             .direction = {{-1.34358856e-07f, 0.0f, 1.0f}},
             .spec =
                 {.max_rpm = 4000.0,
                  .diameter = 0.076,
                  .thrust = 0.334,
                  .torque = 0.032,
                  .inertia = 0.2,
                  .kp = 1.0,
                  .ki = 0.0,
                  .command_sign = -1.0}},
            {.object = nullptr,
             .position = {{-0.218000054f, -0.120000005f, 0.0f}},
             .direction = {{-1.34358856e-07f, 0.0f, 1.0f}},
             .spec =
                 {.max_rpm = 4000.0,
                  .diameter = 0.076,
                  .thrust = 0.334,
                  .torque = 0.032,
                  .inertia = 0.2,
                  .kp = 1.0,
                  .ki = 0.0,
                  .command_sign = -1.0}},
            {.object = nullptr,
             .position = {{0.218000054f, -0.120000005f, 0.0f}},
             .direction = {{-1.34358856e-07f, 0.0f, 1.0f}},
             .spec =
                 {.max_rpm = 4000.0,
                  .diameter = 0.076,
                  .thrust = 0.334,
                  .torque = 0.032,
                  .inertia = 0.2,
                  .kp = 1.0,
                  .ki = 0.0,
                  .command_sign = -1.0}},
        },
    .hulls =
        {
            {
                .object = nullptr,
                .pose = {{{0.0f, 0.0f, 0.0f}}, {{0.0f, 0.0f, 0.0f, 1.0f}}},
                .positions = bluerov_hull_0_positions,
                .position_count = 2514,
                .drag = {{0.544357002f, 0.713886023f, 0.99752301f}},
                .skin = {{0.0544357002f, 0.0713886023f, 0.0997522995f}},
                .centroid = {{-0.00499999989f, 0.0f, 0.0f}},
                .coefficient_frame = {{0.0f, 0.0f, 0.0f, 1.0f}},
                .volume = 0.01092,
            },
        },
    .registered = 0,
    .thruster_count = 8,
    .hull_count = 1,
    .sensors = bluerov_sensors,
    .sensor_count = 4
};

#include "robots/girona500_geometry.inc"
constexpr SensorRec girona500_sensors[] = {
    {.id = 1,
     .kind = SIM_SENSOR_POSE,
     .object = nullptr,
     .pose = {{{0, 0, 0}}, {{0, 0, 0, 1}}},
     .enabled = 1,
     .period_ticks = 6,
     .settings = {.tracking = {7, 7}}},
    {.id = 2,
     .kind = SIM_SENSOR_IMU,
     .object = nullptr,
     .pose = {{{0, 0, 0}}, {{0, 0, 0, 1}}},
     .enabled = 1,
     .period_ticks = 1,
     .settings = {}},
    {.id = 3,
     .kind = SIM_SENSOR_PRESSURE,
     .object = nullptr,
     .pose = {{{0, 0, 0}}, {{0, 0, 0, 1}}},
     .enabled = 1,
     .period_ticks = 6,
     .settings = {}},
    {.id = 4,
     .kind = SIM_SENSOR_CAMERA,
     .object = nullptr,
     .pose =
         {{{0.5f, -0.119999997f, 0.5f}}, {{0.499949157f, 0.500050902f, 0.499949098f, -0.500050843f}}
         },
     .enabled = 1,
     .period_ticks = 12,
     .settings = {.camera = {800, 600, 60.000003546768944, .05, 50}}}
};
extern const Robot girona500;
// Mass, augmented inertia, COM and displacement extracted from the pinned Stonefish Girona500
// model. Fixed convex envelope; default portable rotor inertia, not a newly measured vehicle
// calibration.
constinit const Robot girona500{
    .name = "girona500",
    .asset_file = "robots/girona500.blend",
    .asset_hash = {0xa4, 0x14, 0x0d, 0x55, 0x00, 0x1e, 0x48, 0x0c, 0xb4, 0x39, 0xc8,
                   0xea, 0xb1, 0xe8, 0x05, 0xcf, 0x3a, 0x18, 0xb7, 0xb7, 0x44, 0x8a,
                   0x5e, 0x08, 0xc0, 0xa7, 0xe5, 0x9b, 0x34, 0x9e, 0x3a, 0x51},
    .object =
        {.id = {},
         .body =
             {
                 .id = 1,
                 .name = "girona500",
                 .cls = 0,
                 .motion_type = SIM_DYNAMIC,
                 .tracked = 0,
                 .fuzz = 0,
                 .mass = 865.8697498294008,
                 .inertia = {{97.2002792f, 129.709595f, 138.285263f}},
                 .center_of_mass =
                     {{{-0.0732347518f, 0.000266489282f, 0.0543086417f}},
                      {{-0.701886415f, 0.0899146572f, 0.0899442285f, 0.700842977f}}},
                 .initial_pose = {{{0.0f, 0.0f, 2.0f}}, {{0.0f, 0.0f, 0.0f, 1.0f}}},
                 .initial_velocity = {{0.0f, 0.0f, 0.0f}},
                 .initial_angular_velocity = {{0.0f, 0.0f, 0.0f}},
                 .dry_mass = 143.14603282315827,
                 .volume = 0.14706076542512625,
                 .buoyancy_center = {{-0.0653123483f, 0.000258962973f, -0.174743235f}},
             },
         .colliders = girona500.colliders,
         .collider_count = 1},
    .colliders =
        {
            {.data =
                 {
                     .kind = SIM_MESH,
                     .collisions = 1,
                     .pose = {{{0.0f, 0.0f, 0.0f}}, {{0.0f, 0.0f, 0.0f, 1.0f}}},
                     .size = {{1.0f, 1.0f, 1.0f}},
                     .drag = {{0.0f, 0.0f, 0.0f}},
                     .skin = {{0.0f, 0.0f, 0.0f}},
                     .friction = 1.0,
                     .restitution = 0.0,
                 },
             .geometry =
                 {girona500_collider_0_positions, 574, girona500_collider_0_triangles, 1144},
             .object = nullptr},
        },
    .thrusters =
        {
            {.object = nullptr,
             .position = {{-0.329699993f, -0.258700132f, -0.0210000277f}},
             .direction = {{-1.0f, 1.50995803e-07f, 2.12622737e-20f}},
             .spec =
                 {.max_rpm = 1000.0,
                  .diameter = 0.18,
                  .thrust = 0.48,
                  .torque = 0.05,
                  .inertia = 0.00146468,
                  .kp = 1.0,
                  .ki = 0.5,
                  .command_sign = -1.0}},
            {.object = nullptr,
             .position = {{-0.329699993f, 0.258699894f, -0.0210000277f}},
             .direction = {{-1.0f, 0.0f, -1.50995803e-07f}},
             .spec =
                 {.max_rpm = 1000.0,
                  .diameter = 0.18,
                  .thrust = 0.48,
                  .torque = 0.05,
                  .inertia = 0.00146468,
                  .kp = 1.0,
                  .ki = 0.5,
                  .command_sign = -1.0}},
            {.object = nullptr,
             .position = {{-0.0627000332f, 0.0306999683f, -0.0210000277f}},
             .direction = {{-0.000203292497f, 1.0f, -7.06513583e-06f}},
             .spec =
                 {.max_rpm = 1000.0,
                  .diameter = 0.18,
                  .thrust = 0.48,
                  .torque = 0.05,
                  .inertia = 0.00146468,
                  .kp = 1.0,
                  .ki = 0.5,
                  .command_sign = -1.0}},
            {.object = nullptr,
             .position = {{-0.582700014f, 0.0f, -0.313699961f}},
             .direction = {{-1.78260962e-08f, 0.000203609467f, -1.0f}},
             .spec =
                 {.max_rpm = 1000.0,
                  .diameter = 0.18,
                  .thrust = 0.48,
                  .torque = 0.05,
                  .inertia = 0.00146468,
                  .kp = 1.0,
                  .ki = 0.5,
                  .command_sign = -1.0}},
            {.object = nullptr,
             .position = {{0.534699857f, 0.0f, -0.313699961f}},
             .direction = {{-6.5823464e-08f, -0.000203818054f, -1.0f}},
             .spec =
                 {.max_rpm = 1000.0,
                  .diameter = 0.18,
                  .thrust = 0.48,
                  .torque = 0.05,
                  .inertia = 0.00146468,
                  .kp = 1.0,
                  .ki = 0.5,
                  .command_sign = -1.0}},
        },
    .hulls =
        {
            {
                .object = nullptr,
                .pose = {{{0.0f, 0.0f, 0.0f}}, {{0.0f, 0.0f, 0.0f, 1.0f}}},
                .positions = girona500_hull_0_positions,
                .position_count = 3432,
                .drag = {{1.0f, 1.0f, 1.0f}},
                .skin = {{0.00999999978f, 0.00999999978f, 0.00999999978f}},
                .centroid = {{-0.0653123483f, 0.000258962973f, -0.174743235f}},
                .coefficient_frame = {{0.0f, 0.0f, 0.0f, 1.0f}},
                .volume = 0.14706076542512625,
            },
        },
    .registered = 0,
    .thruster_count = 5,
    .hull_count = 1,
    .sensors = girona500_sensors,
    .sensor_count = 4
};

const Robot *const definitions[] = {&hydrus, &proteus, &bluerov, &girona500};
} // namespace

extern "C" {
uint32_t sim_robot_count(void) {
    return uint32_t(std::size(definitions));
}

const Robot *sim_robot_at(uint32_t i) {
    return i < sim_robot_count() ? definitions[i] : nullptr;
}

const Robot *sim_robot_find(const char *name) {
    if (name)
        for (const auto *r : definitions)
            if (std::strcmp(r->name, name) == 0)
                return r;
    return nullptr;
}

SimResult simulation_robot_config(const Robot *robot, SimulationConfig *out, char *e, uint32_t n) {
    return core::guard(e, n, [&] {
        core::require(
            out && std::find(std::begin(definitions), std::end(definitions), robot) !=
                       std::end(definitions),
            "unknown robot template"
        );
        SimulationConfig c;
        simulation_config_default(&c);
        uint32_t positions = 0, triangles = 0, hull_positions = 0;
        for (uint32_t i = 0; i < robot->object.collider_count; ++i) {
            positions += robot->colliders[i].geometry.position_count;
            triangles += robot->colliders[i].geometry.triangle_count;
        }
        for (uint32_t i = 0; i < robot->hull_count; ++i)
            hull_positions += robot->hulls[i].position_count;
        c.vertex_capacity = std::max(c.vertex_capacity, positions);
        c.triangle_capacity = std::max(c.triangle_capacity, triangles);
        c.hull_vertex_capacity = std::max(c.hull_vertex_capacity, hull_positions);
        *out = c;
    });
}
} // extern "C"
