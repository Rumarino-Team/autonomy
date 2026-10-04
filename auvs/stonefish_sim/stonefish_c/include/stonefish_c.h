#ifndef STONEFISH_C_H
#define STONEFISH_C_H

/* Standard-layout scene ABI shared by C++ and Python ctypes.
   Stonefish classes (btTransform, std::string, Box, ...) stay inside
   stonefish_c.cpp. Do not add constructors or virtual functions here. */

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__GNUC__)
#define SF_API __attribute__((visibility("default")))
#else
#define SF_API
#endif

enum SfBodyPhysics
{
    SF_BODY_SURFACE = 0,
    SF_BODY_FLOATING = 1,
    SF_BODY_SUBMERGED = 2
};

typedef struct SfPose
{
    double xyz[3];
    double rpy[3];
} SfPose;

typedef struct SfMaterial
{
    const char* name;
    double density;
    double restitution;
    double magnetic;
} SfMaterial;

typedef struct SfLook
{
    const char* name;
    double rgb[3];
    double roughness;
    double metalness;
    double reflectivity;
    const char* texture;
    const char* normal_map;
} SfLook;

typedef struct SfFriction
{
    const char* material1;
    const char* material2;
    double static_friction;
    double dynamic_friction;
} SfFriction;

typedef struct SfEnvironment
{
    double ned_latitude;
    double ned_longitude;
    double water_density;
    double jerlov;
    double wave_height;
    double water_temperature;
    double current_xyz[3];
    double sun_azimuth_deg;
    double sun_elevation_deg;
    int particles;
    int pad0;
} SfEnvironment;

typedef struct SfStatic
{
    const char* name;
    const char* material;
    const char* look;
    const char* cls;
    SfPose world;
    SfPose origin;
} SfStatic;

typedef struct SfMesh
{
    const char* physics_path;
    double physics_scale;
    SfPose physics_origin;
    const char* visual_path;
    double visual_scale;
    SfPose visual_origin;
    int convex;
    int pad0;
} SfMesh;

typedef struct SfRobot
{
    const char* name;
    const char* base_link;
    int fixed;
    int self_collisions;
    int physics;
    int buoyant;
    SfPose world;
} SfRobot;

typedef struct SfPart
{
    const char* name;
    const char* material;
    const char* look;
    int physics;
    int buoyant;
    double thickness;
    SfPose origin;
    SfPose compound;
    double mass;
    int internal;
    int pad1;
} SfPart;

typedef struct SfPartBox
{
    SfPart part;
    double dimensions[3];
} SfPartBox;

typedef struct SfPartCylinder
{
    SfPart part;
    double radius;
    double height;
} SfPartCylinder;

typedef struct SfPartMesh
{
    SfPart part;
    const char* path;
    double scale;
    const char* visual_path;
    double visual_scale;
    SfPose visual_origin;
} SfPartMesh;

typedef struct SfPartSphere
{
    SfPart part;
    double radius;
} SfPartSphere;

typedef struct SfThruster
{
    const char* name;
    const char* link;
    SfPose origin;
    double diameter;
    double max_setpoint;
    int right_handed;
    int inverted_setpoint;
    int normalized_setpoint;
    int pad0;
    const char* propeller_mesh;
    double propeller_scale;
    const char* propeller_material;
    const char* propeller_look;
    double kp;
    double ki;
    double ilimit;
    double thrust_forward;
    double thrust_reverse;
    double torque_coeff;
    double time_constant;
} SfThruster;

typedef struct SfSensor
{
    const char* name;
    const char* link;
    SfPose origin;
    double rate;
    int history;
    int pad0;
} SfSensor;

typedef struct SfImu
{
    SfSensor sensor;
    double angular_velocity_range[3];
    double linear_acceleration_range;
    double angle_noise[3];
    double angular_velocity_noise;
    double yaw_drift;
    double linear_acceleration_noise;
} SfImu;

typedef struct SfCamera
{
    SfSensor sensor;
    int resolution_x;
    int resolution_y;
    double horizontal_fov_deg;
} SfCamera;

typedef struct SfWorld SfWorld;

/* simulation_manager is the Stonefish SimulationManager already owned by the sim.
   sf_world_free releases the wrapper only. */
SF_API SfWorld* sf_world_bind(void* simulation_manager);
SF_API void sf_world_free(SfWorld* world);
SF_API void sf_world_set_data_dir(SfWorld* world, const char* data_dir);

SF_API int sf_material(SfWorld* world, const SfMaterial* material);
SF_API int sf_look(SfWorld* world, const SfLook* look);
SF_API int sf_friction(SfWorld* world, const SfFriction* friction);
SF_API int sf_environment(SfWorld* world, const SfEnvironment* environment);

SF_API int sf_static_plane(SfWorld* world, const SfStatic* body, double uv_scale);
SF_API int sf_static_box(SfWorld* world, const SfStatic* body, const double dimensions[3]);
SF_API int sf_static_cylinder(SfWorld* world, const SfStatic* body, double radius, double height);
SF_API int sf_static_sphere(SfWorld* world, const SfStatic* body, double radius);
SF_API int sf_static_mesh(SfWorld* world, const SfStatic* body, const SfMesh* mesh);

/* Parts, then thrusters and sensors, then sf_robot_end. */
SF_API int sf_robot_begin(SfWorld* world, const SfRobot* robot);
SF_API int sf_robot_part_box(SfWorld* world, const SfPartBox* part);
SF_API int sf_robot_part_cylinder(SfWorld* world, const SfPartCylinder* part);
SF_API int sf_robot_part_mesh(SfWorld* world, const SfPartMesh* part);
SF_API int sf_robot_part_sphere(SfWorld* world, const SfPartSphere* part);
SF_API int sf_robot_thruster(SfWorld* world, const SfThruster* thruster);
SF_API int sf_robot_odometry(SfWorld* world, const SfSensor* sensor);
SF_API int sf_robot_imu(SfWorld* world, const SfImu* imu);
SF_API int sf_robot_camera(SfWorld* world, const SfCamera* camera);
SF_API int sf_robot_end(SfWorld* world);

SF_API int sf_world_class_count(const SfWorld* world);
SF_API int sf_world_class_at(const SfWorld* world, int index, char* name, int name_cap, char* cls, int cls_cap);

#ifdef __cplusplus
}
#endif

#endif
