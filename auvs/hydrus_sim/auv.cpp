#include <Python.h>

#include <cmath>
#include <cstdio>
#include <math.h>

#include "../../include/auv.h"
#include "../../stonefish_c/include/stonefish_c.h"

#include "Stonefish/actuators/Actuator.h"
#include "Stonefish/actuators/Thruster.h"
#include "Stonefish/core/ConsoleSimulationApp.h"
#include "Stonefish/core/GraphicalSimulationApp.h"
#include "Stonefish/core/Robot.h"
#include "Stonefish/core/SimulationManager.h"
#include "Stonefish/entities/Entity.h"
#include "Stonefish/entities/MovingEntity.h"
#include "Stonefish/entities/StaticEntity.h"
#include "Stonefish/graphics/OpenGLDataStructs.h"
#include "Stonefish/graphics/OpenGLPipeline.h"
#include "Stonefish/sensors/Sensor.h"
#include "Stonefish/sensors/ScalarSensor.h"
#include "Stonefish/sensors/vision/Camera.h"


#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <dlfcn.h>
#include <exception>
#include <filesystem>
#include <mutex>
#include <optional>
#include <print>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <nlohmann/json.hpp>
#include <fstream>

#ifndef AUV_PLATFORM_CONFIG
#define AUV_PLATFORM_CONFIG "auvs/hydrus_sim/platform.json"
#endif
static constexpr const char* kPlatformConfigPath = AUV_PLATFORM_CONFIG;

volatile std::sig_atomic_t g_stop_requested = 0;

void requestStop(int)
{
    g_stop_requested = 1;
}

void stopIfRequested()
{
    if(g_stop_requested == 0)
        return;
    std::println(stderr, "[hydrus_sim] interrupted; shutting down");
    auv_deinit();
    std::exit(130);
}

struct PlatformConfig {
    std::filesystem::path dataPath;
    std::string blendFile;
    std::string robot;
    sf::Scalar kPhysicsDt;
    sf::Scalar kStepsPerSecond;
    sf::Scalar kRealtimeFactorCap;
    sf::Scalar kTrackingOkSeconds;
    std::chrono::milliseconds kMinRenderInterval;
    bool kConsoleApp = false;
    bool kBBoxOnlyInFrontOfCamera = false;
};

std::optional<PlatformConfig> loadPlatformConfig(const std::filesystem::path& path){
    std::ifstream json_file(path);
    if(!json_file){
        return std::nullopt;
    }

    const nlohmann::json parsed_json = nlohmann::json::parse(json_file);

    PlatformConfig config{
        .dataPath = parsed_json.at("data_path").get<std::string>(),
        .blendFile = parsed_json.at("blend_file").get<std::string>(),
        .robot = parsed_json.contains("robot") ? parsed_json.at("robot").get<std::string>() : "hydrus",
        .kStepsPerSecond = parsed_json.at("steps_per_second").get<double>(),
        .kRealtimeFactorCap = parsed_json.at("realtime_factor_cap").get<double>(),
        .kTrackingOkSeconds = parsed_json.at("tracking_ok_seconds").get<double>(),
        .kMinRenderInterval = std::chrono::milliseconds{
            parsed_json.at("min_render_interval_ms").get<int>()},
        .kConsoleApp = parsed_json.at("console").get<bool>(),
        .kBBoxOnlyInFrontOfCamera = parsed_json.at("bbox_only_in_front_of_camera").get<bool>(),
    };


    if(config.dataPath.empty()){
        std::println(stderr, "[Parser] data_path is empty in {}", path.string());
        return std::nullopt;
    }
    if(config.blendFile.empty()){
        std::println(stderr, "[Parser] blend_file is required in {}", path.string());
        return std::nullopt;
    }
    if(!std::filesystem::is_directory(config.dataPath)){
        std::println(stderr, "[Parser] data_path is not a directory: {}", config.dataPath.string());
        return std::nullopt;
    }
    const std::filesystem::path blend_path = config.dataPath / config.blendFile;
    if(!std::filesystem::is_regular_file(blend_path)){
        std::println(stderr, "[Parser] blend file not found: {}", blend_path.string());
        return std::nullopt;
    }
    if(config.kStepsPerSecond <= sf::Scalar(0)){
        std::println(stderr, "[Parser] steps_per_second must be > 0 in {}", path.string());
        return std::nullopt;
    }
    if(config.kRealtimeFactorCap < sf::Scalar(0)){
        std::println(stderr, "[Parser] realtime_factor_cap must be >= 0 in {}", path.string());
        return std::nullopt;
    }
    if(config.kTrackingOkSeconds < sf::Scalar(0)){
        std::println(stderr, "[Parser] tracking_ok_seconds must be >= 0 in {}", path.string());
        return std::nullopt;
    }
    if(config.kMinRenderInterval.count() < 0){
        std::println(stderr, "[Parser] min_render_interval_ms must be >= 0 in {}", path.string());
        return std::nullopt;
    }

    config.kPhysicsDt = sf::Scalar(1) / config.kStepsPerSecond;
    return config;
}


namespace
{
enum class ObjectCls : AuvObjectCls
{
    cube = 0,
    rect = 1,
    gate = 2,
};

std::optional<ObjectCls> ParseObjectClass(const char* text)
{
    if(text == nullptr || *text == '\0')
        return std::nullopt;

    std::string cls{text};
    for(char& c : cls)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

    if(cls == "cube" || cls == "0")
        return ObjectCls::cube;
    if(cls == "rect" || cls == "rectangle" || cls == "1")
        return ObjectCls::rect;
    if(cls == "gate" || cls == "2")
        return ObjectCls::gate;
    return std::nullopt;
}

MathPose ToMathPose(const sf::Transform& transform)
{
    const sf::Vector3 pos = transform.getOrigin();
    const sf::Quaternion quat = transform.getRotation();
    MathPose pose{};
    pose.pos.buf[0] = static_cast<float>(pos.x());
    pose.pos.buf[1] = static_cast<float>(pos.y());
    pose.pos.buf[2] = static_cast<float>(pos.z());
    pose.quat.buf[0] = static_cast<float>(quat.x());
    pose.quat.buf[1] = static_cast<float>(quat.y());
    pose.quat.buf[2] = static_cast<float>(quat.z());
    pose.quat.buf[3] = static_cast<float>(quat.w());
    return pose;
}

MathPose IdentityPose()
{
    MathPose pose{};
    pose.quat.buf[3] = 1.f;
    return pose;
}

sf::Transform EntityWorldTransform(sf::Entity* entity)
{
    switch(entity->getType())
    {
        case sf::EntityType::STATIC:
            return static_cast<sf::StaticEntity*>(entity)->getTransform();
        case sf::EntityType::SOLID:
        case sf::EntityType::ANIMATED:
            return static_cast<sf::MovingEntity*>(entity)->getOTransform();
        default:
            return sf::I4();
    }
}

bool FillObjectBBox(sf::Entity* entity, MathBoundingBox& bbox)
{
    sf::Vector3 min;
    sf::Vector3 max;
    entity->getAABB(min, max);
    const sf::Vector3 size = max - min;
    if(size.x() <= 0 || size.y() <= 0 || size.z() <= 0)
        return false;

    const sf::Vector3 center = (min + max) * sf::Scalar(0.5);
    bbox = {};
    bbox.pose = ToMathPose(EntityWorldTransform(entity));
    bbox.pose.pos.buf[0] = static_cast<float>(center.x());
    bbox.pose.pos.buf[1] = static_cast<float>(center.y());
    bbox.pose.pos.buf[2] = static_cast<float>(center.z());
    bbox.size.buf[0] = static_cast<float>(size.x());
    bbox.size.buf[1] = static_cast<float>(size.y());
    bbox.size.buf[2] = static_cast<float>(size.z());
    return true;
}

// Stonefish camera frame: +X right, +Y down, +Z along the optical axis.
bool IsInFrontOfCamera(sf::Camera* camera, sf::Entity* entity)
{
    if(camera == nullptr || entity == nullptr)
        return false;

    sf::Vector3 min;
    sf::Vector3 max;
    entity->getAABB(min, max);
    const sf::Transform world_to_camera = camera->getSensorFrame().inverse();

    for(int corner = 0; corner < 8; ++corner)
    {
        const sf::Vector3 world(
            (corner & 1) != 0 ? max.x() : min.x(),
            (corner & 2) != 0 ? max.y() : min.y(),
            (corner & 4) != 0 ? max.z() : min.z());
        const sf::Vector3 local = world_to_camera * world;
        if(local.z() > sf::Scalar(0.05))
            return true;
    }
    return false;
}

bool ProjectObjectBox(
    sf::Camera* camera,
    unsigned int image_width,
    unsigned int image_height,
    float focal_px,
    sf::Entity* entity,
    AuvObject2d& box)
{
    if(camera == nullptr || image_width == 0 || image_height == 0 || focal_px <= 0.f)
        return false;

    sf::Vector3 min;
    sf::Vector3 max;
    entity->getAABB(min, max);
    const sf::Transform world_to_camera = camera->getSensorFrame().inverse();
    const float cx = static_cast<float>(image_width) * 0.5f;
    const float cy = static_cast<float>(image_height) * 0.5f;

    float u_min = 1.0e9f;
    float v_min = 1.0e9f;
    float u_max = -1.0e9f;
    float v_max = -1.0e9f;
    bool any_in_front = false;

    for(int corner = 0; corner < 8; ++corner)
    {
        const sf::Vector3 world(
            (corner & 1) != 0 ? max.x() : min.x(),
            (corner & 2) != 0 ? max.y() : min.y(),
            (corner & 4) != 0 ? max.z() : min.z());
        const sf::Vector3 local = world_to_camera * world;
        if(local.z() <= sf::Scalar(0.05))
            continue;

        const float u = cx + focal_px * static_cast<float>(local.x() / local.z());
        const float v = cy + focal_px * static_cast<float>(local.y() / local.z());
        u_min = std::min(u_min, u);
        v_min = std::min(v_min, v);
        u_max = std::max(u_max, u);
        v_max = std::max(v_max, v);
        any_in_front = true;
    }

    if(!any_in_front)
        return false;
    if(u_max < 0.f || v_max < 0.f
       || u_min >= static_cast<float>(image_width)
       || v_min >= static_cast<float>(image_height))
        return false;

    const auto clamp_pixel = [](float value, unsigned int limit) -> uint32_t {
        if(value <= 0.f)
            return 0;
        const float last = static_cast<float>(limit - 1);
        if(value >= last)
            return limit - 1;
        return static_cast<uint32_t>(value);
    };

    box.top_left.x = clamp_pixel(u_min, image_width);
    box.top_left.y = clamp_pixel(v_min, image_height);
    box.bottom_right.x = clamp_pixel(u_max, image_width);
    box.bottom_right.y = clamp_pixel(v_max, image_height);
    return box.bottom_right.x > box.top_left.x && box.bottom_right.y > box.top_left.y;
}

#ifndef BLENDER_STONEFISH_ROOT
#define BLENDER_STONEFISH_ROOT "."
#endif
#ifndef BLENDER_STONEFISH_CONFIG
#define BLENDER_STONEFISH_CONFIG "blender_stonefish/config.yaml"
#endif

SfPose PoseAt(double x, double y, double z, double roll, double pitch, double yaw)
{
    SfPose pose{};
    pose.xyz[0] = x;
    pose.xyz[1] = y;
    pose.xyz[2] = z;
    pose.rpy[0] = roll;
    pose.rpy[1] = pitch;
    pose.rpy[2] = yaw;
    return pose;
}

SfPart SubmergedPart(const char* name, const char* material, const char* look, double thickness, SfPose origin, SfPose compound)
{
    SfPart part{};
    part.name = name;
    part.material = material;
    part.look = look;
    part.physics = SF_BODY_SUBMERGED;
    part.buoyant = 1;
    part.thickness = thickness;
    part.origin = origin;
    part.compound = compound;
    return part;
}

SfPart BodyPart(
    const char* name,
    const char* material,
    const char* look,
    double thickness,
    double mass,
    int buoyant,
    int internal,
    SfPose origin,
    SfPose compound)
{
    SfPart part = SubmergedPart(name, material, look, thickness, origin, compound);
    part.mass = mass;
    part.buoyant = buoyant;
    part.internal = internal;
    return part;
}

bool CallOk(int status, const char* what);

bool AddMaterial(SfWorld* world, const char* name, double density, double restitution)
{
    const SfMaterial material{name, density, restitution, 0.0};
    return CallOk(sf_material(world, &material), name);
}

bool AddLook(
    SfWorld* world,
    const char* name,
    double red,
    double green,
    double blue,
    double roughness,
    double metalness,
    double reflectivity,
    const char* texture)
{
    const SfLook look{name, {red, green, blue}, roughness, metalness, reflectivity, texture, nullptr};
    return CallOk(sf_look(world, &look), name);
}

bool AddThruster(
    SfWorld* world,
    const char* name,
    const char* link,
    SfPose origin,
    double diameter,
    double max_setpoint,
    int right_handed,
    int inverted,
    const char* propeller,
    double propeller_scale,
    const char* propeller_material,
    const char* propeller_look,
    double kp,
    double ki,
    double ilimit,
    double thrust_forward,
    double thrust_reverse,
    double torque,
    double time_constant)
{
    SfThruster thruster{};
    thruster.name = name;
    thruster.link = link;
    thruster.origin = origin;
    thruster.diameter = diameter;
    thruster.max_setpoint = max_setpoint;
    thruster.right_handed = right_handed;
    thruster.inverted_setpoint = inverted;
    thruster.normalized_setpoint = 1;
    thruster.propeller_mesh = propeller;
    thruster.propeller_scale = propeller_scale;
    thruster.propeller_material = propeller_material;
    thruster.propeller_look = propeller_look;
    thruster.kp = kp;
    thruster.ki = ki;
    thruster.ilimit = ilimit;
    thruster.thrust_forward = thrust_forward;
    thruster.thrust_reverse = thrust_reverse;
    thruster.torque_coeff = torque;
    thruster.time_constant = time_constant;
    return CallOk(sf_robot_thruster(world, &thruster), name);
}

bool AddBox(
    SfWorld* world,
    const char* name,
    const char* material,
    const char* look,
    double x,
    double y,
    double z,
    double mass,
    int buoyant,
    int internal,
    SfPose compound)
{
    SfPartBox box{};
    box.part = BodyPart(name, material, look, -1.0, mass, buoyant, internal, PoseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), compound);
    box.dimensions[0] = x;
    box.dimensions[1] = y;
    box.dimensions[2] = z;
    return CallOk(sf_robot_part_box(world, &box), name);
}

bool AddCylinder(
    SfWorld* world,
    const char* name,
    const char* material,
    const char* look,
    double radius,
    double height,
    double thickness,
    double mass,
    int buoyant,
    int internal,
    SfPose compound)
{
    SfPartCylinder cylinder{};
    cylinder.part = BodyPart(
        name, material, look, thickness, mass, buoyant, internal, PoseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), compound);
    cylinder.radius = radius;
    cylinder.height = height;
    return CallOk(sf_robot_part_cylinder(world, &cylinder), name);
}

bool AddSphere(
    SfWorld* world,
    const char* name,
    const char* material,
    const char* look,
    double radius,
    double mass,
    int buoyant,
    int internal,
    SfPose compound)
{
    SfPartSphere sphere{};
    sphere.part = BodyPart(
        name, material, look, -1.0, mass, buoyant, internal, PoseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), compound);
    sphere.radius = radius;
    return CallOk(sf_robot_part_sphere(world, &sphere), name);
}

bool AddMesh(
    SfWorld* world,
    const char* name,
    const char* material,
    const char* look,
    const char* physics,
    double physics_scale,
    SfPose physics_origin,
    const char* visual,
    double visual_scale,
    SfPose visual_origin,
    double thickness,
    int buoyant,
    SfPose compound)
{
    SfPartMesh mesh{};
    mesh.part = BodyPart(name, material, look, thickness, 0.0, buoyant, 0, physics_origin, compound);
    mesh.path = physics;
    mesh.scale = physics_scale;
    mesh.visual_path = visual;
    mesh.visual_scale = visual_scale;
    mesh.visual_origin = visual_origin;
    return CallOk(sf_robot_part_mesh(world, &mesh), name);
}

bool CallOk(int status, const char* what)
{
    if(status == 0)
        return true;
    std::println(stderr, "[hydrus_sim] {} failed", what);
    return false;
}

bool BuildHydrusRobot(SfWorld* world)
{
    const SfMaterial materials[] = {
        {"acrylic", 1200.0, 0.3, 0.0},
        {"aluminium", 2700.0, 0.8, 0.0},
        {"abs", 1040.0, 0.4, 0.0},
        {"hdpe", 950.0, 0.4, 0.0},
        {"pvc", 1500.0, 0.4, 0.0},
    };
    for(const SfMaterial& material : materials)
    {
        if(!CallOk(sf_material(world, &material), "hydrus material"))
            return false;
    }
    const SfLook looks[] = {
        {"clear", {0.9, 0.9, 0.9}, 0.1, 0.0, 0.5, nullptr, nullptr},
        {"black", {0.0, 0.0, 0.0}, 0.1, 0.0, 0.5, nullptr, nullptr},
        {"green", {0.0, 0.2, 0.0}, 0.1, 0.1, 0.5, nullptr, nullptr},
        {"propeller", {1.0, 1.0, 1.0}, 0.3, 0.0, 0.5, nullptr, nullptr},
    };
    for(const SfLook& look : looks)
    {
        if(!CallOk(sf_look(world, &look), "hydrus look"))
            return false;
    }

    SfRobot robot{};
    robot.name = "HydrusAUV";
    robot.base_link = "Hydrus";
    robot.fixed = 0;
    robot.self_collisions = 0;
    robot.physics = SF_BODY_SUBMERGED;
    robot.buoyant = 1;
    robot.world = PoseAt(0.0, -1.0, 2.0, 0.0, 0.0, 3.14);
    if(!CallOk(sf_robot_begin(world, &robot), "hydrus robot"))
        return false;

    const SfPose mesh_origin = PoseAt(0.0, 0.0, 0.0, -1.5708, 0.0, 0.0);
    SfPartCylinder cabin{};
    cabin.part = SubmergedPart("Cabin", "acrylic", "clear", 0.005, mesh_origin, PoseAt(0.0, 0.0, -0.0775, 0.0, 0.0, 0.0));
    cabin.radius = 0.0825;
    cabin.height = 0.75;
    SfPartBox electronics{};
    electronics.part = SubmergedPart("Box", "aluminium", "black", 0.003, mesh_origin, PoseAt(0.0, 0.0, -0.080, 0.0, 0.0, 0.0));
    electronics.dimensions[0] = 0.22;
    electronics.dimensions[1] = 0.22;
    electronics.dimensions[2] = 0.12;
    SfPartBox dvl{};
    dvl.part = SubmergedPart("DVL", "abs", "green", -1.0, mesh_origin, PoseAt(0.0, 0.0, 0.07, 0.0, 0.0, 0.0));
    dvl.dimensions[0] = 0.1;
    dvl.dimensions[1] = 0.1;
    dvl.dimensions[2] = 0.1;
    SfPartMesh legs_left{};
    legs_left.part = SubmergedPart(
        "LowLegsLeft", "hdpe", "black", -1.0, mesh_origin, PoseAt(0.0, -0.375, -0.08, 0.0, 0.0, 0.0));
    legs_left.path = "models/hydrus_lowlegs.obj";
    legs_left.scale = 0.01;
    SfPartMesh legs_right{};
    legs_right.part = SubmergedPart(
        "LowLegsRight", "hdpe", "black", -1.0, mesh_origin, PoseAt(0.228, -0.375, -0.08, 0.0, 0.0, 0.0));
    legs_right.path = "models/hydrus_lowlegs.obj";
    legs_right.scale = 0.01;
    if(!CallOk(sf_robot_part_cylinder(world, &cabin), "cabin")
       || !CallOk(sf_robot_part_box(world, &electronics), "electronics box")
       || !CallOk(sf_robot_part_box(world, &dvl), "dvl")
       || !CallOk(sf_robot_part_mesh(world, &legs_left), "left legs")
       || !CallOk(sf_robot_part_mesh(world, &legs_right), "right legs"))
        return false;

    const double max_setpoint = 1000.0 / 60.0 * 2.0 * 3.14159265358979323846;
    const SfPose mounts[] = {
        PoseAt(0.198, 0.407, 0.0, 0.0, 0.0, -1.1781),
        PoseAt(-0.198, 0.407, 0.0, 0.0, 0.0, -1.9635),
        PoseAt(0.198, -0.408, 0.0, 0.0, 0.0, 1.1781),
        PoseAt(-0.198, -0.408, 0.0, 0.0, 0.0, 1.9635),
        PoseAt(0.211, 0.169, 0.0, 0.0, -1.571, 0.0),
        PoseAt(-0.211, 0.169, 0.0, 0.0, -1.571, 0.0),
        PoseAt(0.211, -0.169, 0.0, 0.0, -1.571, 0.0),
        PoseAt(-0.211, -0.169, 0.0, 0.0, -1.571, 0.0),
    };
    const char* thruster_names[] = {
        "thruster_0_front_left",
        "thruster_1_front_right",
        "thruster_2_back_left",
        "thruster_3_back_right",
        "thruster_4_depth_front_left",
        "thruster_5_front_right",
        "thruster_6_depth_back_left",
        "thruster_7_back_right",
    };
    for(int i = 0; i < 8; ++i)
    {
        SfThruster thruster{};
        thruster.name = thruster_names[i];
        thruster.link = "Hydrus";
        thruster.origin = mounts[i];
        thruster.diameter = 0.18;
        thruster.max_setpoint = max_setpoint;
        thruster.right_handed = 1;
        thruster.inverted_setpoint = 1;
        thruster.normalized_setpoint = 1;
        thruster.propeller_mesh = "models/propeller.obj";
        thruster.propeller_scale = 0.75;
        thruster.propeller_material = "pvc";
        thruster.propeller_look = "propeller";
        thruster.kp = 1.0;
        thruster.ki = 10.0;
        thruster.ilimit = 5.0;
        thruster.thrust_forward = 0.48;
        thruster.thrust_reverse = 0.48;
        thruster.torque_coeff = 0.05;
        if(!CallOk(sf_robot_thruster(world, &thruster), thruster.name))
            return false;
    }

    SfSensor odometry{};
    odometry.name = "Odometry";
    odometry.link = "Hydrus";
    odometry.origin = PoseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    odometry.rate = 120.0;
    odometry.history = -1;
    SfImu imu{};
    imu.sensor.name = "HydrusCameraIMU";
    imu.sensor.link = "Hydrus";
    imu.sensor.origin = PoseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    imu.sensor.rate = 30.0;
    imu.sensor.history = 1;
    imu.angular_velocity_range[0] = 20.0;
    imu.angular_velocity_range[1] = 20.0;
    imu.angular_velocity_range[2] = 20.0;
    imu.linear_acceleration_range = 30.0;
    imu.angle_noise[0] = 0.001;
    imu.angle_noise[1] = 0.001;
    imu.angle_noise[2] = 0.001;
    imu.angular_velocity_noise = 0.0005;
    imu.yaw_drift = 0.0;
    imu.linear_acceleration_noise = 0.005;
    SfCamera camera{};
    camera.sensor.name = "RGBCamera";
    camera.sensor.link = "Hydrus";
    camera.sensor.origin = PoseAt(0.0, 0.3, 0.0, 1.5708, 0.0, 3.14);
    camera.sensor.rate = 20.0;
    camera.sensor.history = -1;
    camera.resolution_x = 800;
    camera.resolution_y = 600;
    camera.horizontal_fov_deg = 60.0;
    if(!CallOk(sf_robot_odometry(world, &odometry), "odometry")
       || !CallOk(sf_robot_imu(world, &imu), "imu")
       || !CallOk(sf_robot_camera(world, &camera), "camera")
       || !CallOk(sf_robot_end(world), "hydrus robot end"))
        return false;
    return true;
}

bool BuildProteusRobot(SfWorld* world)
{
    const SfPose identity = PoseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    if(!AddLook(world, "blue", 0.0, 0.5, 1.0, 0.3, 0.0, 0.5, nullptr))
        return false;

    SfRobot robot{};
    robot.name = "ProteusAUV";
    robot.base_link = "Proteus";
    robot.physics = SF_BODY_SUBMERGED;
    robot.buoyant = 1;
    robot.world = PoseAt(0.0, -1.0, 2.0, 0.0, 0.0, 3.14);
    if(!CallOk(sf_robot_begin(world, &robot), "proteus robot"))
        return false;
    if(!AddCylinder(world, "TopCabin", "acrylic", "clear", 0.1, 0.3, 0.005, 0.0, 1, 0, PoseAt(0.0, 0.0, -0.05, 1.5708, 0.0, 0.0))
       || !AddCylinder(world, "BottomCabin", "acrylic", "clear", 0.05, 0.31, 0.005, 0.0, 1, 0, PoseAt(0.0, 0.0, 0.13, 1.5708, 0.0, 0.0))
       || !AddBox(world, "LeftRail", "abs", "blue", 0.04, 0.45, 0.04, 0.0, 1, 0, PoseAt(0.17, 0.0, -0.03, 0.0, 0.0, 0.0))
       || !AddBox(world, "RightRail", "abs", "blue", 0.04, 0.45, 0.04, 0.0, 1, 0, PoseAt(-0.17, 0.0, -0.03, 0.0, 0.0, 0.0))
       || !AddMesh(
           world, "ProtySupport1", "abs", "blue", "models/protysupport.obj", 0.001, identity, nullptr, 1.0, identity, -1.0, 1,
           PoseAt(0.0, 0.135, -0.012, 1.5708, 0.0, 0.0))
       || !AddMesh(
           world, "ProtySupport2", "abs", "blue", "models/protysupport.obj", 0.001, identity, nullptr, 1.0, identity, -1.0, 1,
           PoseAt(0.0, -0.135, -0.012, 1.5708, 0.0, 0.0)))
        return false;

    const double max_setpoint = 1000.0 / 60.0 * 2.0 * 3.14159265358979323846;
    const SfPose mounts[] = {
        PoseAt(0.246, 0.0, -0.03, 0.0, 0.0, 4.7123),
        PoseAt(-0.246, 0.0, -0.03, 0.0, 0.0, 4.7123),
        PoseAt(0.165, 0.265, 0.0, 0.0, -1.571, 0.0),
        PoseAt(-0.165, 0.265, 0.0, 0.0, -1.571, 0.0),
        PoseAt(0.165, -0.265, 0.0, 0.0, -1.571, 0.0),
        PoseAt(-0.165, -0.265, 0.0, 0.0, -1.571, 0.0),
    };
    const char* names[] = {
        "thruster_0_left",
        "thruster_1_right",
        "thruster_2_depth_front_left",
        "thruster_3_depth_front_right",
        "thruster_4_depth_back_left",
        "thruster_5_depth_back_right",
    };
    for(int i = 0; i < 6; ++i)
    {
        if(!AddThruster(
               world, names[i], "Proteus", mounts[i], 0.18, max_setpoint, 1, 1, "models/propeller.obj", 0.5, "pvc",
               "propeller", 0.0, 0.0, 0.0, 0.48, 0.48, 0.05, 0.2))
            return false;
    }

    SfSensor odometry{};
    odometry.name = "Odometry";
    odometry.link = "Proteus";
    odometry.origin = identity;
    odometry.rate = 30.0;
    odometry.history = -1;
    SfImu imu{};
    imu.sensor.name = "ProteusCameraIMU";
    imu.sensor.link = "Proteus";
    imu.sensor.origin = identity;
    imu.sensor.rate = 200.0;
    imu.sensor.history = 1;
    imu.angular_velocity_range[0] = 20.0;
    imu.angular_velocity_range[1] = 20.0;
    imu.angular_velocity_range[2] = 20.0;
    imu.linear_acceleration_range = 30.0;
    imu.angle_noise[0] = 0.00016968;
    imu.angle_noise[1] = 0.00016968;
    imu.angle_noise[2] = 0.00016968;
    imu.angular_velocity_noise = 0.00016968;
    imu.yaw_drift = 0.000019393;
    imu.linear_acceleration_noise = 0.002;
    SfCamera camera{};
    camera.sensor.name = "Camera";
    camera.sensor.link = "Proteus";
    camera.sensor.origin = PoseAt(0.0, 0.3, 0.0, 1.5708, 0.0, 3.14);
    camera.sensor.rate = 21.0;
    camera.sensor.history = -1;
    camera.resolution_x = 800;
    camera.resolution_y = 600;
    camera.horizontal_fov_deg = 60.0;
    return CallOk(sf_robot_odometry(world, &odometry), "proteus odometry")
           && CallOk(sf_robot_imu(world, &imu), "proteus imu")
           && CallOk(sf_robot_camera(world, &camera), "proteus camera")
           && CallOk(sf_robot_end(world), "proteus robot end");
}

bool BuildBluerov2Robot(SfWorld* world)
{
    const SfPose identity = PoseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    if(!AddMaterial(world, "Fiberglass", 1800.0, 0.3)
       || !AddMaterial(world, "Neutral", 1000.0, 0.5)
       || !AddMaterial(world, "Steel", 7810.0, 0.9)
       || !AddLook(world, "None", 0.2, 0.2, 0.2, 0.5, 0.0, 0.5, nullptr)
       || !AddLook(world, "blue", 0.0, 0.5, 1.0, 0.3, 0.0, 0.5, nullptr)
       || !AddLook(world, "br2", 1.0, 1.0, 1.0, 0.4, 0.5, 0.5, "bluerov2/br2.png"))
        return false;

    SfRobot robot{};
    robot.name = "bluerov2";
    robot.base_link = "base_link";
    robot.physics = SF_BODY_SUBMERGED;
    robot.buoyant = 1;
    robot.world = PoseAt(0.0, -1.0, 2.0, 0.0, 0.0, 3.14);
    if(!CallOk(sf_robot_begin(world, &robot), "bluerov2 robot"))
        return false;
    if(!AddMesh(
           world, "HullBottom", "Fiberglass", "br2", "bluerov2/bluerov2_phy.obj", 1.0, identity, "bluerov2/bluerov2.obj",
           1.0, identity, 0.005, 0, identity)
       || !AddMesh(
           world, "HeavyFit", "Fiberglass", "black", "bluerov2/bluerov2_ring.obj", 1.0, PoseAt(0.0, 0.0, 0.03, 0.0, 0.0, 0.0),
           "bluerov2/bluerov2_wings.obj", 1.0, identity, 0.005, 0, identity)
       || !AddBox(world, "BackLeft", "Neutral", "None", 0.2, 0.15, 0.091, 0.025, 1, 1, PoseAt(-0.1, -0.1, 0.0, 0.0, 0.0, 0.0))
       || !AddBox(world, "BackRight", "Neutral", "None", 0.2, 0.15, 0.091, 0.025, 1, 1, PoseAt(-0.1, 0.1, 0.0, 0.0, 0.0, 0.0))
       || !AddBox(world, "FrontLeft", "Neutral", "None", 0.2, 0.15, 0.091, 0.025, 1, 1, PoseAt(0.09, -0.1, 0.0, 0.0, 0.0, 0.0))
       || !AddBox(world, "FrontRight", "Neutral", "None", 0.2, 0.15, 0.091, 0.025, 1, 1, PoseAt(0.09, 0.1, 0.0, 0.0, 0.0, 0.0))
       || !AddSphere(world, "WeightCenter", "Steel", "black", 0.01, 2.0, 0, 1, PoseAt(0.0, 0.0, 0.1, 0.0, 0.0, 0.0))
       || !AddSphere(world, "WeightLeft", "Steel", "black", 0.01, 1.0, 0, 1, PoseAt(0.0, -0.075, 0.1, 0.0, 0.0, 0.0))
       || !AddSphere(world, "WeightRight", "Steel", "black", 0.01, 1.0, 0, 1, PoseAt(0.0, 0.075, 0.1, 0.0, 0.0, 0.0)))
        return false;

    const double max_setpoint = 4000.0 / 60.0 * 2.0 * 3.14159265358979323846;
    struct Mount
    {
        const char* name;
        SfPose origin;
        int right_handed;
        int inverted;
        const char* propeller;
    };
    const Mount mounts[] = {
        {"FrontRight", PoseAt(0.1355, 0.1, 0.0725, 0.0, 0.0, -0.7853981634), 1, 1, "bluerov2/ccw.obj"},
        {"FrontLeft", PoseAt(0.1355, -0.1, 0.0725, 0.0, 0.0, 0.7853981634), 1, 1, "bluerov2/ccw.obj"},
        {"BackRight", PoseAt(-0.1475, 0.1, 0.0725, 0.0, 0.0, -2.3561944902), 0, 0, "bluerov2/cw.obj"},
        {"BackLeft", PoseAt(-0.1475, -0.1, 0.0725, 0.0, 0.0, 2.3561944902), 0, 0, "bluerov2/cw.obj"},
        {"DiveFrontRight", PoseAt(0.12, 0.218, 0.0, 0.0, -1.5707963268, 0.0), 1, 0, "bluerov2/cw.obj"},
        {"DiveFrontLeft", PoseAt(0.12, -0.218, 0.0, 0.0, -1.5707963268, 0.0), 0, 1, "bluerov2/ccw.obj"},
        {"DiveBackRight", PoseAt(-0.12, 0.218, 0.0, 0.0, -1.5707963268, 0.0), 0, 1, "bluerov2/ccw.obj"},
        {"DiveBackLeft", PoseAt(-0.12, -0.218, 0.0, 0.0, -1.5707963268, 0.0), 1, 0, "bluerov2/cw.obj"},
    };
    for(const Mount& mount : mounts)
    {
        if(!AddThruster(
               world, mount.name, "base_link", mount.origin, 0.076, max_setpoint, mount.right_handed, mount.inverted,
               mount.propeller, 1.0, "Neutral", "blue", 0.0, 0.0, 0.0, 0.334, 0.334, 0.032, 0.2))
            return false;
    }

    SfSensor odometry{};
    odometry.name = "odometry";
    odometry.link = "base_link";
    odometry.origin = identity;
    odometry.rate = 100.0;
    odometry.history = -1;
    SfImu imu{};
    imu.sensor.name = "imu_filter";
    imu.sensor.link = "base_link";
    imu.sensor.origin = identity;
    imu.sensor.rate = 20.0;
    imu.sensor.history = -1;
    imu.angular_velocity_range[0] = 20.0;
    imu.angular_velocity_range[1] = 20.0;
    imu.angular_velocity_range[2] = 20.0;
    imu.linear_acceleration_range = 30.0;
    imu.angle_noise[0] = 0.000001745;
    imu.angle_noise[1] = 0.000001745;
    imu.angle_noise[2] = 0.000001745;
    imu.angular_velocity_noise = 0.00001745;
    SfCamera left{};
    left.sensor.name = "camera_left";
    left.sensor.link = "base_link";
    left.sensor.origin = PoseAt(0.16, -0.0725, 0.15, 1.571, 0.0, 1.571);
    left.sensor.rate = 30.0;
    left.sensor.history = -1;
    left.resolution_x = 640;
    left.resolution_y = 480;
    left.horizontal_fov_deg = 75.0;
    SfCamera right{};
    right.sensor.name = "camera_right";
    right.sensor.link = "base_link";
    right.sensor.origin = PoseAt(0.16, 0.0725, 0.15, 1.571, 0.0, 1.571);
    right.sensor.rate = 30.0;
    right.sensor.history = -1;
    right.resolution_x = 640;
    right.resolution_y = 480;
    right.horizontal_fov_deg = 75.0;
    return CallOk(sf_robot_odometry(world, &odometry), "bluerov2 odometry")
           && CallOk(sf_robot_imu(world, &imu), "bluerov2 imu")
           && CallOk(sf_robot_camera(world, &left), "bluerov2 left camera")
           && CallOk(sf_robot_camera(world, &right), "bluerov2 right camera")
           && CallOk(sf_robot_end(world), "bluerov2 robot end");
}

bool BuildGirona500Robot(SfWorld* world)
{
    const SfPose identity = PoseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    if(!AddMaterial(world, "Fiberglass", 1800.0, 0.3)
       || !AddMaterial(world, "Neutral", 1000.0, 0.5)
       || !AddMaterial(world, "Aluminium", 2700.0, 0.8))
        return false;

    SfRobot robot{};
    robot.name = "Girona500AUV";
    robot.base_link = "Vehicle";
    robot.physics = SF_BODY_SUBMERGED;
    robot.buoyant = 1;
    robot.world = PoseAt(0.0, -1.0, 2.0, 0.0, 0.0, -1.5708);
    if(!CallOk(sf_robot_begin(world, &robot), "girona500 robot"))
        return false;
    if(!AddMesh(
           world, "HullBottom", "Fiberglass", "yellow", "models/hull_hydro.obj", 1.0, identity, nullptr, 1.0, identity, 0.003, 0,
           PoseAt(-0.049, 0.0, 0.361, 0.0, 0.0, 0.0))
       || !AddMesh(
           world, "HullPort", "Fiberglass", "yellow", "models/hull_hydro.obj", 1.0, identity, nullptr, 1.0, identity, 0.003, 0,
           PoseAt(-0.049, -0.35, -0.339, 0.0, 0.0, 0.0))
       || !AddMesh(
           world, "HullStarboard", "Fiberglass", "yellow", "models/hull_hydro.obj", 1.0, identity, nullptr, 1.0, identity, 0.003, 0,
           PoseAt(-0.049, 0.35, -0.339, 0.0, 0.0, 0.0))
       || !AddMesh(
           world, "VBarStern", "Aluminium", "gray", "models/vbar_hydro.obj", 1.0, identity, nullptr, 1.0, identity, 0.003, 0,
           PoseAt(-0.299, 0.0, 0.211, 0.0, 0.0, 0.0))
       || !AddMesh(
           world, "VBarBow", "Aluminium", "gray", "models/vbar_hydro.obj", 1.0, identity, nullptr, 1.0, identity, 0.003, 0,
           PoseAt(0.251, 0.0, 0.211, 0.0, 0.0, 0.0))
       || !AddMesh(
           world, "DuctSway", "Neutral", "gray", "models/duct_hydro.obj", 1.0, identity, nullptr, 1.0, identity, -1.0, 1,
           PoseAt(-0.0627, 0.0307, -0.019, 0.0, 3.14, 1.57))
       || !AddMesh(
           world, "DuctSurgePort", "Neutral", "gray", "models/duct_hydro.obj", 1.0, identity, nullptr, 1.0, identity, -1.0, 1,
           PoseAt(-0.3297, -0.2587, -0.019, 3.14, 0.0, 0.0))
       || !AddMesh(
           world, "DuctSurgeStarboard", "Neutral", "gray", "models/duct_hydro.obj", 1.0, identity, nullptr, 1.0, identity, -1.0, 1,
           PoseAt(-0.3297, 0.2587, -0.019, 0.0, 0.0, 0.0))
       || !AddMesh(
           world, "DuctHeaveStern", "Neutral", "gray", "models/duct_hydro.obj", 1.0, identity, nullptr, 1.0, identity, -1.0, 1,
           PoseAt(-0.5827, 0.0, -0.3137, 0.0, -1.57, 1.57))
       || !AddMesh(
           world, "DuctHeaveBow", "Neutral", "gray", "models/duct_hydro.obj", 1.0, identity, nullptr, 1.0, identity, -1.0, 1,
           PoseAt(0.5347, 0.0, -0.3137, 0.0, -1.57, -1.57))
       || !AddCylinder(
           world, "BatteryCylinder", "Neutral", "gray", 0.13, 0.6, -1.0, 70.0, 1, 1, PoseAt(-0.099, 0.0, 0.361, 0.0, 1.571, 0.0))
       || !AddCylinder(
           world, "PortCylinder", "Neutral", "gray", 0.13, 1.0, -1.0, 20.0, 1, 1, PoseAt(-0.049, -0.35, -0.339, 0.0, 1.571, 0.0))
       || !AddCylinder(
           world, "StarboardCylinder", "Neutral", "gray", 0.13, 1.0, -1.0, 20.0, 1, 1,
           PoseAt(-0.049, 0.35, -0.339, 0.0, 1.571, 0.0)))
        return false;

    const double max_setpoint = 1000.0 / 60.0 * 2.0 * 3.14159265358979323846;
    const SfPose mounts[] = {
        PoseAt(-0.3297, -0.2587, -0.021, 3.1416, 0.0, 0.0),
        PoseAt(-0.3297, 0.2587, -0.021, 0.0, 0.0, 0.0),
        PoseAt(0.5347, 0.0, -0.3137, 0.0, -1.571, -1.571),
        PoseAt(-0.5827, 0.0, -0.3137, 0.0, -1.571, 1.571),
        PoseAt(-0.0627, 0.0307, -0.021, 0.0, 3.1416, 1.571),
    };
    const char* names[] = {
        "ThrusterSurgePort",
        "ThrusterSurgeStarboard",
        "ThrusterHeaveBow",
        "ThrusterHeaveStern",
        "ThrusterSway",
    };
    for(int i = 0; i < 5; ++i)
    {
        if(!AddThruster(
               world, names[i], "Vehicle", mounts[i], 0.18, max_setpoint, 1, 1, "models/propeller.obj", 1.0, "Neutral",
               "propeller", 1.0, 0.5, 2.0, 0.48, 0.48, 0.05, 0.0))
            return false;
    }

    SfSensor odometry{};
    odometry.name = "Odometry";
    odometry.link = "Vehicle";
    odometry.origin = identity;
    odometry.rate = 30.0;
    odometry.history = -1;
    SfCamera camera{};
    camera.sensor.name = "Camera";
    camera.sensor.link = "Vehicle";
    camera.sensor.origin = PoseAt(0.5, -0.12, 0.5, 0.0, 0.0, 1.571);
    camera.sensor.rate = 10.0;
    camera.sensor.history = -1;
    camera.resolution_x = 800;
    camera.resolution_y = 600;
    camera.horizontal_fov_deg = 60.0;
    return CallOk(sf_robot_odometry(world, &odometry), "girona500 odometry")
           && CallOk(sf_robot_camera(world, &camera), "girona500 camera")
           && CallOk(sf_robot_end(world), "girona500 robot end");
}

bool BuildRobot(SfWorld* world, const std::string& robot)
{
    if(robot.empty() || robot == "hydrus")
        return BuildHydrusRobot(world);
    if(robot == "proteus")
        return BuildProteusRobot(world);
    if(robot == "bluerov2")
        return BuildBluerov2Robot(world);
    if(robot == "girona500")
        return BuildGirona500Robot(world);
    std::println(stderr, "[hydrus_sim] unknown robot '{}'", robot);
    return false;
}

bool RunBlendBuilder(SfWorld* world, const std::filesystem::path& blend, const std::filesystem::path& data)
{
    Dl_info info{};
    if(dladdr(reinterpret_cast<void*>(&sf_world_bind), &info) == 0 || info.dli_fname == nullptr)
    {
        std::println(stderr, "[hydrus_sim] cannot locate the auv library for ctypes");
        return false;
    }
    if(!Py_IsInitialized())
    {
        // The AUV library is dlopened RTLD_LOCAL, so Python extension modules
        // cannot see libpython unless it is also on the global namespace.
        Dl_info python_info{};
        if(dladdr(reinterpret_cast<void*>(&Py_Initialize), &python_info) != 0 && python_info.dli_fname != nullptr)
            dlopen(python_info.dli_fname, RTLD_NOW | RTLD_GLOBAL);
        Py_Initialize();
    }
    const PyGILState_STATE gil = PyGILState_Ensure();
    const std::string bootstrap = std::string("import sys\nsys.path.insert(0, \"") + BLENDER_STONEFISH_ROOT + "\")\n";
    if(PyRun_SimpleString(bootstrap.c_str()) != 0)
    {
        PyErr_Print();
        PyGILState_Release(gil);
        std::println(stderr, "[hydrus_sim] failed to add blender_stonefish to sys.path");
        return false;
    }
    PyObject* module = PyImport_ImportModule("blender_stonefish");
    if(module == nullptr)
    {
        PyErr_Print();
        PyGILState_Release(gil);
        std::println(stderr, "[hydrus_sim] failed to import blender_stonefish");
        return false;
    }
    PyObject* build = PyObject_GetAttrString(module, "build");
    PyObject* result = PyObject_CallFunction(
        build,
        "Kssss",
        static_cast<unsigned long long>(reinterpret_cast<uintptr_t>(world)),
        info.dli_fname,
        blend.c_str(),
        BLENDER_STONEFISH_CONFIG,
        data.c_str());
    const bool ok = result != nullptr && PyLong_Check(result) && PyLong_AsLong(result) == 0;
    if(!ok)
        PyErr_Print();
    Py_XDECREF(result);
    Py_XDECREF(build);
    Py_DECREF(module);
    PyGILState_Release(gil);
    if(!ok)
        std::println(stderr, "[hydrus_sim] blend builder failed for {}", blend.string());
    return ok;
}

class SimManager : public sf::SimulationManager
{
public:
    SimManager(
        sf::Scalar stepsPerSecond,
        std::filesystem::path data,
        std::string blendFile,
        std::string robot)
        : SimulationManager(stepsPerSecond),
          dataPath(std::move(data)),
          blendPath(dataPath / blendFile),
          robotName(std::move(robot))
    {
    }

    ~SimManager() override
    {
        sf_world_free(sceneWorld);
    }

    void BuildScenario() override
    {
        std::unordered_map<std::string, ObjectCls> objectClasses;
        sf_world_free(sceneWorld);
        sceneWorld = sf_world_bind(this);
        sf_world_set_data_dir(sceneWorld, dataPath.c_str());
        if(!RunBlendBuilder(sceneWorld, blendPath, dataPath) || !BuildRobot(sceneWorld, robotName))
        {
            std::println(stderr, "[hydrus_sim] failed to build scenario from {}", blendPath.string());
            return;
        }
        const int count = sf_world_class_count(sceneWorld);
        for(int i = 0; i < count; ++i)
        {
            char name[256];
            char cls[64];
            if(sf_world_class_at(sceneWorld, i, name, sizeof(name), cls, sizeof(cls)) != 0)
                continue;
            const auto mapped = ParseObjectClass(cls);
            if(!mapped)
            {
                std::println(stderr, "[hydrus_sim] static object '{}' has unknown cls '{}'", name, cls);
                continue;
            }
            objectClasses[name] = *mapped;
        }

        odometry = nullptr;
        have_odometry = false;
        camera = nullptr;
        image_width = 0;
        image_height = 0;
        focal_px = 0.f;
        for(unsigned int i = 0; sf::Sensor* sensor = getSensor(i); ++i)
        {
            if(sensor->getType() == sf::SensorType::VISION)
            {
                if(auto* color_camera = dynamic_cast<sf::Camera*>(sensor))
                {
                    camera = color_camera;
                    camera->getResolution(image_width, image_height);
                    const float half_width = static_cast<float>(image_width) * 0.5f;
                    const float half_hfov_rad =
                        static_cast<float>(camera->getHorizontalFOV()) * (static_cast<float>(M_PI) / 360.f);
                    if(half_width > 0.f && half_hfov_rad > 0.f)
                        focal_px = half_width / std::tan(half_hfov_rad);
                }
                continue;
            }

            if(sensor->getType() != sf::SensorType::LINK)
                continue;
            auto* scalar = static_cast<sf::ScalarSensor*>(sensor);
            if(scalar->getScalarSensorType() == sf::ScalarSensorType::ODOM)
            {
                odometry = scalar;
                have_odometry = odometry->getNumOfChannels() >= 10;
            }
        }

        tracked_objects.clear();
        for(unsigned int i = 0; sf::Entity* entity = getEntity(i); ++i)
        {
            if(tracked_objects.size() == AUV_FRAME_MAX_OBJECTS)
                break;
            auto it = objectClasses.find(entity->getName());
            if(it == objectClasses.end())
                continue;
            tracked_objects.push_back({entity, i, it->second});
        }

        thrusters.clear();
        sf::Robot* robot = getRobot(0u);
        if(robot == nullptr)
        {
            std::println(stdout, "[hydrus_sim] no robot in scenario; thrusters will not run");
            return;
        }
        for(size_t i = 0; sf::Actuator* actuator = robot->getActuator(i); ++i)
        {
            if(actuator->getType() != sf::ActuatorType::THRUSTER)
                continue;
            thrusters.push_back(static_cast<sf::Thruster*>(actuator));
        }
    }

    void setThrustorValues(const float* thrustor_values, uint8_t thrustor_values_len)
    {
        const uint8_t count = std::min(thrustor_values_len, static_cast<uint8_t>(thrusters.size()));
        for(uint8_t i = 0; i < count; ++i)
            thrusters[i]->setSetpoint(thrustor_values[i]);
    }

    AuvFrame getAuvFrame()
    {
        AuvFrame frame{};
        frame.camera_pose = IdentityPose();
        frame.timestamp = static_cast<uint64_t>(getSimulationTime() * sf::Scalar(1e9));

        if(have_odometry)
        {
            frame.camera_pose.pos.buf[0] = static_cast<float>(odometry->getLastValue(0));
            frame.camera_pose.pos.buf[1] = static_cast<float>(odometry->getLastValue(1));
            frame.camera_pose.pos.buf[2] = static_cast<float>(odometry->getLastValue(2));
            frame.camera_pose.quat.buf[0] = static_cast<float>(odometry->getLastValue(6));
            frame.camera_pose.quat.buf[1] = static_cast<float>(odometry->getLastValue(7));
            frame.camera_pose.quat.buf[2] = static_cast<float>(odometry->getLastValue(8));
            frame.camera_pose.quat.buf[3] = static_cast<float>(odometry->getLastValue(9));
        }

        for(const TrackedObject& tracked : tracked_objects)
        {
            if(bboxOnlyInFrontOfCamera && !IsInFrontOfCamera(camera, tracked.entity))
                continue;

            AuvObject object{};
            if(!FillObjectBBox(tracked.entity, object.bbox))
                continue;
            object.id = tracked.id;
            object.cls = static_cast<AuvObjectCls>(tracked.cls);
            frame.objects[frame.objects_len++] = object;

            if(frame.objects2d_len >= AUV_FRAME_MAX_OBJECTS)
                continue;
            AuvObject2d box{};
            if(!ProjectObjectBox(camera, image_width, image_height, focal_px, tracked.entity, box))
                continue;
            box.id = tracked.id;
            box.cls = static_cast<AuvObjectCls>(tracked.cls);
            frame.objects2d[frame.objects2d_len++] = box;
        }

        frame.image_width = image_width;
        frame.image_height = image_height;
        frame.tracking_ok = getSimulationTime() < trackingOkSeconds;
        if(!frame.tracking_ok && !logged_tracking_loss)
        {
            logged_tracking_loss = true;
            std::println(stdout, "[hydrus_sim] tracking_ok false after {} s", trackingOkSeconds);
        }
        if(have_odometry)
        {
            // Scenario is NED, so odometry Z is depth, positive down.
            frame.pressure_depth = static_cast<float>(odometry->getLastValue(2));
            frame.pressure_depth_ok = true;
        }

        return frame;
    }


    struct TrackedObject
    {
        sf::Entity* entity;
        uint32_t id;
        ObjectCls cls;
    };

    std::filesystem::path dataPath;
    std::filesystem::path blendPath;
    std::string robotName;
    SfWorld* sceneWorld = nullptr;
    sf::Scalar trackingOkSeconds = std::numeric_limits<uint32_t>::max();;
    bool bboxOnlyInFrontOfCamera = false;
    bool have_odometry = false;
    sf::ScalarSensor* odometry = nullptr;
    sf::Camera* camera = nullptr;
    unsigned int image_width = 0;
    unsigned int image_height = 0;
    float focal_px = 0.f;
    bool logged_tracking_loss = false;
    std::vector<TrackedObject> tracked_objects;
    std::vector<sf::Thruster*> thrusters;
};

class SimApp : public sf::GraphicalSimulationApp
{
public:
    SimApp(std::string dataPath, sf::RenderSettings render_setting, sf::HelperSettings helper_settings, sf::SimulationManager* sim_manager)
        : sf::GraphicalSimulationApp("hydrus_sim", dataPath, render_setting, helper_settings, sim_manager)
    {
    }

    void start(sf::Scalar timeStep)
    {
        autostep_ = false;
        timeStep_ = timeStep;
        Init();
        StartSimulation();
    }

    void updateGraphics()
    {
        if(getState() == sf::SimulationState::FINISHED)
            return;

        const auto now = std::chrono::steady_clock::now();
        if(lastRender.has_value() && now - *lastRender < minRenderInterval)
            return;
        else{
        lastRender = now;
        sf::OpenGLPipeline* pipeline = getGLPipeline();
        SDL_LockMutex(pipeline->getDrawingQueueMutex());
        pipeline->PurgeDrawingQueue();
        pipeline->PurgeSelectedDrawingQueue();
        getSimulationManager()->UpdateDrawingQueue();
        SDL_UnlockMutex(pipeline->getDrawingQueueMutex());
        LoopInternal();
        }
    }

    void shutdown()
    {
        if(cleaned)
            return;
        if(getState() != sf::SimulationState::FINISHED)
            Quit();
        CleanUp();
        cleaned = true;
    }

    std::chrono::milliseconds minRenderInterval{0};

    void setTimeStep(sf::Scalar timeStep)
    {
        timeStep_ = timeStep;
    }

private:
    bool cleaned = false;
    std::optional<std::chrono::steady_clock::time_point> lastRender;
};

class ConsoleSimApp : public sf::ConsoleSimulationApp
{
public:
    ConsoleSimApp(std::string dataPath, sf::SimulationManager* sim_manager)
        : sf::ConsoleSimulationApp("hydrus_sim", dataPath, sim_manager)
    {
    }

    void start(sf::Scalar timeStep)
    {
        autostep_ = false;
        timeStep_ = timeStep;
        Init();
        StartSimulation();
    }

    void shutdown()
    {
        if(cleaned)
            return;
        if(getState() != sf::SimulationState::FINISHED)
            Quit();
        CleanUp();
        cleaned = true;
    }

    void setTimeStep(sf::Scalar timeStep)
    {
        timeStep_ = timeStep;
    }

private:
    bool cleaned = false;
};

struct RealtimeThrottle
{
    void wait(sf::Scalar sim_time)
    {
        if(realtimeFactorCap <= sf::Scalar(0))
            return;

        const auto wall = std::chrono::steady_clock::now();
        if(!base_set)
        {
            base_sim = sim_time;
            base_wall = wall;
            base_set = true;
            return;
        }

        const sf::Scalar sim_elapsed = sim_time - base_sim;
        const sf::Scalar wall_elapsed =
            std::chrono::duration<sf::Scalar>(wall - base_wall).count();
        const sf::Scalar wall_budget = sim_elapsed / realtimeFactorCap;

        if(wall_budget > wall_elapsed)
        {
            const sf::Scalar deficit = wall_budget - wall_elapsed;
            // Sub-ms sleeps lose more to the scheduler than they buy.
            if(deficit < sf::Scalar(0.001))
                return;
            std::this_thread::sleep_for(std::chrono::duration<sf::Scalar>(deficit));
        }
        else
        {
            base_sim = sim_time;
            base_wall = wall;
        }
    }

    sf::Scalar realtimeFactorCap = 0;
    bool base_set = false;
    sf::Scalar base_sim = 0;
    std::chrono::steady_clock::time_point base_wall{};
};

struct SimulationContext
{
    PlatformConfig config;
    SimManager* sim = nullptr;
    SimApp* graphical = nullptr;
    ConsoleSimApp* console = nullptr;
    RealtimeThrottle realtime;
    std::atomic<bool> watcherStop{false};
    std::atomic<bool> scenarioReload{false};
    std::mutex watchMutex;
    std::filesystem::path watchedBlend;
    std::filesystem::file_time_type platformMtime{};
    std::filesystem::file_time_type blendMtime{};
    std::thread watcher;
};

void scenarioWatchLoop(SimulationContext* ctx)
{
    while(!ctx->watcherStop.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if(ctx->watcherStop.load())
            break;

        std::filesystem::path blend;
        std::filesystem::file_time_type platformBase{};
        std::filesystem::file_time_type blendBase{};
        {
            std::lock_guard<std::mutex> lock(ctx->watchMutex);
            blend = ctx->watchedBlend;
            platformBase = ctx->platformMtime;
            blendBase = ctx->blendMtime;
        }

        std::error_code platformError;
        const auto platformNow = std::filesystem::last_write_time(kPlatformConfigPath, platformError);
        if(platformError)
            continue;
        std::error_code blendError;
        const auto blendNow = std::filesystem::last_write_time(blend, blendError);
        if(blendError)
            continue;
        if(platformNow == platformBase && blendNow == blendBase)
            continue;

        std::lock_guard<std::mutex> lock(ctx->watchMutex);
        if(ctx->watchedBlend != blend)
            continue;
        ctx->platformMtime = platformNow;
        ctx->blendMtime = blendNow;
        ctx->scenarioReload.store(true);
    }
}

void noteWatchedFiles(SimulationContext* ctx)
{
    std::lock_guard<std::mutex> lock(ctx->watchMutex);
    ctx->watchedBlend = ctx->sim->blendPath;
    std::error_code ec;
    ctx->platformMtime = std::filesystem::last_write_time(kPlatformConfigPath, ec);
    ctx->blendMtime = std::filesystem::last_write_time(ctx->sim->blendPath, ec);
}

void startScenarioWatcher(SimulationContext* ctx)
{
    noteWatchedFiles(ctx);
    ctx->watcherStop.store(false);
    ctx->scenarioReload.store(false);
    ctx->watcher = std::thread(scenarioWatchLoop, ctx);
}

void stopScenarioWatcher(SimulationContext* ctx)
{
    if(!ctx->watcher.joinable())
        return;
    ctx->watcherStop.store(true);
    ctx->watcher.join();
}

bool reloadScenario(SimulationContext* ctx)
{
    std::optional<PlatformConfig> loaded;
    try
    {
        loaded = loadPlatformConfig(kPlatformConfigPath);
    }
    catch(const std::exception& ex)
    {
        std::println(stderr, "[hydrus_sim] platform reload failed: {}", ex.what());
        return false;
    }
    if(!loaded)
    {
        std::println(stderr, "[hydrus_sim] platform reload failed; keeping scenario");
        return false;
    }
    if(loaded->kConsoleApp != ctx->config.kConsoleApp)
    {
        std::println(stderr, "[hydrus_sim] console change ignored while the app is running");
        loaded->kConsoleApp = ctx->config.kConsoleApp;
    }

    ctx->config = *loaded;
    SimManager* sim = ctx->sim;
    sim->dataPath = ctx->config.dataPath;
    sim->blendPath = ctx->config.dataPath / ctx->config.blendFile;
    sim->robotName = ctx->config.robot;
    sim->trackingOkSeconds = ctx->config.kTrackingOkSeconds;
    sim->bboxOnlyInFrontOfCamera = ctx->config.kBBoxOnlyInFrontOfCamera;
    sim->logged_tracking_loss = false;
    sim->setStepsPerSecond(ctx->config.kStepsPerSecond);
    ctx->realtime.realtimeFactorCap = ctx->config.kRealtimeFactorCap;
    ctx->realtime.base_set = false;
    if(ctx->graphical != nullptr)
    {
        ctx->graphical->minRenderInterval = ctx->config.kMinRenderInterval;
        ctx->graphical->setTimeStep(ctx->config.kPhysicsDt);
    }
    if(ctx->console != nullptr)
        ctx->console->setTimeStep(ctx->config.kPhysicsDt);

    noteWatchedFiles(ctx);
    std::println(stdout, "[hydrus_sim] reloading scenario robot={} blend={}", sim->robotName, sim->blendPath.string());
    sim->RestartScenario();
    if(!sim->StartSimulation())
        std::println(stderr, "[hydrus_sim] scenario restart failed to solve initial conditions");
    return true;
}

SimulationContext* g_simulation_context = nullptr;

sf::RenderSettings DefaultRenderSettings()
{
    sf::RenderSettings s;
    s.windowW = 1200;
    s.windowH = 900;
    s.aa = sf::RenderQuality::HIGH;
    s.shadows = sf::RenderQuality::HIGH;
    s.ao = sf::RenderQuality::HIGH;
    s.atmosphere = sf::RenderQuality::MEDIUM;
    s.ocean = sf::RenderQuality::HIGH;
    s.ssr = sf::RenderQuality::HIGH;
    return s;
}

sf::HelperSettings DefaultHelperSettings()
{
    sf::HelperSettings h;
    h.showFluidDynamics = false;
    h.showCoordSys = false;
    h.showBulletDebugInfo = false;
    h.showSensors = false;
    h.showActuators = false;
    h.showForces = false;
    return h;
}
}

void auv_init(void)
{
    auv_deinit();
    // SDL otherwise installs its own SIGINT handler and Ctrl+C never returns the terminal.
    SDL_SetHint(SDL_HINT_NO_SIGNAL_HANDLERS, "1");
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);

    const std::optional<PlatformConfig> loadedJson = loadPlatformConfig(kPlatformConfigPath);
    if(!loadedJson)
    {
        std::println(stderr, "[hydrus_sim] platform config failed; simulation not started");
        return;
    }

    g_simulation_context = new SimulationContext();
    g_simulation_context->config = *loadedJson;
    const PlatformConfig& config = g_simulation_context->config;
    g_simulation_context->sim = new SimManager(
        config.kStepsPerSecond, config.dataPath, config.blendFile, config.robot);
    g_simulation_context->sim->trackingOkSeconds = config.kTrackingOkSeconds;
    g_simulation_context->sim->bboxOnlyInFrontOfCamera = config.kBBoxOnlyInFrontOfCamera;
    g_simulation_context->realtime.realtimeFactorCap = config.kRealtimeFactorCap;
    if(config.kConsoleApp)
    {
        std::println(stdout, "[hydrus_sim] app=console");
        g_simulation_context->console = new ConsoleSimApp(config.dataPath.string(), g_simulation_context->sim);
        g_simulation_context->console->start(config.kPhysicsDt);
    }
    else
    {
        std::println(stdout, "[hydrus_sim] app=graphical");
        g_simulation_context->graphical = new SimApp(
            config.dataPath.string(), DefaultRenderSettings(), DefaultHelperSettings(), g_simulation_context->sim);
        g_simulation_context->graphical->minRenderInterval = config.kMinRenderInterval;
        g_simulation_context->graphical->start(config.kPhysicsDt);
    }
    startScenarioWatcher(g_simulation_context);
}

void auv_yield_until_next_frame(AuvFrame* frame)
{
    stopIfRequested();
    if(g_simulation_context == nullptr)
    {
        frame->error = AUV_ERROR_NONE;
        return;
    }

    const bool scenarioRestarted =
        g_simulation_context->scenarioReload.exchange(false) && reloadScenario(g_simulation_context);

    g_simulation_context->sim->StepSimulation(g_simulation_context->config.kPhysicsDt);
    if(g_simulation_context->graphical != nullptr)
        g_simulation_context->graphical->updateGraphics();
    g_simulation_context->realtime.wait(g_simulation_context->sim->getSimulationTime());

    const bool finished =
        (g_simulation_context->graphical != nullptr
         && g_simulation_context->graphical->getState() == sf::SimulationState::FINISHED)
        || (g_simulation_context->console != nullptr
            && g_simulation_context->console->getState() == sf::SimulationState::FINISHED);
    stopIfRequested();
    if(finished)
    {
        std::println(stderr, "[hydrus_sim] simulation finished; shutting down");
        auv_deinit();
        std::exit(0);
    }
    *frame = g_simulation_context->sim->getAuvFrame();
    frame->error = scenarioRestarted ? AUV_ERROR_SCENARIO_RESTART : AUV_ERROR_NONE;
}



void auv_set_thrustor_values(const float* thrustor_values, uint8_t thrustor_values_len)
{
    if(g_simulation_context == nullptr)
        return;
    g_simulation_context->sim->setThrustorValues(thrustor_values, thrustor_values_len); 
}

void auv_deinit(void)
{
    if(g_simulation_context == nullptr)
        return;

    stopScenarioWatcher(g_simulation_context);

    if(g_simulation_context->graphical != nullptr)
    {
        g_simulation_context->graphical->shutdown();
        delete g_simulation_context->graphical;
    }
    if(g_simulation_context->console != nullptr)
    {
        g_simulation_context->console->shutdown();
        delete g_simulation_context->console;
    }
    delete g_simulation_context->sim;

    delete g_simulation_context;
    g_simulation_context = nullptr;
}
