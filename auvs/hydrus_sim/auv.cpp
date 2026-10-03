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
#include "Stonefish/core/ScenarioParser.h"
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
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <dlfcn.h>
#include <filesystem>
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
    std::string scenarioFile;
    std::string blendFile;
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
        .scenarioFile = parsed_json.contains("scenario_file") ? parsed_json.at("scenario_file").get<std::string>() : "",
        .blendFile = parsed_json.contains("blend_file") ? parsed_json.at("blend_file").get<std::string>() : "",
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
    if(config.scenarioFile.empty() && config.blendFile.empty()){
        std::println(stderr, "[Parser] scenario_file or blend_file is required in {}", path.string());
        return std::nullopt;
    }
    if(!std::filesystem::is_directory(config.dataPath)){
        std::println(stderr, "[Parser] data_path is not a directory: {}", config.dataPath.string());
        return std::nullopt;
    }
    if(!config.blendFile.empty()){
        const std::filesystem::path blend_path = config.dataPath / config.blendFile;
        if(!std::filesystem::is_regular_file(blend_path)){
            std::println(stderr, "[Parser] blend file not found: {}", blend_path.string());
            return std::nullopt;
        }
    }
    if(!config.scenarioFile.empty()){
        const std::filesystem::path scenario_path = config.dataPath / config.scenarioFile;
        if(!std::filesystem::is_regular_file(scenario_path)){
            std::println(stderr, "[Parser] scenario file not found: {}", scenario_path.string());
            return std::nullopt;
        }
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

class AuvScenarioParser : public sf::ScenarioParser
{
public:
    using ScenarioParser::ScenarioParser;
    std::unordered_map<std::string, ObjectCls> objectClasses;

protected:
    bool ParseStatic(XMLElement* element) override
    {
        const char* type = nullptr;
        element->QueryStringAttribute("type", &type);
        if(type != nullptr && (std::string(type) == "plane" || std::string(type) == "terrain"))
            return ScenarioParser::ParseStatic(element);

        const char* name = nullptr;
        element->QueryStringAttribute("name", &name);
        const char* display_name = name != nullptr ? name : "<unnamed>";

        const char* cls = nullptr;
        if(element->QueryStringAttribute("cls", &cls) != XML_SUCCESS)
            element->QueryStringAttribute("clss", &cls);

        if(cls == nullptr || *cls == '\0')
        {
            std::println(stderr, "[hydrus_sim] static object '{}' is missing a cls tag", display_name);
            return false;
        }

        std::string cls_name{cls};
        for(char& c : cls_name)
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if(cls_name == "scenery")
            return ScenarioParser::ParseStatic(element);

        const auto mapped = ParseObjectClass(cls);
        if(!mapped)
        {
            std::println(stderr, "[hydrus_sim] static object '{}' has unknown cls '{}'", display_name, cls);
            return false;
        }
        if(name == nullptr)
        {
            std::println(stderr, "[hydrus_sim] static object with cls '{}' is missing a name", cls);
            return false;
        }

        objectClasses[name] = *mapped;
        return ScenarioParser::ParseStatic(element);
    }
};

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
        std::string scenarioFile,
        std::string blendFile)
        : SimulationManager(stepsPerSecond),
          dataPath(std::move(data)),
          scenarioPath(scenarioFile.empty() ? std::filesystem::path{} : dataPath / scenarioFile),
          blendPath(blendFile.empty() ? std::filesystem::path{} : dataPath / blendFile)
    {
    }

    ~SimManager() override
    {
        sf_world_free(sceneWorld);
    }

    void BuildScenario() override
    {
        std::unordered_map<std::string, ObjectCls> objectClasses;
        if(!blendPath.empty())
        {
            sf_world_free(sceneWorld);
            sceneWorld = sf_world_bind(this);
            sf_world_set_data_dir(sceneWorld, dataPath.c_str());
            if(!RunBlendBuilder(sceneWorld, blendPath, dataPath) || !BuildHydrusRobot(sceneWorld))
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
        }
        else
        {
            AuvScenarioParser parser(this);
            if(!parser.Parse(scenarioPath.string()))
            {
                std::println(stderr, "Failed to parse scenario {}", scenarioPath.string());
                return;
            }
            objectClasses = std::move(parser.objectClasses);
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
    std::filesystem::path scenarioPath;
    std::filesystem::path blendPath;
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
};

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
        config.kStepsPerSecond, config.dataPath, config.scenarioFile, config.blendFile);
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
}

void auv_yield_until_next_frame(AuvFrame* frame)
{
    stopIfRequested();
    if(g_simulation_context == nullptr)
        return;

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
