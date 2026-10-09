#include "blender_stonefish_cpp/build_scene.hpp"
#include "robot.hpp"

#include <cmath>
#include <cstdio>
#include <iostream>
#include <math.h>

#include "../../include/auv.h"

#include "Stonefish/actuators/Actuator.h"
#include "Stonefish/actuators/Thruster.h"
#include "Stonefish/core/ConsoleSimulationApp.h"
#include "Stonefish/core/GraphicalSimulationApp.h"
#include "Stonefish/core/Robot.h"
#include "Stonefish/core/SimulationApp.h"
#include "Stonefish/core/SimulationManager.h"
#include "Stonefish/entities/Entity.h"
#include "Stonefish/entities/MovingEntity.h"
#include "Stonefish/entities/StaticEntity.h"
#include "Stonefish/graphics/OpenGLDataStructs.h"
#include "Stonefish/graphics/OpenGLPipeline.h"
#include "Stonefish/graphics/OpenGLTrackball.h"
#include <glm/gtc/quaternion.hpp>
#include "Stonefish/sensors/Sensor.h"
#include "Stonefish/sensors/ScalarSensor.h"
#include "Stonefish/sensors/vision/Camera.h"
#include "Stonefish/sensors/vision/ColorCamera.h"


#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <exception>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <poll.h>
#include <sys/inotify.h>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include <fstream>

#ifndef AUV_PLATFORM_CONFIG
#define AUV_PLATFORM_CONFIG "auvs/stonefish_sim/platform.json"
#endif
static constexpr const char* kPlatformConfigPath = AUV_PLATFORM_CONFIG;

volatile std::sig_atomic_t g_stop_requested = 0;
static bool g_seed_goal_pending = true;
void requestStop(int)
{
    g_stop_requested = 1;
}

void stopIfRequested()
{
    if(g_stop_requested == 0)
        return;
    std::cerr << "[stonefish_sim] interrupted; shutting down" << std::endl;
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
    // Blender camera object name. Empty keeps the default trackball.
    std::string viewCamera;
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
    if(parsed_json.contains("view_camera") && !parsed_json["view_camera"].is_null())
    {
        if(!parsed_json["view_camera"].is_string())
        {
            std::cerr << "[Parser] view_camera must be a string in " << path.string() << std::endl;
            return std::nullopt;
        }
        config.viewCamera = parsed_json["view_camera"].get<std::string>();
    }


    if(config.dataPath.empty()){
        std::cerr << "[Parser] data_path is empty in " << path.string() << std::endl;
        return std::nullopt;
    }
    if(config.blendFile.empty()){
        std::cerr << "[Parser] blend_file is required in " << path.string() << std::endl;
        return std::nullopt;
    }
    if(!std::filesystem::is_directory(config.dataPath)){
        std::cerr << "[Parser] data_path is not a directory: " << config.dataPath.string() << std::endl;
        return std::nullopt;
    }
    const std::filesystem::path blend_path = config.dataPath / config.blendFile;
    if(!std::filesystem::is_regular_file(blend_path)){
        std::cerr << "[Parser] blend file not found: " << blend_path.string() << std::endl;
        return std::nullopt;
    }
    if(config.kStepsPerSecond <= sf::Scalar(0)){
        std::cerr << "[Parser] steps_per_second must be > 0 in " << path.string() << std::endl;
        return std::nullopt;
    }
    if(config.kRealtimeFactorCap < sf::Scalar(0)){
        std::cerr << "[Parser] realtime_factor_cap must be >= 0 in " << path.string() << std::endl;
        return std::nullopt;
    }
    if(config.kTrackingOkSeconds < sf::Scalar(0)){
        std::cerr << "[Parser] tracking_ok_seconds must be >= 0 in " << path.string() << std::endl;
        return std::nullopt;
    }
    if(config.kMinRenderInterval.count() < 0){
        std::cerr << "[Parser] min_render_interval_ms must be >= 0 in " << path.string() << std::endl;
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
        if(local.z() > sf::Scalar(0.05)){
            std::cout << entity->getName() << std::endl;
            return true;
        }
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

#ifndef BLENDER_STONEFISH_CONFIG
#define BLENDER_STONEFISH_CONFIG "auvs/stonefish_sim/stonefish_config.yaml"
#endif

glm::mat3 RotationAbout(const glm::vec3& axis, float angle)
{
    const float c = std::cos(angle);
    const float s = std::sin(angle);
    if(axis.x != 0.f)
        return glm::mat3(glm::vec3(1.f, 0.f, 0.f), glm::vec3(0.f, c, s), glm::vec3(0.f, -s, c));
    if(axis.y != 0.f)
        return glm::mat3(glm::vec3(c, 0.f, -s), glm::vec3(0.f, 1.f, 0.f), glm::vec3(s, 0.f, c));
    return glm::mat3(glm::vec3(c, s, 0.f), glm::vec3(-s, c, 0.f), glm::vec3(0.f, 0.f, 1.f));
}

// Blender camera local -Z looks forward and local +Y is up. Mesh import flips local Z
// and stores the object rotation as (-roll, pitch, -yaw), so the view uses that frame.
bool ViewRotation(const double location[3], const double rotation[3], glm::vec3& eye, glm::vec3& forward, glm::quat& rotation_out)
{
    const glm::mat3 object =
        RotationAbout(glm::vec3(0.f, 0.f, 1.f), static_cast<float>(-rotation[2]))
        * RotationAbout(glm::vec3(0.f, 1.f, 0.f), static_cast<float>(rotation[1]))
        * RotationAbout(glm::vec3(1.f, 0.f, 0.f), static_cast<float>(-rotation[0]));
    forward = glm::normalize(object * glm::vec3(0.f, 0.f, 1.f));
    glm::vec3 up = object * glm::vec3(0.f, 1.f, 0.f);
    up = up - glm::dot(up, forward) * forward;
    eye = glm::vec3(
        static_cast<float>(-location[0]), static_cast<float>(location[1]), static_cast<float>(-location[2]));
    if(glm::dot(up, up) < 1e-8f)
        return false;
    up = glm::normalize(up);
    const glm::vec3 side = glm::normalize(glm::cross(up, forward));
    const glm::mat3 world_from_local(side, -forward, up);
    rotation_out = glm::quat_cast(glm::transpose(world_from_local));
    return true;
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
        destroyCameraPreview();
    }

    void destroyCameraPreview()
    {
        if(cameraPreviewEventWatchInstalled)
        {
            SDL_DelEventWatch(WatchCameraPreviewEvent, this);
            cameraPreviewEventWatchInstalled = false;
        }
        if(cameraPreviewContext != nullptr)
        {
            if(cameraPreviewWindow != nullptr)
                SDL_GL_MakeCurrent(cameraPreviewWindow, cameraPreviewContext);
            if(cameraPreviewTexture != 0)
                glDeleteTextures(1, &cameraPreviewTexture);
            if(cameraPreviewBuffer != 0)
                glDeleteBuffers(1, &cameraPreviewBuffer);
            if(cameraPreviewVao != 0)
                glDeleteVertexArrays(1, &cameraPreviewVao);
            if(cameraPreviewProgram != 0)
                glDeleteProgram(cameraPreviewProgram);
            if(mainGLWindow != nullptr && mainGLContext != nullptr)
                SDL_GL_MakeCurrent(mainGLWindow, mainGLContext);
            SDL_GL_DeleteContext(cameraPreviewContext);
            cameraPreviewContext = nullptr;
        }
        if(cameraPreviewWindow != nullptr)
        {
            SDL_DestroyWindow(cameraPreviewWindow);
            cameraPreviewWindow = nullptr;
        }
        cameraPreviewTexture = 0;
        cameraPreviewBuffer = 0;
        cameraPreviewVao = 0;
        cameraPreviewProgram = 0;
        mainGLWindow = nullptr;
        mainGLContext = nullptr;
    }

    void BuildScenario() override
    {
        std::unordered_map<std::string, ObjectCls> objectClasses;
        blender_stonefish_cpp::Scene scene;
        if(pendingScene)
        {
            scene = std::move(*pendingScene);
            pendingScene.reset();
        }
        else
            scene = blender_stonefish_cpp::ReadScene(blendPath, BLENDER_STONEFISH_CONFIG);
        const auto nativeClasses = blender_stonefish_cpp::BuildScene(
            *this, scene, BLENDER_STONEFISH_CONFIG, dataPath, blendPath);
        viewCameras.clear();
        for(const auto& camera : scene.cameras)
            RememberViewCamera(camera.name, camera.location.data(), camera.rotation.data());
        builtScene = std::move(scene);
        hasBuiltScene = true;
        if(!BuildRobot(*this, dataPath, robotName))
            throw std::runtime_error("[stonefish_sim] failed to build robot " + robotName);
        for(const auto& [name, cls] : nativeClasses)
        {
            const auto mapped = ParseObjectClass(cls.c_str());
            if(!mapped)
                throw std::runtime_error("[stonefish_sim] unknown object class " + cls + " for " + name);
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

        if(auto* colorCamera = dynamic_cast<sf::ColorCamera*>(camera))
        {
            colorCamera->InstallNewDataHandler([this](sf::ColorCamera* source) {
                const auto* pixels = static_cast<const uint8_t*>(source->getImageDataPointer());
                unsigned int width = 0;
                unsigned int height = 0;
                source->getResolution(width, height);
                if(pixels == nullptr || width == 0 || height == 0)
                    return;

                std::lock_guard<std::mutex> lock(cameraPreviewFrameMutex);
                cameraPreviewWidth = width;
                cameraPreviewHeight = height;
                const size_t rowBytes = static_cast<size_t>(width) * 3;
                const size_t sourceStride = (rowBytes + 3u) & ~size_t(3u);
                cameraPreviewFrame.resize(rowBytes * height);
                for(unsigned int row = 0; row < height; ++row)
                    std::memcpy(
                        cameraPreviewFrame.data() + static_cast<size_t>(row) * rowBytes,
                        pixels + static_cast<size_t>(row) * sourceStride,
                        rowBytes);
                cameraPreviewFrameDirty.store(true);
            });
            if(sf::SimulationApp::getApp() != nullptr && sf::SimulationApp::getApp()->hasGraphics())
                createCameraPreviewWindow();
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
            std::cout << "[stonefish_sim] no robot in scenario; thrusters will not run" << std::endl;
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

    void presentCameraPreview()
    {
        if(cameraPreviewCloseRequested.exchange(false))
        {
            destroyCameraPreview();
            return;
        }
        if(cameraPreviewWindow == nullptr)
            return;
        if(!cameraPreviewFrameDirty.exchange(false))
            return;

        std::lock_guard<std::mutex> lock(cameraPreviewFrameMutex);
        if(cameraPreviewFrame.empty() || cameraPreviewWidth == 0 || cameraPreviewHeight == 0)
            return;

        if(SDL_GL_MakeCurrent(cameraPreviewWindow, cameraPreviewContext) != 0)
            return;
        glViewport(0, 0, static_cast<GLsizei>(cameraPreviewWidth), static_cast<GLsizei>(cameraPreviewHeight));
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(cameraPreviewProgram);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, cameraPreviewTexture);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RGB8,
            static_cast<GLsizei>(cameraPreviewWidth),
            static_cast<GLsizei>(cameraPreviewHeight),
            0,
            GL_RGB,
            GL_UNSIGNED_BYTE,
            cameraPreviewFrame.data());
        glBindVertexArray(cameraPreviewVao);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        SDL_GL_SwapWindow(cameraPreviewWindow);
        SDL_GL_MakeCurrent(mainGLWindow, mainGLContext);
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
            std::cout << "[stonefish_sim] tracking_ok false after " << trackingOkSeconds << " s" << std::endl;
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
    SDL_Window* cameraPreviewWindow = nullptr;
    SDL_Window* mainGLWindow = nullptr;
    SDL_GLContext mainGLContext = nullptr;
    SDL_GLContext cameraPreviewContext = nullptr;
    GLuint cameraPreviewTexture = 0;
    GLuint cameraPreviewBuffer = 0;
    GLuint cameraPreviewVao = 0;
    GLuint cameraPreviewProgram = 0;
    std::atomic<bool> cameraPreviewCloseRequested{false};
    std::atomic<bool> cameraPreviewFrameDirty{false};
    bool cameraPreviewEventWatchInstalled = false;
    std::mutex cameraPreviewFrameMutex;
    std::vector<uint8_t> cameraPreviewFrame;
    unsigned int cameraPreviewWidth = 0;
    unsigned int cameraPreviewHeight = 0;
    struct ViewCamera
    {
        std::string name;
        glm::vec3 eye{0.f};
        glm::vec3 forward{0.f, 1.f, 0.f};
        glm::quat rotation{1.f, 0.f, 0.f, 0.f};
    };
    std::vector<ViewCamera> viewCameras;

    static int WatchCameraPreviewEvent(void* userdata, SDL_Event* event)
    {
        auto* self = static_cast<SimManager*>(userdata);
        if(self != nullptr && self->cameraPreviewWindow != nullptr
           && event->type == SDL_WINDOWEVENT
           && event->window.event == SDL_WINDOWEVENT_CLOSE
           && event->window.windowID == SDL_GetWindowID(self->cameraPreviewWindow))
            self->cameraPreviewCloseRequested.store(true);
        return 1;
    }

    void createCameraPreviewWindow()
    {
        if(cameraPreviewWindow != nullptr)
            return;
        mainGLWindow = SDL_GL_GetCurrentWindow();
        mainGLContext = SDL_GL_GetCurrentContext();
        const std::string title = robotName + " Camera";
        cameraPreviewWindow = SDL_CreateWindow(
            title.c_str(),
            SDL_WINDOWPOS_UNDEFINED,
            SDL_WINDOWPOS_UNDEFINED,
            static_cast<int>(image_width),
            static_cast<int>(image_height),
            SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
        if(cameraPreviewWindow == nullptr)
        {
            std::cerr << "[stonefish_sim] could not create camera preview window: " << SDL_GetError() << std::endl;
            return;
        }
        cameraPreviewContext = SDL_GL_CreateContext(cameraPreviewWindow);
        if(cameraPreviewContext == nullptr)
        {
            std::cerr << "[stonefish_sim] could not create camera preview context: " << SDL_GetError() << std::endl;
            SDL_GL_MakeCurrent(mainGLWindow, mainGLContext);
            SDL_DestroyWindow(cameraPreviewWindow);
            cameraPreviewWindow = nullptr;
            return;
        }
        static constexpr char vertexShaderSource[] =
            "#version 430 core\n"
            "layout(location=0) in vec2 position;\n"
            "layout(location=1) in vec2 texCoordIn;\n"
            "out vec2 texCoord;\n"
            "void main(){ gl_Position=vec4(position,0.0,1.0); texCoord=texCoordIn; }\n";
        static constexpr char fragmentShaderSource[] =
            "#version 430 core\n"
            "in vec2 texCoord;\n"
            "uniform sampler2D cameraImage;\n"
            "out vec4 color;\n"
            "void main(){ color=vec4(texture(cameraImage,texCoord).rgb,1.0); }\n";
        auto compileShader = [](GLenum type, const char* source) -> GLuint {
            const GLuint shader = glCreateShader(type);
            glShaderSource(shader, 1, &source, nullptr);
            glCompileShader(shader);
            GLint compiled = GL_FALSE;
            glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
            if(compiled == GL_FALSE)
            {
                glDeleteShader(shader);
                return 0;
            }
            return shader;
        };
        const GLuint vertexShader = compileShader(GL_VERTEX_SHADER, vertexShaderSource);
        const GLuint fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentShaderSource);
        if(vertexShader == 0 || fragmentShader == 0)
        {
            std::cerr << "[stonefish_sim] could not compile camera preview shaders" << std::endl;
            if(vertexShader != 0)
                glDeleteShader(vertexShader);
            if(fragmentShader != 0)
                glDeleteShader(fragmentShader);
            SDL_GL_MakeCurrent(mainGLWindow, mainGLContext);
            SDL_GL_DeleteContext(cameraPreviewContext);
            cameraPreviewContext = nullptr;
            SDL_DestroyWindow(cameraPreviewWindow);
            cameraPreviewWindow = nullptr;
            return;
        }
        cameraPreviewProgram = glCreateProgram();
        glAttachShader(cameraPreviewProgram, vertexShader);
        glAttachShader(cameraPreviewProgram, fragmentShader);
        glLinkProgram(cameraPreviewProgram);
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        GLint linked = GL_FALSE;
        glGetProgramiv(cameraPreviewProgram, GL_LINK_STATUS, &linked);
        if(linked == GL_FALSE)
        {
            std::cerr << "[stonefish_sim] could not link camera preview shader program" << std::endl;
            glDeleteProgram(cameraPreviewProgram);
            cameraPreviewProgram = 0;
            SDL_GL_MakeCurrent(mainGLWindow, mainGLContext);
            SDL_GL_DeleteContext(cameraPreviewContext);
            cameraPreviewContext = nullptr;
            SDL_DestroyWindow(cameraPreviewWindow);
            cameraPreviewWindow = nullptr;
            return;
        }
        static constexpr GLfloat vertices[] = {
            -1.f, -1.f, 0.f, 1.f,
             1.f, -1.f, 1.f, 1.f,
             1.f,  1.f, 1.f, 0.f,
            -1.f, -1.f, 0.f, 1.f,
             1.f,  1.f, 1.f, 0.f,
            -1.f,  1.f, 0.f, 0.f,
        };
        glGenVertexArrays(1, &cameraPreviewVao);
        glBindVertexArray(cameraPreviewVao);
        glGenBuffers(1, &cameraPreviewBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, cameraPreviewBuffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), reinterpret_cast<void*>(2 * sizeof(GLfloat)));
        glGenTextures(1, &cameraPreviewTexture);
        glBindTexture(GL_TEXTURE_2D, cameraPreviewTexture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glUseProgram(cameraPreviewProgram);
        glUniform1i(glGetUniformLocation(cameraPreviewProgram, "cameraImage"), 0);
        SDL_GL_SetSwapInterval(0);
        SDL_GL_MakeCurrent(mainGLWindow, mainGLContext);
        SDL_AddEventWatch(WatchCameraPreviewEvent, this);
        cameraPreviewEventWatchInstalled = true;
    }

    void RememberViewCamera(const std::string& name, const double location[3], const double rotation[3])
    {
        for(const auto& camera : viewCameras)
        {
            if(camera.name != name)
                continue;
            std::cerr << "[stonefish_sim] duplicate view camera '" << name << "'" << std::endl;
            return;
        }
        ViewCamera camera;
        camera.name = name;
        if(!ViewRotation(location, rotation, camera.eye, camera.forward, camera.rotation))
        {
            std::cerr << "[stonefish_sim] skipping view camera '" << name << "': look direction and up are parallel"
                      << std::endl;
            return;
        }
        viewCameras.push_back(std::move(camera));
    }
    // Set by a reload before RestartScenario so BuildScenario does not parse the blend again.
    std::optional<blender_stonefish_cpp::Scene> pendingScene;
    blender_stonefish_cpp::Scene builtScene;
    bool hasBuiltScene = false;
};

class SimApp : public sf::GraphicalSimulationApp
{
public:
    SimApp(std::string dataPath, sf::RenderSettings render_setting, sf::HelperSettings helper_settings, sf::SimulationManager* sim_manager)
        : sf::GraphicalSimulationApp("stonefish_sim", dataPath, render_setting, helper_settings, sim_manager)
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
        : sf::ConsoleSimulationApp("stonefish_sim", dataPath, sim_manager)
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

constexpr unsigned kReloadPlatform = 1u << 0;
constexpr unsigned kReloadBlend = 1u << 1;
constexpr unsigned kReloadYaml = 1u << 2;
constexpr unsigned kReloadTextures = 1u << 3;
constexpr unsigned kReloadAll = kReloadPlatform | kReloadBlend | kReloadYaml | kReloadTextures;
constexpr auto kReloadDebounce = std::chrono::milliseconds(200);

struct SimulationContext
{
    PlatformConfig config;
    SimManager* sim = nullptr;
    SimApp* graphical = nullptr;
    ConsoleSimApp* console = nullptr;
    RealtimeThrottle realtime;
    std::atomic<bool> watcherStop{false};
    std::atomic<bool> scenarioReload{false};
    std::atomic<unsigned> reloadReasons{0};
    std::atomic<bool> watchPathsChanged{false};
    std::mutex watchMutex;
    std::filesystem::path watchedBlend;
    std::filesystem::path watchedTextures;
    std::thread watcher;
};

struct DirWatch
{
    int wd = -1;
    std::filesystem::path dir;
    struct Rule
    {
        std::string name;
        unsigned reason = 0;
        bool any_file = false;
    };
    std::vector<Rule> rules;
};

std::filesystem::path WatchDir(const std::filesystem::path& path)
{
    const auto dir = path.parent_path();
    return dir.empty() ? std::filesystem::path(".") : dir;
}

void AddFileRule(std::vector<DirWatch>& watches, const std::filesystem::path& file, unsigned reason)
{
    const auto dir = WatchDir(file);
    for(auto& watch : watches)
    {
        if(watch.dir != dir)
            continue;
        watch.rules.push_back({file.filename().string(), reason, false});
        return;
    }
    DirWatch watch;
    watch.dir = dir;
    watch.rules.push_back({file.filename().string(), reason, false});
    watches.push_back(std::move(watch));
}

void ArmWatches(int fd, std::vector<DirWatch>& watches, const std::filesystem::path& blend, const std::filesystem::path& textures)
{
    for(const auto& watch : watches)
        if(watch.wd >= 0)
            inotify_rm_watch(fd, watch.wd);
    watches.clear();

    AddFileRule(watches, kPlatformConfigPath, kReloadPlatform);
    if(!blend.empty())
        AddFileRule(watches, blend, kReloadBlend);
    AddFileRule(watches, BLENDER_STONEFISH_CONFIG, kReloadYaml);
    std::error_code textures_error;
    if(std::filesystem::is_directory(textures, textures_error))
    {
        DirWatch watch;
        watch.dir = textures;
        watch.rules.push_back({{}, kReloadTextures, true});
        watches.push_back(std::move(watch));
    }

    static std::unordered_set<std::string> logged_failures;
    for(auto& watch : watches)
    {
        watch.wd = inotify_add_watch(fd, watch.dir.c_str(), IN_CLOSE_WRITE | IN_MOVED_TO);
        if(watch.wd < 0)
        {
            if(logged_failures.insert(watch.dir.string()).second)
                std::cerr << "[stonefish_sim] inotify watch failed for " << watch.dir.string() << ": " << std::strerror(errno) << std::endl;
        }
        else
            logged_failures.erase(watch.dir.string());
    }
}

bool AllWatchesArmed(const std::vector<DirWatch>& watches)
{
    if(watches.empty())
        return false;
    for(const auto& watch : watches)
        if(watch.wd < 0)
            return false;
    return true;
}

unsigned ReasonsForEvent(const std::vector<DirWatch>& watches, const inotify_event* event)
{
    if((event->mask & IN_Q_OVERFLOW) != 0)
        return kReloadAll;
    if((event->mask & (IN_CLOSE_WRITE | IN_MOVED_TO)) == 0 || (event->mask & IN_ISDIR) != 0 || event->len == 0)
        return 0;
    const std::string name(event->name);
    for(const auto& watch : watches)
    {
        if(watch.wd != event->wd)
            continue;
        unsigned reasons = 0;
        for(const auto& rule : watch.rules)
            if(rule.any_file || rule.name == name)
                reasons |= rule.reason;
        return reasons;
    }
    return 0;
}

void scenarioWatchLoop(SimulationContext* ctx)
{
    const int fd = inotify_init1(IN_NONBLOCK | IN_CLOEXEC);
    if(fd < 0)
    {
        std::cerr << "[stonefish_sim] inotify_init failed: " << std::strerror(errno) << std::endl;
        return;
    }
    const struct CloseFd
    {
        int fd;
        ~CloseFd() { if(fd >= 0) close(fd); }
    } close_fd{fd};

    std::vector<DirWatch> watches;
    unsigned pending = 0;
    auto deadline = std::chrono::steady_clock::time_point::max();
    alignas(inotify_event) char buffer[16 * 1024];

    while(!ctx->watcherStop.load())
    {
        if(ctx->watchPathsChanged.exchange(false) || !AllWatchesArmed(watches))
        {
            std::filesystem::path blend;
            std::filesystem::path textures;
            {
                std::lock_guard<std::mutex> lock(ctx->watchMutex);
                blend = ctx->watchedBlend;
                textures = ctx->watchedTextures;
            }
            ArmWatches(fd, watches, blend, textures);
        }

        const auto now = std::chrono::steady_clock::now();
        if(pending != 0 && now >= deadline)
        {
            ctx->reloadReasons.fetch_or(pending);
            ctx->scenarioReload.store(true);
            pending = 0;
            deadline = std::chrono::steady_clock::time_point::max();
        }

        int timeout_ms = 50;
        if(pending != 0)
        {
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
            if(remaining < timeout_ms)
                timeout_ms = remaining < 0 ? 0 : static_cast<int>(remaining);
        }
        pollfd poll_fd{fd, POLLIN, 0};
        const int polled = poll(&poll_fd, 1, timeout_ms);
        if(ctx->watcherStop.load())
            break;
        if(polled < 0)
        {
            if(errno == EINTR)
                continue;
            std::cerr << "[stonefish_sim] inotify poll failed: " << std::strerror(errno) << std::endl;
            break;
        }
        if(polled == 0 || (poll_fd.revents & POLLIN) == 0)
            continue;

        while(true)
        {
            const ssize_t bytes = read(fd, buffer, sizeof(buffer));
            if(bytes < 0)
            {
                if(errno == EAGAIN || errno == EINTR)
                    break;
                std::cerr << "[stonefish_sim] inotify read failed: " << std::strerror(errno) << std::endl;
                break;
            }
            if(bytes == 0)
                break;
            for(ssize_t offset = 0; offset < bytes;)
            {
                const auto* event = reinterpret_cast<const inotify_event*>(buffer + offset);
                const unsigned reasons = ReasonsForEvent(watches, event);
                if(reasons != 0)
                {
                    pending |= reasons;
                    deadline = std::chrono::steady_clock::now() + kReloadDebounce;
                }
                offset += static_cast<ssize_t>(sizeof(inotify_event) + event->len);
            }
        }
    }
}

void noteWatchedFiles(SimulationContext* ctx)
{
    std::lock_guard<std::mutex> lock(ctx->watchMutex);
    ctx->watchedBlend = ctx->sim->blendPath;
    ctx->watchedTextures = ctx->sim->dataPath / "textures";
    ctx->watchPathsChanged.store(true);
}

void startScenarioWatcher(SimulationContext* ctx)
{
    ctx->watcherStop.store(false);
    ctx->scenarioReload.store(false);
    ctx->reloadReasons.store(0);
    noteWatchedFiles(ctx);
    ctx->watcher = std::thread(scenarioWatchLoop, ctx);
}

void stopScenarioWatcher(SimulationContext* ctx)
{
    if(!ctx->watcher.joinable())
        return;
    ctx->watcherStop.store(true);
    ctx->watcher.join();
}

bool SceneIdentityChanged(const PlatformConfig& current, const PlatformConfig& loaded)
{
    return current.robot != loaded.robot || current.dataPath != loaded.dataPath || current.blendFile != loaded.blendFile;
}

void ApplyViewCamera(SimulationContext* ctx)
{
    if(ctx == nullptr || ctx->graphical == nullptr || ctx->config.viewCamera.empty())
        return;
    auto* trackball = ctx->sim->getTrackball();
    if(trackball == nullptr)
        return;
    const SimManager::ViewCamera* selected = nullptr;
    for(const auto& camera : ctx->sim->viewCameras)
    {
        if(camera.name == ctx->config.viewCamera)
            selected = &camera;
    }
    if(selected == nullptr)
    {
        std::cerr << "[stonefish_sim] view camera '" << ctx->config.viewCamera << "' was not found. Blender cameras:";
        if(ctx->sim->viewCameras.empty())
            std::cerr << " (none)";
        for(const auto& camera : ctx->sim->viewCameras)
            std::cerr << ' ' << camera.name;
        std::cerr << std::endl;
        return;
    }
    glm::vec3 center(0.f);
    glm::quat rotation(1.f, 0.f, 0.f, 0.f);
    GLfloat radius = 5.f;
    trackball->getOrbit(center, rotation, radius);
    center = selected->eye + selected->forward * radius;
    trackball->setOrbit(center, selected->rotation, radius);
    std::cout << "[stonefish_sim] view camera " << selected->name << std::endl;
}

void ApplyRuntimeParams(SimulationContext* ctx, const PlatformConfig& loaded)
{
    ctx->config.kPhysicsDt = loaded.kPhysicsDt;
    ctx->config.kStepsPerSecond = loaded.kStepsPerSecond;
    ctx->config.kRealtimeFactorCap = loaded.kRealtimeFactorCap;
    ctx->config.kTrackingOkSeconds = loaded.kTrackingOkSeconds;
    ctx->config.kMinRenderInterval = loaded.kMinRenderInterval;
    const bool camera_changed = ctx->config.viewCamera != loaded.viewCamera;
    ctx->config.viewCamera = loaded.viewCamera;
    ctx->config.kBBoxOnlyInFrontOfCamera = loaded.kBBoxOnlyInFrontOfCamera;
    ctx->sim->trackingOkSeconds = loaded.kTrackingOkSeconds;
    ctx->sim->bboxOnlyInFrontOfCamera = loaded.kBBoxOnlyInFrontOfCamera;
    ctx->sim->setStepsPerSecond(loaded.kStepsPerSecond);
    ctx->realtime.realtimeFactorCap = loaded.kRealtimeFactorCap;
    ctx->realtime.base_set = false;
    if(ctx->graphical != nullptr)
    {
        ctx->graphical->minRenderInterval = loaded.kMinRenderInterval;
        ctx->graphical->setTimeStep(loaded.kPhysicsDt);
    }
    if(ctx->console != nullptr)
        ctx->console->setTimeStep(loaded.kPhysicsDt);
    if(camera_changed)
        ApplyViewCamera(ctx);
}

bool SameVec(const blender_stonefish_cpp::Vec3& a, const blender_stonefish_cpp::Vec3& b)
{
    for(size_t i = 0; i < a.size(); ++i)
        if(std::abs(a[i] - b[i]) > 1e-8)
            return false;
    return true;
}

bool SameAssets(const blender_stonefish_cpp::Scene& previous, const blender_stonefish_cpp::Scene& next)
{
    if(previous.objects.size() != next.objects.size())
        return false;
    std::unordered_map<std::string, const blender_stonefish_cpp::Object*> by_name;
    for(const auto& object : previous.objects)
        if(!by_name.emplace(object.name, &object).second)
            return false;
    for(const auto& object : next.objects)
    {
        const auto it = by_name.find(object.name);
        if(it == by_name.end() || it->second->mesh == nullptr || object.mesh == nullptr)
            return false;
        const auto& prior = *it->second;
        if(prior.material != object.material || prior.look != object.look || prior.cls != object.cls || prior.convex != object.convex)
            return false;
        if(blender_stonefish_cpp::MeshFingerprint(*prior.mesh) != blender_stonefish_cpp::MeshFingerprint(*object.mesh))
            return false;
    }
    return true;
}

sf::Transform ObjectTransform(const blender_stonefish_cpp::Object& object)
{
    const auto& position = object.position;
    const auto& rotation = object.rotation;
    return sf::Transform(sf::Quaternion(rotation[2], rotation[1], rotation[0]), sf::Vector3(position[0], position[1], position[2]));
}

// Applies pose edits only after every named static is found, so a miss leaves the world untouched.
std::optional<size_t> ApplyStaticPoses(
    SimManager* sim, const blender_stonefish_cpp::Scene& previous, const blender_stonefish_cpp::Scene& next)
{
    struct Update
    {
        sf::StaticEntity* entity;
        sf::Transform transform;
    };
    std::vector<Update> updates;
    for(const auto& object : next.objects)
    {
        const blender_stonefish_cpp::Object* prior = nullptr;
        for(const auto& candidate : previous.objects)
        {
            if(candidate.name == object.name)
            {
                prior = &candidate;
                break;
            }
        }
        if(prior != nullptr && SameVec(prior->position, object.position) && SameVec(prior->rotation, object.rotation))
            continue;
        sf::StaticEntity* static_entity = nullptr;
        for(unsigned int i = 0; sf::Entity* candidate = sim->getEntity(i); ++i)
        {
            if(candidate->getName() != object.name)
                continue;
            static_entity = dynamic_cast<sf::StaticEntity*>(candidate);
            break;
        }
        if(static_entity == nullptr)
            return std::nullopt;
        updates.push_back({static_entity, ObjectTransform(object)});
    }
    for(const auto& update : updates)
        update.entity->setTransform(update.transform);
    return updates.size();
}

bool reloadScenario(SimulationContext* ctx)
{
    // Clear only the bits we observed so a save that lands mid-reload is kept for the next frame.
    const unsigned reasons = ctx->reloadReasons.load();
    if(reasons == 0)
        return false;
    ctx->reloadReasons.fetch_and(~reasons);

    std::optional<PlatformConfig> loaded;
    try
    {
        loaded = loadPlatformConfig(kPlatformConfigPath);
    }
    catch(const std::exception& ex)
    {
        std::cerr << "[stonefish_sim] platform reload failed: " << ex.what() << std::endl;
        return false;
    }
    if(!loaded)
    {
        std::cerr << "[stonefish_sim] platform reload failed; keeping scenario" << std::endl;
        return false;
    }
    if(loaded->kConsoleApp != ctx->config.kConsoleApp)
    {
        std::cerr << "[stonefish_sim] console change ignored while the app is running" << std::endl;
        loaded->kConsoleApp = ctx->config.kConsoleApp;
    }

    const bool identity_changed = SceneIdentityChanged(ctx->config, *loaded);
    const bool structural = identity_changed || (reasons & (kReloadYaml | kReloadTextures)) != 0;
    std::optional<blender_stonefish_cpp::Scene> decoded;
    if((reasons & (kReloadBlend | kReloadYaml)) != 0 || identity_changed)
    {
        try
        {
            // Reject a bad blend before any live entity is moved or destroyed.
            decoded = blender_stonefish_cpp::ReadScene(loaded->dataPath / loaded->blendFile, BLENDER_STONEFISH_CONFIG);
        }
        catch(const std::exception& ex)
        {
            std::cerr << "[stonefish_sim] native scene reload rejected; keeping scenario: " << ex.what() << std::endl;
            return false;
        }
    }

    if(!structural && !decoded)
    {
        ApplyRuntimeParams(ctx, *loaded);
        std::cout << "[stonefish_sim] updated simulation timing without reloading meshes" << std::endl;
        return false;
    }
    if(!structural && decoded && ctx->sim->hasBuiltScene && SameAssets(ctx->sim->builtScene, *decoded))
    {
        const std::optional<size_t> moved = ApplyStaticPoses(ctx->sim, ctx->sim->builtScene, *decoded);
        if(moved)
        {
            ctx->sim->viewCameras.clear();
            for(const auto& camera : decoded->cameras)
                ctx->sim->RememberViewCamera(camera.name, camera.location.data(), camera.rotation.data());
            ApplyRuntimeParams(ctx, *loaded);
            if(!ctx->config.viewCamera.empty())
                ApplyViewCamera(ctx);
            ctx->sim->builtScene = std::move(*decoded);
            ctx->sim->hasBuiltScene = true;
            if(*moved == 0)
                std::cout << "[stonefish_sim] blend reload found no mesh or pose changes" << std::endl;
            else
                std::cout << "[stonefish_sim] updated " << *moved << " static poses without reloading meshes" << std::endl;
            return false;
        }
        ctx->sim->pendingScene = std::move(decoded);
    }
    else if(decoded)
        ctx->sim->pendingScene = std::move(decoded);
    else if(ctx->sim->hasBuiltScene)
        ctx->sim->pendingScene = ctx->sim->builtScene;

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
    std::cout << "[stonefish_sim] reloading scenario robot=" << sim->robotName << " blend=" << sim->blendPath.string() << std::endl;
    glm::vec3 savedCenter(0.f);
    glm::quat savedRotation(1.f, 0.f, 0.f, 0.f);
    GLfloat savedRadius = 5.f;
    const bool keepTrackball = ctx->graphical != nullptr && sim->getTrackball() != nullptr;
    if(keepTrackball)
        sim->getTrackball()->getOrbit(savedCenter, savedRotation, savedRadius);
    try { sim->RestartScenario(); }
    catch(const std::exception& ex)
    {
        std::cerr << "[stonefish_sim] native scene restart failed: " << ex.what() << std::endl;
        requestStop(0);
        return false;
    }
    if(keepTrackball && sim->getTrackball() != nullptr)
        sim->getTrackball()->setOrbit(savedCenter, savedRotation, savedRadius);
    if(!ctx->config.viewCamera.empty())
        ApplyViewCamera(ctx);
    if(!sim->StartSimulation())
        std::cerr << "[stonefish_sim] scenario restart failed to solve initial conditions" << std::endl;
    g_seed_goal_pending = true;
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
    try
    {
    auv_deinit();
    // SDL otherwise installs its own SIGINT handler and Ctrl+C never returns the terminal.
    SDL_SetHint(SDL_HINT_NO_SIGNAL_HANDLERS, "1");
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);

    const std::optional<PlatformConfig> loadedJson = loadPlatformConfig(kPlatformConfigPath);
    if(!loadedJson)
    {
        std::cerr << "[stonefish_sim] platform config failed; simulation not started" << std::endl;
        return;
    }

    g_seed_goal_pending = true;
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
        std::cout << "[stonefish_sim] app=console" << std::endl;
        g_simulation_context->console = new ConsoleSimApp(config.dataPath.string(), g_simulation_context->sim);
        g_simulation_context->console->start(config.kPhysicsDt);
    }
    else
    {
        std::cout << "[stonefish_sim] app=graphical" << std::endl;
        g_simulation_context->graphical = new SimApp(
            config.dataPath.string(), DefaultRenderSettings(), DefaultHelperSettings(), g_simulation_context->sim);
        g_simulation_context->graphical->minRenderInterval = config.kMinRenderInterval;
        g_simulation_context->graphical->start(config.kPhysicsDt);
        ApplyViewCamera(g_simulation_context);
    }
    startScenarioWatcher(g_simulation_context);
    }
    catch(const std::exception& ex)
    {
        std::cerr << "[stonefish_sim] native initialization failed: " << ex.what() << std::endl;
        auv_deinit();
    }
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
    stopIfRequested();

    g_simulation_context->sim->StepSimulation(g_simulation_context->config.kPhysicsDt);
    if(g_simulation_context->graphical != nullptr)
    {
        g_simulation_context->graphical->updateGraphics();
        g_simulation_context->sim->presentCameraPreview();
    }
    g_simulation_context->realtime.wait(g_simulation_context->sim->getSimulationTime());

    const bool finished =
        (g_simulation_context->graphical != nullptr
         && g_simulation_context->graphical->getState() == sf::SimulationState::FINISHED)
        || (g_simulation_context->console != nullptr
            && g_simulation_context->console->getState() == sf::SimulationState::FINISHED);
    stopIfRequested();
    if(finished)
    {
        std::cerr << "[stonefish_sim] simulation finished; shutting down" << std::endl;
        auv_deinit();
        std::exit(0);
    }
    *frame = g_simulation_context->sim->getAuvFrame();
    // A reload also requests a new goal. Report the restart first so the mission
    // drops the old sim clock; the goal is seeded on the following frame.
    if(scenarioRestarted)
        frame->error = AUV_ERROR_SCENARIO_RESTART;
    else if(g_seed_goal_pending)
    {
        g_seed_goal_pending = false;
        frame->error = AUV_ERROR_SEED_GOAL;
    }
    else
        frame->error = AUV_ERROR_NONE;
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
        g_simulation_context->sim->destroyCameraPreview();
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
