#include <cmath>
#include <cstdio>
#include <math.h>

#include "auv.h"

#include "actuators/Actuator.h"
#include "actuators/Thruster.h"
#include "core/ConsoleSimulationApp.h"
#include "core/GraphicalSimulationApp.h"
#include "core/Robot.h"
#include "core/ScenarioParser.h"
#include "core/SimulationManager.h"
#include "entities/Entity.h"
#include "entities/MovingEntity.h"
#include "entities/SolidEntity.h"
#include "entities/StaticEntity.h"
#include "entities/forcefields/Ocean.h"
#include "entities/forcefields/Atmosphere.h"
#include "graphics/OpenGLDataStructs.h"
#include "graphics/OpenGLPipeline.h"
#include "graphics/OpenGLTrackball.h"
#include "sensors/Sensor.h"
#include "sensors/ScalarSensor.h"
#include "sensors/VisionSensor.h"
#include "sensors/vision/Camera.h"


#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <print>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>


static std::filesystem::path dataPath = "auvs/hydrus_sim/data/";
static std::filesystem::path scenarioPath = "scenarios/hydrus_env.scn";
static constexpr sf::Scalar kStepsPerSecond = 360.0;
static constexpr sf::Scalar kPhysicsDt = sf::Scalar(1) / kStepsPerSecond;
static constexpr sf::Scalar kRealtimeFactorCap = 1.0;
static constexpr auto kMinRenderInterval = std::chrono::milliseconds(16);
static constexpr bool kConsoleApp = false;

static constexpr sf::Scalar kJerlovWater = sf::Scalar(0.28);
static constexpr sf::Scalar kSunAzimuthDeg = sf::Scalar(-63.69);
static constexpr sf::Scalar kSunElevationDeg = sf::Scalar(14.62);
static constexpr GLfloat kViewExposureEv = 0.f;
static constexpr const char* kTrackballRobotName = "HydrusAUV";

namespace
{
void ApplyViewDefaults(sf::SimulationManager* sim)
{
    sim->setSolidDisplayMode(sf::DisplayMode::GRAPHICAL);
    if(sf::Ocean* ocean = sim->getOcean())
    {
        ocean->setWaterType(kJerlovWater);
        ocean->setParticles(false);
        ocean->setRenderable(true);
    }
    if(sf::Atmosphere* atmosphere = sim->getAtmosphere())
        atmosphere->SetSunPosition(kSunAzimuthDeg, kSunElevationDeg);
    if(sf::OpenGLTrackball* trackball = sim->getTrackball())
    {
        trackball->setExposureCompensation(kViewExposureEv);
        for(size_t i = 0; sf::Robot* robot = sim->getRobot(static_cast<unsigned int>(i)); ++i)
        {
            if(robot->getName() != kTrackballRobotName)
                continue;
            if(sf::SolidEntity* base = robot->getBaseLink())
                trackball->GlueToMoving(base);
            break;
        }
    }
}

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

constexpr sf::Scalar kPi = 3.14159265358979323846;

void FillImu(sf::ScalarSensor* imu, AuvReactiveFrame& frame)
{
    frame.quat.buf[3] = 1.f;
    if(imu == nullptr || imu->getNumOfChannels() < 9)
        return;

    const sf::Scalar roll = imu->getLastValue(0);
    const sf::Scalar pitch = imu->getLastValue(1);
    const sf::Scalar yaw = imu->getLastValue(2);
    sf::Matrix3 basis;
    basis.setEulerYPR(yaw, pitch, roll);
    sf::Quaternion quat;
    basis.getRotation(quat);
    frame.quat.buf[0] = static_cast<float>(quat.x());
    frame.quat.buf[1] = static_cast<float>(quat.y());
    frame.quat.buf[2] = static_cast<float>(quat.z());
    frame.quat.buf[3] = static_cast<float>(quat.w());

    frame.gyro.buf[0] = static_cast<float>(imu->getLastValue(3));
    frame.gyro.buf[1] = static_cast<float>(imu->getLastValue(4));
    frame.gyro.buf[2] = static_cast<float>(imu->getLastValue(5));
    frame.accel.buf[0] = static_cast<float>(imu->getLastValue(6));
    frame.accel.buf[1] = static_cast<float>(imu->getLastValue(7));
    frame.accel.buf[2] = static_cast<float>(imu->getLastValue(8));
}

uint32_t ClampPixel(sf::Scalar value, unsigned limit)
{
    if(value <= 0)
        return 0;
    if(value >= sf::Scalar(limit - 1))
        return limit - 1;
    return static_cast<uint32_t>(value);
}

const char* ObjectClsName(ObjectCls cls)
{
    switch(cls)
    {
        case ObjectCls::cube:
            return "cube";
        case ObjectCls::rect:
            return "rect";
        case ObjectCls::gate:
            return "gate";
    }
    return "unknown";
}

// Same camera frame as detection_mocker: +X right, +Y down, +Z forward.
// Stonefish looks along sensor +Z with +X to the right, but sensor +Y is up
// when this camera is level (view up is -Y). Negate Y so image-down matches
// the mocker. Corners behind the camera are dropped; the rest are clamped
// into the image, which is how a partial gate still yields a box.
bool ProjectAabb(
    const sf::Transform& camera_world,
    sf::Scalar fov_h_deg,
    unsigned res_x,
    unsigned res_y,
    const sf::Vector3& aabb_min,
    const sf::Vector3& aabb_max,
    AuvPoint2u& top_left,
    AuvPoint2u& bottom_right)
{
    if(res_x < 2 || res_y < 2 || fov_h_deg <= 0)
        return false;

    const sf::Scalar tan_half_h = std::tan(fov_h_deg * kPi / sf::Scalar(360));
    if(tan_half_h <= 0)
        return false;
    const sf::Scalar focal = (sf::Scalar(res_x) * 0.5) / tan_half_h;
    const sf::Transform world_to_camera = camera_world.inverse();
    const sf::Scalar xs[2] = {aabb_min.x(), aabb_max.x()};
    const sf::Scalar ys[2] = {aabb_min.y(), aabb_max.y()};
    const sf::Scalar zs[2] = {aabb_min.z(), aabb_max.z()};

    bool any_in_front = false;
    sf::Scalar u_min = 0;
    sf::Scalar u_max = 0;
    sf::Scalar v_min = 0;
    sf::Scalar v_max = 0;
    for(sf::Scalar x : xs)
    {
        for(sf::Scalar y : ys)
        {
            for(sf::Scalar z : zs)
            {
                sf::Vector3 camera_point = world_to_camera * sf::Vector3(x, y, z);
                camera_point.setY(-camera_point.y());
                // detection_mocker range is 0.1 m to 50 m in front of the camera.
                if(camera_point.z() <= sf::Scalar(0.1) || camera_point.z() >= sf::Scalar(50))
                    continue;
                const sf::Scalar u = focal * camera_point.x() / camera_point.z() + sf::Scalar(res_x) * 0.5;
                const sf::Scalar v = focal * camera_point.y() / camera_point.z() + sf::Scalar(res_y) * 0.5;
                if(!any_in_front)
                {
                    u_min = u_max = u;
                    v_min = v_max = v;
                    any_in_front = true;
                }
                else
                {
                    u_min = std::min(u_min, u);
                    u_max = std::max(u_max, u);
                    v_min = std::min(v_min, v);
                    v_max = std::max(v_max, v);
                }
            }
        }
    }
    if(!any_in_front)
        return false;

    top_left.x = ClampPixel(u_min, res_x);
    top_left.y = ClampPixel(v_min, res_y);
    bottom_right.x = ClampPixel(u_max, res_x);
    bottom_right.y = ClampPixel(v_max, res_y);
    return bottom_right.x > top_left.x && bottom_right.y > top_left.y;
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

class SimManager : public sf::SimulationManager
{
public:
    SimManager(sf::Scalar stepsPerSecond, std::filesystem::path scenarioPath)
        : SimulationManager(stepsPerSecond),
          scenarioPath(std::move(scenarioPath))
    {
    }

    void BuildScenario() override
    {
        AuvScenarioParser parser(this);
        if(!parser.Parse(scenarioPath.string()))
        {
            std::println(stderr, "Failed to parse scenario {}", scenarioPath.string());
            return;
        }
        const auto objectClasses = std::move(parser.objectClasses);

        odometry = nullptr;
        have_odometry = false;
        imu = nullptr;
        pressure = nullptr;
        camera = nullptr;
        for(unsigned int i = 0; sf::Sensor* sensor = getSensor(i); ++i)
        {
            if(sensor->getType() != sf::SensorType::LINK)
                continue;
            auto* scalar = static_cast<sf::ScalarSensor*>(sensor);
            if(scalar->getScalarSensorType() == sf::ScalarSensorType::ODOM && odometry == nullptr)
            {
                odometry = scalar;
                have_odometry = odometry->getNumOfChannels() >= 10;
            }
            else if(scalar->getScalarSensorType() == sf::ScalarSensorType::IMU && imu == nullptr
                    && scalar->getNumOfChannels() >= 9)
            {
                imu = scalar;
            }
            else if(scalar->getScalarSensorType() == sf::ScalarSensorType::PRESSURE && pressure == nullptr)
            {
                pressure = scalar;
            }
        }
        for(unsigned int i = 0; sf::Sensor* sensor = getSensor(i); ++i)
        {
            if(sensor->getType() != sf::SensorType::VISION || camera != nullptr)
                continue;
            auto* vision = static_cast<sf::VisionSensor*>(sensor);
            if(vision->getVisionSensorType() == sf::VisionSensorType::COLOR_CAMERA)
                camera = static_cast<sf::Camera*>(vision);
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
            std::println(
                stdout,
                "[hydrus_sim] tracking '{}' as {}",
                entity->getName(),
                ObjectClsName(it->second));
        }

        thrusters.clear();
        sf::Robot* robot = getRobot(0u);
        if(robot == nullptr)
        {
            std::println(stdout, "[hydrus_sim] no robot in scenario; thrusters will not run");
        }
        else
        {
            for(size_t i = 0; sf::Actuator* actuator = robot->getActuator(i); ++i)
            {
                if(actuator->getType() != sf::ActuatorType::THRUSTER)
                    continue;
                thrusters.push_back(static_cast<sf::Thruster*>(actuator));
            }
        }

        ApplyViewDefaults(this);
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
        frame.tracking_ok = true;

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
            AuvObject object{};
            if(!FillObjectBBox(tracked.entity, object.bbox))
                continue;
            object.id = tracked.id;
            object.cls = static_cast<AuvObjectCls>(tracked.cls);
            frame.objects[frame.objects_len++] = object;
        }

        return frame;
    }

    AuvReactiveFrame getAuvReactiveFrame()
    {
        AuvReactiveFrame frame{};
        frame.timestamp = static_cast<uint64_t>(getSimulationTime() * sf::Scalar(1e9));
        FillImu(imu, frame);

        unsigned res_x = 0;
        unsigned res_y = 0;
        sf::Scalar fov_h_deg = 0;
        sf::Transform camera_world = sf::I4();
        if(camera != nullptr)
        {
            camera->getResolution(res_x, res_y);
            fov_h_deg = camera->getHorizontalFOV();
            camera_world = camera->getSensorFrame();
        }
        frame.image_width = res_x;
        frame.image_height = res_y;
        // Stonefish reports gauge pressure in Pa, so depth below the surface is P / (rho * g).
        if(pressure != nullptr && getOcean() != nullptr)
        {
            const sf::Scalar rho_g = getOcean()->getLiquid().density * getGravity().getZ();
            if(rho_g > 0)
            {
                frame.pressure_depth = static_cast<float>(pressure->getLastValue(0) / rho_g);
                frame.pressure_depth_ok = true;
            }
        }

        for(const TrackedObject& tracked : tracked_objects)
        {
            if(frame.object_len == AUV_FRAME_MAX_OBJECTS || camera == nullptr)
                break;
            sf::Vector3 aabb_min;
            sf::Vector3 aabb_max;
            tracked.entity->getAABB(aabb_min, aabb_max);
            Object2DYolo box{};
            if(!ProjectAabb(camera_world, fov_h_deg, res_x, res_y, aabb_min, aabb_max, box.top_left, box.bottom_right))
                continue;
            box.id = tracked.id;
            box.cls = static_cast<AuvObjectCls>(tracked.cls);
            frame.objects2d[frame.object_len++] = box;
        }

        return frame;
    }


    struct TrackedObject
    {
        sf::Entity* entity;
        uint32_t id;
        ObjectCls cls;
    };

    std::filesystem::path scenarioPath;
    bool have_odometry = false;
    sf::ScalarSensor* odometry = nullptr;
    sf::ScalarSensor* imu = nullptr;
    sf::ScalarSensor* pressure = nullptr;
    sf::Camera* camera = nullptr;
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
        if(lastRender.has_value() && now - *lastRender < kMinRenderInterval)
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
        if(kRealtimeFactorCap <= sf::Scalar(0))
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
        const sf::Scalar wall_budget = sim_elapsed / kRealtimeFactorCap;

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

    bool base_set = false;
    sf::Scalar base_sim = 0;
    std::chrono::steady_clock::time_point base_wall{};
};

struct SimulationContext
{
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

    g_simulation_context = new SimulationContext();
    g_simulation_context->sim = new SimManager(kStepsPerSecond, dataPath / scenarioPath);
    if(kConsoleApp)
    {
        std::println(stdout, "[hydrus_sim] app=console");
        g_simulation_context->console = new ConsoleSimApp(dataPath.string(), g_simulation_context->sim);
        g_simulation_context->console->start(kPhysicsDt);
    }
    else
    {
        std::println(stdout, "[hydrus_sim] app=graphical");
        g_simulation_context->graphical = new SimApp(
            dataPath.string(), DefaultRenderSettings(), DefaultHelperSettings(), g_simulation_context->sim);
        g_simulation_context->graphical->start(kPhysicsDt);
    }
}

static bool advance_simulation()
{
    if(g_simulation_context == nullptr)
        return false;

    g_simulation_context->sim->StepSimulation(kPhysicsDt);
    if(g_simulation_context->graphical != nullptr)
        g_simulation_context->graphical->updateGraphics();
    g_simulation_context->realtime.wait(g_simulation_context->sim->getSimulationTime());

    const bool finished =
        (g_simulation_context->graphical != nullptr
         && g_simulation_context->graphical->getState() == sf::SimulationState::FINISHED)
        || (g_simulation_context->console != nullptr
            && g_simulation_context->console->getState() == sf::SimulationState::FINISHED);
    if(finished)
    {
        std::println(stderr, "[hydrus_sim] simulation finished; restart for another run");
        auv_deinit();
        return false;
    }
    return true;
}

void auv_yield_until_next_frame(AuvFrame* frame)
{
    if(!advance_simulation())
        return;
    *frame = g_simulation_context->sim->getAuvFrame();
}

void auv_yield_until_reactive_frame(AuvReactiveFrame* frame)
{
    if(!advance_simulation())
        return;
    *frame = g_simulation_context->sim->getAuvReactiveFrame();
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
