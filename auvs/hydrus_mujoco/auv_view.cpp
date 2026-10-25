#include <cmath>

#include "auv_view.h"

#include "core/FeatherstoneRobot.h"
#include "core/GraphicalSimulationApp.h"
#include "core/Robot.h"
#include "core/ScenarioParser.h"
#include "core/SimulationManager.h"
#include "entities/FeatherstoneEntity.h"
#include "entities/SolidEntity.h"
#include "entities/forcefields/Ocean.h"
#include "graphics/OpenGLOcean.h"
#include "graphics/OpenGLOceanParticles.h"
#include "graphics/OpenGLPipeline.h"
#include "graphics/OpenGLTrackball.h"
#include "sensors/Sensor.h"
#include "sensors/VisionSensor.h"
#include "sensors/vision/ColorCamera.h"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {

constexpr char kStonefishData[] = "auvs/stonefish_sim/data/";
constexpr auto kMinRenderInterval = std::chrono::milliseconds(16);

sf::RenderSettings defaultRenderSettings() {
    sf::RenderSettings settings;
    settings.windowW = 1200;
    settings.windowH = 900;
    settings.aa = sf::RenderQuality::HIGH;
    settings.shadows = sf::RenderQuality::HIGH;
    settings.ao = sf::RenderQuality::HIGH;
    settings.atmosphere = sf::RenderQuality::MEDIUM;
    settings.ocean = sf::RenderQuality::HIGH;
    settings.ssr = sf::RenderQuality::HIGH;
    return settings;
}

sf::HelperSettings defaultHelperSettings() {
    sf::HelperSettings settings;
    settings.showFluidDynamics = false;
    settings.showCoordSys = false;
    settings.showBulletDebugInfo = false;
    settings.showSensors = false;
    settings.showActuators = false;
    settings.showForces = false;
    return settings;
}

// Bullet is never stepped. simulationTime is private, so the post-tick
// callback is the clock that does not integrate.
class ViewManager : public sf::SimulationManager {
public:
    ViewManager(std::string scenario, std::string robot_name)
        : SimulationManager(360),
          scenario_(std::move(scenario)),
          robot_name_(std::move(robot_name)) {
    }

    void BuildScenario() override {
        const std::filesystem::path scenario = std::filesystem::path(kStonefishData) / scenario_;
        sf::ScenarioParser parser(this);
        if (!parser.Parse(scenario.string())) {
            std::fprintf(stderr, "[auv_mujoco] failed to parse %s\n", scenario.c_str());
            return;
        }

        sf::Robot* robot = getRobot(robot_name_);
        if (robot == nullptr || robot->getType() != sf::RobotType::FEATHERSTONE) {
            std::fprintf(stderr, "[auv_mujoco] scenario is missing robot '%s'\n", robot_name_.c_str());
            return;
        }
        dynamics_ = static_cast<sf::FeatherstoneRobot*>(robot)->getDynamics();
        hull_ = robot->getBaseLink();

        for (unsigned int i = 0; sf::Sensor* sensor = getSensor(i); ++i) {
            if (sensor->getType() != sf::SensorType::VISION)
                continue;
            auto* vision = static_cast<sf::VisionSensor*>(sensor);
            if (vision->getVisionSensorType() != sf::VisionSensorType::COLOR_CAMERA)
                continue;
            camera_ = static_cast<sf::ColorCamera*>(vision);
            break;
        }
        if (camera_ == nullptr)
            std::fprintf(stderr, "[auv_mujoco] scenario has no color camera\n");
    }

    void finishSetup() {
        if (hull_ != nullptr) {
            if (sf::OpenGLTrackball* trackball = getTrackball())
                trackball->GlueToMoving(hull_);
        }
        if (camera_ == nullptr)
            return;
        camera_->setDisplayOnScreen(true, 16, 16, 0.4f);
        camera_->InstallNewDataHandler([this](sf::ColorCamera* camera) { storeImage(camera); });
    }

    void syncBody(const double pos[3], const double quat_wxyz[4], const double lin[3], const double ang[3]) {
        if (dynamics_ == nullptr)
            return;
        const sf::Quaternion rotation(quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]);
        dynamics_->setBaseTransform(sf::Transform(rotation, sf::Vector3(pos[0], pos[1], pos[2])));
        if (btMultiBody* body = dynamics_->getMultiBody()) {
            body->setBaseVel(sf::Vector3(lin[0], lin[1], lin[2]));
            body->setBaseOmega(sf::Vector3(ang[0], ang[1], ang[2]));
        }
    }

    // The color camera stays disabled here so its 20 Hz period is counted
    // only by the explicit Update in the render path.
    void advanceTime(sf::Scalar dt) {
        const bool camera_was_enabled = camera_ != nullptr && camera_->isEnabled();
        if (camera_was_enabled)
            camera_->setEnabled(false);
        SimulationPostTickCallback(getDynamicsWorld(), dt);
        if (camera_was_enabled)
            camera_->setEnabled(true);
    }

    sf::ColorCamera* colorCamera() const { return camera_; }

    // The color camera's view shares the hull's ocean particles. The hull
    // deletes that set, and the ocean destructor would delete it again.
    void detachSharedParticles() {
        if (camera_ == nullptr)
            return;
        sf::Ocean* water = getOcean();
        if (water == nullptr || water->getOpenGLOcean() == nullptr)
            return;
        sf::OpenGLView* view = camera_->getOpenGLView();
        if (view == nullptr)
            return;
        water->getOpenGLOcean()->AssignParticles(view, std::make_shared<sf::OpenGLOceanParticles>(1, 1.f));
    }

private:
    void storeImage(sf::ColorCamera* camera) {
        unsigned width = 0;
        unsigned height = 0;
        camera->getResolution(width, height);
        const auto* pixels = static_cast<const std::uint8_t*>(camera->getImageDataPointer());
        if (pixels == nullptr || width == 0 || height == 0)
            return;
        const std::size_t count = static_cast<std::size_t>(width) * height * 3;
        image_.assign(pixels, pixels + count);
        image_width_ = width;
        image_height_ = height;
        if (!image_logged_) {
            std::fprintf(stdout, "[auv_mujoco] camera image %ux%u\n", image_width_, image_height_);
            std::fflush(stdout);
            image_logged_ = true;
        }
    }

    std::string scenario_;
    std::string robot_name_;
    sf::FeatherstoneEntity* dynamics_ = nullptr;
    sf::MovingEntity* hull_ = nullptr;
    sf::ColorCamera* camera_ = nullptr;
    std::vector<std::uint8_t> image_;
    unsigned image_width_ = 0;
    unsigned image_height_ = 0;
    bool image_logged_ = false;
};

class ViewApp : public sf::GraphicalSimulationApp {
public:
    ViewApp(ViewManager* manager)
        : sf::GraphicalSimulationApp(
            "auv_mujoco",
            kStonefishData,
            defaultRenderSettings(),
            defaultHelperSettings(),
            manager),
          manager_(manager) {
    }

    void start() {
        autostep_ = false;
        timeStep_ = 0;
        Init();
        StartSimulation();
        manager_->finishSetup();
    }

    int present(sf::Scalar camera_dt) {
        if (getState() == sf::SimulationState::FINISHED)
            return -1;
        const auto now = std::chrono::steady_clock::now();
        if (last_render_.has_value() && now - *last_render_ < kMinRenderInterval)
            return 0;
        last_render_ = now;

        sf::OpenGLPipeline* pipeline = getGLPipeline();
        SDL_LockMutex(pipeline->getDrawingQueueMutex());
        pipeline->PurgeDrawingQueue();
        pipeline->PurgeSelectedDrawingQueue();
        getSimulationManager()->UpdateDrawingQueue();
        SDL_UnlockMutex(pipeline->getDrawingQueueMutex());
        if (manager_->colorCamera() != nullptr)
            manager_->colorCamera()->Update(camera_dt);
        // LoopInternal polls the window and calls Render plus DrawDisplay.
        LoopInternal();
        if (getState() == sf::SimulationState::FINISHED)
            return -1;
        return 1;
    }

    void shutdown() {
        if (cleaned_)
            return;
        if (getState() != sf::SimulationState::FINISHED)
            Quit();
        CleanUp();
        cleaned_ = true;
    }

    void WindowEvent(SDL_Event* event) override {
        if (event->window.event == SDL_WINDOWEVENT_CLOSE)
            Quit();
        else
            sf::GraphicalSimulationApp::WindowEvent(event);
    }

private:
    ViewManager* manager_ = nullptr;
    bool cleaned_ = false;
    std::optional<std::chrono::steady_clock::time_point> last_render_;
};

}  // namespace

struct StonefishView {
    ViewManager* manager = nullptr;
    ViewApp* app = nullptr;
};

StonefishView* stonefish_view_create(const char* scenario, const char* robot_name) {
    if (scenario == nullptr || scenario[0] == '\0' || robot_name == nullptr || robot_name[0] == '\0') {
        std::fprintf(stderr, "[auv_mujoco] stonefish view needs a scenario and a robot name\n");
        return nullptr;
    }
    std::fprintf(stdout, "[auv_mujoco] stonefish view scenario=%s robot=%s\n", scenario, robot_name);
    std::fflush(stdout);
    auto* view = new StonefishView();
    view->manager = new ViewManager(scenario, robot_name);
    view->app = new ViewApp(view->manager);
    view->app->start();
    return view;
}

void stonefish_view_sync(
    StonefishView* view,
    const double pos[3],
    const double quat_wxyz[4],
    const double lin[3],
    const double ang[3],
    double dt) {
    if (view == nullptr || view->manager == nullptr)
        return;
    view->manager->syncBody(pos, quat_wxyz, lin, ang);
    view->manager->advanceTime(static_cast<sf::Scalar>(dt));
}

int stonefish_view_present(StonefishView* view, double camera_dt) {
    if (view == nullptr || view->app == nullptr)
        return -1;
    return view->app->present(static_cast<sf::Scalar>(camera_dt));
}

void stonefish_view_destroy(StonefishView* view) {
    if (view == nullptr)
        return;
    // The manager destructor releases ocean textures, which needs the GL
    // context that shutdown() deletes.
    if (view->manager != nullptr)
        view->manager->detachSharedParticles();
    delete view->manager;
    view->manager = nullptr;
    if (view->app != nullptr) {
        view->app->shutdown();
        delete view->app;
    }
    delete view;
}
