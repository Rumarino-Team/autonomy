#include "build_scene.hpp"
#include "../stonefish_c/include/stonefish_c.h"
#include <Stonefish/core/GraphicalSimulationApp.h>
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/core/Robot.h>
#include <Stonefish/entities/Entity.h>
#include <Stonefish/actuators/Actuator.h>
#include <cmath>
#include <iostream>
#include <stdexcept>

bool BuildRobot(SfWorld*, const std::string&);
class Manager : public sf::SimulationManager {
public:
    Manager(const char* data, const char* config, const char* robot)
        : SimulationManager(360), data(data), config(config), robot(robot) {}
    ~Manager() override { sf_world_free(world); }
    void BuildScenario() override {
        sf_world_free(world);
        world = nullptr;
        classes = blender_stonefish_cpp::BuildScene(*this, data / "pool_scene.blend", config, data);
        world = sf_world_bind(this);
        sf_world_set_data_dir(world, data.c_str());
        if (!BuildRobot(world, robot)) throw std::runtime_error("robot construction failed");
    }
    std::filesystem::path data, config;
    std::string robot;
    SfWorld* world = nullptr;
    std::unordered_map<std::string, std::string> classes;
};
sf::RenderSettings Settings() {
    sf::RenderSettings settings;
    settings.windowW = 640;
    settings.windowH = 480;
    return settings;
}
class App : public sf::GraphicalSimulationApp {
public:
    App(const std::string& data, sf::SimulationManager* sim)
        : GraphicalSimulationApp("Native Blender smoke", data, Settings(), sf::HelperSettings{}, sim) {}
    ~App() override { Quit(); CleanUp(); }
    void Initialize() { autostep_ = false; timeStep_ = 1.0 / 360; setMaxPhysicsThreads(1); Init(); }
};
int main(int argc, char** argv) {
    if (argc < 3 || argc > 4) { std::cerr << "Usage: stonefish_blend_smoke data-dir config.yaml [robot]\n"; return 2; }
    try {
        Manager manager(argv[1], argv[2], argc == 4 ? argv[3] : "hydrus");
        App app(argv[1], &manager);
        app.Initialize();
        size_t statics = 0;
        for (unsigned i = 0; auto* entity = manager.getEntity(i); ++i) {
            if (entity->getType() == sf::EntityType::STATIC) ++statics;
            sf::Vector3 min, max;
            entity->getAABB(min, max);
            for (int j = 0; j < 3; ++j)
                if (!std::isfinite(min[j]) || !std::isfinite(max[j])) throw std::runtime_error("invalid entity bounds");
        }
        if (statics != 20 || manager.classes.size() != 2 || !manager.getRobot(0u)) throw std::runtime_error("unexpected pool scene contents");
        if (!manager.StartSimulation()) throw std::runtime_error("failed to solve initial conditions");
        for (int i = 0; i < 36; ++i) manager.StepSimulation(1.0 / 360);
        if (!(manager.getSimulationTime() > 0)) throw std::runtime_error("simulation did not advance");
        manager.RestartScenario();
        if (!manager.StartSimulation()) throw std::runtime_error("failed to restart simulation");
        for (int i = 0; i < 36; ++i) manager.StepSimulation(1.0 / 360);
        statics = 0;
        for (unsigned i = 0; auto* entity = manager.getEntity(i); ++i)
            if (entity->getType() == sf::EntityType::STATIC) ++statics;
        if (statics != 20 || manager.classes.size() != 2 || !manager.getRobot(0u)) throw std::runtime_error("restart did not rebuild the pool and robot");
        std::cout << "PASS: " << statics << " static entities, " << manager.classes.size()
                  << " tracked classes, robot " << manager.getRobot(0u)->getName() << ", physics advanced to "
                  << manager.getSimulationTime() << " seconds; restart also passed\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
