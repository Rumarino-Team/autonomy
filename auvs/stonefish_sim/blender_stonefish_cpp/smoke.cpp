#include "build_scene.hpp"
#include "../robot.hpp"
#include <Stonefish/core/GraphicalSimulationApp.h>
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/core/Robot.h>
#include <Stonefish/entities/Entity.h>
#include <Stonefish/entities/solids/Compound.h>
#include <Stonefish/actuators/Actuator.h>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

class Manager : public sf::SimulationManager {
public:
    Manager(const char* data, const char* config, const char* robot)
        : SimulationManager(360), data(data), config(config), robot(robot) {}
    void BuildScenario() override {
        classes = blender_stonefish_cpp::BuildScene(*this, data / "pool_scene.blend", config, data);
        if (!BuildRobot(*this, data, robot)) throw std::runtime_error("robot construction failed");
    }
    void ReleaseScenario() {
        DestroyScenario();
        // Upstream DestroyScenario deletes the world without clearing its
        // pointer. Prevent the manager destructor from deleting it twice.
        dynamicsWorld = nullptr;
    }
    std::filesystem::path data, config;
    std::string robot;
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
    ~App() override {
        Quit();
        // Destroy bodies while the graphics context and app still exist. The
        // manager outlives this app and its destructor may release graphics.
        static_cast<Manager*>(getSimulationManager())->ReleaseScenario();
        CleanUp();
    }
    void Initialize() { autostep_ = false; timeStep_ = 1.0 / 360; setMaxPhysicsThreads(1); Init(); }
};
int main(int argc, char** argv) {
    if (argc < 3 || argc > 4) { std::cerr << "Usage: stonefish_blend_smoke data-dir config.yaml [robot]\n"; return 2; }
    try {
        Manager manager(argv[1], argv[2], argc == 4 ? argv[3] : "hydrus");
        App app(argv[1], &manager);
        app.Initialize();
        if(manager.robot == "proteus") {
            auto* robot = manager.getRobot(0u);
            auto* body = dynamic_cast<sf::Compound*>(robot->getBaseLink());
            if(!body) throw std::runtime_error("Proteus must remain a compound body");
            double mass = 0, volume = 0;
            sf::Vector3 cg(0, 0, 0);
            size_t external = 0, appearance = 0;
            for(size_t i = 0; ; ++i) {
                const auto part = body->getPart(i);
                if(!part.solid) break;
                const auto pmass = part.solid->getMass();
                if(part.isExternal) {
                    ++external;
                    mass += pmass;
                    cg += (part.origin * part.solid->getCG2OTransform().inverse()).getOrigin() * pmass;
                    if(part.solid->isBuoyant()) volume += part.solid->getVolume();
                } else {
                    ++appearance;
                    if(pmass != 0 || part.solid->isBuoyant())
                        throw std::runtime_error("appearance changed Proteus mass or buoyancy");
                }
            }
            if(external != 6 || appearance != 12 || !body->isDisplayingInternalParts())
                throw std::runtime_error("unexpected Proteus proxy/appearance structure");
            if(std::abs(body->getMass() - mass) > 1e-9 || std::abs(body->getVolume() - volume) > 1e-9
                || (body->getCG2OTransform().inverse().getOrigin() - cg / mass).length() > 1e-9)
                throw std::runtime_error("appearance changed inherited body properties");
            size_t actuators = 0;
            while(robot->getActuator(actuators)) ++actuators;
            if(actuators != 6) throw std::runtime_error("Proteus actuator ordering/count changed");
            const auto center = body->getCG2OTransform().inverse().getOrigin();
            const auto inertia = body->getInertia();
            std::cout << std::setprecision(12) << "PROTEUS_PHYSICS mass_kg=" << mass
                      << " volume_m3=" << volume << " cg_body_m="
                      << center.x() << ',' << center.y() << ',' << center.z()
                      << " principal_inertia_kg_m2=" << inertia.x() << ',' << inertia.y() << ',' << inertia.z()
                      << " external_proxies=" << external << " appearance_groups=" << appearance
                      << " actuators=" << actuators << '\n';
        }
        size_t statics = 0;
        for (unsigned i = 0; auto* entity = manager.getEntity(i); ++i) {
            if (entity->getType() == sf::EntityType::STATIC) ++statics;
            sf::Vector3 min, max;
            entity->getAABB(min, max);
            for (int j = 0; j < 3; ++j)
                if (!std::isfinite(min[j]) || !std::isfinite(max[j])) throw std::runtime_error("invalid entity bounds");
        }
        // The user's pool is editable. Compare reload against this loaded scene,
        // rather than assuming the original checked-in obstacle count.
        const auto initialStatics = statics;
        const auto initialClasses = manager.classes;
        if (statics == 0 || manager.classes.empty() || !manager.getRobot(0u)) throw std::runtime_error("unexpected pool scene contents");
        if (!manager.StartSimulation()) throw std::runtime_error("failed to solve initial conditions");
        for (int i = 0; i < 36; ++i) manager.StepSimulation(1.0 / 360);
        if (!(manager.getSimulationTime() > 0)) throw std::runtime_error("simulation did not advance");
        manager.RestartScenario();
        if (!manager.StartSimulation()) throw std::runtime_error("failed to restart simulation");
        for (int i = 0; i < 36; ++i) manager.StepSimulation(1.0 / 360);
        statics = 0;
        for (unsigned i = 0; auto* entity = manager.getEntity(i); ++i)
            if (entity->getType() == sf::EntityType::STATIC) ++statics;
        if (statics != initialStatics || manager.classes != initialClasses || !manager.getRobot(0u)) throw std::runtime_error("restart did not rebuild the pool and robot");
        std::cout << "PASS: " << statics << " static entities, " << manager.classes.size()
                  << " tracked classes, robot " << manager.getRobot(0u)->getName() << ", physics advanced to "
                  << manager.getSimulationTime() << " seconds; restart also passed\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
