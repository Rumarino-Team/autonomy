#include "build_scene.hpp"
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/core/MaterialManager.h>
#include <Stonefish/core/NED.h>
#include <Stonefish/entities/statics/Obstacle.h>
#include <Stonefish/entities/forcefields/Ocean.h>
#include <Stonefish/entities/forcefields/Atmosphere.h>
#include <Stonefish/entities/forcefields/Uniform.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <map>
#include <stdexcept>

namespace blender_stonefish_cpp {
std::unordered_map<std::string, std::string> BuildScene(
    sf::SimulationManager& manager, const std::filesystem::path& blend,
    const std::filesystem::path& config_path, const std::filesystem::path& data) {
    // Decode and validate every selected mesh before changing the simulation.
    auto scene = ReadScene(blend, config_path);
    auto config = YAML::LoadFile(config_path.string());
    for (const auto& object : scene.objects) {
        if (!config["materials"][object.material] || !config["looks"][object.look])
            throw std::runtime_error("[blender_cpp] " + object.name + ": unknown material or look");
    }
    auto path = [&](const YAML::Node& value) -> std::string {
        if (!value || value.as<std::string>().empty()) return {};
        return (data / value.as<std::string>()).string();
    };
    std::unordered_map<std::string, std::string> materials, looks, classes;
    for (const auto& entry : config["materials"]) {
        auto name = entry.first.as<std::string>();
        auto props = entry.second;
        materials[name] = manager.getMaterialManager()->CreateMaterial(
            name, props["density"].as<double>(), props["restitution"].as<double>(), props["magnetic"].as<double>(0));
    }
    for (const auto& entry : config["looks"]) {
        auto name = entry.first.as<std::string>();
        auto props = entry.second;
        auto color = props["color"];
        looks[name] = manager.CreateLook(name,
            sf::Color(color[0].as<float>(), color[1].as<float>(), color[2].as<float>()),
            props["roughness"].as<float>(0.5f), props["metalness"].as<float>(0), props["reflectivity"].as<float>(0.5f),
            path(props["texture"]), path(props["normal_map"]));
    }
    for (const auto& pair : config["friction"]["pairs"]) {
        auto a = pair[0].as<std::string>(), b = pair[1].as<std::string>();
        if (!materials.contains(a) || !materials.contains(b)
            || !manager.SetMaterialsInteraction(materials.at(a), materials.at(b), pair[2].as<double>(), pair[3].as<double>()))
            throw std::runtime_error("[blender_cpp] invalid friction pair: " + a + "/" + b);
    }
    const auto env = config["environment"], ocean = env["ocean"], sun = env["sun"], ned = env["ned"];
    manager.getNED()->Init(ned["latitude"].as<double>(), ned["longitude"].as<double>(), 0);
    if (ocean["enabled"].as<bool>(true)) {
        auto water = manager.getMaterialManager()->CreateFluid("Water", ocean["water_density"].as<double>(), 1.308e-3, 1.55);
        manager.EnableOcean(ocean["wave_height"].as<double>(), manager.getMaterialManager()->getFluid(water));
        auto* medium = manager.getOcean();
        medium->setWaterType(ocean["jerlov"].as<double>());
        medium->SetConditions(env["atmosphere"]["temperature"].as<double>());
        medium->setParticles(ocean["particles"].as<bool>(true));
        auto current = ocean["current"]["velocity"];
        medium->AddVelocityField(new sf::Uniform(sf::Vector3(current[0].as<double>(), current[1].as<double>(), current[2].as<double>())));
    }
    manager.EnableAtmosphere();
    manager.getAtmosphere()->SetSunPosition(sun["azimuth"].as<double>(), sun["elevation"].as<double>());
    std::map<const Mesh*, std::filesystem::path> mesh_paths;
    for (const auto& object : scene.objects) {
        auto it = mesh_paths.find(object.mesh.get());
        if (it == mesh_paths.end()) {
            auto file = data / "models" / "blender_cpp" / ("mesh_" + std::to_string(mesh_paths.size()) + ".obj");
            WriteObj(*object.mesh, file);
            it = mesh_paths.emplace(object.mesh.get(), file).first;
        }
        auto* obstacle = new sf::Obstacle(object.name, it->second.string(), 1.0, sf::I4(),
                                          object.convex, materials.at(object.material), looks.at(object.look));
        const auto& p = object.position;
        const auto& r = object.rotation;
        manager.AddStaticEntity(obstacle, sf::Transform(sf::Quaternion(r[2], r[1], r[0]), sf::Vector3(p[0], p[1], p[2])));
        if (!object.cls.empty() && object.cls != "scenery") classes.emplace(obstacle->getName(), object.cls);
    }
    std::cout << "[blender_cpp] built " << scene.objects.size() << " static entities, " << mesh_paths.size()
              << " unique meshes, " << classes.size() << " tracked objects from " << blend << '\n';
    return classes;
}
}
