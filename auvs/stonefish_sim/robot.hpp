#pragma once

#include <filesystem>
#include <string>

namespace sf {
class SimulationManager;
}

bool BuildRobot(sf::SimulationManager& sim, const std::filesystem::path& dataPath, const std::string& robot);
