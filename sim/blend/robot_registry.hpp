#pragma once
#include "robots/definition.hpp"
#include <string_view>

namespace sim {
// Fixed supported robots. Configuration is compiled into the reader; the asset
// hash binds it to an exact saved .blend, including mounts and visual geometry.
// These are configuration records, not a plug-in or modular robot interface.
std::span<const RobotDefinition *const> robot_definitions();
const RobotDefinition *robot_definition(std::string_view name);
std::string robot_configuration_hash(const RobotDefinition &);
std::string_view robot_asset_hash(std::string_view name);
} // namespace sim
