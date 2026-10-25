#pragma once

#include <atomic>
#include <filesystem>
#include <thread>

namespace blend_watcher {

void start(const std::filesystem::path& blend_file);
void stop();
std::atomic<bool>& reloadPending();

}  // namespace blend_watcher
