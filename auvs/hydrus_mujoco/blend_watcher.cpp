#include "blend_watcher.hpp"

#include <chrono>
#include <cerrno>
#include <cstring>
#include <cstdio>
#include <poll.h>
#include <sys/inotify.h>
#include <unistd.h>

namespace blend_watcher {
namespace {

std::atomic<bool> g_reload_pending{false};
std::atomic<bool> g_stop{false};
std::thread g_thread;
int g_inotify_fd = -1;

void watchLoop(std::filesystem::path blend_file) {
    g_inotify_fd = inotify_init1(IN_NONBLOCK | IN_CLOEXEC);
    if (g_inotify_fd < 0) {
        std::perror("[auv_mujoco] inotify_init");
        return;
    }
    const int watch = inotify_add_watch(
        g_inotify_fd,
        blend_file.parent_path().c_str(),
        IN_CLOSE_WRITE | IN_MOVED_TO | IN_DELETE_SELF | IN_MOVE_SELF);
    if (watch < 0) {
        std::perror("[auv_mujoco] inotify_add_watch");
        close(g_inotify_fd);
        g_inotify_fd = -1;
        return;
    }

    using clock = std::chrono::steady_clock;
    auto deadline = clock::now();
    bool armed = false;
    alignas(inotify_event) char buffer[16 * 1024];

    while (!g_stop.load()) {
        const int timeout_ms = armed ? 50 : 500;
        pollfd poll_fd{.fd = g_inotify_fd, .events = POLLIN};
        const int polled = poll(&poll_fd, 1, timeout_ms);
        if (polled > 0 && (poll_fd.revents & POLLIN) != 0) {
            while (true) {
                const ssize_t bytes = read(g_inotify_fd, buffer, sizeof(buffer));
                if (bytes < 0) {
                    if (errno == EAGAIN || errno == EINTR) break;
                    std::perror("[auv_mujoco] inotify read");
                    break;
                }
                if (bytes == 0) break;
                for (ssize_t offset = 0; offset < bytes;) {
                    const auto* event = reinterpret_cast<const inotify_event*>(buffer + offset);
                    if (event->len > 0) {
                        const std::string name(event->name);
                        if (name == blend_file.filename().string()) {
                            armed = true;
                            deadline = clock::now() + std::chrono::milliseconds(200);
                        }
                    }
                    offset += static_cast<ssize_t>(sizeof(inotify_event) + event->len);
                }
            }
        }
        if (armed && clock::now() >= deadline) {
            g_reload_pending.store(true);
            armed = false;
            std::fprintf(stdout, "[auv_mujoco] blend reload scheduled\n");
        }
    }

    if (g_inotify_fd >= 0) {
        close(g_inotify_fd);
        g_inotify_fd = -1;
    }
}

}  // namespace

void start(const std::filesystem::path& blend_file) {
    stop();
    g_stop.store(false);
    g_reload_pending.store(false);
    g_thread = std::thread(watchLoop, blend_file);
}

void stop() {
    g_stop.store(true);
    if (g_thread.joinable()) g_thread.join();
}

std::atomic<bool>& reloadPending() { return g_reload_pending; }

}  // namespace blend_watcher
