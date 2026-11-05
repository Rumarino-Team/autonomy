#include "wire.hpp"
#include <dlfcn.h>
#include <fcntl.h>
#include <memory>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
extern char **environ;
namespace {
using namespace sim;
struct Model {
  Json source;
};
struct World {
  int fd = -1;
  pid_t process = -1;
  std::vector<SimBodyState> states;
  std::vector<SimContact> contacts;
  uint64_t hash = 0;
  ~World() {
    if (fd >= 0)
      ::close(fd);
    if (process > 0) {
      ::kill(process, SIGTERM);
      while (::waitpid(process, nullptr, 0) < 0 && errno == EINTR) {
      }
    }
  }
};
void update(World &w, const Json &reply) {
  if (reply.contains("error"))
    throw std::runtime_error(reply.at("error").get<std::string>());
  w.states.clear();
  w.contacts.clear();
  for (auto &s : reply.at("states"))
    w.states.push_back(state(s));
  for (auto &c : reply.at("contacts"))
    w.contacts.push_back({c.at("a"), c.at("b"), vec(c.at("position")),
                          vec(c.at("normal")), c.at("force")});
  w.hash = reply.at("hash");
}
void request(World &w, const Json &message) {
  send_message(w.fd, message);
  update(w, receive_message(w.fd));
}
SimResult model_create(const SimModelDesc *d, void **out, char *e, uint32_t n) {
  if (!out)
    return SIM_INVALID;
  *out = nullptr;
  return guarded(e, n, [&] {
    validate(d);
    require(d->integrator == SIM_INTEGRATOR_DEFAULT,
            "Stonefish exposes its default integrator only");
    require(d->gravity.x == 0 && d->gravity.y == 0 && d->gravity.z >= 0,
            "Stonefish requires NED gravity along +Z");
    auto m = std::make_unique<Model>();
    m->source = encode(*d);
    *out = m.release();
  });
}
SimResult world_create(const void *model, void **out, char *e, uint32_t n) {
  if (!out || !model)
    return SIM_INVALID;
  *out = nullptr;
  return guarded(e, n, [&] {
    auto w = std::make_unique<World>();
    int sockets[2];
    if (::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sockets))
      throw std::system_error(errno, std::generic_category());
    w->fd = sockets[0];
    timeval timeout{120, 0};
    setsockopt(w->fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(w->fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    Dl_info info{};
    require(dladdr(reinterpret_cast<void *>(&world_create), &info) &&
                info.dli_fname,
            "cannot locate Stonefish worker");
    auto path = std::filesystem::absolute(info.dli_fname).parent_path() /
                "sim_stonefish_worker";
    std::string executable = path.string(), fd = "3";
    char *args[] = {executable.data(), const_cast<char *>("--fd"), fd.data(),
                    nullptr};
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    // The socket pair is CLOEXEC; dup2 clears that flag on the worker endpoint.
    if (sockets[1] == 3) {
      int replacement = fcntl(sockets[1], F_DUPFD_CLOEXEC, 4);
      ::close(sockets[1]);
      sockets[1] = replacement;
    }
    posix_spawn_file_actions_adddup2(&actions, sockets[1], 3);
    posix_spawn_file_actions_addclose(&actions, sockets[1]);
    int result = posix_spawn(&w->process, executable.c_str(), &actions, nullptr,
                             args, environ);
    posix_spawn_file_actions_destroy(&actions);
    ::close(sockets[1]);
    if (result)
      throw std::system_error(result, std::generic_category(),
                              "spawn Stonefish worker");
    request(*w, {{"version", 1},
                 {"op", "init"},
                 {"model", static_cast<const Model *>(model)->source}});
    *out = w.release();
  });
}
SimResult reset(void *p, const SimBodyState *states, uint32_t count, char *e,
                uint32_t n) {
  if (!p || (!states && count))
    return SIM_INVALID;
  return guarded(e, n, [&] {
    Json j = {{"version", 1}, {"op", "reset"}, {"states", Json::array()}};
    for (uint32_t i = 0; i < count; ++i)
      j["states"].push_back(json(states[i]));
    request(*static_cast<World *>(p), j);
  });
}
SimResult step(void *p, const SimWrench *forces, uint32_t count, char *e,
               uint32_t n) {
  if (!p || (!forces && count))
    return SIM_INVALID;
  return guarded(e, n, [&] {
    Json j = {{"version", 1}, {"op", "step"}, {"forces", Json::array()}};
    for (uint32_t i = 0; i < count; ++i)
      j["forces"].push_back({{"id", forces[i].id},
                             {"force", json(forces[i].force)},
                             {"torque", json(forces[i].torque)}});
    request(*static_cast<World *>(p), j);
  });
}
} // namespace
extern "C" SimResult sim_backend_get_api(uint32_t version, uint32_t size,
                                         SimBackendApi *api) {
  try {
    if (!api || version != SIM_BACKEND_ABI_VERSION || size != sizeof(*api))
      return SIM_UNSUPPORTED;
    *api = {};
    api->struct_size = sizeof(*api);
    api->abi_version = version;
    api->capabilities = SIM_CAP_PROCESS_ISOLATED | SIM_CAP_NATIVE_HYDRO;
    api->integrators = 1u << SIM_INTEGRATOR_DEFAULT;
    std::snprintf(api->name, sizeof(api->name), "stonefish");
    Dl_info identity{};
    if (!dladdr(reinterpret_cast<void *>(&world_create), &identity) ||
        !identity.dli_fname)
      return SIM_BACKEND_ERROR;
    auto worker = std::filesystem::path(identity.dli_fname).parent_path() /
                  "sim_stonefish_worker";
    auto adjacent = worker.parent_path() / "libStonefish.so";
    auto engine = file_digest(std::filesystem::is_regular_file(adjacent)
                                  ? adjacent.c_str()
                                  : SIM_ENGINE_LIBRARY_PATH),
         binary = file_digest(worker.c_str());
    if (!engine || !binary)
      return SIM_BACKEND_ERROR;
    std::snprintf(api->version, sizeof(api->version),
                  "stonefish/%016llx/%016llx",
                  static_cast<unsigned long long>(engine),
                  static_cast<unsigned long long>(binary));
    api->model_create = model_create;
    api->model_destroy = [](void *p) { delete static_cast<Model *>(p); };
    api->world_create = world_create;
    api->world_destroy = [](void *p) { delete static_cast<World *>(p); };
    api->reset = reset;
    api->step = step;
    api->read_state = [](const void *p, SimBodyState *out, uint32_t c,
                         uint32_t *n) {
      return p ? copy_out(static_cast<const World *>(p)->states, out, c, n)
               : SIM_INVALID;
    };
    api->read_contacts = [](const void *p, SimContact *out, uint32_t c,
                            uint32_t *n) {
      return p ? copy_out(static_cast<const World *>(p)->contacts, out, c, n)
               : SIM_INVALID;
    };
    api->state_hash = [](const void *p) {
      return p ? static_cast<const World *>(p)->hash : 0;
    };
    return SIM_OK;
  } catch (...) {
    return SIM_BACKEND_ERROR;
  }
}
