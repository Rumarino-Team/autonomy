#include "wire.hpp"
#include <dlfcn.h>
#include <fcntl.h>
#include <filesystem>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
extern char **environ;
using namespace core;

struct PhysicsWorld {
    const Simulation *source;

    explicit PhysicsWorld(const Simulation &m)
        : source(&m),
          reply(model_allocator(m)),
          incoming(model_allocator(m)),
          outgoing(model_allocator(m)) {
    }

    int fd = -1;
    pid_t process = -1;
    WorkerReply reply;
    WireBuffer incoming, outgoing;
    Buffer<SimBodyState> &states = reply.states;
    Buffer<SimContact> &contacts = reply.contacts;
    uint64_t &hash = reply.hash;

    ~PhysicsWorld() {
        // EOF lets the worker destroy native resources and temporary meshes
        // normally.
        if (fd >= 0) {
            ::shutdown(fd, SHUT_RDWR);
            ::close(fd);
        }
        if (process <= 0)
            return;
        for (unsigned attempt = 0; attempt < 2000; ++attempt) {
            auto result = ::waitpid(process, nullptr, WNOHANG);
            if (result == process || (result < 0 && errno == ECHILD))
                return;
            if (result < 0 && errno != EINTR)
                break;
            ::usleep(1000);
        }
        ::kill(process, SIGTERM);
        while (::waitpid(process, nullptr, 0) < 0 && errno == EINTR) {
        }
    }
};

namespace {
void update(PhysicsWorld &w) {
    receive_frame(w.fd, w.incoming);
    read_worker_reply(w.incoming, w.reply);
}

void request(PhysicsWorld &w) {
    send_frame(w.fd, w.outgoing.data(), w.outgoing.size());
    update(w);
}

SimResult world_create(const Simulation *scene, PhysicsWorld **out, char *e, uint32_t n) {
    if (!out)
        return SIM_INVALID;
    *out = nullptr;
    if (!scene)
        return SIM_INVALID;
    return guard(e, n, [&] {
        validate_model(scene);
        const Simulation *model = scene;
        require(
            model->settings.integrator == SIM_INTEGRATOR_DEFAULT,
            "Stonefish exposes its default integrator only"
        );
        require(
            model->settings.gravity.buf[0] == 0 && model->settings.gravity.buf[1] == 0 &&
                model->settings.gravity.buf[2] >= 0,
            "Stonefish requires NED gravity along +Z"
        );
        auto w = std::make_unique<PhysicsWorld>(*scene);
        int sockets[2];
        if (::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sockets))
            throw std::runtime_error("socket creation failed");
        w->fd = sockets[0];
        struct ChildFd {
            int fd;
            ~ChildFd() {
                if (fd >= 0)
                    ::close(fd);
            }
        } child{sockets[1]};
        timeval timeout{120, 0};
        setsockopt(w->fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        setsockopt(w->fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
        Dl_info info{};
        require(
            dladdr(reinterpret_cast<void *>(&world_create), &info) && info.dli_fname,
            "cannot locate Stonefish worker"
        );
        auto path =
            std::filesystem::absolute(info.dli_fname).parent_path() / "simulator_stonefish_worker";
        std::string executable = path.string(), fd = "3";
        char *args[] = {executable.data(), const_cast<char *>("--fd"), fd.data(), nullptr};
        posix_spawn_file_actions_t actions;
        require(
            posix_spawn_file_actions_init(&actions) == 0,
            "spawn action allocation failed",
            SIM_NO_MEMORY
        );
        struct Actions {
            posix_spawn_file_actions_t *p;
            ~Actions() {
                posix_spawn_file_actions_destroy(p);
            }
        } actions_owner{&actions};
        // The socket pair is CLOEXEC; dup2 clears that flag on the worker endpoint.
        if (sockets[1] == 3) {
            int replacement = fcntl(sockets[1], F_DUPFD_CLOEXEC, 4);
            require(replacement >= 0, "cannot duplicate worker socket", SIM_BACKEND_ERROR);
            ::close(child.fd);
            child.fd = replacement;
            sockets[1] = replacement;
        }
        require(
            posix_spawn_file_actions_adddup2(&actions, sockets[1], 3) == 0,
            "worker socket action failed",
            SIM_BACKEND_ERROR
        );
        require(
            posix_spawn_file_actions_addclose(&actions, sockets[1]) == 0,
            "worker close action failed",
            SIM_BACKEND_ERROR
        );
        // Keep worker diagnostics out of the host's machine-readable result stream.
        require(
            posix_spawn_file_actions_adddup2(&actions, STDERR_FILENO, STDOUT_FILENO) == 0,
            "worker diagnostics action failed",
            SIM_BACKEND_ERROR
        );
        int result = posix_spawn(&w->process, executable.c_str(), &actions, nullptr, args, environ);
        ::close(child.fd);
        child.fd = -1;
        if (result)
            throw std::runtime_error("cannot spawn Stonefish worker");
        encode_scene(*model, w->outgoing);
        send_frame(w->fd, w->outgoing.data(), w->outgoing.size());
        update(*w);
        *out = w.release();
    });
}

SimResult reset(PhysicsWorld *p, const SimBodyState *states, uint32_t count, char *e, uint32_t n) {
    if (!p || (!states && count))
        return SIM_INVALID;
    return guard(e, n, [&] {
        auto &w = *static_cast<PhysicsWorld *>(p);
        validate_reset(*w.source, states, count);
        worker_reset(w.outgoing, states, count);
        request(w);
    });
}

SimResult step(PhysicsWorld *p, const SimWrench *forces, uint32_t count, char *e, uint32_t n) {
    if (!p || (!forces && count))
        return SIM_INVALID;
    return guard(e, n, [&] {
        auto &w = *static_cast<PhysicsWorld *>(p);
        validate_wrenches(*w.source, forces, count);
        worker_step(w.outgoing, forces, count);
        request(w);
    });
}

} // namespace

extern "C" SimResult sim_backend_get_api(uint32_t version, uint32_t size, PhysicsBackendApi *api) {
    if (!api || version != PHYSICS_BACKEND_ABI_VERSION || size != sizeof(*api))
        return SIM_UNSUPPORTED;
    *api = {};
    api->struct_size = sizeof(*api);
    api->abi_version = 5;
    api->capabilities = SIM_CAP_PROCESS_ISOLATED | SIM_CAP_NATIVE_HYDRO;
    api->integrators = 1u << SIM_INTEGRATOR_DEFAULT;
    std::snprintf(api->name, sizeof api->name, "stonefish");
    std::snprintf(api->version, sizeof api->version, "worker-v2/float");
    api->world_create = world_create;
    api->world_destroy = [](PhysicsWorld *p) { delete p; };
    api->reset = reset;
    api->step = step;
    api->read_state = [](const PhysicsWorld *p, SimBodyState *out, uint32_t c, uint32_t *n) {
        return p ? copy_out(p->states, out, c, n) : SIM_INVALID;
    };
    api->read_contacts = [](const PhysicsWorld *p, SimContact *out, uint32_t c, uint32_t *n) {
        return p ? copy_out(p->contacts, out, c, n) : SIM_INVALID;
    };
    api->state_hash = [](const PhysicsWorld *p) { return p ? p->hash : uint64_t(0); };
    return SIM_OK;
}
