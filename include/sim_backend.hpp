#ifndef AUTONOMY_SIM_BACKEND_HPP
#define AUTONOMY_SIM_BACKEND_HPP

#include "sim_backend.h"
#include <cstdint>
#include <memory>
#include <utility>

/** Per-world simulation handle (replaces opaque `void *` in the C ABI). */
class SimWorld {
public:
  virtual ~SimWorld() = default;

  virtual SimResult reset(const SimBodyState *states, uint32_t state_count,
                          char *err, uint32_t err_cap) = 0;
  virtual SimResult step(const SimWrench *wrenches, uint32_t wrench_count,
                         char *err, uint32_t err_cap) = 0;
  virtual SimResult read_state(SimBodyState *states, uint32_t capacity,
                               uint32_t *count) const = 0;
  virtual SimResult read_contacts(SimContact *contacts, uint32_t capacity,
                                  uint32_t *count) const = 0;
  virtual uint64_t state_hash() const = 0;
};

/** Backend factory and metadata (replaces `SimBackendApi` function pointers). */
class SimBackend {
public:
  virtual ~SimBackend() = default;

  virtual uint32_t abi_version() const = 0;
  virtual uint32_t capabilities() const = 0;
  virtual uint32_t integrators() const = 0;
  virtual const char *name() const = 0;
  virtual const char *version() const = 0;

  virtual SimResult world_create(const SimSceneView *scene, SimWorld **world,
                               char *err, uint32_t err_cap) = 0;
  virtual void world_destroy(SimWorld *world) = 0;

  SimResult world_create(const SimSceneView *scene,
                         std::unique_ptr<SimWorld> &out, char *err,
                         uint32_t err_cap) {
    SimWorld *raw = nullptr;
    const SimResult code = world_create(scene, &raw, err, err_cap);
    if (code != SIM_OK) {
      out.reset();
      return code;
    }
    out.reset(raw);
    return SIM_OK;
  }
};

namespace sim_backend {

/** Adapts a compiled C `SimBackendApi` vtable for C++ callers. */
class FromCApi final : public SimBackend {
public:
  explicit FromCApi(SimBackendApi api) : api_(api) {}

  uint32_t abi_version() const override { return api_.abi_version; }
  uint32_t capabilities() const override { return api_.capabilities; }
  uint32_t integrators() const override { return api_.integrators; }
  const char *name() const override { return api_.name; }
  const char *version() const override { return api_.version; }

  SimResult world_create(const SimSceneView *scene, SimWorld **world, char *err,
                         uint32_t err_cap) override {
    if (!api_.world_create)
      return SIM_BACKEND_ERROR;
    void *opaque = nullptr;
    const SimResult code = api_.world_create(scene, &opaque, err, err_cap);
    if (code != SIM_OK) {
      *world = nullptr;
      return code;
    }
    *world = new CWorldHandle(api_, opaque);
    return SIM_OK;
  }

  void world_destroy(SimWorld *world) override { delete world; }

private:
  class CWorldHandle final : public SimWorld {
  public:
    CWorldHandle(SimBackendApi api, void *world) : api_(api), world_(world) {}

    ~CWorldHandle() override {
      if (world_ && api_.world_destroy)
        api_.world_destroy(world_);
    }

    SimResult reset(const SimBodyState *states, uint32_t state_count, char *err,
                    uint32_t err_cap) override {
      if (!api_.reset)
        return SIM_BACKEND_ERROR;
      return api_.reset(world_, states, state_count, err, err_cap);
    }

    SimResult step(const SimWrench *wrenches, uint32_t wrench_count, char *err,
                   uint32_t err_cap) override {
      if (!api_.step)
        return SIM_BACKEND_ERROR;
      return api_.step(world_, wrenches, wrench_count, err, err_cap);
    }

    SimResult read_state(SimBodyState *states, uint32_t capacity,
                         uint32_t *count) const override {
      if (!api_.read_state)
        return SIM_BACKEND_ERROR;
      return api_.read_state(world_, states, capacity, count);
    }

    SimResult read_contacts(SimContact *contacts, uint32_t capacity,
                            uint32_t *count) const override {
      if (!api_.read_contacts)
        return SIM_BACKEND_ERROR;
      return api_.read_contacts(world_, contacts, capacity, count);
    }

    uint64_t state_hash() const override {
      if (!api_.state_hash)
        return 0;
      return api_.state_hash(world_);
    }

  private:
    SimBackendApi api_;
    void *world_;
  };

  SimBackendApi api_;
};

/** Fills a C `SimBackendApi` from a C++ backend (for DSO export). */
inline SimResult fill_c_api(SimBackend &backend, SimBackendApi *out) {
  if (!out)
    return SIM_INVALID;
  *out = {};
  out->struct_size = static_cast<uint32_t>(sizeof(SimBackendApi));
  out->abi_version = backend.abi_version();
  out->capabilities = backend.capabilities();
  out->integrators = backend.integrators();
  std::snprintf(out->name, sizeof out->name, "%s", backend.name());
  std::snprintf(out->version, sizeof out->version, "%s", backend.version());

  struct BackendState {
    SimBackend *self;
  };

  auto *state = new BackendState{&backend};
  out->world_create = [](const SimSceneView *scene, void **world, char *err,
                         uint32_t err_cap) -> SimResult {
    auto *s = static_cast<BackendState *>(nullptr); // set below
    (void)scene;
    (void)world;
    (void)err;
    (void)err_cap;
    (void)s;
    return SIM_UNSUPPORTED;
  };
  // The C callback table needs stable function pointers; implementations
  // should use a small thunk type defined alongside their backend subclass.
  (void)state;
  return SIM_UNSUPPORTED;
}

} // namespace sim_backend

#endif
