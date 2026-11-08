#include "runtime_internal.hpp"
#include "dynlib.h"
#include <numbers>
using namespace core;

namespace {
void read(SimulationRuntime &r) {
    char e[256];
    uint32_t count = 0;
    r.bodies.resize(r.model->object_count);
    check(
        r.backend.read_state(r.world, r.bodies.data(), uint32_t(r.bodies.size()), &count),
        "read states failed"
    );
    require(count == r.bodies.size(), "wrong state count");
    for (uint32_t i = 0; i < count; ++i) {
        auto &s = r.bodies[i];
        require(
            s.id == r.model->object_table[i]->body.id && finite(s.linear_velocity) &&
                finite(s.angular_velocity) && finite(s.center_of_mass),
            "invalid backend state"
        );
        valid_pose(s.pose);
    }
    auto result = r.backend.read_contacts(r.world, nullptr, 0, &count);
    require(result == SIM_OK || result == SIM_BUFFER_TOO_SMALL, "contact sizing failed");
    r.contacts.resize(count);
    check(
        r.backend.read_contacts(r.world, r.contacts.data(), count, &count), "read contacts failed"
    );
    require(count == r.contacts.size(), "wrong contact count");
    for (auto &c : r.contacts)
        require(
            finite(c.position) && finite(c.normal) && std::isfinite(c.normal_force),
            "invalid backend contact"
        );
    (void)e;
}

void reset(SimulationRuntime &r, const SimBodyState *s, uint32_t count) {
    require((s && count == r.model->object_count) || (!s && !count), "reset requires all states");
    char e[512];
    check(r.backend.reset(r.world, s, count, e, sizeof e), e);
    r.tick = 0;
    std::fill(std::begin(r.rotors), std::end(r.rotors), RotorState{});
    std::fill(std::begin(r.commands), std::end(r.commands), 0);
    std::fill(std::begin(r.input), std::end(r.input), 0);
    std::fill(r.fluid.begin(), r.fluid.end(), SimWrench{});
    std::fill(r.wrenches.begin(), r.wrenches.end(), SimWrench{});
    arena_reset(&r.scratch);
    for (auto &m : r.modules)
        if (m.api.reset)
            check(m.api.reset(m.state, e, sizeof e), e);
    read(r);
    for (size_t i = 0; i < r.bodies.size(); ++i) {
        r.previous_velocity[i] = r.bodies[i].linear_velocity;
        r.previous_angular_velocity[i] = r.bodies[i].angular_velocity;
    }
    sample_sensors(r);
}
} // namespace

extern "C" {
SimResult physics_backend_open(PhysicsBackend *b, const char *path, char *e, uint32_t n) {
    return guard(e, n, [&] {
        require(b && !b->library && path, "invalid backend loader arguments");
        auto *lib = dynlib_open(path);
        require(lib, "cannot load backend shared library", SIM_BACKEND_ERROR);
        try {
            auto *symbol = dynlib_symbol(lib, "sim_backend_get_api");
            require(symbol, "missing backend entrypoint", SIM_UNSUPPORTED);
            PhysicsBackendGetApi get;
            static_assert(sizeof(get) == sizeof(symbol));
            std::memcpy(&get, &symbol, sizeof get);
            PhysicsBackendApi api{};
            check(get(5, sizeof api, &api), "backend ABI v5 required");
            require(
                api.struct_size == sizeof(api) && api.abi_version == 5 && api.world_create &&
                    api.world_destroy && api.reset && api.step && api.read_state &&
                    api.read_contacts && api.state_hash,
                "incomplete backend API",
                SIM_UNSUPPORTED
            );
            b->api = api;
            b->library = lib;
        } catch (...) {
            dynlib_close(lib);
            throw;
        }
    });
}

void physics_backend_close(PhysicsBackend *b) {
    if (b) {
        if (b->library)
            dynlib_close(b->library);
        *b = {};
    }
}

SimResult simulation_runtime_create(
    const Simulation *m,
    const PhysicsBackend *b,
    const ForceRegistry *registry,
    SimulationRuntime **out,
    char *e,
    uint32_t n
) {
    if (out)
        *out = nullptr;
    return guard(e, n, [&] {
        require(out && b && b->library, "missing backend");
        require(
            b->api.struct_size == sizeof(PhysicsBackendApi) &&
                b->api.abi_version == PHYSICS_BACKEND_ABI_VERSION && b->api.world_create &&
                b->api.world_destroy && b->api.reset && b->api.step && b->api.read_state &&
                b->api.read_contacts && b->api.state_hash,
            "backend ABI v5 required",
            SIM_UNSUPPORTED
        );
        validate_model(m);
        require(
            b->api.integrators & (1u << m->settings.integrator),
            "unsupported integrator",
            SIM_UNSUPPORTED
        );
        require(
            !m->settings.native_hydrodynamics || (b->api.capabilities & SIM_CAP_NATIVE_HYDRO),
            "native hydrodynamics unsupported",
            SIM_UNSUPPORTED
        );
        auto r = std::make_unique<SimulationRuntime>(m, b);
        r->bodies.resize(m->object_count);
        r->wrenches.resize(m->object_count);
        r->fluid.resize(m->object_count);
        r->previous_velocity.resize(m->object_count);
        r->previous_angular_velocity.resize(m->object_count);
        r->samples.resize(m->sensor_count);
        r->detections.resize(bytes(m->sensor_count, m->object_count));
        r->scratch_memory.resize(model_config(*m).scratch_bytes);
        arena_init(&r->scratch, r->scratch_memory.p, r->scratch_memory.size());
        uint32_t builtin_count;
        auto *builtin = builtin_modules(builtin_count);
        require(
            !registry || (registry->count <= 4096 && (!registry->count || registry->entries)),
            "invalid force registry"
        );
        if (registry) {
            for (uint32_t i = 0; i < registry->count; ++i) {
                const auto &a = registry->entries[i];
                require(
                    std::memchr(a.name, 0, 64) && a.name[0] && std::memchr(a.contribution, 0, 64) &&
                        a.evaluate && bool(a.create) == bool(a.destroy),
                    "invalid callback descriptor"
                );
                for (uint32_t j = 0; j < i; ++j)
                    require(
                        std::strcmp(a.name, registry->entries[j].name) != 0,
                        "duplicate registered force"
                    );
                for (uint32_t j = 0; j < builtin_count; ++j)
                    require(
                        std::strcmp(a.name, builtin[j].name) != 0,
                        "builtin force cannot be replaced"
                    );
            }
        }
        auto find = [&](const char *name) -> const ForceModule * {
            for (uint32_t j = 0; j < builtin_count; ++j)
                if (!std::strcmp(name, builtin[j].name))
                    return builtin + j;
            if (registry)
                for (uint32_t j = 0; j < registry->count; ++j)
                    if (!std::strcmp(name, registry->entries[j].name))
                        return registry->entries + j;
            return nullptr;
        };
        auto add = [&](const char *name) {
            const auto *f = find(name);
            require(f, "unknown force module");
            if (m->settings.native_hydrodynamics && !std::strcmp(name, "hydrodynamics"))
                return;
            for (auto &i : r->modules) {
                require(std::strcmp(i.api.name, name) != 0, "duplicate force selection");
                require(
                    !f->contribution[0] || std::strcmp(i.api.contribution, f->contribution) != 0,
                    "duplicate physical contribution"
                );
            }
            require(
                !m->settings.native_hydrodynamics ||
                    std::strcmp(f->contribution, "hydrodynamics") != 0,
                "duplicate native hydrodynamics provider"
            );
            ModuleInstance instance{*f, nullptr, 0};
            r->modules.push_back(instance);
            auto &entry = r->modules[r->modules.size() - 1];
            if (f->create) {
                entry.created = 1;
                char msg[256];
                check(f->create(m, f->user, &entry.state, msg, sizeof msg), msg);
            }
        };
        if (m->module_count)
            for (uint32_t i = 0; i < m->module_count; ++i)
                add(m->module_table[i]->name);
        else
            for (uint32_t i = 0; i < builtin_count; ++i)
                add(builtin[i].name);
        bool gravity = false;
        for (auto &i : r->modules)
            gravity |= !std::strcmp(i.api.contribution, "gravity");
        require(gravity, "gravity compensation provider is required");
        char msg[512];
        check(r->backend.world_create(m, &r->world, msg, sizeof msg), msg);
        reset(*r, nullptr, 0);
        *out = r.release();
    });
}

void simulation_runtime_destroy(SimulationRuntime *r) {
    delete r;
}

SimResult simulation_runtime_reset(
    SimulationRuntime *r, const SimBodyState *s, uint32_t count, char *e, uint32_t n
) {
    return guard(e, n, [&] {
        require(r, "null runtime");
        reset(*r, s, count);
    });
}

SimResult simulation_runtime_step(
    SimulationRuntime *r, const double *input, uint32_t count, char *e, uint32_t n
) {
    return guard(e, n, [&] {
        require(
            r && count == r->model->robot.thruster_count && (!count || input),
            "invalid command count"
        );
        require(r->tick < UINT64_MAX, "tick overflow");
        for (uint32_t i = 0; i < count; ++i)
            require(
                std::isfinite(input[i]) && input[i] >= -1 && input[i] <= 1, "command outside [-1,1]"
            );
        for (uint32_t i = 0; i < count; ++i)
            r->commands[i] = r->input[i] = input[i];
        const auto &m = *r->model;
        for (uint32_t i = 0; i < m.fault_count; ++i) {
            auto &f = *m.fault_table[i];
            if (r->tick >= f.start && r->tick < f.end &&
                (f.kind == SIM_FAULT_DEAD || f.kind == SIM_FAULT_WEAK))
                r->commands[f.thruster] *= f.kind == SIM_FAULT_DEAD ? 0 : f.magnitude;
        }
        for (uint32_t i = 0; i < m.object_count; ++i) {
            r->wrenches[i] = {m.object_table[i]->body.id, {}, {}};
            r->previous_velocity[i] = r->bodies[i].linear_velocity;
            r->previous_angular_velocity[i] = r->bodies[i].angular_velocity;
        }
        arena_reset(&r->scratch);
        struct ScratchReset {
            ArenaAllocator *a;
            ~ScratchReset() {
                arena_reset(a);
            }
        } scratch_reset{&r->scratch};
        SimForceContext ctx{
            &m,
            r->bodies.p,
            m.object_count,
            r->commands,
            count,
            r->tick,
            double(m.settings.dt_numerator) / m.settings.dt_denominator,
            r->rotors,
            r->fluid.p,
            &r->scratch
        };
        char msg[512];
        for (auto &i : r->modules)
            check(
                i.api.evaluate(&ctx, i.state, r->wrenches.p, m.object_count, msg, sizeof msg), msg
            );
        for (auto &w : r->wrenches)
            require(finite(w.force) && finite(w.torque), "non-finite force output");
        check(r->backend.step(r->world, r->wrenches.p, m.object_count, msg, sizeof msg), msg);
        ++r->tick;
        read(*r);
        for (auto &s : r->bodies)
            require(
                norm(s.pose.pos) <= m.settings.bounds.position &&
                    norm(s.linear_velocity) <= m.settings.bounds.speed &&
                    norm(s.angular_velocity) <= m.settings.bounds.angular_speed,
                "simulation bound exceeded"
            );
        sample_sensors(*r);
    });
}

SimResult
simulation_runtime_states(const SimulationRuntime *r, const SimBodyState **p, uint32_t *n) {
    if (!r || !p || !n)
        return SIM_INVALID;
    *p = r->bodies.p;
    *n = uint32_t(r->bodies.size());
    return SIM_OK;
}

SimResult
simulation_runtime_contacts(const SimulationRuntime *r, const SimContact **p, uint32_t *n) {
    if (!r || !p || !n)
        return SIM_INVALID;
    *p = r->contacts.p;
    *n = uint32_t(r->contacts.size());
    return SIM_OK;
}

SimResult simulation_runtime_rotors(const SimulationRuntime *r, const RotorState **p, uint32_t *n) {
    if (!r || !p || !n)
        return SIM_INVALID;
    *p = r->rotors;
    *n = r->model->robot.thruster_count;
    return SIM_OK;
}

SimResult
simulation_runtime_wrenches(const SimulationRuntime *r, const SimWrench **p, uint32_t *n) {
    if (!r || !p || !n)
        return SIM_INVALID;
    *p = r->wrenches.p;
    *n = uint32_t(r->wrenches.size());
    return SIM_OK;
}

uint64_t simulation_runtime_tick(const SimulationRuntime *r) {
    return r ? r->tick : 0;
}

uint64_t simulation_runtime_hash(const SimulationRuntime *r) {
    if (!r)
        return 0;
    Hash h;
    h.scalar(r->tick);
    h.scalar(r->backend.state_hash(r->world));
    for (auto &s : r->bodies)
        h.state(s);
    for (uint32_t i = 0; i < r->model->robot.thruster_count; ++i) {
        auto &a = r->rotors[i];
        h.scalar(a.omega);
        h.scalar(a.integral);
        h.scalar(a.torque);
        h.scalar(a.thrust);
        h.scalar(r->commands[i]);
    }
    for (auto &w : r->fluid) {
        h.vec(w.force);
        h.vec(w.torque);
    }
    for (auto &v : r->previous_velocity)
        h.vec(v);
    for (auto &v : r->previous_angular_velocity)
        h.vec(v);
    hash_sensors(*r, h);
    return h.value;
}
}

extern "C" SimResult
simulation_runtime_observe(SimulationRuntime *r, Observation *out, char *e, uint32_t n) {
    return guard(e, n, [&] {
        require(r && out, "null observation arguments");
        *out = r->observation;
    });
}

extern "C" SimResult simulation_runtime_sensor_samples(
    const SimulationRuntime *r, const SensorSample **out, uint32_t *count
) {
    if (!r || !out || !count)
        return SIM_INVALID;
    *out = r->samples.data();
    *count = uint32_t(r->samples.size());
    return SIM_OK;
}
