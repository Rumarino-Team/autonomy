#pragma once
#include "internal.hpp"
using namespace core;

struct ModuleInstance {
    ForceModule api;
    void *state;
    uint32_t created;
};

struct SimulationRuntime {
    const Simulation *model;
    PhysicsBackendApi backend;
    PhysicsWorld *world = nullptr;
    Buffer<SimBodyState> bodies;
    Buffer<SimContact> contacts;
    Buffer<SimWrench> wrenches, fluid;
    Buffer<MathVector3f> previous_velocity, previous_angular_velocity;
    Buffer<SensorSample> samples;
    Buffer<ObjectObservation> detections;
    Observation observation{};
    Buffer<ModuleInstance> modules;
    Buffer<uint8_t> scratch_memory;
    ArenaAllocator scratch{};
    RotorState rotors[8]{};
    double commands[8]{}, input[8]{};
    uint64_t tick = 0;

    SimulationRuntime(const Simulation *m, const PhysicsBackend *b)
        : model(m),
          backend(b->api),
          bodies(model_config(*m).allocator),
          contacts(model_config(*m).allocator),
          wrenches(model_config(*m).allocator),
          fluid(model_config(*m).allocator),
          previous_velocity(model_config(*m).allocator),
          previous_angular_velocity(model_config(*m).allocator),
          samples(model_config(*m).allocator),
          detections(model_config(*m).allocator),
          modules(model_config(*m).allocator),
          scratch_memory(model_config(*m).allocator) {
    }

    ~SimulationRuntime() {
        if (world)
            backend.world_destroy(world);
        for (size_t i = modules.size(); i > 0; --i) {
            auto &m = modules[i - 1];
            if (m.created && m.api.destroy)
                m.api.destroy(m.state);
        }
    }
};

namespace core {
void sample_sensors(SimulationRuntime &);
void hash_sensors(const SimulationRuntime &, Hash &);
} // namespace core
