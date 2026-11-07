#include "auv_simulator.h"
#include "scene_presets.h"
#include <atomic>
#include <cassert>
#include <cstdlib>
#include <thread>
#include <vector>
#include <cstring>

namespace {
struct Allocations {
    size_t live = 0, calls = 0;
    bool fail = false;

    static void *allocate(void *u, size_t size, size_t alignment) {
        auto &a = *static_cast<Allocations *>(u);
        if (a.fail)
            return nullptr;
        void *p = nullptr;
        if (posix_memalign(&p, alignment < sizeof(void *) ? sizeof(void *) : alignment, size))
            return nullptr;
        ++a.live;
        ++a.calls;
        return p;
    }

    static void release(void *u, void *p, size_t, size_t) {
        if (p) {
            --static_cast<Allocations *>(u)->live;
            free(p);
        }
    }
};

AuvSimulatorResult run(const char *backend, unsigned index) {
    char error[512]{};
    Simulation model{};
    auto *robot = sim_robot_find("hydrus");
    SimulationConfig config;
    assert(simulation_robot_config(robot, &config, error, sizeof error) == SIM_OK);
    Allocations allocations;
    config.allocator = {&allocations, Allocations::allocate, Allocations::release};
    assert(simulation_init(&model, &config, error, sizeof error) == SIM_OK);
    assert(simulation_add_robot(&model, robot, 1, nullptr, error, sizeof error) == SIM_OK);
    ObjectBlock *object = nullptr;
    assert(
        simulation_draft_object(&model, 99, &object, error, sizeof error) == SIM_INVALID && !object
    );
    assert(simulation_draft_object(&model, 1, &object, error, sizeof error) == SIM_OK);
    object->body.initial_pose.pos.buf[0] = float(index) * .01f;
    assert(simulation_finalize(&model, error, sizeof error) == SIM_OK);
    assert(
        simulation_draft_object(&model, 1, &object, error, sizeof error) == SIM_INVALID && !object
    );
    AuvSimulatorOptions options{
        sizeof(options), AUV_SIMULATOR_EXTENSION_VERSION, backend, 120, 6, 0, 0
    };
    options.version = 0;
    assert(auv_simulator_init_scene(&model, &options, error, sizeof error) == SIM_UNSUPPORTED);
    options.version = AUV_SIMULATOR_EXTENSION_VERSION;
    const size_t model_allocations = allocations.live;
    allocations.fail = true;
    assert(auv_simulator_init_scene(&model, &options, error, sizeof error) == SIM_NO_MEMORY);
    allocations.fail = false;
    assert(allocations.live == model_allocations);
    assert(simulation_validate(&model, error, sizeof error) == SIM_OK);
    assert(auv_simulator_init_scene(&model, &options, error, sizeof error) == SIM_OK);
    float commands[8]{};
    commands[index % 8] = .02f;
    auv_set_thrustor_values(commands, 8);
    AuvFrame frame;
    for (unsigned i = 0; i < 4; ++i)
        auv_yield_until_next_frame(&frame);
    const size_t warm = allocations.calls;
    for (unsigned i = 0; i < 8; ++i)
        auv_yield_until_next_frame(&frame);
    assert(allocations.calls == warm);
    AuvSimulatorResult result{};
    assert(auv_simulator_read_result(&result, sizeof result, error, sizeof error) == SIM_OK);
    assert(result.status == 0 && result.tick == 72 && result.has_final_state && result.hash);
    assert(auv_simulator_read_result(&result, 0, error, sizeof error) == SIM_UNSUPPORTED);
    auv_deinit();
    // Adapter destruction must not destroy the borrowed model.
    assert(simulation_validate(&model, error, sizeof error) == SIM_OK);
    simulation_destroy(&model);
    assert(allocations.live == 0);
    return result;
}
} // namespace

int main(int argc, char **argv) {
    assert(argc == 2);
    // Match the Zig coordinator: keep engine code and its immutable native
    // registration caches alive until all threads have joined.
    PhysicsBackend pinned{};
    char error[512]{};
    assert(physics_backend_open(&pinned, argv[1], error, sizeof error) == SIM_OK);
    AuvSimulatorResult reference[8];
    for (unsigned i = 0; i < 8; ++i)
        reference[i] = run(argv[1], i);
    std::vector<std::thread> workers;
    std::atomic<unsigned> ready{0};
    for (unsigned i = 0; i < 8; ++i)
        workers.emplace_back([&, i] {
            ready.fetch_add(1);
            while (ready.load() != 8)
                std::this_thread::yield();
            for (unsigned repeat = 0; repeat < 3; ++repeat) {
                auto result = run(argv[1], i);
                assert(result.tick == reference[i].tick && result.hash == reference[i].hash);
                assert(
                    result.min_depth == reference[i].min_depth &&
                    result.max_depth == reference[i].max_depth
                );
                assert(!memcmp(
                    &result.robot.pose.pos.buf,
                    &reference[i].robot.pose.pos.buf,
                    sizeof result.robot.pose.pos.buf
                ));
            }
        });
    for (auto &worker : workers)
        worker.join();
    physics_backend_close(&pinned);
}
