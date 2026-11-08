// Exercise the public C application ABI, including unloading every generation
// while one native MuJoCo world remains alive throughout the mission.
#include "application.h"
#include "auv_simulator.h"
#include "scene_presets.h"
#include "dynlib.h"
#include "../data_binary.hpp"
#include <filesystem>
#include <iostream>
#include <memory>
#include <set>
#include <stdexcept>
#include <vector>

namespace {
void require(bool ok, const char *what) {
    if (!ok)
        throw std::runtime_error(what);
}

bool cancelled() {
    return false;
}

void telemetry(const uint8_t *, size_t) {
}

bool teleop(float *) {
    return false;
}

struct Generation {
    void *library = nullptr;
    const AutonomyApplicationApi *api = nullptr;
    void *instance = nullptr;
    std::filesystem::path path;

    void close() {
        if (instance) {
            api->destroy(instance);
            instance = nullptr;
        }
        if (library) {
            dynlib_close(library);
            library = nullptr;
        }
        if (!path.empty())
            std::filesystem::remove(path);
    }

    ~Generation() {
        close();
    }

    void load(const char *source, unsigned index, const AutonomyHostServices &services) {
        path = std::filesystem::temp_directory_path() /
               ("autonomy-app-" + std::to_string(reinterpret_cast<uintptr_t>(this)) + "-" +
                std::to_string(index) + ".so");
        std::filesystem::copy_file(source, path, std::filesystem::copy_options::overwrite_existing);
        library = dynlib_open(path.string().c_str());
        require(library, "open application");
        using Get = const AutonomyApplicationApi *(*)(uint32_t);
        auto get = reinterpret_cast<Get>(dynlib_symbol(library, "autonomy_application_api"));
        require(get, "get API");
        require(!get(999), "reject unsupported host ABI");
        api = get(AUTONOMY_APPLICATION_ABI);
        require(api && api->struct_size == sizeof(*api), "API layout");
        char error[512]{};
        require(api->create(&services, &instance, error, sizeof error) == 0, error);
    }
};

std::vector<uint8_t> capture(Generation &g) {
    char error[512]{};
    size_t n = 0;
    require(g.api->capture(g.instance, nullptr, 0, &n, error, sizeof error) == 0, "capture length");
    require(n < 64 * 1024 * 1024, "capture limit");
    std::vector<uint8_t> b(n);
    require(g.api->capture(g.instance, b.data(), b.size(), &n, error, sizeof error) == 0, error);
    return b;
}

AuvSimulatorResult result() {
    AuvSimulatorResult s{};
    char error[512]{};
    require(auv_simulator_read_result(&s, sizeof s, error, sizeof error) == SIM_OK, error);
    return s;
}
} // namespace

int main(int argc, char **argv) {
    if (argc != 3)
        return 2;
    Simulation model{};
    bool active = false;
    try {
        char plugin_path[4096]{};
        require(
            dynlib_module_path(
                reinterpret_cast<void *>(&auv_init), plugin_path, sizeof plugin_path
            ),
            "plugin path"
        );
        const char *args[]{"application-test", "run"};
        AutonomyAuvCallbacks auv{
            auv_init,
            auv_yield_until_next_frame,
            auv_set_thrustor_values,
            auv_deinit,
            auv_simulation_status
        };
        AutonomyHostServices services{
            sizeof(services),
            AUTONOMY_APPLICATION_ABI,
            2,
            args,
            cancelled,
            telemetry,
            teleop,
            &auv,
            plugin_path,
            argv[2],
            argv[0],
            argv[1]
        };
        auto current = std::make_unique<Generation>();
        current->load(argv[1], 0, services);
        AutonomyLaunch launch{};
        current->api->describe(current->instance, &launch);
        char error[512]{};
        require(
            current->api->build_scene(current->instance, &model, error, sizeof error) == 0, error
        );
        AuvSimulatorOptions options{
            sizeof(options),
            AUV_SIMULATOR_EXTENSION_VERSION,
            argv[2],
            launch.max_ticks,
            launch.frame_ticks,
            0,
            0
        };
        require(auv_simulator_init_scene(&model, &options, error, sizeof error) == SIM_OK, error);
        active = true;
        current->api->activate(current->instance);
        std::set<std::string> stages;
        unsigned generation = 1;
        bool done = false;
        for (unsigned frame = 0; frame < 5000; ++frame) {
            const int status = current->api->update(current->instance, error, sizeof error);
            require(status >= 0, error);
            if (status == 1) {
                done = true;
                break;
            }
            if (frame % 37 != 0)
                continue;
            auto bytes = capture(*current);
            auto state = autonomy::data::decode(bytes, 1, 2);
            stages.insert(state.field("runner")->field("stage")->text);
            auto candidate = std::make_unique<Generation>();
            candidate->load(argv[1], generation++, services);
            const auto before = result();
            auto malformed = bytes;
            malformed.back() ^= 1;
            require(
                candidate->api->restore(
                    candidate->instance, malformed.data(), malformed.size(), error, sizeof error
                ) != 0,
                "reject bad checksum"
            );
            malformed = bytes;
            malformed[12] = 99; // header schema, checksum covers payload only
            require(
                candidate->api->restore(
                    candidate->instance, malformed.data(), malformed.size(), error, sizeof error
                ) != 0,
                "reject unknown schema"
            );
            if (generation % 3 == 0) {
                // Exercise schema-1 -> schema-2 migration with a real missing field.
                auto *mission = const_cast<autonomy::data::Value *>(state.field("mission"));
                for (auto i = mission->fields.begin(); i != mission->fields.end(); ++i)
                    if (i->first == "log_counter") {
                        mission->fields.erase(i);
                        break;
                    }
                autonomy::data::Writer payload;
                payload.value(state);
                autonomy::data::Writer legacy;
                for (char c : std::string("AUVDATA1"))
                    legacy.uint(uint8_t(c), 1);
                legacy.uint(1, 4);
                legacy.uint(1, 4);
                legacy.uint(payload.bytes.size(), 8);
                legacy.uint(autonomy::data::crc32(payload.bytes.data(), payload.bytes.size()), 4);
                legacy.bytes.insert(legacy.bytes.end(), payload.bytes.begin(), payload.bytes.end());
                require(
                    candidate->api->restore(
                        candidate->instance,
                        legacy.bytes.data(),
                        legacy.bytes.size(),
                        error,
                        sizeof error
                    ) == 0,
                    error
                );
            }
            require(
                candidate->api->restore(
                    candidate->instance, bytes.data(), bytes.size(), error, sizeof error
                ) == 0,
                error
            );
            current.reset();
            current = std::move(candidate);
            current->api->activate(current->instance);
            auv_simulator_rebase_pacing();
            const auto after = result();
            require(
                before.tick == after.tick && before.hash == after.hash, "reload changed physics"
            );
        }
        require(done, "mission completion");
        require(
            stages.contains("pass_first") && stages.contains("circle_second"),
            "reload both persistent maneuver phases"
        );
        auto final = result();
        require(final.tick == 14718, "pre-refactor mission ticks");
        std::cout << "APPLICATION_RELOAD_OK generations=" << generation << " tick=" << final.tick
                  << " hash=" << final.hash << "\n";
        current.reset();
        auv_deinit();
        active = false;
        simulation_destroy(&model);
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << "\n";
        if (active)
            auv_deinit();
        simulation_destroy(&model);
        return 1;
    }
}
