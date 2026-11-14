#include "scene_presets.h"
#include "data_binary.hpp"
using V = autonomy::data::Value;
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {
void expect(bool condition, const char *message) {
    if (!condition)
        throw std::runtime_error(message);
}

bool near(float actual, float expected) {
    return std::fabs(actual - expected) < 1e-4f;
}

const UnderwaterBody *find_body(const Simulation &scene, uint32_t id) {
    for (uint32_t i = 0; i < scene.object_count; ++i)
        if (scene.object_table[i]->body.id == id)
            return &scene.object_table[i]->body;
    return nullptr;
}
} // namespace

int main(int argc, char **argv) {
    if (argc > 1) {
        const Robot *robot = sim_robot_find("hydrus");
        char error[512] = {};
        Simulation loaded{};
        SimResult result = simulation_scene_load(&loaded, argv[1], robot, error, sizeof error);
        if (result != SIM_OK) {
            std::cerr << (error[0] ? error : "snapshot load failed") << '\n';
            return 1;
        }
        const UnderwaterBody *marker = find_body(loaded, 3);
        expect(marker, "exported marker");
        std::cout << "MARKER " << marker->initial_pose.pos.buf[0] << " "
                  << marker->initial_pose.pos.buf[1] << " " << marker->initial_pose.pos.buf[2]
                  << '\n';
        expect(
            !(near(marker->initial_pose.pos.buf[0], 0.9985f) &&
              near(marker->initial_pose.pos.buf[1], 20.1525f) &&
              near(marker->initial_pose.pos.buf[2], 1.1675f)),
            "marker must come from the blend snapshot, not the compiled box"
        );
        simulation_destroy(&loaded);
        std::cout << "POOL_SNAPSHOT_OK\n";
        return 0;
    }
    const auto path = std::filesystem::temp_directory_path() / "autonomy-snapshot-test.bin";
    try {
        {

            autonomy::data::write(
                path.string().c_str(),
                6,
                V::object(
                    {{"version", 1},
                     {"vehicle", "hydrus"},
                     {"bodies",
                      V::array(
                          {V::object(
                               {{"id", 1},
                                {"name", "hydrus"},
                                {"class", 0},
                                {"motion", 1},
                                {"tracked", false},
                                {"fuzz", false},
                                {"profile", "hydrus"},
                                {"pose",
                                 V::object(
                                     {{"position", V::array({1, 2, 3})},
                                      {"orientation", V::array({0, 0, 0, 1})}}
                                 )},
                                {"shapes", V::array({})}}
                           ),
                           V::object(
                               {{"id", 100},
                                {"name", "Pool west wall"},
                                {"class", 3},
                                {"motion", 0},
                                {"tracked", true},
                                {"fuzz", false},
                                {"pose",
                                 V::object(
                                     {{"position", V::array({4, 5, 6})},
                                      {"orientation", V::array({0, 0, 0, 1})}}
                                 )},
                                {"shapes",
                                 V::array({V::object(
                                     {{"kind", 0},
                                      {"collisions", true},
                                      {"pose",
                                       V::object(
                                           {{"position", V::array({0, 0, 0})},
                                            {"orientation", V::array({0, 0, 0, 1})}}
                                       )},
                                      {"size", V::array({0.15, 14.3, 1.64})}}
                                 )})}}
                           )}
                      )}}
                )
            );
        }
        const Robot *robot = sim_robot_find("hydrus");
        expect(robot, "missing hydrus preset");
        char error[512] = {};
        Simulation loaded{};
        SimResult result =
            simulation_scene_load(&loaded, path.string().c_str(), robot, error, sizeof error);
        if (result != SIM_OK)
            throw std::runtime_error(error[0] ? error : "snapshot load failed");
        const auto &vehicle = loaded.robot.object.body.initial_pose.pos;
        expect(
            near(vehicle.buf[0], 1.f) && near(vehicle.buf[1], 2.f) && near(vehicle.buf[2], 3.f),
            "vehicle pose"
        );
        const UnderwaterBody *wall = find_body(loaded, 100);
        expect(wall && std::strcmp(wall->name, "Pool west wall") == 0, "wall identity");
        expect(wall->cls == 3 && wall->tracked == 1, "wall class");
        expect(
            near(wall->initial_pose.pos.buf[0], 4.f) && near(wall->initial_pose.pos.buf[1], 5.f) &&
                near(wall->initial_pose.pos.buf[2], 6.f),
            "wall pose"
        );
        const ObjectBlock *wall_object = nullptr;
        for (uint32_t i = 0; i < loaded.object_count; ++i)
            if (loaded.object_table[i]->body.id == 100)
                wall_object = loaded.object_table[i];
        expect(
            wall_object && wall_object->collider_count == 1 &&
                wall_object->colliders[0].data.collisions == 1,
            "wall collision"
        );
        simulation_destroy(&loaded);

        Simulation preset{};
        result = simulation_scene_create(&preset, "robosub_course", robot, error, sizeof error);
        if (result != SIM_OK)
            throw std::runtime_error(error[0] ? error : "preset create failed");
        const UnderwaterBody *marker = find_body(preset, 3);
        expect(marker, "preset marker");
        expect(
            near(marker->initial_pose.pos.buf[0], 0.9985f) &&
                near(marker->initial_pose.pos.buf[1], 20.1525f) &&
                near(marker->initial_pose.pos.buf[2], 1.1675f),
            "preset marker pose"
        );
        simulation_destroy(&preset);
        std::filesystem::remove(path);
        std::cout << "SNAPSHOT_OK\n";
        return 0;
    } catch (const std::exception &exception) {
        std::cerr << exception.what() << '\n';
        std::filesystem::remove(path);
        return 1;
    }
}
