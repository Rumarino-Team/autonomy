#include "robots.h"
#include "internal.hpp"
using namespace core;

int main() {
    // SI wrench allocation at zero inflow. Columns include the configured shaft
    // sign, COM lever arm, and reaction torque used by the runtime rotor model.
    const auto &r = *sim_robot_find("hydrus");
    SimulationSettings settings;
    simulation_settings_default(&settings);
    double columns[8][6]{}, augmented[6][12]{};
    for (uint32_t i = 0; i < r.thruster_count; ++i) {
        const auto &t = r.thrusters[i];
        const auto &s = t.spec;
        const double maximum = settings.water_density * std::pow(s.diameter, 4) * s.thrust *
                               std::pow(s.max_rpm / 60, 2);
        const auto force = t.direction * (s.command_sign * maximum);
        const auto moment = cross(t.position - r.object.body.center_of_mass.pos, force) -
                            force * (s.diameter * s.torque / s.thrust);
        for (uint32_t k = 0; k < 3; ++k) {
            columns[i][k] = force.buf[k];
            columns[i][k + 3] = moment.buf[k];
        }
    }
    for (unsigned i = 0; i < 6; ++i) {
        augmented[i][i + 6] = 1;
        for (unsigned j = 0; j < 6; ++j)
            for (uint32_t k = 0; k < r.thruster_count; ++k)
                augmented[i][j] += columns[k][i] * columns[k][j];
    }
    for (unsigned k = 0; k < 6; ++k) {
        unsigned pivot = k;
        for (unsigned i = k + 1; i < 6; ++i)
            if (std::abs(augmented[i][k]) > std::abs(augmented[pivot][k]))
                pivot = i;
        if (std::abs(augmented[pivot][k]) < 1e-10)
            return 1;
        for (unsigned j = 0; j < 12; ++j)
            std::swap(augmented[k][j], augmented[pivot][j]);
        const double factor = augmented[k][k];
        for (double &v : augmented[k])
            v /= factor;
        for (unsigned i = 0; i < 6; ++i)
            if (i != k) {
                const double f = augmented[i][k];
                for (unsigned j = 0; j < 12; ++j)
                    augmented[i][j] -= f * augmented[k][j];
            }
    }
    auto forward = rotate(r.sensors[3].pose.quat, v(0, 0, 1));
    forward = forward / norm(forward);
    std::printf(
        "{\n  \"teleop_port\":8080,\n  \"camera_forward\":[%.17g,%.17g,%.17g],\n  "
        "\"rotation_limit\":[0.3,0,3.141592653589793],\n  \"tam\":[\n",
        double(forward.buf[0]),
        double(forward.buf[1]),
        double(forward.buf[2])
    );
    for (uint32_t i = 0; i < r.thruster_count; ++i) {
        std::printf("    [");
        for (unsigned j = 0; j < 6; ++j) {
            double value = 0;
            for (unsigned k = 0; k < 6; ++k)
                value += columns[i][k] * augmented[k][j + 6];
            std::printf("%s%.17g", j ? "," : "", value);
        }
        std::printf("]%s\n", i + 1 < r.thruster_count ? "," : "");
    }
    const double weight = r.object.body.dry_mass - settings.water_density * r.object.body.volume;
    std::printf("  ],\n  \"thruster_output_scale\":1,\n  \"thruster_curve\":\"quadratic_rpm\",\n");
    std::printf(
        "  \"odometry\":{\n    \"rotation_gate\":0.5,\n    \"kp\":[100,100,100,25,30,60],\n    "
        "\"ki\":[0,0,1,0,0,0],\n    \"kd\":[110,110,140,12,16,50],\n    "
        "\"derivative_on_measurement\":true,\n    \"translation_frame\":\"world\",\n    "
        "\"integral_limit\":[0,0,15,0,0,0],\n    \"output_limit\":[180,400,150,20,20,70],\n    "
        "\"feedforward_world_force\":[%.17g,%.17g,%.17g]\n  },\n",
        -weight * settings.gravity.buf[0],
        -weight * settings.gravity.buf[1],
        -weight * settings.gravity.buf[2]
    );
    std::printf("  "
                "\"ibvs\":{\"center_deadband\":0.05,\"target_height\":0.9,\"interaction\":[[0,0,0],"
                "[0,0,1],[0,0,0],[0,0,0],[0,0,0],[-1,0,0]]}\n}\n");
}
