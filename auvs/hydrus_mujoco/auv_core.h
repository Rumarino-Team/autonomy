#pragma once

#include <mujoco/mujoco.h>

#include <cstdint>
#include <string>
#include <vector>

namespace auv {

struct Triangle {
    mjtNum v[3][3];
};

using Faces = std::vector<Triangle>;

struct HullPart {
    int geom = -1;
    Faces faces;
    mjtNum centroid[3]{};
    mjtNum volume = 0;
    mjtNum cd[3]{};
    mjtNum cf[3]{};
    mjtNum area = 0;
};

// Propeller model. Defaults match hydrus_auv.scn. An XML numeric named thruster_spec
// overrides them, in order: max_rpm, diameter, thrust_coeff, torque_coeff, rotor_inertia, kp, ki.
struct ThrusterSpec {
    double max_rpm = 1000.0;
    double diameter = 0.18;
    double thrust_coeff = 0.48;
    double torque_coeff = 0.05;
    double rotor_inertia = 0.00146468;
    double kp = 1.0;
    double ki = 10.0;
};

// Per-environment physical variation. The defaults reproduce Stonefish.
struct Randomization {
    double dry_mass_scale = 1;
    double volume_scale = 1;
    double drag_scale = 1;
    double thrust_scale = 1;
    double rotor_inertia_scale = 1;
    // Uniform water velocity in the world frame [m/s].
    double current[3] = {0, 0, 0};
};

// Compiled scene plus everything derived from it. Read-only once loaded, so any number of
// SimulationManager instances on any threads can share it.
class AuvModel {
public:
    static AuvModel* load(const char* xml_path, std::string& error);
    ~AuvModel();
    AuvModel(const AuvModel&) = delete;
    AuvModel& operator=(const AuvModel&) = delete;

    mjModel* model = nullptr;
    int body = -1;
    int qpos_adr = -1;
    int qvel_adr = -1;
    // thruster_0 .. thruster_{n-1}. A missing index before a later thruster is a load error.
    std::vector<int> thruster_site;
    ThrusterSpec thruster;
    int camera_site = -1;
    int gyro_adr = -1;
    int accel_adr = -1;
    double water_density = 1025;
    double dry_mass = 0;
    double volume = 0;
    mjtNum cb[3]{};
    int hydro_prescaler = 1;
    std::vector<HullPart> hull;

    int thrusterCount() const { return static_cast<int>(thruster_site.size()); }

private:
    AuvModel() = default;
};

struct Rotor {
    double omega = 0;
    double integral = 0;
    double torque = 0;
};

// One vehicle: MuJoCo state plus the Stonefish thruster and fluid model.
class SimulationManager {
public:
    explicit SimulationManager(const AuvModel& model);
    ~SimulationManager();
    SimulationManager(const SimulationManager&) = delete;
    SimulationManager& operator=(const SimulationManager&) = delete;

    // Back to the pose in the XML, at rest.
    void reset();
    // pos world, quat wxyz, velocities in the world frame.
    void reset(const mjtNum pos[3], const mjtNum quat[4], const mjtNum lin_vel[3], const mjtNum ang_vel[3]);
    void setRandomization(const Randomization& randomization);
    void setThrusters(const float* values, int count);
    void step();

    const AuvModel& auvModel() const { return model_; }
    const mjModel* model() const { return model_.model; }
    mjData* data() const { return data_; }
    const mjtNum* position() const;
    const mjtNum* quaternion() const;
    // World-frame velocities of the body frame origin.
    void velocity(mjtNum lin[3], mjtNum ang[3]) const;
    const mjtNum* gyro() const;
    const mjtNum* accel() const;

private:
    void clearActuation();
    void applyForces();
    void computeHydrodynamics();
    void applyThrusters();

    const AuvModel& model_;
    mjData* data_ = nullptr;
    Randomization randomization_;
    std::vector<double> cmd_;
    std::vector<Rotor> rotor_;
    uint64_t hydro_counter_ = 0;
    mjtNum hydro_force_[3]{};
    mjtNum hydro_torque_[3]{};
    std::vector<mjtNum> part_min_;
    std::vector<mjtNum> part_max_;
};

double numericOr(const mjModel* model, const char* name, int index, double fallback);

}  // namespace auv
