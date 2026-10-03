#pragma once

#include <mujoco/mujoco.h>

#include <cstdint>
#include <string>
#include <vector>

namespace hydrus {

constexpr int kThrusters = 8;

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
// HydrusSim instances on any threads can share it.
class HydrusModel {
public:
    static HydrusModel* load(const char* xml_path, std::string& error);
    ~HydrusModel();
    HydrusModel(const HydrusModel&) = delete;
    HydrusModel& operator=(const HydrusModel&) = delete;

    mjModel* model = nullptr;
    int body = -1;
    int qpos_adr = -1;
    int qvel_adr = -1;
    int thruster_site[kThrusters]{};
    int camera_site = -1;
    int gyro_adr = -1;
    int accel_adr = -1;
    double water_density = 1025;
    double dry_mass = 0;
    double volume = 0;
    mjtNum cb[3]{};
    int hydro_prescaler = 1;
    std::vector<HullPart> hull;

private:
    HydrusModel() = default;
};

struct Rotor {
    double omega = 0;
    double integral = 0;
    double torque = 0;
};

// One Hydrus vehicle: MuJoCo state plus the Stonefish thruster and fluid model.
class HydrusSim {
public:
    explicit HydrusSim(const HydrusModel& model);
    ~HydrusSim();
    HydrusSim(const HydrusSim&) = delete;
    HydrusSim& operator=(const HydrusSim&) = delete;

    // Back to the pose in hydrus.xml, at rest.
    void reset();
    // pos world, quat wxyz, velocities in the world frame.
    void reset(const mjtNum pos[3], const mjtNum quat[4], const mjtNum lin_vel[3], const mjtNum ang_vel[3]);
    void setRandomization(const Randomization& randomization);
    void setThrusters(const float* values, int count);
    void step();

    const HydrusModel& hydrusModel() const { return model_; }
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

    const HydrusModel& model_;
    mjData* data_ = nullptr;
    Randomization randomization_;
    double cmd_[kThrusters]{};
    Rotor rotor_[kThrusters]{};
    uint64_t hydro_counter_ = 0;
    mjtNum hydro_force_[3]{};
    mjtNum hydro_torque_[3]{};
    std::vector<mjtNum> part_min_;
    std::vector<mjtNum> part_max_;
};

double numericOr(const mjModel* model, const char* name, int index, double fallback);

}  // namespace hydrus
