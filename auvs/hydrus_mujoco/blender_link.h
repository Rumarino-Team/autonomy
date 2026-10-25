#pragma once

// Latest vehicle poses for an open Blender scene. One little-endian UDP
// datagram, small enough for a single packet:
//
//   char magic[4] = "AMJB"
//   uint16 version = 1
//   uint16 count
//   double sim_time
//   repeated count times:
//     char name[32]
//     double pos[3]
//     double quat[4]   // w x y z
//
// Positions and quaternions are already in the pool blend's Z-up frame:
// Blender (x, y, z) = (-north, east, -down).

#include <mujoco/mujoco.h>

namespace blender_link {

constexpr int kMaxPoses = 10;

#pragma pack(push, 1)
struct Pose {
    char name[32];
    double pos[3];
    double quat[4];  // w x y z
};
#pragma pack(pop)

// Vehicle body, each fuzz: body, and the camera site when they exist, in the
// Blender frame. Returns how many were written to out (at most kMaxPoses).
int collectPoses(const mjModel* model, const mjData* data, int vehicle_body, int camera_site, Pose out[kMaxPoses]);

class Publisher {
public:
    // Null when HYDRUS_MUJOCO_HEADLESS=1, the port is invalid, or the socket
    // cannot be opened. A missing listener is not a failure.
    static Publisher* open();
    ~Publisher();

    Publisher(const Publisher&) = delete;
    Publisher& operator=(const Publisher&) = delete;

    // Vehicle body, plus gate, marker, and the camera site when they exist.
    // Sends at most one datagram per 1/30 s of simulation time.
    void publish(const mjModel* model, const mjData* data, int vehicle_body, int camera_site);

private:
    Publisher(int fd, int port);

    int fd_ = -1;
    int port_ = 0;
    double next_time_ = 0;
    bool started_ = false;
};

}  // namespace blender_link
