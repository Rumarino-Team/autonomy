#pragma once

// Deterministic run recording for the Blender take importer. JSON lines:
//
//   {"t":"header","version":1,"seed":1,"fps":24,"timestep":...,"every":15,"objects":[...],...}
//   {"t":"sample","step":12,"time":0.0333,"poses":{"hydrus":[px,py,pz,qw,qx,qy,qz],...},
//    "ned":[px,py,pz,qx,qy,qz,qw],"vel":[vx,vy,vz,wx,wy,wz],"thr":[...],
//    "boxes":[[id,cls,x0,y0,x1,y1],...],"hash":"0123456789abcdef"}
//   {"t":"event","step":...,"time":...,"text":"..."}
//   {"t":"end","step":...,"time":...,"reason":"..."}
//
// Poses use the Blender frame of blender_link. ned and vel are the vehicle body in
// the simulator world frame, in the init13 layout.

// Before auv.h: include/math.h would otherwise shadow the system <math.h>.
#include <cmath>

#include "auv.h"
#include "auv_core.h"

#include <cstdint>
#include <cstdio>

namespace recorder {

class Recorder {
public:
    // Writes the header and the step 0 sample. Null if the file cannot be opened.
    static Recorder* open(
        const char* path,
        double fps,
        uint64_t seed,
        const auv::SimulationManager& sim,
        const AuvFrame& frame,
        const float* thrusters,
        int thruster_count);
    ~Recorder();

    Recorder(const Recorder&) = delete;
    Recorder& operator=(const Recorder&) = delete;

    // After each physics step. Samples every `every` steps.
    void step(const auv::SimulationManager& sim, const AuvFrame& frame, const float* thrusters, int thruster_count);
    void event(const char* text);
    // Writes the end line and closes the file. Later calls do nothing.
    void stop(const char* reason);

private:
    Recorder(std::FILE* file, int every);
    void sample(const auv::SimulationManager& sim, const AuvFrame& frame, const float* thrusters, int thruster_count);

    std::FILE* file_ = nullptr;
    int every_ = 1;
    uint64_t step_ = 0;
    double time_ = 0;
};

}  // namespace recorder
