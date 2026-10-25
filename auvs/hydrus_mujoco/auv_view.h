#pragma once

// Stonefish window for one MuJoCo vehicle. Loaded with RTLD_DEEPBIND so
// Stonefish's TinyXML is not replaced by the copy inside libmujoco.

#ifdef __cplusplus
extern "C" {
#endif

typedef struct StonefishView StonefishView;

// scenario is relative to auvs/stonefish_sim/data/, for example
// "scenarios/open_space_proteus.scn". robot_name is the scenario robot.
StonefishView* stonefish_view_create(const char* scenario, const char* robot_name);
// quat_wxyz is MuJoCo order. dt is the physics step, in seconds.
void stonefish_view_sync(
    StonefishView* view,
    const double pos[3],
    const double quat_wxyz[4],
    const double lin[3],
    const double ang[3],
    double dt);
// 1 if a frame was drawn, 0 if this call was skipped, -1 if the window closed.
int stonefish_view_present(StonefishView* view, double camera_dt);
void stonefish_view_destroy(StonefishView* view);

#ifdef __cplusplus
}
#endif
