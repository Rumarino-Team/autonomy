#include "scene_presets.h"
#include "object_classes.h"
#include <stdio.h>
#include <string.h>

static SimResult
box(Simulation *s,
    uint32_t id,
    const char *name,
    uint32_t cls,
    uint32_t tracked,
    uint32_t collisions,
    float x,
    float y,
    float z,
    float hx,
    float hy,
    float hz,
    char *error,
    uint32_t capacity) {
    UnderwaterBody body;
    underwater_body_default(&body);
    body.id = id;
    body.cls = cls;
    body.tracked = tracked;
    body.fuzz = id == 3;
    body.initial_pose.pos = (MathVector3f){{x, y, z}};
    snprintf(body.name, sizeof body.name, "%s", name);
    Collider collider;
    collider_default(&collider);
    collider.data.size = (MathVector3f){{hx, hy, hz}};
    collider.data.collisions = collisions;
    ObjectBlock *object = NULL;
    return simulation_add_object(s, &body, &collider, 1, 0, &object, error, capacity);
}

SimResult simulation_scene_create(
    Simulation *s, const char *scene, const Robot *robot, char *error, uint32_t capacity
) {
    if (!s || !scene || (strcmp(scene, "robot_only") && strcmp(scene, "robosub_course"))) {
        if (error && capacity)
            snprintf(error, capacity, "unknown scene (robot_only or robosub_course)");
        return SIM_INVALID;
    }
    SimulationConfig config;
    SimResult result = simulation_robot_config(robot, &config, error, capacity);
    if (result != SIM_OK)
        return result;
    result = simulation_init(s, &config, error, capacity);
    if (result != SIM_OK)
        return result;
    /* The camera looks along body -Y. Yaw 180 turns that toward pool +Y, where
     * the gate sits. The saved Hydrus pose stays in place for robot_only. */
    MathPose start = {0};
    start.pos = (MathVector3f){{0.f, 2.f, 2.f}};
    start.quat = (MathQuaternionf){{0.f, 0.f, 1.f, 0.f}};
    result = simulation_add_robot(
        s, robot, 1, !strcmp(scene, "robosub_course") ? &start : NULL, error, capacity
    );
    if (result != SIM_OK)
        goto failed;
    if (!strcmp(scene, "robosub_course")) {
#define BOX(...)                                                                                   \
    do {                                                                                           \
        result = box(s, __VA_ARGS__, error, capacity);                                             \
        if (result != SIM_OK)                                                                      \
            goto failed;                                                                           \
    } while (0)
        /* Boxes are the pool_scene.blend bounds, converted NED = (-x, y, -z).
         * Every one is tracked so the camera can name it. The opening is a
         * detection proxy with no contact; the posts, bar, walls, and floor collide. */
        BOX(2, "gate opening", AUV_ROBOSUB_GATE_OPENING, 1, 0, 1.0465f, 8.f, 2.3935f, 1.4915f, .05f, .8865f);
        BOX(3, "marker", AUV_ROBOSUB_MARKER, 1, 1, .9985f, 20.1525f, 1.1675f, .2175f, .5335f, .1895f);
        BOX(10, "gate left post", AUV_ROBOSUB_GATE, 1, 1, 2.573f, 8.f, 2.220f, .035f, .040f, .783f);
        BOX(11, "gate right post", AUV_ROBOSUB_GATE, 1, 1, -.480f, 8.f, 2.220f, .035f, .040f, .783f);
        BOX(12, "gate top bar", AUV_ROBOSUB_GATE, 1, 1, 1.0465f, 8.f, 1.472f, 1.5615f, .040f, .035f);
        BOX(100, "Pool west wall", AUV_ROBOSUB_WALL, 1, 1, 6.45f, 10.f, 1.64f, .15f, 14.3f, 1.64f);
        BOX(101, "Pool east wall", AUV_ROBOSUB_WALL, 1, 1, -6.45f, 10.f, 1.64f, .15f, 14.3f, 1.64f);
        BOX(102, "Pool south wall", AUV_ROBOSUB_WALL, 1, 1, 0.f, -4.45f, 1.64f, 6.6f, .15f, 1.64f);
        BOX(103, "Pool north wall", AUV_ROBOSUB_WALL, 1, 1, 0.f, 24.45f, 1.64f, 6.6f, .15f, 1.64f);
        BOX(104, "Pool floor", AUV_ROBOSUB_FLOOR, 1, 1, 0.f, 10.f, 3.38f, 6.6f, 14.6f, .1f);
        BOX(110, "Bin", AUV_ROBOSUB_BIN, 1, 1, 2.677f, 10.0755f, 2.931f, .305f, .4575f, .079f);
        BOX(111, "Bin_Copy", AUV_ROBOSUB_BIN, 1, 1, 1.677f, 10.5755f, 2.931f, .305f, .4575f, .079f);
        BOX(112, "Board", AUV_ROBOSUB_BOARD, 1, 1, 1.505f, 12.f, 1.f, .005f, .3f, .3f);
        BOX(113, "GateSphere", AUV_ROBOSUB_BUOY, 1, 1, -3.264f, 18.936f, 1.673f, .35f, .35f, .35f);
#undef BOX
    }
    result = simulation_finalize(s, error, capacity);
    if (result == SIM_OK)
        return result;
failed:
    simulation_destroy(s);
    return result;
}
