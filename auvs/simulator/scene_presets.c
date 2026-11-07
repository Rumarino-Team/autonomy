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
    result = simulation_add_robot(s, robot, 1, NULL, error, capacity);
    if (result != SIM_OK)
        goto failed;
    if (!strcmp(scene, "robosub_course")) {
#define BOX(...)                                                                                   \
    do {                                                                                           \
        result = box(s, __VA_ARGS__, error, capacity);                                             \
        if (result != SIM_OK)                                                                      \
            goto failed;                                                                           \
    } while (0)
        /* Detection proxy for the opening has no contact response. The frame is
         * separate collision geometry, so the vehicle must pass through it. */
        BOX(2, "gate opening", AUV_ROBOSUB_GATE_OPENING, 1, 0, 0, -6, 2, 1.5f, .05f, 1);
        BOX(3, "marker", AUV_ROBOSUB_MARKER, 1, 1, 0, -14, 2, .8f, .6f, .6f);
        BOX(10, "gate left post", AUV_ROBOSUB_GATE, 0, 1, -1.65f, -6, 2, .15f, .15f, 1.25f);
        BOX(11, "gate right post", AUV_ROBOSUB_GATE, 0, 1, 1.65f, -6, 2, .15f, .15f, 1.25f);
        BOX(12, "gate top bar", AUV_ROBOSUB_GATE, 0, 1, 0, -6, .85f, 1.8f, .15f, .1f);
        BOX(100, "floor", 0, 0, 1, 0, -10, 5, 12, 18, .1f);
#undef BOX
    }
    result = simulation_finalize(s, error, capacity);
    if (result == SIM_OK)
        return result;
failed:
    simulation_destroy(s);
    return result;
}
