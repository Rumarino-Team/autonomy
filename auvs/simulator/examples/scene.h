#ifndef SIM_EXAMPLE_SCENE_H
#define SIM_EXAMPLE_SCENE_H
#include "sensors.h"
#include <string.h>

/* A programmatic compound vehicle; no asset loader or controller is needed. */
static inline SimResult
example_scene(Simulation *model, uint32_t thrusters, char *error, uint32_t capacity) {
    SimResult result = simulation_init(model, NULL, error, capacity);
    if (result != SIM_OK)
        return result;
    memset(&model->settings.gravity, 0, sizeof(model->settings.gravity));
    UnderwaterBody body;
    underwater_body_default(&body);
    body.id = 17;
    body.motion_type = SIM_DYNAMIC;
    body.mass = body.dry_mass = 2;
    body.initial_pose.pos.buf[2] = 2;
    strcpy(body.name, "compound vehicle");
    body.tracked = 1;
    Collider colliders[5];
    for (uint32_t i = 0; i < 5; ++i) {
        collider_default(&colliders[i]);
        colliders[i].data.collisions = 0;
        for (unsigned axis = 0; axis < 3; ++axis)
            colliders[i].data.size.buf[axis] = .1f;
        colliders[i].data.pose.pos.buf[0] = ((float)i - 2) * .25f;
    }
    ObjectBlock *object = NULL;
    result = simulation_add_object(model, &body, colliders, 5, 1, &object, error, capacity);
    if (result != SIM_OK)
        return result;
    for (uint32_t i = 0; i < thrusters; ++i) {
        Thruster t = {0};
        t.object = object;
        t.direction.buf[0] = 1;
        rotor_spec_default(&t.spec);
        result = simulation_add_thruster(model, &t, error, capacity);
        if (result != SIM_OK)
            return result;
    }
    return example_sensors(model, object, error, capacity);
}
#endif
