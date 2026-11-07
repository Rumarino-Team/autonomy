#ifndef SIM_EXAMPLE_SENSORS_H
#define SIM_EXAMPLE_SENSORS_H
#include "sim.h"

/* Programmatic fixture: four independently owned sensors, sampled each tick. */
static inline SimResult
example_sensors(Simulation *model, const ObjectBlock *object, char *error, uint32_t capacity) {
    const SensorRec *sensors[4];
    for (uint32_t kind = 0; kind < 4; ++kind) {
        SensorRec sensor;
        sensor_default(&sensor, kind);
        sensor.id = kind + 1;
        sensor.object = object;
        sensor.period_ticks = 1;
        SimResult result = simulation_add_sensor(model, &sensor, &sensors[kind], error, capacity);
        if (result != SIM_OK)
            return result;
    }
    SensorRouting routing = {sensors[0], sensors[1], sensors[2], sensors[3]};
    return simulation_set_sensor_routing(model, &routing, error, capacity);
}
#endif
