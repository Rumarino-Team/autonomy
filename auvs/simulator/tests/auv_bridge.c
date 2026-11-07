#define _POSIX_C_SOURCE 200809L
#include "auv.h"
#include <dlfcn.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#define CHECK(x)                                                                                   \
    do {                                                                                           \
        if (!(x)) {                                                                                \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x);                                \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)

int main(int argc, char **argv) {
    CHECK(argc == 3);
    void *library = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
    CHECK(library);
    AuvInitFunc *init;
    AuvDeinitFunc *destroy;
    AuvYieldUntilNextFrameFunc *next;
    AuvSetThrustorsInputFunc *commands;
    bool (*status)(AuvSimulationStatus *);
#define LOAD(field, name)                                                                          \
    do {                                                                                           \
        void *symbol = dlsym(library, name);                                                       \
        CHECK(symbol);                                                                             \
        memcpy(&(field), &symbol, sizeof(field));                                                  \
    } while (0)
    LOAD(init, "auv_init");
    LOAD(destroy, "auv_deinit");
    LOAD(next, "auv_yield_until_next_frame");
    LOAD(commands, "auv_set_thrustor_values");
    LOAD(status, "auv_simulation_status");
#undef LOAD
    setenv("AUTONOMY_SIM_BACKEND", argv[2], 1);
    setenv("AUTONOMY_SIM_SCENE", "robot_only", 1);
    setenv("AUTONOMY_SIM_FRAME_TICKS", "6", 1);
    setenv("AUTONOMY_SIM_MAX_TICKS", "12", 1);
    setenv("AUTONOMY_SIM_REALTIME", "0", 1);
    const char *names[] = {"hydrus", "proteus", "bluerov", "girona500"};
    const uint8_t counts[] = {8, 6, 8, 5};
    AuvSimulationStatus s;
    AuvFrame frame;
    CHECK(!status(NULL));
    for (unsigned i = 0; i < 4; ++i)
        for (unsigned repeat = 0; repeat < 2; ++repeat) {
            setenv("AUTONOMY_SIM_ROBOT", names[i], 1);
            init();
            CHECK(status(&s) && s.code == 0);
            next(&frame);
            CHECK(status(&s) && s.code == 0 && s.tick == 6);
            CHECK(
                frame.timestamp == 16666666 && frame.position_ok.x && frame.position_ok.y &&
                frame.position_ok.z
            );
            CHECK(
                frame.attitude_ok.x && frame.pressure_depth_ok && frame.image_width == 800 &&
                frame.image_height == 600
            );
            CHECK(isfinite(frame.camera_pose.pos.buf[2]) && frame.objects_len == 0);
            double qnorm = 0;
            for (unsigned k = 0; k < 4; ++k)
                qnorm += frame.camera_pose.quat.buf[k] * frame.camera_pose.quat.buf[k];
            CHECK(fabs(qnorm - 1) < 1e-4);
            float values[8] = {0};
            values[0] = .05f;
            commands(values, counts[i]);
            CHECK(status(&s) && s.code == 0);
            next(&frame);
            CHECK(frame.timestamp == 33333333 && status(&s) && s.tick == 12);
            next(&frame);
            CHECK(
                status(&s) && s.code == 1 && s.tick == 12 && !frame.position_ok.x &&
                !frame.objects_len
            );
            destroy();
            destroy();
        }
    setenv("AUTONOMY_SIM_ROBOT", "hydrus", 1);
    init();
    float values[8] = {0};
    commands(values, 7);
    CHECK(status(&s) && s.code == 2);
    next(&frame);
    CHECK(!frame.position_ok.x);
    destroy();
    init();
    values[0] = NAN;
    commands(values, 8);
    CHECK(status(&s) && s.code == 2);
    destroy();
    init();
    values[0] = 1.01f;
    commands(values, 8);
    CHECK(status(&s) && s.code == 2);
    destroy();
    init();
    next(NULL);
    CHECK(status(&s) && s.code == 2);
    destroy();
    const char *keys[] = {
        "AUTONOMY_SIM_ROBOT",
        "AUTONOMY_SIM_SCENE",
        "AUTONOMY_SIM_BACKEND",
        "AUTONOMY_SIM_FRAME_TICKS",
        "AUTONOMY_SIM_MAX_TICKS",
        "AUTONOMY_SIM_REALTIME"
    };
    const char *invalid[] = {"missing", "missing", "/no/such/backend.so", "0", "13", "2"};
    const char *valid[] = {"hydrus", "robot_only", argv[2], "6", "12", "0"};
    for (unsigned i = 0; i < 6; ++i) {
        setenv(keys[i], invalid[i], 1);
        init();
        CHECK(status(&s) && s.code == 2 && s.message[0]);
        destroy();
        setenv(keys[i], valid[i], 1);
    }
    setenv("AUTONOMY_SIM_REALTIME", "1", 1);
    init();
    CHECK(status(&s) && s.code == 0);
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);
    next(&frame);
    next(&frame);
    clock_gettime(CLOCK_MONOTONIC, &end);
    const double elapsed = end.tv_sec - start.tv_sec + (end.tv_nsec - start.tv_nsec) * 1e-9;
    CHECK(elapsed >= .02);
    destroy();
    dlclose(library);
    return 0;
}
