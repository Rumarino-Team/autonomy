#include "auv.h"
#include <dlfcn.h>
#include <cmath>
#include <iostream>
#include <stdexcept>

template<class T> T* Symbol(void* library, const char* name) {
    auto* symbol = reinterpret_cast<T*>(dlsym(library, name));
    if (!symbol) throw std::runtime_error(std::string("missing plugin entry point: ") + name);
    return symbol;
}
int main(int argc, char** argv) {
    if (argc != 2) { std::cerr << "Usage: stonefish_blend_plugin_smoke native-plugin.so\n"; return 2; }
    void* library = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
    if (!library) { std::cerr << dlerror() << '\n'; return 1; }
    AuvDeinitFunc* deinit = nullptr;
    try {
        auto* init = Symbol<AuvInitFunc>(library, "auv_init");
        auto* next = Symbol<AuvYieldUntilNextFrameFunc>(library, "auv_yield_until_next_frame");
        auto* thrust = Symbol<AuvSetThrustorsInputFunc>(library, "auv_set_thrustor_values");
        deinit = Symbol<AuvDeinitFunc>(library, "auv_deinit");
        init();
        float values[8]{};
        thrust(values, 8);
        AuvFrame frame{};
        for (int i = 0; i < 36; ++i) next(&frame);
        if (frame.objects_len != 2 || frame.image_width != 800 || frame.image_height != 600 || !frame.timestamp)
            throw std::runtime_error("unexpected plugin frame, tracked objects, or camera resolution");
        for (float p : frame.camera_pose.pos.buf)
            if (!std::isfinite(p)) throw std::runtime_error("invalid odometry");
        for (unsigned i = 0; i < frame.objects_len; ++i)
            for (float size : frame.objects[i].bbox.size.buf)
                if (!std::isfinite(size) || size <= 0) throw std::runtime_error("invalid bounding box");
        std::cout << "PASS: plugin loaded through AUV API, produced " << unsigned(frame.objects_len)
                  << " tracked objects and " << frame.image_width << 'x' << frame.image_height
                  << " camera metadata over 36 frames\n";
        deinit();
        dlclose(library);
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        if (deinit) deinit();
        dlclose(library);
        return 1;
    }
}
