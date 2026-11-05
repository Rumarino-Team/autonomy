#include "auv.h"
#include "runtime.hpp"
#include "blender_link.hpp"
#include <charconv>
#include <chrono>
#include <cstdlib>
#include <dlfcn.h>
#include <memory>
#include <thread>
namespace {
using namespace sim;
struct Context {
  std::unique_ptr<World> world;
  AuvSimulationStatus status{};
  std::ofstream trace;
  uint64_t limit = 21600;
  std::vector<double> commands;
  std::unique_ptr<BlenderPublisher> blender;
  bool realtime = false;
  std::chrono::steady_clock::time_point started;
};
thread_local Context context;
const char *environment(const char *key, const char *fallback) {
  auto p = std::getenv(key);
  return p && *p ? p : fallback;
}
uint64_t integer(const char *key, uint64_t fallback) {
  auto s = std::getenv(key);
  if (!s)
    return fallback;
  uint64_t out;
  auto [end, error] = std::from_chars(s, s + std::strlen(s), out);
  require(error == std::errc() && !*end, "invalid simulation integer setting");
  return out;
}
void error(const char *message) {
  context.status.code = 2;
  std::snprintf(context.status.message, sizeof(context.status.message), "%s",
                message);
}
void vector(MathVector3f &dst, SimVec3 v) {
  dst.buf[0] = v.x;
  dst.buf[1] = v.y;
  dst.buf[2] = v.z;
}
void quaternion(MathQuaternionf &dst, SimQuat q) {
  dst.buf[0] = q.x;
  dst.buf[1] = q.y;
  dst.buf[2] = q.z;
  dst.buf[3] = q.w;
}
void pose(MathPose &dst, SimPose p) {
  vector(dst.pos, p.position);
  quaternion(dst.quat, p.orientation);
}
} // namespace
extern "C" {
void auv_deinit() {
  if (context.trace) {
    context.trace << Json{{"t", "end"},
                          {"step", context.status.tick},
                          {"status", context.status.code}}
                  << '\n';
    context.trace.close();
  }
  context.world.reset();
  context.blender.reset();
  context.commands.clear();
}
void auv_init() {
  auv_deinit();
  context.status = {};
  try {
    auto path =
        environment("AUTONOMY_SIM_SCENARIO", "auvs/stonefish_sim/data/pool_scene.blend");
    auto scene = Scene::load(path);
    if (std::getenv("AUTONOMY_SIM_SEED"))
      scene = Scene::parse(
          draw_case(scene.document, integer("AUTONOMY_SIM_SEED", 0)), path);
    Dl_info info{};
    require(dladdr(reinterpret_cast<void *>(&auv_init), &info) &&
                info.dli_fname,
            "cannot locate simulation plugin");
    auto backend_path =
        std::filesystem::absolute(info.dli_fname).parent_path() /
        "libsim_backend_mujoco.so";
    auto backend = std::make_shared<Backend>(
        environment("AUTONOMY_SIM_BACKEND", backend_path.c_str()));
    context.world = std::make_unique<World>(std::move(scene), backend);
    context.limit = integer("AUTONOMY_SIM_MAX_TICKS", 21600);
    require(context.limit > 0, "simulation tick budget must be positive");
    context.commands.assign(context.world->scene.thrusters.size(), 0);
    require(context.commands.size() <= 8,
            "AUV bridge supports at most eight thrusters");
    context.realtime = integer("AUTONOMY_SIM_REALTIME", 0) != 0;
    context.started = std::chrono::steady_clock::now();
    if (integer("AUTONOMY_SIM_BLENDER", 0)) {
      auto port = integer("AUTONOMY_SIM_BLENDER_PORT", 9760);
      require(port > 0 && port <= 65535, "invalid Blender UDP port");
      context.blender = std::make_unique<BlenderPublisher>(port);
      context.blender->publish(*context.world, context.world->observe());
    }
    if (auto path = std::getenv("AUTONOMY_SIM_TRACE")) {
      context.trace.open(path);
      require(bool(context.trace), "cannot open simulation trace");
      context.trace << Json{{"t", "header"},
                            {"version", 1},
                            {"backend", backend->api.name},
                            {"backend_version", backend->api.version},
                            {"build_fingerprint", backend->build_fingerprint},
                            {"scenario", context.world->scene.document},
                            {"fingerprint", context.world->scene.fingerprint()}}
                    << '\n';
      context.trace << context.world->sample() << '\n';
    }
  } catch (const std::exception &e) {
    error(e.what());
  }
}
void auv_set_thrustor_values(const float *input, uint8_t count) {
  if (context.status.code)
    return;
  if (!input || count != context.commands.size()) {
    error("thruster command count does not match scenario");
    return;
  }
  for (size_t i = 0; i < count; ++i) {
    if (!std::isfinite(input[i])) {
      error("non-finite controller command");
      return;
    }
    context.commands[i] = std::clamp(double(input[i]), -1.0, 1.0);
  }
}
void auv_yield_until_next_frame(AuvFrame *frame) {
  if (!frame)
    return;
  *frame = {};
  frame->camera_pose.quat.buf[3] = 1;
  frame->imu_quat.buf[3] = 1;
  if (context.status.code || !context.world)
    return;
  try {
    auto &w = *context.world;
    if (w.tick >= context.limit) {
      context.status.code = 1;
      std::snprintf(context.status.message, sizeof(context.status.message),
                    "simulation tick budget exhausted");
      return;
    }
    w.step(context.commands);
    if (context.realtime)
      std::this_thread::sleep_until(context.started + std::chrono::nanoseconds(w.timestamp_ns()));
    context.status.tick = w.tick;
    auto o = w.observe();
    if (context.blender)
      context.blender->publish(w, o);
    // Existing mission controllers consume the vehicle origin and body attitude
    // here.
    pose(frame->camera_pose, w.bodies[w.scene.camera.body].pose);
    quaternion(frame->imu_quat, o.imu);
    frame->timestamp = w.timestamp_ns();
    frame->position_ok = {o.position_ok, o.position_ok, o.position_ok};
    frame->attitude_ok = {o.attitude_ok, o.attitude_ok, o.attitude_ok};
    vector(frame->accel, o.accel);
    vector(frame->gyro, o.gyro);
    frame->pressure_depth = o.depth;
    frame->pressure_depth_ok = o.depth_ok;
    frame->image_width = w.scene.camera.width;
    frame->image_height = w.scene.camera.height;
    for (auto &object : o.objects) {
      if (frame->objects_len >= AUV_FRAME_MAX_OBJECTS)
        break;
      auto &out = frame->objects[frame->objects_len++];
      out.id = object.id;
      out.cls = object.cls;
      pose(out.bbox.pose, object.pose);
      vector(out.bbox.size, object.size);
      if (object.visible && frame->objects2d_len < AUV_FRAME_MAX_OBJECTS) {
        auto &out2 = frame->objects2d[frame->objects2d_len++];
        out2.id = object.id;
        out2.cls = object.cls;
        out2.top_left = {object.x0, object.y0};
        out2.bottom_right = {object.x1, object.y1};
      }
    }
    if (context.trace)
      context.trace << w.sample() << '\n';
  } catch (const std::exception &e) {
    error(e.what());
  }
}
bool auv_simulation_status(AuvSimulationStatus *out) {
  if (!out)
    return false;
  *out = context.status;
  return true;
}
}
