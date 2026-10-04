#include <cmath>
#include <cstdlib>

#include "auv.h"
#include "auv_core.h"

#include <mujoco/mujoco.h>

#include "auv_view.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <dlfcn.h>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>

namespace {

constexpr char kDefaultModelPath[] = "auvs/hydrus_mujoco/proteus.xml";

const char* modelPath() {
    const char* path = std::getenv("AUV_MJCF");
    if (path != nullptr && path[0] != '\0')
        return path;
    return kDefaultModelPath;
}
constexpr double kPi = 3.14159265358979323846;
constexpr double kRealtimeFactorCap = 1.0;
enum class ObjectCls : AuvObjectCls {
    cube = 0,
    rect = 1,
    gate = 2,
};

const char* objectClsName(ObjectCls cls) {
    switch (cls) {
        case ObjectCls::cube:
            return "cube";
        case ObjectCls::rect:
            return "rect";
        case ObjectCls::gate:
            return "gate";
    }
    return "unknown";
}

struct TrackedObject {
    int geom = -1;
    uint32_t id = 0;
    ObjectCls cls = ObjectCls::cube;
    MathBoundingBox bbox{};
    mjtNum aabb_min[3]{};
    mjtNum aabb_max[3]{};
};

struct RealtimeThrottle {
    void wait(mjtNum sim_time) {
        if (kRealtimeFactorCap <= 0)
            return;
        const auto wall = std::chrono::steady_clock::now();
        if (!base_set) {
            base_sim = sim_time;
            base_wall = wall;
            base_set = true;
            return;
        }
        const mjtNum sim_elapsed = sim_time - base_sim;
        const mjtNum wall_elapsed = std::chrono::duration<mjtNum>(wall - base_wall).count();
        const mjtNum wall_budget = sim_elapsed / kRealtimeFactorCap;
        if (wall_budget > wall_elapsed) {
            const mjtNum deficit = wall_budget - wall_elapsed;
            if (deficit < 0.001)
                return;
            std::this_thread::sleep_for(std::chrono::duration<mjtNum>(deficit));
        } else {
            base_sim = sim_time;
            base_wall = wall;
        }
    }

    bool base_set = false;
    mjtNum base_sim = 0;
    std::chrono::steady_clock::time_point base_wall{};
};

void writeQuat(MathQuaternionf& dst, const mjtNum wxyz[4]) {
    dst.buf[0] = static_cast<float>(wxyz[1]);
    dst.buf[1] = static_cast<float>(wxyz[2]);
    dst.buf[2] = static_cast<float>(wxyz[3]);
    dst.buf[3] = static_cast<float>(wxyz[0]);
}

void writeVec3(MathVector3f& dst, mjtNum x, mjtNum y, mjtNum z) {
    dst.buf[0] = static_cast<float>(x);
    dst.buf[1] = static_cast<float>(y);
    dst.buf[2] = static_cast<float>(z);
}

void includePoint(mjtNum min[3], mjtNum max[3], const mjtNum point[3], bool& any) {
    if (!any) {
        for (int i = 0; i < 3; ++i)
            min[i] = max[i] = point[i];
        any = true;
        return;
    }
    for (int i = 0; i < 3; ++i) {
        min[i] = std::min(min[i], point[i]);
        max[i] = std::max(max[i], point[i]);
    }
}

void geomWorldAabb(const mjModel* model, const mjData* data, int geom, mjtNum min[3], mjtNum max[3]) {
    const mjtNum* pos = data->geom_xpos + 3 * geom;
    const mjtNum* mat = data->geom_xmat + 9 * geom;
    bool any = false;
    auto addLocal = [&](mjtNum x, mjtNum y, mjtNum z) {
        const mjtNum local[3] = {x, y, z};
        mjtNum world[3];
        mju_mulMatVec3(world, mat, local);
        mju_addTo3(world, pos);
        includePoint(min, max, world, any);
    };

    const mjtNum* size = model->geom_size + 3 * geom;
    switch (model->geom_type[geom]) {
        case mjGEOM_BOX:
        case mjGEOM_CYLINDER: {
            const mjtNum rx = size[0];
            const mjtNum ry = (model->geom_type[geom] == mjGEOM_CYLINDER) ? size[0] : size[1];
            const mjtNum rz = (model->geom_type[geom] == mjGEOM_CYLINDER) ? size[1] : size[2];
            for (int ix = -1; ix <= 1; ix += 2) {
                for (int iy = -1; iy <= 1; iy += 2) {
                    for (int iz = -1; iz <= 1; iz += 2)
                        addLocal(ix * rx, iy * ry, iz * rz);
                }
            }
            break;
        }
        case mjGEOM_MESH: {
            const int mesh = model->geom_dataid[geom];
            const int vert_adr = model->mesh_vertadr[mesh];
            const int nvert = model->mesh_vertnum[mesh];
            for (int i = 0; i < nvert; ++i) {
                const float* vert = model->mesh_vert + 3 * (vert_adr + i);
                addLocal(vert[0], vert[1], vert[2]);
            }
            break;
        }
        default:
            addLocal(0, 0, 0);
            break;
    }
    if (!any) {
        for (int i = 0; i < 3; ++i)
            min[i] = max[i] = pos[i];
    }
}

uint32_t clampPixel(double value, unsigned limit) {
    if (limit < 2)
        return 0;
    if (value <= 0)
        return 0;
    if (value >= static_cast<double>(limit - 1))
        return limit - 1;
    return static_cast<uint32_t>(value);
}

// Same projection as stonefish_sim: camera +X right, +Y up, +Z forward, then Y
// is negated so image +Y is down. Corners must sit 0.1 m to 50 m in front.
bool projectAabb(
    const mjtNum* cam_pos,
    const mjtNum* cam_mat,
    double fov_h_deg,
    unsigned res_x,
    unsigned res_y,
    const mjtNum aabb_min[3],
    const mjtNum aabb_max[3],
    AuvPoint2u& top_left,
    AuvPoint2u& bottom_right) {
    if (res_x < 2 || res_y < 2 || fov_h_deg <= 0)
        return false;
    const double tan_half_h = std::tan(fov_h_deg * kPi / 360.0);
    if (tan_half_h <= 0)
        return false;
    const double focal = (static_cast<double>(res_x) * 0.5) / tan_half_h;
    const mjtNum xs[2] = {aabb_min[0], aabb_max[0]};
    const mjtNum ys[2] = {aabb_min[1], aabb_max[1]};
    const mjtNum zs[2] = {aabb_min[2], aabb_max[2]};

    bool any_in_front = false;
    double u_min = 0;
    double u_max = 0;
    double v_min = 0;
    double v_max = 0;
    for (mjtNum x : xs) {
        for (mjtNum y : ys) {
            for (mjtNum z : zs) {
                const mjtNum world[3] = {x - cam_pos[0], y - cam_pos[1], z - cam_pos[2]};
                mjtNum cam[3];
                mju_mulMatTVec3(cam, cam_mat, world);
                cam[1] = -cam[1];
                if (cam[2] <= 0.1 || cam[2] >= 50)
                    continue;
                const double u = focal * cam[0] / cam[2] + static_cast<double>(res_x) * 0.5;
                const double v = focal * cam[1] / cam[2] + static_cast<double>(res_y) * 0.5;
                if (!any_in_front) {
                    u_min = u_max = u;
                    v_min = v_max = v;
                    any_in_front = true;
                } else {
                    u_min = std::min(u_min, u);
                    u_max = std::max(u_max, u);
                    v_min = std::min(v_min, v);
                    v_max = std::max(v_max, v);
                }
            }
        }
    }
    if (!any_in_front)
        return false;
    top_left.x = clampPixel(u_min, res_x);
    top_left.y = clampPixel(v_min, res_y);
    bottom_right.x = clampPixel(u_max, res_x);
    bottom_right.y = clampPixel(v_max, res_y);
    return bottom_right.x > top_left.x && bottom_right.y > top_left.y;
}

bool headlessRequested() {
    const char* env = std::getenv("HYDRUS_MUJOCO_HEADLESS");
    return env != nullptr && std::strcmp(env, "1") == 0;
}

struct StonefishViewApi {
    void* handle = nullptr;
    StonefishView* view = nullptr;
    void (*sync)(StonefishView*, const double*, const double*, const double*, const double*, double) = nullptr;
    int (*present)(StonefishView*, double) = nullptr;
    void (*destroy)(StonefishView*) = nullptr;
};

struct StonefishScene {
    const char* scenario;
    const char* robot;
};

StonefishScene stonefishSceneFor(const mjModel* model, int body) {
    const char* name = mj_id2name(model, mjOBJ_BODY, body);
    if (name != nullptr && std::strcmp(name, "proteus") == 0)
        return {"scenarios/open_space_proteus.scn", "ProteusAUV"};
    if (name != nullptr && std::strcmp(name, "bluerov") == 0)
        return {"scenarios/pool_bluerov2.scn", "bluerov2"};
    return {"scenarios/open_space_env.scn", "HydrusAUV"};
}

// MuJoCo and Stonefish both export TinyXML. Loading the view with RTLD_DEEPBIND
// keeps Stonefish bound to its own copy.
bool openStonefishView(StonefishViewApi& api, StonefishScene scene) {
    Dl_info info{};
    if (dladdr(reinterpret_cast<void*>(&auv_init), &info) == 0 || info.dli_fname == nullptr) {
        std::fprintf(stderr, "[auv_mujoco] cannot locate the plugin to load the Stonefish view\n");
        return false;
    }
    const auto path = std::filesystem::path(info.dli_fname).parent_path() / "libauv_mujoco_view.so";
    api.handle = dlopen(path.c_str(), RTLD_NOW | RTLD_DEEPBIND);
    if (api.handle == nullptr) {
        std::fprintf(stderr, "[auv_mujoco] %s\n", dlerror());
        return false;
    }
    auto* create = reinterpret_cast<StonefishView* (*)(const char*, const char*)>(
        dlsym(api.handle, "stonefish_view_create"));
    api.sync = reinterpret_cast<decltype(api.sync)>(dlsym(api.handle, "stonefish_view_sync"));
    api.present = reinterpret_cast<decltype(api.present)>(dlsym(api.handle, "stonefish_view_present"));
    api.destroy = reinterpret_cast<decltype(api.destroy)>(dlsym(api.handle, "stonefish_view_destroy"));
    if (create == nullptr || api.sync == nullptr || api.present == nullptr || api.destroy == nullptr) {
        std::fprintf(stderr, "[auv_mujoco] Stonefish view is missing an entry point\n");
        dlclose(api.handle);
        api = {};
        return false;
    }
    api.view = create(scene.scenario, scene.robot);
    return api.view != nullptr;
}

void closeStonefishView(StonefishViewApi& api) {
    if (api.destroy != nullptr && api.view != nullptr)
        api.destroy(api.view);
    api.view = nullptr;
    if (api.handle != nullptr)
        dlclose(api.handle);
    api = {};
}

struct Simulation {
    auv::AuvModel* vehicle = nullptr;
    auv::SimulationManager* core = nullptr;
    const mjModel* model = nullptr;
    mjData* data = nullptr;
    std::mutex mu;
    RealtimeThrottle realtime;

    int body = -1;
    unsigned image_width = 800;
    unsigned image_height = 600;
    double fov_h_deg = 60;
    TrackedObject tracked[2]{};
    int tracked_len = 0;

    StonefishViewApi stonefish;
    double camera_dt = 0;
};

Simulation* g_sim = nullptr;


void cacheObject(Simulation& sim, const char* name, ObjectCls cls, uint32_t id) {
    const int geom = mj_name2id(sim.model, mjOBJ_GEOM, name);
    if (geom < 0 || sim.tracked_len >= 2) {
        std::fprintf(stderr, "[auv_mujoco] missing geom '%s'\n", name);
        return;
    }
    TrackedObject& tracked = sim.tracked[sim.tracked_len++];
    tracked.geom = geom;
    tracked.id = id;
    tracked.cls = cls;
    geomWorldAabb(sim.model, sim.data, geom, tracked.aabb_min, tracked.aabb_max);
    const mjtNum* mat = sim.data->geom_xmat + 9 * geom;
    mjtNum quat[4];
    mju_mat2Quat(quat, mat);
    tracked.bbox = {};
    writeQuat(tracked.bbox.pose.quat, quat);
    writeVec3(
        tracked.bbox.pose.pos,
        0.5 * (tracked.aabb_min[0] + tracked.aabb_max[0]),
        0.5 * (tracked.aabb_min[1] + tracked.aabb_max[1]),
        0.5 * (tracked.aabb_min[2] + tracked.aabb_max[2]));
    writeVec3(
        tracked.bbox.size,
        tracked.aabb_max[0] - tracked.aabb_min[0],
        tracked.aabb_max[1] - tracked.aabb_min[1],
        tracked.aabb_max[2] - tracked.aabb_min[2]);
    std::fprintf(stdout, "[auv_mujoco] tracking '%s' as %s\n", name, objectClsName(cls));
}

AuvFrame makeFrame(const Simulation& sim) {
    AuvFrame frame{};
    frame.camera_pose.quat.buf[3] = 1.f;
    frame.imu_quat.buf[3] = 1.f;
    frame.timestamp = static_cast<uint64_t>(sim.data->time * 1e9);
    frame.tracking_ok = true;
    frame.error = AUV_ERROR_NONE;
    frame.image_width = sim.image_width;
    frame.image_height = sim.image_height;
    const mjtNum* pos = sim.core->position();
    writeVec3(frame.camera_pose.pos, pos[0], pos[1], pos[2]);
    writeQuat(frame.camera_pose.quat, sim.core->quaternion());
    writeQuat(frame.imu_quat, sim.core->quaternion());
    // Scenario is NED, so body Z is depth, positive down.
    if (pos[2] > 0) {
        frame.pressure_depth = static_cast<float>(pos[2]);
        frame.pressure_depth_ok = true;
    }
    if (const mjtNum* gyro = sim.core->gyro())
        writeVec3(frame.gyro, gyro[0], gyro[1], gyro[2]);
    if (const mjtNum* accel = sim.core->accel())
        writeVec3(frame.accel, accel[0], accel[1], accel[2]);
    for (int i = 0; i < sim.tracked_len && frame.objects_len < AUV_FRAME_MAX_OBJECTS; ++i) {
        AuvObject object{};
        object.bbox = sim.tracked[i].bbox;
        object.id = sim.tracked[i].id;
        object.cls = static_cast<AuvObjectCls>(sim.tracked[i].cls);
        frame.objects[frame.objects_len++] = object;
    }
    const int camera_site = sim.vehicle->camera_site;
    if (camera_site < 0)
        return frame;
    const mjtNum* cam_pos = sim.data->site_xpos + 3 * camera_site;
    const mjtNum* cam_mat = sim.data->site_xmat + 9 * camera_site;
    for (int i = 0; i < sim.tracked_len && frame.objects2d_len < AUV_FRAME_MAX_OBJECTS; ++i) {
        AuvObject2d box{};
        if (!projectAabb(
                cam_pos,
                cam_mat,
                sim.fov_h_deg,
                sim.image_width,
                sim.image_height,
                sim.tracked[i].aabb_min,
                sim.tracked[i].aabb_max,
                box.top_left,
                box.bottom_right))
            continue;
        box.id = sim.tracked[i].id;
        box.cls = static_cast<AuvObjectCls>(sim.tracked[i].cls);
        frame.objects2d[frame.objects2d_len++] = box;
    }
    return frame;
}

void syncView(Simulation& sim) {
    if (sim.stonefish.view == nullptr)
        return;
    const mjtNum* pos_mj = sim.core->position();
    const mjtNum* quat_mj = sim.core->quaternion();
    mjtNum linear_mj[3];
    mjtNum angular_mj[3];
    sim.core->velocity(linear_mj, angular_mj);
    double pos[3];
    double quat[4];
    double linear[3];
    double angular[3];
    for (int i = 0; i < 3; ++i) {
        pos[i] = pos_mj[i];
        linear[i] = linear_mj[i];
        angular[i] = angular_mj[i];
    }
    for (int i = 0; i < 4; ++i)
        quat[i] = quat_mj[i];
    const double dt = sim.model->opt.timestep;
    sim.stonefish.sync(sim.stonefish.view, pos, quat, linear, angular, dt);
    sim.camera_dt += dt;
}

}  // namespace

void auv_init(void) {
    auv_deinit();

    std::string error;
    const char* path = modelPath();
    auv::AuvModel* vehicle = auv::AuvModel::load(path, error);
    if (vehicle == nullptr) {
        std::fprintf(stderr, "[auv_mujoco] failed to load %s: %s\n", path, error.c_str());
        return;
    }

    auto* sim = new Simulation();
    sim->vehicle = vehicle;
    sim->core = new auv::SimulationManager(*vehicle);
    sim->model = vehicle->model;
    sim->data = sim->core->data();
    sim->body = vehicle->body;
    const mjModel* model = sim->model;
    sim->image_width = static_cast<unsigned>(auv::numericOr(model, "camera_spec", 0, 800));
    sim->image_height = static_cast<unsigned>(auv::numericOr(model, "camera_spec", 1, 600));
    sim->fov_h_deg = auv::numericOr(model, "camera_spec", 2, 60);

    cacheObject(*sim, "gate", ObjectCls::gate, 1);
    cacheObject(*sim, "marker", ObjectCls::cube, 2);
    std::fprintf(
        stdout,
        "[auv_mujoco] dry mass=%.3f kg, inertial mass=%.3f kg, volume=%.1f cm3, fluid forces every %d steps\n",
        vehicle->dry_mass,
        model->body_mass[sim->body],
        vehicle->volume * 1e6,
        vehicle->hydro_prescaler);
    for (const auv::HullPart& part : vehicle->hull) {
        std::fprintf(
            stdout,
            "[auv_mujoco] hull '%s': %zu faces, %.4f m2\n",
            mj_id2name(model, mjOBJ_GEOM, part.geom),
            part.faces.size(),
            part.area);
    }

    const bool headless = headlessRequested();
    std::fprintf(stdout, "[auv_mujoco] render=%s\n", headless ? "headless" : "stonefish");
    std::fflush(stdout);
    g_sim = sim;
    if (!headless && !openStonefishView(sim->stonefish, stonefishSceneFor(model, sim->body)))
        std::fprintf(stderr, "[auv_mujoco] continuing without the Stonefish window\n");
}

void auv_deinit(void) {
    if (g_sim == nullptr)
        return;
    Simulation* sim = g_sim;
    g_sim = nullptr;
    closeStonefishView(sim->stonefish);
    delete sim->core;
    delete sim->vehicle;
    delete sim;
}

namespace {

bool step_simulation() {
    if (g_sim == nullptr)
        return false;

    mjtNum sim_time = 0;
    {
        std::lock_guard<std::mutex> lock(g_sim->mu);
        g_sim->core->step();
        syncView(*g_sim);
        sim_time = g_sim->data->time;
    }
    if (g_sim->stonefish.view != nullptr) {
        const int presented = g_sim->stonefish.present(g_sim->stonefish.view, g_sim->camera_dt);
        if (presented > 0)
            g_sim->camera_dt = 0;
        if (presented < 0) {
            std::fprintf(stderr, "[auv_mujoco] simulation finished; restart for another run\n");
            auv_deinit();
            return false;
        }
    }
    g_sim->realtime.wait(sim_time);
    return true;
}

}  // namespace

void auv_yield_until_next_frame(AuvFrame* frame) {
    if (!step_simulation())
        return;
    std::lock_guard<std::mutex> lock(g_sim->mu);
    *frame = makeFrame(*g_sim);
}

void auv_set_thrustor_values(const float* thrustor_values, uint8_t thrustor_values_len) {
    if (g_sim == nullptr)
        return;
    std::lock_guard<std::mutex> lock(g_sim->mu);
    g_sim->core->setThrusters(thrustor_values, thrustor_values_len);
}
