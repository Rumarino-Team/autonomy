#include "auv.h"
#include "hydrus_core.h"

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

namespace {

using hydrus::kThrusters;

constexpr char kModelPath[] = "auvs/hydrus_mujoco/hydrus.xml";
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

struct Simulation {
    hydrus::HydrusModel* hydrus_model = nullptr;
    hydrus::HydrusSim* core = nullptr;
    const mjModel* model = nullptr;
    mjData* data = nullptr;
    std::mutex mu;
    std::thread viewer;
    std::atomic<bool> running{false};
    std::atomic<bool> finished{false};
    RealtimeThrottle realtime;

    int hydrus_body = -1;
    unsigned image_width = 800;
    unsigned image_height = 600;
    double fov_h_deg = 60;
    TrackedObject tracked[2]{};
    int tracked_len = 0;

    GLFWwindow* window = nullptr;
    mjvCamera cam{};
    mjvOption opt{};
    mjvPerturb pert{};
    mjvScene scn{};
    mjrContext con{};
    bool button_left = false;
    bool button_middle = false;
    bool button_right = false;
    double lastx = 0;
    double lasty = 0;
};

Simulation* g_sim = nullptr;

bool headlessRequested() {
    const char* env = std::getenv("HYDRUS_MUJOCO_HEADLESS");
    return env != nullptr && std::strcmp(env, "1") == 0;
}

void cacheObject(Simulation& sim, const char* name, ObjectCls cls, uint32_t id) {
    const int geom = mj_name2id(sim.model, mjOBJ_GEOM, name);
    if (geom < 0 || sim.tracked_len >= 2) {
        std::fprintf(stderr, "[hydrus_mujoco] missing geom '%s'\n", name);
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
    std::fprintf(stdout, "[hydrus_mujoco] tracking '%s' as %s\n", name, objectClsName(cls));
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
    const int camera_site = sim.hydrus_model->camera_site;
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

void detachTrackingCamera(Simulation& sim) {
    if (sim.cam.type != mjCAMERA_TRACKING || sim.hydrus_body < 0)
        return;
    const mjtNum* pos = sim.data->xpos + 3 * sim.hydrus_body;
    sim.cam.lookat[0] = pos[0];
    sim.cam.lookat[1] = pos[1];
    sim.cam.lookat[2] = pos[2];
    sim.cam.type = mjCAMERA_FREE;
}

void mouseButton(GLFWwindow* window, int, int, int) {
    auto* sim = static_cast<Simulation*>(glfwGetWindowUserPointer(window));
    sim->button_left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    sim->button_middle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    sim->button_right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    glfwGetCursorPos(window, &sim->lastx, &sim->lasty);
}

void mouseMove(GLFWwindow* window, double xpos, double ypos) {
    auto* sim = static_cast<Simulation*>(glfwGetWindowUserPointer(window));
    if (!sim->button_left && !sim->button_middle && !sim->button_right)
        return;

    const double dx = xpos - sim->lastx;
    const double dy = ypos - sim->lasty;
    sim->lastx = xpos;
    sim->lasty = ypos;

    int height = 1;
    glfwGetWindowSize(window, nullptr, &height);
    if (height <= 0)
        height = 1;

    const bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
    mjtMouse action;
    if (sim->button_right)
        action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (sim->button_left)
        action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    std::lock_guard<std::mutex> lock(sim->mu);
    if (action == mjMOUSE_MOVE_V || action == mjMOUSE_MOVE_H)
        detachTrackingCamera(*sim);
    // The rendered up axis is flipped for NED, so invert vertical drag.
    mjv_moveCamera(sim->model, action, dx / height, -dy / height, &sim->cam);
}

void mouseScroll(GLFWwindow* window, double, double yoffset) {
    auto* sim = static_cast<Simulation*>(glfwGetWindowUserPointer(window));
    std::lock_guard<std::mutex> lock(sim->mu);
    mjv_moveCamera(sim->model, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &sim->cam);
}

void viewerMain(Simulation* sim) {
    if (!glfwInit()) {
        std::fprintf(stderr, "[hydrus_mujoco] glfwInit failed; continuing headless\n");
        return;
    }
    glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
    sim->window = glfwCreateWindow(1200, 900, "hydrus_mujoco", nullptr, nullptr);
    if (sim->window == nullptr) {
        std::fprintf(stderr, "[hydrus_mujoco] GLFW window failed; continuing headless\n");
        glfwTerminate();
        return;
    }
    glfwMakeContextCurrent(sim->window);
    glfwSwapInterval(1);
    glfwSetWindowUserPointer(sim->window, sim);
    glfwSetMouseButtonCallback(sim->window, mouseButton);
    glfwSetCursorPosCallback(sim->window, mouseMove);
    glfwSetScrollCallback(sim->window, mouseScroll);

    mjv_defaultCamera(&sim->cam);
    mjv_defaultOption(&sim->opt);
    mjv_defaultPerturb(&sim->pert);
    mjv_defaultScene(&sim->scn);
    mjr_defaultContext(&sim->con);
    mjv_makeScene(sim->model, &sim->scn, 4000);
    mjr_makeContext(sim->model, &sim->con, mjFONTSCALE_150);
    sim->cam.type = mjCAMERA_TRACKING;
    sim->cam.trackbodyid = sim->hydrus_body;
    sim->cam.distance = 8;
    sim->cam.azimuth = 140;
    sim->cam.elevation = -35;

    while (sim->running.load() && !glfwWindowShouldClose(sim->window)) {
        {
            std::lock_guard<std::mutex> lock(sim->mu);
            mjv_updateScene(sim->model, sim->data, &sim->opt, &sim->pert, &sim->cam, mjCAT_ALL, &sim->scn);
            // MuJoCo treats +Z as up. This world is NED, so flip the camera up axis.
            for (mjvGLCamera& cam : sim->scn.camera) {
                cam.up[0] = -cam.up[0];
                cam.up[1] = -cam.up[1];
                cam.up[2] = -cam.up[2];
            }
        }
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(sim->window, &viewport.width, &viewport.height);
        mjr_render(viewport, &sim->scn, &sim->con);
        glfwSwapBuffers(sim->window);
        glfwPollEvents();
    }

    const bool user_closed = sim->running.load() && glfwWindowShouldClose(sim->window);
    mjr_freeContext(&sim->con);
    mjv_freeScene(&sim->scn);
    glfwDestroyWindow(sim->window);
    sim->window = nullptr;
    glfwTerminate();
    if (user_closed)
        sim->finished.store(true);
}

}  // namespace

void auv_init(void) {
    auv_deinit();

    std::string error;
    hydrus::HydrusModel* hydrus_model = hydrus::HydrusModel::load(kModelPath, error);
    if (hydrus_model == nullptr) {
        std::fprintf(stderr, "[hydrus_mujoco] failed to load %s: %s\n", kModelPath, error.c_str());
        return;
    }

    auto* sim = new Simulation();
    sim->hydrus_model = hydrus_model;
    sim->core = new hydrus::HydrusSim(*hydrus_model);
    sim->model = hydrus_model->model;
    sim->data = sim->core->data();
    sim->hydrus_body = hydrus_model->body;
    const mjModel* model = sim->model;
    sim->image_width = static_cast<unsigned>(hydrus::numericOr(model, "camera_spec", 0, 800));
    sim->image_height = static_cast<unsigned>(hydrus::numericOr(model, "camera_spec", 1, 600));
    sim->fov_h_deg = hydrus::numericOr(model, "camera_spec", 2, 60);

    cacheObject(*sim, "gate", ObjectCls::gate, 1);
    cacheObject(*sim, "marker", ObjectCls::cube, 2);
    std::fprintf(
        stdout,
        "[hydrus_mujoco] dry mass=%.3f kg, inertial mass=%.3f kg, volume=%.1f cm3, fluid forces every %d steps\n",
        hydrus_model->dry_mass,
        model->body_mass[sim->hydrus_body],
        hydrus_model->volume * 1e6,
        hydrus_model->hydro_prescaler);
    for (const hydrus::HullPart& part : hydrus_model->hull) {
        std::fprintf(
            stdout,
            "[hydrus_mujoco] hull '%s': %zu faces, %.4f m2\n",
            mj_id2name(model, mjOBJ_GEOM, part.geom),
            part.faces.size(),
            part.area);
    }

    const bool headless = headlessRequested();
    std::fprintf(stdout, "[hydrus_mujoco] app=%s\n", headless ? "console" : "graphical");
    std::fflush(stdout);
    sim->running.store(true);
    g_sim = sim;
    if (!headless)
        sim->viewer = std::thread(viewerMain, sim);
}

void auv_deinit(void) {
    if (g_sim == nullptr)
        return;
    Simulation* sim = g_sim;
    g_sim = nullptr;
    sim->running.store(false);
    if (sim->viewer.joinable())
        sim->viewer.join();
    delete sim->core;
    delete sim->hydrus_model;
    delete sim;
}

namespace {

bool advanceSimulation() {
    if (g_sim == nullptr)
        return false;

    mjtNum sim_time = 0;
    bool stop = false;
    {
        std::lock_guard<std::mutex> lock(g_sim->mu);
        stop = g_sim->finished.load();
        if (!stop)
            g_sim->core->step();
        sim_time = g_sim->data->time;
    }
    if (stop) {
        std::fprintf(stderr, "[hydrus_mujoco] simulation finished; restart for another run\n");
        auv_deinit();
        return false;
    }
    g_sim->realtime.wait(sim_time);
    return true;
}

}  // namespace

void auv_yield_until_next_frame(AuvFrame* frame) {
    if (!advanceSimulation())
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
