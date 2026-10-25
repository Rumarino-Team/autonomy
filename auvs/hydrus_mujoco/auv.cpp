#include <cmath>
#include <cstdlib>

#include "auv.h"
#include "auv_batch.h"
#include "auv_core.h"

#include <mujoco/mujoco.h>

#include "blender_link.h"
#include "recorder.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr char kDefaultModelPath[] = "auvs/hydrus_mujoco/proteus.xml";

const char* modelPath() {
    const char* path = std::getenv("AUV_MJCF");
    if (path != nullptr && path[0] != '\0')
        return path;
    return kDefaultModelPath;
}
constexpr double kPi = 3.14159265358979323846;
constexpr double kDefaultRealtimeFactor = 4.0;

double realtimeFactorFromEnv() {
    const char* env = std::getenv("AUV_REALTIME_FACTOR");
    if (env == nullptr || env[0] == '\0')
        return kDefaultRealtimeFactor;
    char* end = nullptr;
    const double value = std::strtod(env, &end);
    if (end == env || *end != '\0' || !std::isfinite(value) || value < 0.0) {
        std::fprintf(stderr, "[auv_mujoco] AUV_REALTIME_FACTOR '%s' is invalid\n", env);
        return kDefaultRealtimeFactor;
    }
    return value;
}
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

constexpr int kMaxFuzzObjects = 8;
constexpr uint8_t kFuzzPos = 1;
constexpr uint8_t kFuzzRoll = 2;
constexpr uint8_t kFuzzPitch = 4;
constexpr uint8_t kFuzzYaw = 8;

struct CatalogBody {
    char name[32]{};
    double rest[7]{};
    uint32_t id = 0;
    uint8_t cls = 0;
    uint8_t dof = 0;
    uint8_t tracked = 0;
};

struct FuzzSlot {
    CatalogBody object{};
    int body = -1;
    int mocap = -1;
    int geom = -1;
};

FuzzSlot g_fuzz[kMaxFuzzObjects];
int g_fuzz_len = 0;

void parseFuzzText(const char* text, CatalogBody& object) {
    object.cls = 0;
    object.dof = 0;
    object.tracked = 0;
    char buf[160];
    std::snprintf(buf, sizeof(buf), "%s", text == nullptr ? "" : text);
    char* save = nullptr;
    for (char* tok = strtok_r(buf, " \t", &save); tok != nullptr; tok = strtok_r(nullptr, " \t", &save)) {
        if (std::strcmp(tok, "pos") == 0)
            object.dof = static_cast<uint8_t>(object.dof | kFuzzPos);
        else if (std::strcmp(tok, "roll") == 0)
            object.dof = static_cast<uint8_t>(object.dof | kFuzzRoll);
        else if (std::strcmp(tok, "pitch") == 0)
            object.dof = static_cast<uint8_t>(object.dof | kFuzzPitch);
        else if (std::strcmp(tok, "yaw") == 0)
            object.dof = static_cast<uint8_t>(object.dof | kFuzzYaw);
        else if (std::strcmp(tok, "cube") == 0) {
            object.cls = static_cast<AuvObjectCls>(ObjectCls::cube);
            object.tracked = 1;
        } else if (std::strcmp(tok, "rect") == 0) {
            object.cls = static_cast<AuvObjectCls>(ObjectCls::rect);
            object.tracked = 1;
        } else if (std::strcmp(tok, "gate") == 0) {
            object.cls = static_cast<AuvObjectCls>(ObjectCls::gate);
            object.tracked = 1;
        }
    }
}

void buildFuzzCatalog(const mjModel* model) {
    g_fuzz_len = 0;
    std::memset(g_fuzz, 0, sizeof(g_fuzz));
    if (model == nullptr)
        return;
    FuzzSlot found[kMaxFuzzObjects];
    int n = 0;
    for (int t = 0; t < model->ntext; ++t) {
        const char* text_name = mj_id2name(model, mjOBJ_TEXT, t);
        if (text_name == nullptr || std::strncmp(text_name, "fuzz:", 5) != 0 || text_name[5] == '\0')
            continue;
        if (n >= kMaxFuzzObjects) {
            std::fprintf(stderr, "[auv_mujoco] ignoring fuzz texts past %d objects\n", kMaxFuzzObjects);
            break;
        }
        const char* body_name = text_name + 5;
        const int body = mj_name2id(model, mjOBJ_BODY, body_name);
        if (body < 0) {
            std::fprintf(stderr, "[auv_mujoco] fuzz text '%s' has no body\n", text_name);
            continue;
        }
        const int mocap = model->body_mocapid[body];
        if (mocap < 0) {
            std::fprintf(stderr, "[auv_mujoco] fuzz body '%s' is not mocap\n", body_name);
            continue;
        }
        int geom = -1;
        for (int g = 0; g < model->ngeom; ++g) {
            if (model->geom_bodyid[g] == body) {
                geom = g;
                break;
            }
        }
        FuzzSlot& slot = found[n++];
        slot = {};
        slot.body = body;
        slot.mocap = mocap;
        slot.geom = geom;
        std::snprintf(slot.object.name, sizeof(slot.object.name), "%s", body_name);
        const mjtNum* pos = model->body_pos + 3 * body;
        const mjtNum* quat = model->body_quat + 4 * body;
        slot.object.rest[0] = pos[0];
        slot.object.rest[1] = pos[1];
        slot.object.rest[2] = pos[2];
        slot.object.rest[3] = quat[1];
        slot.object.rest[4] = quat[2];
        slot.object.rest[5] = quat[3];
        slot.object.rest[6] = quat[0];
        parseFuzzText(model->text_data + model->text_adr[t], slot.object);
    }
    std::sort(found, found + n, [](const FuzzSlot& a, const FuzzSlot& b) {
        return std::strcmp(a.object.name, b.object.name) < 0;
    });
    for (int i = 0; i < n; ++i) {
        found[i].object.id = static_cast<uint32_t>(i + 1);
        g_fuzz[i] = found[i];
        const CatalogBody& object = g_fuzz[i].object;
        std::fprintf(
            stdout,
            "[auv_mujoco] fuzz '%s' id %u rest %.4f %.4f %.4f dof %u class %s\n",
            object.name,
            object.id,
            object.rest[0],
            object.rest[1],
            object.rest[2],
            object.dof,
            object.tracked ? objectClsName(static_cast<ObjectCls>(object.cls)) : "none");
    }
    g_fuzz_len = n;
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
        if (factor <= 0)
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
        const mjtNum wall_budget = sim_elapsed / factor;
        if (wall_budget <= wall_elapsed)
            return;
        // Keep the original baseline. A short sleep that the kernel rounds up
        // would otherwise be accepted as the new pace and the cap would stick
        // below the requested factor.
        const auto wake = base_wall + std::chrono::duration<mjtNum>(wall_budget);
        const mjtNum deficit = wall_budget - wall_elapsed;
        if (deficit > 0.008)
            std::this_thread::sleep_for(std::chrono::duration<mjtNum>(deficit - 0.002));
        while (std::chrono::steady_clock::now() < wake) {
        }
    }

    double factor = kDefaultRealtimeFactor;
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
    TrackedObject tracked[kMaxFuzzObjects]{};
    int tracked_len = 0;

    blender_link::Publisher* blender = nullptr;
};

Simulation* g_sim = nullptr;
uint32_t localization_epoch = 0;


void cacheCatalog(Simulation& sim) {
    sim.tracked_len = 0;
    for (int i = 0; i < g_fuzz_len; ++i) {
        if (g_fuzz[i].object.tracked == 0 || g_fuzz[i].geom < 0)
            continue;
        if (sim.tracked_len >= kMaxFuzzObjects)
            break;
        TrackedObject& tracked = sim.tracked[sim.tracked_len++];
        tracked.geom = g_fuzz[i].geom;
        tracked.id = g_fuzz[i].object.id;
        tracked.cls = static_cast<ObjectCls>(g_fuzz[i].object.cls);
        std::fprintf(
            stdout, "[auv_mujoco] tracking '%s' as %s\n", g_fuzz[i].object.name, objectClsName(tracked.cls));
    }
}

AuvFrame makeFrame(const Simulation& sim) {
    AuvFrame frame{};
    frame.camera_pose.quat.buf[3] = 1.f;
    frame.imu_quat.buf[3] = 1.f;
    frame.timestamp = static_cast<uint64_t>(sim.data->time * 1e9);
    frame.tracking_ok = true;
    frame.position_timestamp = frame.timestamp;
    frame.imu_timestamp = frame.timestamp;
    frame.observations_timestamp = frame.timestamp;
    frame.imu_ok = true;
    frame.localization_epoch = localization_epoch;
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
        frame.pressure_timestamp = frame.timestamp;
    }
    if (const mjtNum* gyro = sim.core->gyro()) {
        writeVec3(frame.gyro, gyro[0], gyro[1], gyro[2]);
        frame.gyro_ok = true;
    }
    if (const mjtNum* accel = sim.core->accel())
        writeVec3(frame.accel, accel[0], accel[1], accel[2]);
    mjtNum aabb_min[kMaxFuzzObjects][3]{};
    mjtNum aabb_max[kMaxFuzzObjects][3]{};
    for (int i = 0; i < sim.tracked_len && frame.objects_len < AUV_FRAME_MAX_OBJECTS; ++i) {
        geomWorldAabb(sim.model, sim.data, sim.tracked[i].geom, aabb_min[i], aabb_max[i]);
        mjtNum quat[4];
        mju_mat2Quat(quat, sim.data->geom_xmat + 9 * sim.tracked[i].geom);
        AuvObject object{};
        object.id = sim.tracked[i].id;
        object.cls = static_cast<AuvObjectCls>(sim.tracked[i].cls);
        writeQuat(object.bbox.pose.quat, quat);
        writeVec3(
            object.bbox.pose.pos,
            0.5 * (aabb_min[i][0] + aabb_max[i][0]),
            0.5 * (aabb_min[i][1] + aabb_max[i][1]),
            0.5 * (aabb_min[i][2] + aabb_max[i][2]));
        writeVec3(
            object.bbox.size,
            aabb_max[i][0] - aabb_min[i][0],
            aabb_max[i][1] - aabb_min[i][1],
            aabb_max[i][2] - aabb_min[i][2]);
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
                aabb_min[i],
                aabb_max[i],
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
    if (sim.blender == nullptr)
        return;
    sim.blender->publish(sim.model, sim.data, sim.body, sim.vehicle->camera_site);
}

struct BatchTracked {
    int geom = -1;
    uint32_t id = 0;
    ObjectCls cls = ObjectCls::cube;
};

enum FuzzKind : uint8_t {
    kKindNone = 0,
    kKindTracking = 1,
    kKindImu = 2,
    kKindGyro = 3,
    kKindDepth = 4,
    kKindDead = 5,
    kKindWeak = 6,
};

struct FuzzFault {
    uint32_t start = 0;
    uint32_t end = 0;
    uint8_t kind = kKindNone;
    uint8_t thruster = 0;
    float magnitude = 0;
};

// One case per batch world. The thread bound to that world is the only writer.
struct EnvFuzz {
    bool active = false;
    bool frame_waiting = false;
    uint8_t sticky = AUV_ERROR_NONE;
    uint64_t seed = 0;
    uint64_t step = 0;
    FuzzFault faults[4]{};
};

void clearEnv(EnvFuzz& env) {
    env.active = false;
    env.frame_waiting = false;
    env.sticky = AUV_ERROR_NONE;
    env.seed = 0;
    env.step = 0;
    for (FuzzFault& fault : env.faults) {
        fault.start = 0;
        fault.end = 0;
        fault.kind = kKindNone;
        fault.thruster = 0;
        fault.magnitude = 0;
    }
}

int g_prepared_envs = 0;
AuvBatch* g_batch = nullptr;
std::mutex g_batch_mu;
int g_live = 0;
std::vector<int> g_free_envs;
std::vector<float> g_commands;
std::vector<EnvFuzz> g_env_fuzz;
int g_nthr = 0;
BatchTracked g_tracked[kMaxFuzzObjects]{};
int g_tracked_len = 0;
unsigned g_image_width = 800;
unsigned g_image_height = 600;
double g_fov_h_deg = 60;
thread_local int g_env = -1;
recorder::Recorder* g_recorder = nullptr;
int g_record_env = -1;

bool fillThrusterGeometry(const auv::AuvModel& vehicle, const mjData* data, AuvThrusterGeometry* geometry) {
    const int count = vehicle.thrusterCount();
    if (count < 1 || count > AUV_MAX_THRUSTERS)
        return false;
    *geometry = {};
    geometry->len = static_cast<uint8_t>(count);
    const auto* rotation = data->xmat + 9 * vehicle.body;
    const auto* origin = data->xpos + 3 * vehicle.body;
    const auto* center = data->xipos + 3 * vehicle.body;
    mjtNum delta[3], local[3];
    mju_sub3(delta, center, origin);
    mju_mulMatTVec3(local, rotation, delta);
    writeVec3(geometry->center_of_mass, local[0], local[1], local[2]);
    for (int i = 0; i < count; ++i) {
        const int site = vehicle.thruster_site[i];
        const auto* position = data->site_xpos + 3 * site;
        const auto* matrix = data->site_xmat + 9 * site;
        mju_sub3(delta, position, origin);
        mju_mulMatTVec3(local, rotation, delta);
        writeVec3(geometry->mounts[i].position, local[0], local[1], local[2]);
        const mjtNum axis[3] = {matrix[0], matrix[3], matrix[6]};
        mju_mulMatTVec3(local, rotation, axis);
        writeVec3(geometry->mounts[i].direction, local[0], local[1], local[2]);
    }
    return true;
}

bool createBatchLocked() {
    AuvBatch* batch = auv_batch_create(modelPath(), g_prepared_envs, 0);
    if (batch == nullptr)
        return false;
    const auv::AuvModel* vehicle = auv_batch_model(batch);
    const mjModel* model = vehicle->model;
    g_batch = batch;
    g_nthr = vehicle->thrusterCount();
    g_commands.assign(static_cast<size_t>(g_prepared_envs * g_nthr), 0.f);
    g_env_fuzz.assign(static_cast<size_t>(g_prepared_envs), EnvFuzz{});
    g_free_envs.clear();
    g_free_envs.reserve(static_cast<size_t>(g_prepared_envs));
    for (int i = g_prepared_envs - 1; i >= 0; --i)
        g_free_envs.push_back(i);
    g_live = 0;
    g_image_width = static_cast<unsigned>(auv::numericOr(model, "camera_spec", 0, 800));
    g_image_height = static_cast<unsigned>(auv::numericOr(model, "camera_spec", 1, 600));
    g_fov_h_deg = auv::numericOr(model, "camera_spec", 2, 60);
    buildFuzzCatalog(model);
    g_tracked_len = 0;
    for (int i = 0; i < g_fuzz_len; ++i) {
        if (g_fuzz[i].object.tracked == 0 || g_fuzz[i].geom < 0)
            continue;
        if (g_tracked_len >= kMaxFuzzObjects)
            break;
        BatchTracked& tracked = g_tracked[g_tracked_len++];
        tracked.geom = g_fuzz[i].geom;
        tracked.id = g_fuzz[i].object.id;
        tracked.cls = static_cast<ObjectCls>(g_fuzz[i].object.cls);
        std::fprintf(
            stdout, "[auv_mujoco] tracking '%s' as %s\n", g_fuzz[i].object.name, objectClsName(tracked.cls));
    }
    ++localization_epoch;
    std::fprintf(
        stdout,
        "[auv_mujoco] batch worlds=%d headless, dry mass=%.3f kg, fluid forces every %d steps\n",
        g_prepared_envs,
        vehicle->dry_mass,
        vehicle->hydro_prescaler);
    for (const auv::HullPart& part : vehicle->hull) {
        std::fprintf(
            stdout,
            "[auv_mujoco] hull '%s': %zu faces, %.4f m2\n",
            mj_id2name(model, mjOBJ_GEOM, part.geom),
            part.faces.size(),
            part.area);
    }
    std::fflush(stdout);
    return true;
}

void bindBatchThread() {
    std::lock_guard<std::mutex> lock(g_batch_mu);
    if (g_env >= 0)
        return;
    if (g_batch == nullptr && !createBatchLocked())
        return;
    if (g_free_envs.empty()) {
        std::fprintf(stderr, "[auv_mujoco] batch is full (%d)\n", g_prepared_envs);
        return;
    }
    g_env = g_free_envs.back();
    g_free_envs.pop_back();
    ++g_live;
}

void releaseBatchThread() {
    std::lock_guard<std::mutex> lock(g_batch_mu);
    if (g_env < 0)
        return;
    const int env = g_env;
    g_env = -1;
    if (env < static_cast<int>(g_env_fuzz.size()))
        clearEnv(g_env_fuzz[static_cast<size_t>(env)]);
    if (--g_live > 0) {
        g_free_envs.push_back(env);
        return;
    }
    delete g_recorder;
    g_recorder = nullptr;
    g_record_env = -1;
    g_env_fuzz.clear();
    g_free_envs.clear();
    auv_batch_destroy(g_batch);
    g_batch = nullptr;
    g_commands.clear();
    g_nthr = 0;
    g_tracked_len = 0;
    g_fuzz_len = 0;
}

struct VisibleObject {
    uint32_t id = 0;
    AuvObjectCls cls = 0;
    mjtNum aabb_min[3]{};
    mjtNum aabb_max[3]{};
    mjtNum quat[4] = {1, 0, 0, 0};
};

AuvFrame makeBatchFrame(int env) {
    AuvFrame frame{};
    frame.camera_pose.quat.buf[3] = 1.f;
    frame.imu_quat.buf[3] = 1.f;
    auv::SimulationManager* sim = auv_batch_env(g_batch, env);
    const auv::AuvModel* vehicle = auv_batch_model(g_batch);
    if (sim == nullptr || vehicle == nullptr)
        return frame;
    const mjModel* model = vehicle->model;
    mjData* data = sim->data();
    frame.timestamp = static_cast<uint64_t>(data->time * 1e9);
    frame.tracking_ok = true;
    frame.position_timestamp = frame.timestamp;
    frame.imu_timestamp = frame.timestamp;
    frame.observations_timestamp = frame.timestamp;
    frame.imu_ok = true;
    frame.localization_epoch = localization_epoch;
    frame.error = AUV_ERROR_NONE;
    frame.image_width = g_image_width;
    frame.image_height = g_image_height;
    const mjtNum* pos = sim->position();
    writeVec3(frame.camera_pose.pos, pos[0], pos[1], pos[2]);
    writeQuat(frame.camera_pose.quat, sim->quaternion());
    writeQuat(frame.imu_quat, sim->quaternion());
    if (pos[2] > 0) {
        frame.pressure_depth = static_cast<float>(pos[2]);
        frame.pressure_depth_ok = true;
        frame.pressure_timestamp = frame.timestamp;
    }
    if (const mjtNum* gyro = sim->gyro()) {
        writeVec3(frame.gyro, gyro[0], gyro[1], gyro[2]);
        frame.gyro_ok = true;
    }
    if (const mjtNum* accel = sim->accel())
        writeVec3(frame.accel, accel[0], accel[1], accel[2]);

    VisibleObject visible[kMaxFuzzObjects];
    int visible_len = 0;
    for (int i = 0; i < g_tracked_len; ++i) {
        VisibleObject& entry = visible[visible_len++];
        entry.id = g_tracked[i].id;
        entry.cls = static_cast<AuvObjectCls>(g_tracked[i].cls);
        geomWorldAabb(model, data, g_tracked[i].geom, entry.aabb_min, entry.aabb_max);
        mju_mat2Quat(entry.quat, data->geom_xmat + 9 * g_tracked[i].geom);
    }

    for (int i = 0; i < visible_len && frame.objects_len < AUV_FRAME_MAX_OBJECTS; ++i) {
        const VisibleObject& entry = visible[i];
        AuvObject object{};
        object.id = entry.id;
        object.cls = entry.cls;
        writeQuat(object.bbox.pose.quat, entry.quat);
        writeVec3(
            object.bbox.pose.pos,
            0.5 * (entry.aabb_min[0] + entry.aabb_max[0]),
            0.5 * (entry.aabb_min[1] + entry.aabb_max[1]),
            0.5 * (entry.aabb_min[2] + entry.aabb_max[2]));
        writeVec3(
            object.bbox.size,
            entry.aabb_max[0] - entry.aabb_min[0],
            entry.aabb_max[1] - entry.aabb_min[1],
            entry.aabb_max[2] - entry.aabb_min[2]);
        frame.objects[frame.objects_len++] = object;
    }
    const int camera_site = vehicle->camera_site;
    if (camera_site < 0)
        return frame;
    const mjtNum* cam_pos = data->site_xpos + 3 * camera_site;
    const mjtNum* cam_mat = data->site_xmat + 9 * camera_site;
    for (int i = 0; i < visible_len && frame.objects2d_len < AUV_FRAME_MAX_OBJECTS; ++i) {
        AuvObject2d box{};
        if (!projectAabb(
                cam_pos,
                cam_mat,
                g_fov_h_deg,
                g_image_width,
                g_image_height,
                visible[i].aabb_min,
                visible[i].aabb_max,
                box.top_left,
                box.bottom_right))
            continue;
        box.id = visible[i].id;
        box.cls = visible[i].cls;
        frame.objects2d[frame.objects2d_len++] = box;
    }
    return frame;
}

uint64_t splitmix64(uint64_t x) {
    uint64_t z = x + 0x9E3779B97F4A7C15ULL;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

double unitChannel(uint64_t seed, uint64_t channel) {
    const uint64_t bits = splitmix64(seed + channel * 0x9E3779B97F4A7C15ULL);
    return static_cast<double>(bits >> 11) * (1.0 / 9007199254740992.0);
}

double uniformChannel(uint64_t seed, uint64_t channel, double lo, double hi) {
    return lo + (hi - lo) * unitChannel(seed, channel);
}

double unitStep(uint64_t seed, uint64_t step, uint64_t channel) {
    const uint64_t mixed =
        splitmix64(seed + step * 0xD1B54A32D192ED03ULL + channel * 0x94D049BB133111EBULL);
    return static_cast<double>(mixed >> 11) * (1.0 / 9007199254740992.0);
}

void quatFromEulerF32(double roll, double pitch, double yaw, double out[4]) {
    const float hr = static_cast<float>(roll) * 0.5f;
    const float hp = static_cast<float>(pitch) * 0.5f;
    const float hy = static_cast<float>(yaw) * 0.5f;
    const float sr = std::sin(hr);
    const float cr = std::cos(hr);
    const float sp = std::sin(hp);
    const float cp = std::cos(hp);
    const float sy = std::sin(hy);
    const float cy = std::cos(hy);
    out[0] = static_cast<double>(sr * cp * cy - cr * sp * sy);
    out[1] = static_cast<double>(cr * sp * cy + sr * cp * sy);
    out[2] = static_cast<double>(cr * cp * sy - sr * sp * cy);
    out[3] = static_cast<double>(cr * cp * cy + sr * sp * sy);
}

void quatToEuler(const double q[4], double euler[3]) {
    const double x = q[0];
    const double y = q[1];
    const double z = q[2];
    const double w = q[3];
    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    euler[0] = std::atan2(sinr_cosp, cosr_cosp);
    const double sinp = 2.0 * (w * y - z * x);
    const double half_pi = kPi / 2.0;
    euler[1] = std::fabs(sinp) >= 1.0 ? std::copysign(half_pi, sinp) : std::asin(sinp);
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    euler[2] = std::atan2(siny_cosp, cosy_cosp);
}

void eulerToQuat(double roll, double pitch, double yaw, double q[4]) {
    const double hr = roll * 0.5;
    const double hp = pitch * 0.5;
    const double hy = yaw * 0.5;
    const double sr = std::sin(hr);
    const double cr = std::cos(hr);
    const double sp = std::sin(hp);
    const double cp = std::cos(hp);
    const double sy = std::sin(hy);
    const double cy = std::cos(hy);
    q[0] = sr * cp * cy - cr * sp * sy;
    q[1] = cr * sp * cy + sr * cp * sy;
    q[2] = cr * cp * sy - sr * sp * cy;
    q[3] = cr * cp * cy + sr * sp * sy;
}

void perturbObject(uint64_t seed, uint64_t channel, const CatalogBody& object, double* dst) {
    dst[0] = object.rest[0];
    dst[1] = object.rest[1];
    dst[2] = object.rest[2];
    if ((object.dof & kFuzzPos) != 0) {
        dst[0] += uniformChannel(seed, channel, -1.5, 1.5);
        dst[1] += uniformChannel(seed, channel + 1, -1.5, 1.5);
        dst[2] = std::max(0.5, object.rest[2] + uniformChannel(seed, channel + 2, -0.4, 0.4));
    }
    double quat[4] = {object.rest[3], object.rest[4], object.rest[5], object.rest[6]};
    const uint8_t turn = static_cast<uint8_t>(kFuzzRoll | kFuzzPitch | kFuzzYaw);
    if ((object.dof & turn) != 0) {
        double euler[3];
        quatToEuler(quat, euler);
        if ((object.dof & kFuzzRoll) != 0)
            euler[0] += uniformChannel(seed, channel + 3, -0.15, 0.15);
        if ((object.dof & kFuzzPitch) != 0)
            euler[1] += uniformChannel(seed, channel + 4, -0.15, 0.15);
        if ((object.dof & kFuzzYaw) != 0)
            euler[2] += uniformChannel(seed, channel + 5, -0.4, 0.4);
        eulerToQuat(euler[0], euler[1], euler[2], quat);
    }
    dst[3] = quat[0];
    dst[4] = quat[1];
    dst[5] = quat[2];
    dst[6] = quat[3];
}

uint32_t saturatingAdd(uint32_t a, uint32_t b) {
    const uint64_t sum = static_cast<uint64_t>(a) + b;
    return sum > 0xffffffffULL ? 0xffffffffu : static_cast<uint32_t>(sum);
}

void drawCase(uint64_t seed, double init13[13], double rand8[8], double* poses, FuzzFault faults[4]) {
    std::fill(init13, init13 + 13, 0.0);
    std::fill(rand8, rand8 + 8, 1.0);
    init13[0] = uniformChannel(seed, 0, -1, 1);
    init13[1] = -1 + uniformChannel(seed, 1, -1, 1);
    init13[2] = std::clamp(2 + uniformChannel(seed, 2, -0.5, 0.5), 0.7, 2.8);
    double quat[4];
    quatFromEulerF32(
        uniformChannel(seed, 3, -0.15, 0.15),
        uniformChannel(seed, 4, -0.15, 0.15),
        3.14 + uniformChannel(seed, 5, -0.4, 0.4),
        quat);
    init13[3] = quat[0];
    init13[4] = quat[1];
    init13[5] = quat[2];
    init13[6] = quat[3];
    for (int axis = 0; axis < 6; ++axis)
        init13[7 + axis] = uniformChannel(seed, static_cast<uint64_t>(6 + axis), -0.2, 0.2);

    rand8[0] = uniformChannel(seed, 20, 0.97, 1.03);
    rand8[1] = uniformChannel(seed, 21, 0.97, 1.03);
    rand8[2] = uniformChannel(seed, 22, 0.7, 1.4);
    rand8[3] = uniformChannel(seed, 23, 0.8, 1.2);
    rand8[4] = uniformChannel(seed, 24, 0.7, 1.5);
    const double theta = uniformChannel(seed, 25, -kPi, kPi);
    const double speed = uniformChannel(seed, 26, 0, 0.15);
    rand8[5] = speed * std::cos(theta);
    rand8[6] = speed * std::sin(theta);
    rand8[7] = 0;

    for (int i = 0; i < g_fuzz_len; ++i)
        perturbObject(seed, 30 + static_cast<uint64_t>(i) * 8, g_fuzz[i].object, poses + 7 * i);

    const int fault_count = static_cast<int>(uniformChannel(seed, 40, 0, 5));
    for (int i = 0; i < fault_count && i < 4; ++i) {
        const uint64_t base = 50 + static_cast<uint64_t>(i) * 8;
        const int kind = static_cast<int>(uniformChannel(seed, base, 0, 6)) + 1;
        const auto start = static_cast<uint32_t>(uniformChannel(seed, base + 1, 0, 10800));
        const auto duration = static_cast<uint32_t>(uniformChannel(seed, base + 2, 72, 720));
        const auto thruster = static_cast<uint8_t>(uniformChannel(seed, base + 3, 0, 8));
        float magnitude = 0;
        if (kind == kKindGyro)
            magnitude = static_cast<float>(uniformChannel(seed, base + 4, -0.05, 0.05));
        else if (kind == kKindDepth)
            magnitude = static_cast<float>(uniformChannel(seed, base + 4, -0.3, 0.3));
        else if (kind == kKindWeak)
            magnitude = static_cast<float>(uniformChannel(seed, base + 4, 0.3, 0.7));
        faults[i].start = start;
        faults[i].end = saturatingAdd(start, duration);
        faults[i].kind = static_cast<uint8_t>(kind);
        faults[i].thruster = thruster;
        faults[i].magnitude = magnitude;
    }
}

void writeMocap(mjModel* model, mjData* data, const double* poses) {
    for (int i = 0; i < g_fuzz_len; ++i) {
        const int m = g_fuzz[i].mocap;
        const double* pose = poses + 7 * i;
        mju_copy3(data->mocap_pos + 3 * m, pose);
        data->mocap_quat[4 * m] = pose[6];
        data->mocap_quat[4 * m + 1] = pose[3];
        data->mocap_quat[4 * m + 2] = pose[4];
        data->mocap_quat[4 * m + 3] = pose[5];
    }
    if (g_fuzz_len > 0)
        mj_forward(model, data);
}

bool faultActive(const FuzzFault& fault, uint64_t step) {
    if (fault.kind == kKindNone)
        return false;
    const uint32_t at = step > 0xffffffffULL ? 0xffffffffu : static_cast<uint32_t>(step);
    return at >= fault.start && at < fault.end;
}

bool finitePose(const AuvFrame& frame) {
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(frame.camera_pose.pos.buf[i]))
            return false;
    }
    for (int i = 0; i < 4; ++i) {
        if (!std::isfinite(frame.camera_pose.quat.buf[i]))
            return false;
    }
    return true;
}

void applySensorFaults(const EnvFuzz& env, AuvFrame& frame) {
    for (const FuzzFault& fault : env.faults) {
        if (!faultActive(fault, env.step))
            continue;
        switch (fault.kind) {
            case kKindTracking:
                frame.tracking_ok = false;
                frame.objects_len = 0;
                frame.objects2d_len = 0;
                break;
            case kKindImu:
                frame.imu_ok = false;
                break;
            case kKindGyro:
                if (frame.gyro_ok) {
                    const float noise = static_cast<float>(unitStep(env.seed, env.step, 1) - 0.5);
                    frame.gyro.buf[0] += fault.magnitude + noise * 0.01f;
                }
                break;
            case kKindDepth:
                if (frame.pressure_depth_ok) {
                    const float noise = static_cast<float>(unitStep(env.seed, env.step, 2) - 0.5);
                    frame.pressure_depth += fault.magnitude + noise * 0.02f;
                }
                break;
            default:
                break;
        }
    }
}

void scaleStoredCommands(EnvFuzz& env, float* commands, int count) {
    for (const FuzzFault& fault : env.faults) {
        if (!faultActive(fault, env.step))
            continue;
        if (fault.kind != kKindDead && fault.kind != kKindWeak)
            continue;
        if (fault.thruster >= count)
            continue;
        const float scale = fault.kind == kKindDead ? 0.f : fault.magnitude;
        commands[fault.thruster] *= scale;
    }
    for (int i = 0; i < count; ++i) {
        if (!std::isfinite(commands[i])) {
            std::fill(commands, commands + count, 0.f);
            if (env.sticky == AUV_ERROR_NONE)
                env.sticky = AUV_ERROR_NON_FINITE;
            return;
        }
    }
}

void finishFuzzFrame(int env, AuvFrame& frame) {
    if (env < 0 || env >= static_cast<int>(g_env_fuzz.size()))
        return;
    EnvFuzz& fuzz = g_env_fuzz[static_cast<size_t>(env)];
    if (!fuzz.active)
        return;
    if (!finitePose(frame))
        fuzz.sticky = AUV_ERROR_NON_FINITE;
    else if (fuzz.sticky == AUV_ERROR_NONE && frame.camera_pose.pos.buf[2] < 0.2f)
        fuzz.sticky = AUV_ERROR_SURFACED;
    applySensorFaults(fuzz, frame);
    if (fuzz.sticky != AUV_ERROR_NONE)
        frame.error = fuzz.sticky;
    fuzz.frame_waiting = true;
}

}  // namespace

void auv_init(void) {
    if (g_prepared_envs != 0) {
        bindBatchThread();
        return;
    }
    auv_deinit();
    ++localization_epoch;

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

    buildFuzzCatalog(model);
    cacheCatalog(*sim);
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

    sim->realtime.factor = realtimeFactorFromEnv();
    const bool headless = headlessRequested();
    if (!headless)
        sim->blender = blender_link::Publisher::open();
    std::fprintf(
        stdout,
        "[auv_mujoco] render=%s realtime_factor=%.3g\n",
        sim->blender != nullptr ? "blender" : "headless",
        sim->realtime.factor);
    std::fflush(stdout);
    if (!headless && sim->blender == nullptr)
        std::fprintf(stderr, "[auv_mujoco] continuing without the Blender link\n");
    g_sim = sim;
}

void auv_deinit(void) {
    if (g_env >= 0) {
        releaseBatchThread();
        return;
    }
    if (g_sim == nullptr)
        return;
    Simulation* sim = g_sim;
    g_sim = nullptr;
    delete sim->blender;
    delete sim->core;
    delete sim->vehicle;
    delete sim;
    g_fuzz_len = 0;
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
    g_sim->realtime.wait(sim_time);
    return true;
}

}  // namespace

void auv_yield_until_next_frame(AuvFrame* frame) {
    if (frame == nullptr) return;
    if (g_env >= 0) {
        *frame = {};
        if (g_batch == nullptr)
            return;
        const float* cmd = g_commands.data() + static_cast<std::ptrdiff_t>(g_env * g_nthr);
        auv_batch_step_env(g_batch, g_env, cmd, 1, nullptr);
        *frame = makeBatchFrame(g_env);
        if (g_recorder != nullptr && g_env == g_record_env)
            g_recorder->step(*auv_batch_env(g_batch, g_env), *frame, cmd, g_nthr);
        finishFuzzFrame(g_env, *frame);
        return;
    }
    *frame = {};
    if (!step_simulation())
        return;
    std::lock_guard<std::mutex> lock(g_sim->mu);
    *frame = makeFrame(*g_sim);
}

void auv_set_thrustor_values(const float* thrustor_values, uint8_t thrustor_values_len) {
    if (g_env >= 0) {
        if (g_nthr <= 0)
            return;
        float* dst = g_commands.data() + static_cast<std::ptrdiff_t>(g_env * g_nthr);
        std::fill(dst, dst + g_nthr, 0.f);
        if (thrustor_values != nullptr && thrustor_values_len > 0)
            std::copy_n(thrustor_values, std::min<int>(thrustor_values_len, g_nthr), dst);
        if (g_env < static_cast<int>(g_env_fuzz.size())) {
            EnvFuzz& fuzz = g_env_fuzz[static_cast<size_t>(g_env)];
            if (fuzz.active && fuzz.frame_waiting) {
                scaleStoredCommands(fuzz, dst, g_nthr);
                fuzz.step += 1;
                fuzz.frame_waiting = false;
            }
        }
        return;
    }
    if (g_sim == nullptr)
        return;
    std::lock_guard<std::mutex> lock(g_sim->mu);
    g_sim->core->setThrusters(thrustor_values, thrustor_values_len);
}

uint32_t auv_abi_version(void) { return AUV_ABI_VERSION; }

bool auv_get_thruster_geometry(AuvThrusterGeometry* geometry) {
    if (geometry == nullptr)
        return false;
    if (g_env >= 0) {
        const auv::AuvModel* vehicle = auv_batch_model(g_batch);
        auv::SimulationManager* sim = auv_batch_env(g_batch, g_env);
        if (vehicle == nullptr || sim == nullptr)
            return false;
        return fillThrusterGeometry(*vehicle, sim->data(), geometry);
    }
    if (g_sim == nullptr)
        return false;
    std::lock_guard<std::mutex> lock(g_sim->mu);
    return fillThrusterGeometry(*g_sim->vehicle, g_sim->data, geometry);
}

bool auv_prepare_batch(int num_envs) {
    if (num_envs <= 0)
        return false;
    std::lock_guard<std::mutex> lock(g_batch_mu);
    if (g_prepared_envs != 0 || g_batch != nullptr || g_sim != nullptr)
        return false;
    g_prepared_envs = num_envs;
    return true;
}

bool auv_begin_fuzz(uint64_t seed) {
    if (g_env < 0 || g_batch == nullptr || g_env >= static_cast<int>(g_env_fuzz.size()))
        return false;
    const auv::AuvModel* vehicle = auv_batch_model(g_batch);
    auv::SimulationManager* sim = auv_batch_env(g_batch, g_env);
    if (vehicle == nullptr || sim == nullptr)
        return false;
    double init13[13];
    double rand8[8];
    double poses[kMaxFuzzObjects * 7]{};
    FuzzFault faults[4]{};
    drawCase(seed, init13, rand8, poses, faults);
    auv_batch_reset_env(g_batch, g_env, init13, rand8);
    writeMocap(vehicle->model, sim->data(), poses);
    if (g_nthr > 0) {
        float* dst = g_commands.data() + static_cast<std::ptrdiff_t>(g_env * g_nthr);
        std::fill(dst, dst + g_nthr, 0.f);
    }
    EnvFuzz& env = g_env_fuzz[static_cast<size_t>(g_env)];
    clearEnv(env);
    env.active = true;
    env.seed = seed;
    std::copy(faults, faults + 4, env.faults);
    return true;
}

bool auv_record_start(const char* path, double fps) {
    if (g_env < 0 || g_batch == nullptr)
        return false;
    std::lock_guard<std::mutex> lock(g_batch_mu);
    if (g_recorder != nullptr)
        return false;
    auv::SimulationManager* sim = auv_batch_env(g_batch, g_env);
    if (sim == nullptr || g_env >= static_cast<int>(g_env_fuzz.size()) || !g_env_fuzz[static_cast<size_t>(g_env)].active)
        return false;
    const uint64_t seed = g_env_fuzz[static_cast<size_t>(g_env)].seed;
    const float* cmd = g_commands.data() + static_cast<std::ptrdiff_t>(g_env * g_nthr);
    g_recorder = recorder::Recorder::open(path, fps, seed, *sim, makeBatchFrame(g_env), cmd, g_nthr);
    if (g_recorder == nullptr)
        return false;
    g_record_env = g_env;
    return true;
}

void auv_record_event(const char* text) {
    if (g_recorder != nullptr && g_env == g_record_env)
        g_recorder->event(text);
}

void auv_record_stop(const char* reason) {
    if (g_recorder == nullptr || g_env != g_record_env)
        return;
    g_recorder->stop(reason);
}
