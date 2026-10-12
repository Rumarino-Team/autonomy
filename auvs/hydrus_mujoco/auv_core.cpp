#include "auv_core.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace auv {

namespace {

constexpr double kPi = 3.14159265358979323846;
// Stonefish MechanicalPI integral clamp. Not part of thruster_spec.
constexpr double kRotorILimit = 5.0;
constexpr int kHullUserFields = 7;

// xfrc_applied is force then torque, both at the body center of mass, world frame.
void addWrench(mjData* data, int body, const mjtNum torque[3], const mjtNum force[3]) {
    mjtNum* dst = data->xfrc_applied + 6 * body;
    dst[0] += force[0];
    dst[1] += force[1];
    dst[2] += force[2];
    dst[3] += torque[0];
    dst[4] += torque[1];
    dst[5] += torque[2];
}

void setVec(mjtNum dst[3], mjtNum x, mjtNum y, mjtNum z) {
    dst[0] = x;
    dst[1] = y;
    dst[2] = z;
}

void midpoint(mjtNum dst[3], const mjtNum a[3], const mjtNum b[3]) {
    for (int i = 0; i < 3; ++i)
        dst[i] = 0.5 * (a[i] + b[i]);
}

void splitTriangle(const Triangle& t, Faces& out) {
    Triangle m{};
    midpoint(m.v[0], t.v[0], t.v[1]);
    midpoint(m.v[1], t.v[1], t.v[2]);
    midpoint(m.v[2], t.v[2], t.v[0]);
    Triangle a{};
    mju_copy3(a.v[0], t.v[0]);
    mju_copy3(a.v[1], m.v[0]);
    mju_copy3(a.v[2], m.v[2]);
    out.push_back(a);
    mju_copy3(a.v[0], t.v[1]);
    mju_copy3(a.v[1], m.v[1]);
    mju_copy3(a.v[2], m.v[0]);
    out.push_back(a);
    mju_copy3(a.v[0], t.v[2]);
    mju_copy3(a.v[1], m.v[2]);
    mju_copy3(a.v[2], m.v[1]);
    out.push_back(a);
    out.push_back(m);
}

void subdivide(Faces& faces) {
    Faces out;
    out.reserve(faces.size() * 4);
    for (const Triangle& t : faces)
        splitTriangle(t, out);
    faces.swap(out);
}

mjtNum triangleArea(const Triangle& t) {
    mjtNum e1[3];
    mjtNum e2[3];
    mjtNum n[3];
    mju_sub3(e1, t.v[1], t.v[0]);
    mju_sub3(e2, t.v[2], t.v[0]);
    mju_cross(n, e1, e2);
    return 0.5 * mju_norm3(n);
}

mjtNum averageArea(const Faces& faces) {
    mjtNum sum = 0;
    for (const Triangle& t : faces)
        sum += triangleArea(t);
    return faces.empty() ? 0 : sum / static_cast<mjtNum>(faces.size());
}

// Stonefish OpenGLContent::Refine: split faces larger than threshold times the mean area.
void refine(Faces& faces, mjtNum threshold) {
    constexpr mjtNum kMinArea = 0.01 * 0.01;
    for (;;) {
        const mjtNum limit = threshold * std::max(averageArea(faces), kMinArea);
        Faces out;
        bool split = false;
        for (const Triangle& t : faces) {
            if (triangleArea(t) > limit) {
                splitTriangle(t, out);
                split = true;
            } else {
                out.push_back(t);
            }
        }
        faces.swap(out);
        if (!split)
            return;
    }
}

void pushQuad(Faces& faces, const mjtNum a[3], const mjtNum b[3], const mjtNum c[3], const mjtNum d[3]) {
    Triangle t{};
    mju_copy3(t.v[0], a);
    mju_copy3(t.v[1], b);
    mju_copy3(t.v[2], c);
    faces.push_back(t);
    mju_copy3(t.v[1], c);
    mju_copy3(t.v[2], d);
    faces.push_back(t);
}

// Stonefish OpenGLContent::BuildBox with three subdivisions. Faces wind outward.
Faces buildBox(const mjtNum half[3]) {
    const mjtNum x = half[0];
    const mjtNum y = half[1];
    const mjtNum z = half[2];
    mjtNum v1[3], v2[3], v3[3], v4[3], v5[3], v6[3], v7[3], v8[3];
    setVec(v1, -x, -y, -z);
    setVec(v2, -x, y, -z);
    setVec(v3, x, y, -z);
    setVec(v4, x, -y, -z);
    setVec(v5, x, y, z);
    setVec(v6, x, -y, z);
    setVec(v7, -x, -y, z);
    setVec(v8, -x, y, z);
    Faces faces;
    pushQuad(faces, v1, v2, v3, v4);
    pushQuad(faces, v4, v3, v5, v6);
    pushQuad(faces, v7, v8, v2, v1);
    pushQuad(faces, v6, v5, v8, v7);
    pushQuad(faces, v5, v3, v2, v8);
    pushQuad(faces, v4, v6, v7, v1);
    for (int i = 0; i < 3; ++i)
        subdivide(faces);
    return faces;
}

// Stonefish OpenGLContent::BuildCylinder: axis +Z, radius corrected so the polygon keeps the
// true cross-section area, then subdivided twice.
Faces buildCylinder(mjtNum radius, mjtNum height) {
    const int slices = static_cast<int>(std::max(std::ceil(2.0 * kPi * radius / 0.1), 32.0));
    const mjtNum dphi = 2.0 * kPi / slices;
    const mjtNum r = std::sqrt(dphi * radius * radius / std::sin(dphi));
    const mjtNum h = 0.5 * height;
    auto ring = [&](int i, mjtNum z, mjtNum out[3]) {
        const mjtNum a = static_cast<mjtNum>(i) / slices * 2.0 * kPi;
        setVec(out, std::sin(a) * r, -std::cos(a) * r, z);
    };
    Faces faces;
    const mjtNum top_center[3] = {0, 0, h};
    const mjtNum bottom_center[3] = {0, 0, -h};
    for (int i = 0; i < slices; ++i) {
        mjtNum t0[3], b0[3], t1[3], b1[3];
        ring(i, h, t0);
        ring(i, -h, b0);
        ring(i + 1, h, t1);
        ring(i + 1, -h, b1);
        Triangle t{};
        mju_copy3(t.v[0], t0);
        mju_copy3(t.v[1], b0);
        mju_copy3(t.v[2], t1);
        faces.push_back(t);
        mju_copy3(t.v[0], b0);
        mju_copy3(t.v[1], b1);
        mju_copy3(t.v[2], t1);
        faces.push_back(t);
        mju_copy3(t.v[0], top_center);
        mju_copy3(t.v[1], t0);
        mju_copy3(t.v[2], t1);
        faces.push_back(t);
        mju_copy3(t.v[0], bottom_center);
        mju_copy3(t.v[1], b1);
        mju_copy3(t.v[2], b0);
        faces.push_back(t);
    }
    subdivide(faces);
    subdivide(faces);
    return faces;
}

Faces meshFaces(const mjModel* model, int mesh) {
    Faces faces;
    const int vert_adr = model->mesh_vertadr[mesh];
    const int face_adr = model->mesh_faceadr[mesh];
    const int nface = model->mesh_facenum[mesh];
    faces.reserve(nface);
    for (int i = 0; i < nface; ++i) {
        const int* face = model->mesh_face + 3 * (face_adr + i);
        Triangle t{};
        for (int k = 0; k < 3; ++k) {
            const float* vert = model->mesh_vert + 3 * (vert_adr + face[k]);
            setVec(t.v[k], vert[0], vert[1], vert[2]);
        }
        faces.push_back(t);
    }
    refine(faces, 3.0);
    return faces;
}

// Signed volume and volume centroid of a closed triangle mesh.
mjtNum signedVolume(const Faces& faces, mjtNum centroid[3]) {
    mjtNum vol6 = 0;
    mju_zero3(centroid);
    for (const Triangle& t : faces) {
        mjtNum c[3];
        mju_cross(c, t.v[1], t.v[2]);
        const mjtNum v = mju_dot3(t.v[0], c);
        vol6 += v;
        for (int i = 0; i < 3; ++i)
            centroid[i] += v * (t.v[0][i] + t.v[1][i] + t.v[2][i]) / 4.0;
    }
    if (vol6 != 0)
        mju_scl3(centroid, centroid, 1.0 / vol6);
    return vol6 / 6.0;
}

void buildHull(AuvModel& vehicle) {
    const mjModel* model = vehicle.model;
    if (model->nuser_geom < kHullUserFields || vehicle.body < 0)
        return;
    const int geom_adr = model->body_geomadr[vehicle.body];
    const int geom_num = model->body_geomnum[vehicle.body];
    for (int i = 0; i < geom_num; ++i) {
        const int geom = geom_adr + i;
        const mjtNum* user = model->geom_user + model->nuser_geom * geom;
        // Volume is the buoyant displacement. A non-buoyant shell still has drag coefficients.
        const bool has_drag = user[1] != 0 || user[2] != 0 || user[3] != 0;
        if (user[0] <= 0 && !has_drag)
            continue;
        HullPart part;
        part.geom = geom;
        part.volume = user[0];
        mju_copy3(part.cd, user + 1);
        mju_copy3(part.cf, user + 4);
        const mjtNum* size = model->geom_size + 3 * geom;
        switch (model->geom_type[geom]) {
            case mjGEOM_BOX:
                part.faces = buildBox(size);
                break;
            case mjGEOM_CYLINDER:
                part.faces = buildCylinder(size[0], 2.0 * size[1]);
                break;
            case mjGEOM_MESH:
                if (model->geom_dataid[geom] >= 0)
                    part.faces = meshFaces(model, model->geom_dataid[geom]);
                break;
            default:
                break;
        }
        if (part.faces.empty()) {
            std::fprintf(stderr, "[auv_mujoco] hull geom %d has no drag mesh\n", geom);
            continue;
        }
        if (signedVolume(part.faces, part.centroid) < 0) {
            for (Triangle& t : part.faces) {
                mjtNum tmp[3];
                mju_copy3(tmp, t.v[1]);
                mju_copy3(t.v[1], t.v[2]);
                mju_copy3(t.v[2], tmp);
            }
        }
        for (const Triangle& t : part.faces)
            part.area += triangleArea(t);
        vehicle.hull.push_back(std::move(part));
    }
}

// Stonefish SolidEntity::CorrectHydrodynamicForces: weight the per-axis coefficients by the
// direction of the summed force in the body frame.
void scaleByCoefficients(mjtNum vec[3], const mjtNum* body_mat, const mjtNum coeff[3], mjtNum scale) {
    mjtNum dir[3];
    mju_mulMatTVec3(dir, body_mat, vec);
    const mjtNum len = mju_norm3(dir);
    mjtNum c = coeff[0];
    if (len > 1e-12)
        c = (std::abs(dir[0]) * coeff[0] + std::abs(dir[1]) * coeff[1] + std::abs(dir[2]) * coeff[2]) / len;
    mju_scl3(vec, vec, scale * c);
}

void worldPoint(mjtNum out[3], const mjtNum* pos, const mjtNum* mat, const mjtNum local[3]) {
    mju_mulMatVec3(out, mat, local);
    mju_addTo3(out, pos);
}

// True when name is exactly thruster_<index> and index is at or past the first missing site.
bool thrusterPastGap(const char* name, int first_missing) {
    constexpr char kPrefix[] = "thruster_";
    constexpr size_t kPrefixLen = sizeof(kPrefix) - 1;
    if (name == nullptr || std::strncmp(name, kPrefix, kPrefixLen) != 0)
        return false;
    const char* rest = name + kPrefixLen;
    if (*rest == '\0')
        return false;
    char* end = nullptr;
    const long index = std::strtol(rest, &end, 10);
    return end != rest && *end == '\0' && index >= first_missing;
}

// Prefer the generic numeric name, then the hydrus_* name the original model uses.
double numericAlias(const mjModel* model, const char* name, const char* legacy, int index, double fallback) {
    if (mj_name2id(model, mjOBJ_NUMERIC, name) >= 0)
        return numericOr(model, name, index, fallback);
    return numericOr(model, legacy, index, fallback);
}

// Stonefish MechanicalPI::Update. The damping torque is the propeller torque of the last step.
double updateRotor(Rotor& rotor, double setpoint, double inertia, double dt, double kp, double ki, double omega_limit) {
    const double error = setpoint - rotor.omega;
    const double tau = kp * error + ki * rotor.integral;
    rotor.integral = std::clamp(rotor.integral + error * dt, -kRotorILimit, kRotorILimit);
    const double damping = std::abs(rotor.torque);
    const double tau_d = rotor.omega > 0 ? damping : -damping;
    rotor.omega = std::clamp(rotor.omega + (tau - tau_d) / inertia * dt, -omega_limit, omega_limit);
    return rotor.omega;
}

}  // namespace

double numericOr(const mjModel* model, const char* name, int index, double fallback) {
    const int id = mj_name2id(model, mjOBJ_NUMERIC, name);
    if (id < 0 || index < 0 || index >= model->numeric_size[id])
        return fallback;
    return model->numeric_data[model->numeric_adr[id] + index];
}

AuvModel* AuvModel::load(const char* xml_path, std::string& error) {
    char buffer[1024] = {};
    mjModel* model = mj_loadXML(xml_path, nullptr, buffer, sizeof(buffer));
    if (model == nullptr) {
        error = buffer;
        return nullptr;
    }
    auto* loaded = new AuvModel();
    loaded->model = model;
    const char* body_name = nullptr;
    const char* body_candidates[] = {"auv", "hydrus", "proteus", "bluerov"};
    for (const char* candidate : body_candidates) {
        if (mj_name2id(model, mjOBJ_BODY, candidate) >= 0) {
            body_name = candidate;
            break;
        }
    }
    loaded->body = body_name == nullptr ? -1 : mj_name2id(model, mjOBJ_BODY, body_name);
    if (loaded->body < 0) {
        error = "missing body 'auv', 'hydrus', 'proteus', or 'bluerov'";
        delete loaded;
        return nullptr;
    }
    const int joint = model->body_jntadr[loaded->body];
    if (joint < 0 || model->jnt_type[joint] != mjJNT_FREE) {
        error = std::string("body '") + body_name + "' needs a free joint";
        delete loaded;
        return nullptr;
    }
    loaded->qpos_adr = model->jnt_qposadr[joint];
    loaded->qvel_adr = model->jnt_dofadr[joint];

    const char* inertia_name = nullptr;
    if (mj_name2id(model, mjOBJ_NUMERIC, "inertia") >= 0)
        inertia_name = "inertia";
    else if (mj_name2id(model, mjOBJ_NUMERIC, "hydrus_inertia") >= 0)
        inertia_name = "hydrus_inertia";
    if (inertia_name != nullptr) {
        for (int i = 0; i < 3; ++i)
            model->body_inertia[3 * loaded->body + i] = numericOr(model, inertia_name, i, 0);
        mjData* data = mj_makeData(model);
        mj_setConst(model, data);
        mj_deleteData(data);
    }

    loaded->camera_site = mj_name2id(model, mjOBJ_SITE, "camera");
    for (int i = 0;; ++i) {
        const std::string name = "thruster_" + std::to_string(i);
        const int site = mj_name2id(model, mjOBJ_SITE, name.c_str());
        if (site < 0)
            break;
        loaded->thruster_site.push_back(site);
    }
    const int first_missing = loaded->thrusterCount();
    for (int s = 0; s < model->nsite; ++s) {
        const char* name = mj_id2name(model, mjOBJ_SITE, s);
        if (!thrusterPastGap(name, first_missing))
            continue;
        error = "missing site 'thruster_" + std::to_string(first_missing) + "'";
        delete loaded;
        return nullptr;
    }
    if (mj_name2id(model, mjOBJ_NUMERIC, "thruster_spec") >= 0) {
        ThrusterSpec& spec = loaded->thruster;
        spec.max_rpm = numericOr(model, "thruster_spec", 0, spec.max_rpm);
        spec.diameter = numericOr(model, "thruster_spec", 1, spec.diameter);
        spec.thrust_coeff = numericOr(model, "thruster_spec", 2, spec.thrust_coeff);
        spec.torque_coeff = numericOr(model, "thruster_spec", 3, spec.torque_coeff);
        spec.rotor_inertia = numericOr(model, "thruster_spec", 4, spec.rotor_inertia);
        spec.kp = numericOr(model, "thruster_spec", 5, spec.kp);
        spec.ki = numericOr(model, "thruster_spec", 6, spec.ki);
    }
    const int gyro = mj_name2id(model, mjOBJ_SENSOR, "gyro");
    const int accel = mj_name2id(model, mjOBJ_SENSOR, "accel");
    if (gyro >= 0)
        loaded->gyro_adr = model->sensor_adr[gyro];
    if (accel >= 0)
        loaded->accel_adr = model->sensor_adr[accel];

    loaded->water_density = numericOr(model, "water_density", 0, 1025);
    const double steps_per_second = 1.0 / model->opt.timestep;
    const double hydro_rate = numericOr(model, "hydro_rate", 0, 50);
    loaded->hydro_prescaler = std::max(1, static_cast<int>(std::lround(steps_per_second / hydro_rate)));
    loaded->volume = numericAlias(model, "volume", "hydrus_volume", 0, 0);
    for (int i = 0; i < 3; ++i)
        loaded->cb[i] = numericAlias(model, "cb", "hydrus_cb", i, 0);
    loaded->dry_mass = numericAlias(model, "dry_mass", "hydrus_dry_mass", 0, model->body_mass[loaded->body]);
    buildHull(*loaded);
    return loaded;
}

AuvModel::~AuvModel() {
    if (model != nullptr)
        mj_deleteModel(model);
}

SimulationManager::SimulationManager(const AuvModel& model)
    : model_(model),
      data_(mj_makeData(model.model)),
      cmd_(model.thruster_site.size()),
      rotor_(model.thruster_site.size()),
      part_min_(model.hull.size()),
      part_max_(model.hull.size()) {
    reset();
}

SimulationManager::~SimulationManager() {
    mj_deleteData(data_);
}

void SimulationManager::clearActuation() {
    for (size_t i = 0; i < cmd_.size(); ++i) {
        cmd_[i] = 0;
        rotor_[i] = {};
    }
    hydro_counter_ = 0;
    mju_zero3(hydro_force_);
    mju_zero3(hydro_torque_);
}

void SimulationManager::reset() {
    mj_resetData(model_.model, data_);
    clearActuation();
    mj_forward(model_.model, data_);
}

void SimulationManager::reset(const mjtNum pos[3], const mjtNum quat[4], const mjtNum lin_vel[3], const mjtNum ang_vel[3]) {
    const mjModel* m = model_.model;
    mj_resetData(m, data_);
    clearActuation();
    mjtNum* qpos = data_->qpos + model_.qpos_adr;
    mju_copy3(qpos, pos);
    mju_copy4(qpos + 3, quat);
    mju_normalize4(qpos + 3);
    // Free joint velocity: linear in the world frame, angular in the body frame.
    mjtNum* qvel = data_->qvel + model_.qvel_adr;
    mju_copy3(qvel, lin_vel);
    mjtNum neg[4];
    mju_negQuat(neg, qpos + 3);
    mju_rotVecQuat(qvel + 3, ang_vel, neg);
    mj_forward(m, data_);
}

void SimulationManager::setRandomization(const Randomization& randomization) {
    randomization_ = randomization;
}

void SimulationManager::setThrusters(const float* values, int count) {
    for (int i = 0; i < static_cast<int>(cmd_.size()); ++i)
        cmd_[i] = (values != nullptr && i < count) ? values[i] : 0.0;
}

void SimulationManager::step() {
    mj_step1(model_.model, data_);
    applyForces();
    mj_step2(model_.model, data_);
}

const mjtNum* SimulationManager::position() const {
    return data_->xpos + 3 * model_.body;
}

const mjtNum* SimulationManager::quaternion() const {
    return data_->xquat + 4 * model_.body;
}

void SimulationManager::velocity(mjtNum lin[3], mjtNum ang[3]) const {
    mjtNum vel[6];
    mj_objectVelocity(model_.model, data_, mjOBJ_XBODY, model_.body, vel, 0);
    mju_copy3(ang, vel);
    mju_copy3(lin, vel + 3);
}

const mjtNum* SimulationManager::gyro() const {
    return model_.gyro_adr >= 0 ? data_->sensordata + model_.gyro_adr : nullptr;
}

const mjtNum* SimulationManager::accel() const {
    return model_.accel_adr >= 0 ? data_->sensordata + model_.accel_adr : nullptr;
}

// Stonefish Compound::ComputeHydrodynamicForces for a submerged compound: buoyancy at the
// center of buoyancy, plus per-face form drag and skin friction on every external part.
// While crossing the surface Stonefish clips each face against the water; here each part's
// buoyancy scales with how much of its height is under water, and dry faces get no drag.
void SimulationManager::computeHydrodynamics() {
    const mjModel* model = model_.model;
    const mjData* data = data_;
    const int body = model_.body;
    const std::vector<HullPart>& hull = model_.hull;
    mju_zero3(hydro_force_);
    mju_zero3(hydro_torque_);

    const mjtNum* cg = data->xipos + 3 * body;
    const mjtNum* body_mat = data->xmat + 9 * body;
    mjtNum vel[6];
    mj_objectVelocity(model, data, mjOBJ_BODY, body, vel, 0);
    const mjtNum* ang = vel;
    const mjtNum* lin = vel + 3;
    const mjtNum rho = model_.water_density;
    const mjtNum gvec[3] = {model->opt.gravity[0], model->opt.gravity[1], model->opt.gravity[2]};
    const mjtNum volume_scale = randomization_.volume_scale;
    const mjtNum drag_scale = randomization_.drag_scale;
    const mjtNum* current = randomization_.current;

    mjtNum min_z = 0;
    mjtNum max_z = 0;
    bool any = false;
    for (size_t p = 0; p < hull.size(); ++p) {
        const HullPart& part = hull[p];
        const mjtNum* gpos = data->geom_xpos + 3 * part.geom;
        const mjtNum* gmat = data->geom_xmat + 9 * part.geom;
        bool part_any = false;
        for (const Triangle& t : part.faces) {
            for (const auto& v : t.v) {
                mjtNum w[3];
                worldPoint(w, gpos, gmat, v);
                if (!part_any) {
                    part_min_[p] = part_max_[p] = w[2];
                    part_any = true;
                } else {
                    part_min_[p] = std::min(part_min_[p], w[2]);
                    part_max_[p] = std::max(part_max_[p], w[2]);
                }
            }
        }
        if (!any) {
            min_z = part_min_[p];
            max_z = part_max_[p];
            any = true;
        } else {
            min_z = std::min(min_z, part_min_[p]);
            max_z = std::max(max_z, part_max_[p]);
        }
    }
    if (!any || max_z <= 0)
        return;
    const bool inside = min_z > 0;

    auto addAt = [&](const mjtNum point[3], const mjtNum force[3]) {
        mjtNum arm[3];
        mjtNum moment[3];
        mju_sub3(arm, point, cg);
        mju_cross(moment, arm, force);
        mju_addTo3(hydro_force_, force);
        mju_addTo3(hydro_torque_, moment);
    };

    if (inside) {
        mjtNum cb[3];
        worldPoint(cb, data->xpos + 3 * body, body_mat, model_.cb);
        mjtNum fb[3];
        mju_scl3(fb, gvec, -model_.volume * volume_scale * rho);
        addAt(cb, fb);
    }

    for (size_t p = 0; p < hull.size(); ++p) {
        const HullPart& part = hull[p];
        const mjtNum* gpos = data->geom_xpos + 3 * part.geom;
        const mjtNum* gmat = data->geom_xmat + 9 * part.geom;

        if (!inside && part_max_[p] > 0) {
            const mjtNum span = part_max_[p] - part_min_[p];
            const mjtNum wet = part_min_[p] > 0 || span <= 0 ? 1.0 : part_max_[p] / span;
            mjtNum center[3];
            worldPoint(center, gpos, gmat, part.centroid);
            mjtNum fb[3];
            mju_scl3(fb, gvec, -wet * part.volume * volume_scale * rho);
            addAt(center, fb);
        }

        mjtNum fdq[3] = {0, 0, 0};
        mjtNum tdq[3] = {0, 0, 0};
        mjtNum fdf[3] = {0, 0, 0};
        mjtNum tdf[3] = {0, 0, 0};
        for (const Triangle& t : part.faces) {
            mjtNum p1[3], p2[3], p3[3];
            worldPoint(p1, gpos, gmat, t.v[0]);
            worldPoint(p2, gpos, gmat, t.v[1]);
            worldPoint(p3, gpos, gmat, t.v[2]);
            mjtNum e1[3], e2[3], fn[3];
            mju_sub3(e1, p2, p1);
            mju_sub3(e2, p3, p1);
            mju_cross(fn, e1, e2);
            const mjtNum len2 = mju_dot3(fn, fn);
            if (len2 < 1e-12)
                continue;
            const mjtNum len = std::sqrt(len2);
            const mjtNum area = 0.5 * len;
            mju_scl3(fn, fn, 1.0 / len);
            mjtNum fc[3];
            for (int i = 0; i < 3; ++i)
                fc[i] = (p1[i] + p2[i] + p3[i]) / 3.0;
            if (!inside && fc[2] <= 0)
                continue;

            mjtNum arm[3];
            mju_sub3(arm, fc, cg);
            // Fluid velocity relative to the face.
            mjtNum vc[3];
            mju_cross(vc, ang, arm);
            mju_addTo3(vc, lin);
            mju_sub3(vc, current, vc);
            const mjtNum vc_n = mju_dot3(vc, fn);

            // Stonefish scales this by |v| and the normal speed, so it grows with speed cubed.
            if (vc_n < -1e-12) {
                mjtNum q[3];
                mju_scl3(q, vc, mju_norm3(vc) * -vc_n * area);
                mju_addTo3(fdq, q);
                mjtNum m[3];
                mju_cross(m, arm, q);
                mju_addTo3(tdq, m);
            }

            mjtNum vt[3];
            mju_addScl3(vt, vc, fn, -vc_n);
            if (mju_dot3(vt, vt) > 1e-9) {
                mjtNum s[3];
                mju_scl3(s, vt, area);
                mju_addTo3(fdf, s);
                mjtNum m[3];
                mju_cross(m, arm, s);
                mju_addTo3(tdf, m);
            }
        }
        scaleByCoefficients(fdq, body_mat, part.cd, 0.5 * rho * drag_scale);
        scaleByCoefficients(tdq, body_mat, part.cd, 0.5 * rho * drag_scale);
        scaleByCoefficients(fdf, body_mat, part.cf, rho * drag_scale);
        scaleByCoefficients(tdf, body_mat, part.cf, rho * drag_scale);
        mju_addTo3(hydro_force_, fdq);
        mju_addTo3(hydro_force_, fdf);
        mju_addTo3(hydro_torque_, tdq);
        mju_addTo3(hydro_torque_, tdf);
    }
}

// Stonefish Thruster::Update with the FDThrust model for a right-handed propeller.
void SimulationManager::applyThrusters() {
    const mjModel* model = model_.model;
    mjData* data = data_;
    const int body = model_.body;
    const mjtNum* cg = data->xipos + 3 * body;
    mjtNum vel[6];
    mj_objectVelocity(model, data, mjOBJ_BODY, body, vel, 0);
    const mjtNum* ang = vel;
    const mjtNum* lin = vel + 3;
    const ThrusterSpec& spec = model_.thruster;
    const double max_omega = spec.max_rpm / 60.0 * 2.0 * kPi;
    const double omega_limit = 2.0 * max_omega;
    const double rho = model_.water_density;
    const double d = spec.diameter;
    const double dt = model->opt.timestep;
    const double inertia = spec.rotor_inertia * randomization_.rotor_inertia_scale;
    const double gain = randomization_.thrust_scale;

    for (int i = 0; i < static_cast<int>(cmd_.size()); ++i) {
        Rotor& rotor = rotor_[i];
        const double setpoint = -std::clamp(cmd_[i], -1.0, 1.0) * max_omega;
        const double omega = updateRotor(rotor, setpoint, inertia, dt, spec.kp, spec.ki, omega_limit);
        const int site = model_.thruster_site[i];
        if (site < 0)
            continue;
        const mjtNum* pos = data->site_xpos + 3 * site;
        if (pos[2] <= 0) {
            rotor.torque = 0;
            continue;
        }
        const mjtNum* mat = data->site_xmat + 9 * site;
        const mjtNum axis[3] = {mat[0], mat[3], mat[6]};
        mjtNum arm[3];
        mju_sub3(arm, pos, cg);
        mjtNum point_vel[3];
        mju_cross(point_vel, ang, arm);
        mju_addTo3(point_vel, lin);
        mjtNum inflow[3];
        mju_sub3(inflow, point_vel, randomization_.current);
        const double u = mju_dot3(axis, inflow);

        const double n = omega / (2.0 * kPi);
        const double thrust = gain * rho * d * d * d * std::abs(n) * (d * spec.thrust_coeff * n - spec.thrust_coeff * u);
        rotor.torque = -gain * rho * d * d * d * d * std::abs(n) * (d * spec.torque_coeff * n - spec.torque_coeff * u);

        mjtNum force[3];
        mjtNum torque[3];
        mju_scl3(force, axis, thrust);
        mju_scl3(torque, axis, rotor.torque);
        mjtNum moment[3];
        mju_cross(moment, arm, force);
        mju_addTo3(torque, moment);
        addWrench(data, body, torque, force);
    }
}

void SimulationManager::applyForces() {
    const mjModel* model = model_.model;
    mju_zero(data_->xfrc_applied, 6 * model->nbody);
    const int body = model_.body;

    if (hydro_counter_ % static_cast<uint64_t>(model_.hydro_prescaler) == 0)
        computeHydrodynamics();
    ++hydro_counter_;
    addWrench(data_, body, hydro_torque_, hydro_force_);

    // MuJoCo pulls on the full inertial mass; Stonefish only weighs the dry mass.
    mjtNum lift[3];
    mju_scl3(lift, model->opt.gravity, model_.dry_mass * randomization_.dry_mass_scale - model->body_mass[body]);
    const mjtNum no_torque[3] = {0, 0, 0};
    addWrench(data_, body, no_torque, lift);

    applyThrusters();
}

}  // namespace auv
