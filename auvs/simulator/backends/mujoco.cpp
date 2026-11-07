#include "../internal.hpp"
#include "bindings.hpp"
#include <mujoco/mujoco.h>
using namespace core;

struct BodyBinding {
    int data;
    const ObjectBlock *body;
};

struct ShapeBinding {
    int data;
    const Collider *shape;
};

struct PhysicsWorld {
    const Simulation &source;
    mjModel *mj = nullptr;
    mjData *data = nullptr;
    BindingPool<BodyBinding> ids;
    BindingPool<ShapeBinding> geoms;
    Buffer<SimBodyState> states;
    Buffer<SimContact> contacts;
    Buffer<uint8_t> seen;
    mutable Buffer<mjtNum> hash_state;

    explicit PhysicsWorld(const Simulation &m)
        : source(m),
          ids(model_allocator(m)),
          geoms(model_allocator(m)),
          states(model_allocator(m)),
          contacts(model_allocator(m)),
          seen(model_allocator(m)),
          hash_state(model_allocator(m)) {
        ids.init(m.object_count);
        size_t count = 0;
        for (uint32_t i = 0; i < m.object_count; ++i)
            count += m.object_table[i]->collider_count;
        geoms.init(count);
    }

    ~PhysicsWorld() {
        if (data)
            mj_deleteData(data);
        if (mj)
            mj_deleteModel(mj);
    }
};

namespace {
void xyz(double *out, MathVector3f v) {
    out[0] = v.buf[0];
    out[1] = v.buf[1];
    out[2] = v.buf[2];
}

void quaternion(double *out, MathQuaternionf q) {
    // The simulator ABI stores xyzw; MuJoCo stores wxyz. The shared model
    // uses NED coordinates before it reaches this backend.
    out[0] = q.buf[3];
    out[1] = q.buf[0];
    out[2] = q.buf[1];
    out[3] = q.buf[2];
}

void transform(double *pos, double *quat, MathPose p) {
    xyz(pos, p.pos);
    quaternion(quat, p.quat);
}

void refresh(PhysicsWorld &w) {
    auto m = w.mj;
    auto d = w.data;
    mj_forward(m, d);
    w.states.clear();
    w.contacts.clear();
    for (size_t i = 0; i < w.ids.size(); ++i) {
        int id = w.ids[i];
        auto p = d->xpos + 3 * id, q = d->xquat + 4 * id, c = d->xipos + 3 * id;
        double v[6];
        mj_objectVelocity(m, d, mjOBJ_BODY, id, v, 0);
        w.states.push_back(
            {w.source.object_table[i]->body.id,
             {core::v(p[0], p[1], p[2]), core::q(q[1], q[2], q[3], q[0])},
             core::v(c[0], c[1], c[2]),
             core::v(v[3], v[4], v[5]),
             core::v(v[0], v[1], v[2])}
        );
    }
    for (int i = 0; i < d->ncon; ++i) {
        auto &c = d->contact[i];
        int a = m->geom_bodyid[c.geom[0]], b = m->geom_bodyid[c.geom[1]];
        if (a == b)
            continue;
        double f[6]{};
        mj_contactForce(m, d, i, f);
        auto id = [&](int k) {
            for (uint32_t i = 0; i < w.source.object_count; ++i)
                if (w.ids[i] == k)
                    return w.source.object_table[i]->body.id;
            return UINT32_MAX;
        };
        w.contacts.push_back(
            {id(a),
             id(b),
             v(c.pos[0], c.pos[1], c.pos[2]),
             v(c.frame[0], c.frame[1], c.frame[2]),
             f[0]}
        );
    }
}

void apply_state(PhysicsWorld &w, const SimBodyState *states, uint32_t count) {
    auto m = w.mj;
    auto d = w.data;
    require(
        (!states && count == 0) || (states && count == w.ids.size()), "reset needs all body states"
    );
    validate_reset(w.source, states, count);
    mj_resetData(m, d);
    for (size_t i = 0; i < w.ids.size(); ++i) {
        auto &b = w.source.object_table[i]->body;
        SimBodyState s =
            states ? states[i]
                   : SimBodyState{
                         b.id, b.initial_pose, {}, b.initial_velocity, b.initial_angular_velocity
                     };
        require(
            s.id == b.id && finite(s.linear_velocity) && finite(s.angular_velocity),
            "invalid reset body"
        );
        valid_pose(s.pose);
        int id = w.ids[i];
        if (b.motion_type == SIM_DYNAMIC) {
            int joint = m->body_jntadr[id], qa = m->jnt_qposadr[joint], va = m->jnt_dofadr[joint];
            d->qpos[qa] = s.pose.pos.buf[0];
            d->qpos[qa + 1] = s.pose.pos.buf[1];
            d->qpos[qa + 2] = s.pose.pos.buf[2];
            d->qpos[qa + 3] = s.pose.quat.buf[3];
            d->qpos[qa + 4] = s.pose.quat.buf[0];
            d->qpos[qa + 5] = s.pose.quat.buf[1];
            d->qpos[qa + 6] = s.pose.quat.buf[2];
            // MuJoCo freejoint translation is the body origin, while the ABI velocity
            // is at COM.
            auto offset = rotate(s.pose.quat, b.center_of_mass.pos);
            auto velocity = s.linear_velocity - cross(s.angular_velocity, offset);
            d->qvel[va] = velocity.buf[0];
            d->qvel[va + 1] = velocity.buf[1];
            d->qvel[va + 2] = velocity.buf[2];
            auto angular = rotate(conjugate(s.pose.quat), s.angular_velocity);
            d->qvel[va + 3] = angular.buf[0];
            d->qvel[va + 4] = angular.buf[1];
            d->qvel[va + 5] = angular.buf[2];
        } else if (b.motion_type == SIM_KINEMATIC) {
            int k = m->body_mocapid[id];
            d->mocap_pos[3 * k] = s.pose.pos.buf[0];
            d->mocap_pos[3 * k + 1] = s.pose.pos.buf[1];
            d->mocap_pos[3 * k + 2] = s.pose.pos.buf[2];
            d->mocap_quat[4 * k] = s.pose.quat.buf[3];
            d->mocap_quat[4 * k + 1] = s.pose.quat.buf[0];
            d->mocap_quat[4 * k + 2] = s.pose.quat.buf[1];
            d->mocap_quat[4 * k + 3] = s.pose.quat.buf[2];
        } else if (states)
            require(
                norm(s.pose.pos - b.initial_pose.pos) == 0 &&
                    s.pose.quat.buf[0] == b.initial_pose.quat.buf[0] &&
                    s.pose.quat.buf[1] == b.initial_pose.quat.buf[1] &&
                    s.pose.quat.buf[2] == b.initial_pose.quat.buf[2] &&
                    s.pose.quat.buf[3] == b.initial_pose.quat.buf[3],
                "static bodies cannot move on reset; compile a new model"
            );
    }
    refresh(w);
}

SimResult world_create(const Simulation *scene, PhysicsWorld **out, char *error, uint32_t n) {
    if (!out)
        return SIM_INVALID;
    *out = nullptr;
    if (!scene)
        return SIM_INVALID;
    return guard(error, n, [&] {
        validate_model(scene);
        auto world = std::make_unique<PhysicsWorld>(*scene);
        const auto &desc = world->source.settings;
        require(!desc.native_hydrodynamics, "MuJoCo does not expose native hydrodynamics");
        // mjSpec is MuJoCo's editable model description. Fill its numeric fields
        // and asset buffers directly; mj_compile performs the model processing
        // without serializing or parsing XML. The deleter also handles failures.
        std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(mj_makeSpec(), mj_deleteSpec);
        require(spec != nullptr, "cannot allocate MuJoCo spec");
        spec->compiler.degree = 0;
        spec->option.timestep = double(desc.dt_numerator) / desc.dt_denominator;
        xyz(spec->option.gravity, desc.gravity);
        spec->option.integrator = mjINT_IMPLICITFAST;
        spec->option.enableflags &= ~mjENBL_SLEEP;
        auto root = mjs_findBody(spec.get(), "world");
        for (size_t i = 0; i < world->source.object_count; ++i) {
            auto &b = world->source.object_table[i]->body;
            // A static body has no joint. A dynamic body gets six free degrees of
            // freedom. A kinematic body is positioned through MuJoCo's mocap arrays.
            // Generated names preserve the ABI's stable IDs across compilation.
            auto body = mjs_addBody(root, nullptr);
            mjs_setName(body->element, ("b" + std::to_string(b.id)).c_str());
            transform(body->pos, body->quat, b.initial_pose);
            body->mocap = b.motion_type == SIM_KINEMATIC;
            if (b.motion_type == SIM_DYNAMIC) {
                mjs_addFreeJoint(body);
                body->explicitinertial = true;
                transform(body->ipos, body->iquat, b.center_of_mass);
                body->mass = b.mass;
                // Effective calibrated inertia can violate the rigid-body triangle
                // inequality checked by the compiler. Compile with valid isotropic
                // moments, then install the requested principal moments below.
                double placeholder =
                    std::max({b.inertia.buf[0], b.inertia.buf[1], b.inertia.buf[2]});
                xyz(body->inertia, v(placeholder, placeholder, placeholder));
            }
            for (uint32_t k = 0; k < world->source.object_table[i]->collider_count; ++k) {
                const auto &collider = world->source.object_table[i]->colliders[k];
                const auto &g = collider.data;
                require(g.restitution == 0, "MuJoCo adapter currently requires zero restitution");
                auto geom = mjs_addGeom(body, nullptr);
                auto suffix = std::to_string(i) + '_' + std::to_string(k);
                mjs_setName(geom->element, ("g" + suffix).c_str());
                // Geom poses are relative to the body origin. Mass and inertia come
                // from the body calibration, so collider volume contributes no mass.
                transform(geom->pos, geom->quat, g.pose);
                geom->density = 0;
                geom->contype = geom->conaffinity = g.collisions;
                xyz(geom->friction, v(g.friction, 0, 0));
                if (g.kind == SIM_MESH) {
                    auto mesh = mjs_addMesh(spec.get(), nullptr);
                    auto name = "m" + suffix;
                    mjs_setName(mesh->element, name.c_str());
                    // Mesh asset buffers store float xyz triples and int index triples.
                    // Copy this shape's ranges; triangle indices are already local to
                    // those vertices, even when the scene contains several meshes.
                    auto &vertices = *mesh->uservert;
                    vertices.reserve(size_t(collider.geometry.position_count) * 3);
                    for (uint32_t v = 0; v < collider.geometry.position_count; ++v) {
                        auto &p = collider.geometry.positions[v];
                        vertices.insert(
                            vertices.end(), {float(p.buf[0]), float(p.buf[1]), float(p.buf[2])}
                        );
                    }
                    auto &faces = *mesh->userface;
                    faces.reserve(size_t(collider.geometry.triangle_count) * 3);
                    for (uint32_t t = 0; t < collider.geometry.triangle_count; ++t) {
                        auto &f = collider.geometry.triangles[t];
                        faces.insert(faces.end(), {int(f.a), int(f.b), int(f.c)});
                    }
                    geom->type = mjGEOM_MESH;
                    mjs_setString(geom->meshname, name.c_str());
                } else {
                    geom->type = g.kind == SIM_BOX      ? mjGEOM_BOX
                                 : g.kind == SIM_SPHERE ? mjGEOM_SPHERE
                                                        : mjGEOM_CYLINDER;
                    if (g.kind == SIM_BOX)
                        xyz(geom->size, g.size);
                    else if (g.kind == SIM_SPHERE)
                        geom->size[0] = g.size.buf[0];
                    else
                        // MuJoCo uses [radius, half length]; the ABI keeps half length in
                        // z.
                        xyz(geom->size, v(g.size.buf[0], g.size.buf[2], 0));
                }
            }
        }
        world->mj = mj_compile(spec.get(), nullptr);
        if (!world->mj)
            throw std::runtime_error(mjs_getError(spec.get()));
        // The compiled mjModel owns its arrays independently of the builder.
        spec.reset();
        for (uint32_t bi = 0; bi < world->source.object_count; ++bi) {
            const auto &b = world->source.object_table[bi]->body;
            int id = mj_name2id(world->mj, mjOBJ_BODY, ("b" + std::to_string(b.id)).c_str());
            world->ids.push_back(id);
            world->ids.block(world->ids.size() - 1).body =
                world->source.object_table[world->ids.size() - 1];
            if (b.motion_type == SIM_DYNAMIC) {
                auto p = world->mj->body_inertia + 3 * id;
                p[0] = b.inertia.buf[0];
                p[1] = b.inertia.buf[1];
                p[2] = b.inertia.buf[2];
            }
        }
        for (size_t i = 0; i < world->source.object_count; ++i) {
            auto &b = world->source.object_table[i]->body;
            for (uint32_t k = 0; k < world->source.object_table[i]->collider_count; ++k) {
                auto name = "g" + std::to_string(i) + '_' + std::to_string(k);
                world->geoms.push_back(mj_name2id(world->mj, mjOBJ_GEOM, name.c_str()));
                world->geoms.block(world->geoms.size() - 1).shape =
                    &world->source.object_table[i]->colliders[k];
            }
        }
        world->seen.resize(world->source.object_count);
        world->states.reserve(world->source.object_count);
        world->hash_state.resize(mj_stateSize(world->mj, mjSTATE_INTEGRATION));
        switch (desc.integrator) {
        case SIM_INTEGRATOR_EULER:
            world->mj->opt.integrator = mjINT_EULER;
            break;
        case SIM_INTEGRATOR_IMPLICIT:
            world->mj->opt.integrator = mjINT_IMPLICIT;
            break;
        case SIM_INTEGRATOR_RK4:
            world->mj->opt.integrator = mjINT_RK4;
            break;
        default:
            world->mj->opt.integrator = mjINT_IMPLICITFAST;
            break;
        }
        // Recompute derived model constants after replacing the compiled inertia.
        // Runtime positions, velocities and solver history live in a separate
        // mjData, which apply_state initializes from the resolved scene.
        auto constants = mj_makeData(world->mj);
        require(constants, "cannot allocate MuJoCo constants");
        mj_setConst(world->mj, constants);
        mj_deleteData(constants);
        world->data = mj_makeData(world->mj);
        require(world->data, "cannot allocate MuJoCo world");
        apply_state(*world, nullptr, 0);
        *out = world.release();
    });
}

SimResult reset(PhysicsWorld *p, const SimBodyState *s, uint32_t c, char *e, uint32_t n) {
    if (!p)
        return SIM_INVALID;
    return guard(e, n, [&] { apply_state(*static_cast<PhysicsWorld *>(p), s, c); });
}

SimResult step(PhysicsWorld *p, const SimWrench *input, uint32_t count, char *e, uint32_t n) {
    if (!p || (!input && count))
        return SIM_INVALID;
    return guard(e, n, [&] {
        auto &w = *static_cast<PhysicsWorld *>(p);
        auto m = w.mj;
        validate_wrenches(w.source, input, count);
        // The host calculates thruster/fluid forces and torques at each body's
        // COM in world axes. Replace last tick's applied wrenches, then let MuJoCo
        // integrate those forces together with gravity and contact constraints.
        mju_zero(w.data->xfrc_applied, 6 * m->nbody);
        std::fill(w.seen.begin(), w.seen.end(), 0);
        for (uint32_t i = 0; i < count; ++i) {
            auto &f = input[i];
            require(finite(f.force) && finite(f.torque), "invalid or duplicate wrench");
            auto index = i < w.source.object_count && w.source.object_table[i]->body.id == f.id
                             ? i
                             : core::index(w.source, f.id);
            require(!w.seen[index], "duplicate wrench");
            w.seen[index] = 1;
            auto out = w.data->xfrc_applied + 6 * w.ids[index];
            out[0] = f.force.buf[0];
            out[1] = f.force.buf[1];
            out[2] = f.force.buf[2];
            out[3] = f.torque.buf[0];
            out[4] = f.torque.buf[1];
            out[5] = f.torque.buf[2];
        }
        mj_step(m, w.data);
        refresh(w);
    });
}

uint64_t hash(const PhysicsWorld *p) {
    if (!p)
        return 0;
    auto &w = *static_cast<const PhysicsWorld *>(p);
    try {
        auto &s = w.hash_state;
        mj_getState(w.mj, w.data, s.data(), mjSTATE_INTEGRATION);
        Hash h;
        for (double v : s)
            h.scalar(v);
        return h.value;
    } catch (...) {
        return 0;
    }
}

} // namespace

extern "C" SimResult sim_backend_get_api(uint32_t version, uint32_t size, PhysicsBackendApi *api) {
    if (!api || version != PHYSICS_BACKEND_ABI_VERSION || size != sizeof(*api))
        return SIM_UNSUPPORTED;
    *api = {};
    api->struct_size = sizeof(*api);
    api->abi_version = 5;
    api->capabilities = SIM_CAP_THREADS;
    api->integrators = (1u << (SIM_INTEGRATOR_RK4 + 1)) - 1;
    std::snprintf(api->name, sizeof api->name, "mujoco");
    std::snprintf(api->version, sizeof api->version, "%s", mj_versionString());
    api->world_create = world_create;
    api->world_destroy = [](PhysicsWorld *p) { delete p; };
    api->reset = reset;
    api->step = step;
    api->read_state = [](const PhysicsWorld *p, SimBodyState *out, uint32_t c, uint32_t *n) {
        return p ? copy_out(p->states, out, c, n) : SIM_INVALID;
    };
    api->read_contacts = [](const PhysicsWorld *p, SimContact *out, uint32_t c, uint32_t *n) {
        return p ? copy_out(p->contacts, out, c, n) : SIM_INVALID;
    };
    api->state_hash = hash;
    return SIM_OK;
}
