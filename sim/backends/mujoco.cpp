#include "image.hpp"
#include <dlfcn.h>
#include <memory>
#include <mujoco/mujoco.h>
#include <set>
namespace {
using namespace sim;
struct BodyBinding {
  using Value = int;
  DataBlock id;
  int data = -1;
  const ObjectBlock *body = nullptr;
};
struct ShapeBinding {
  using Value = int;
  DataBlock id;
  int data = -1;
  const Collider *shape = nullptr;
};
struct World {
  Memory memory;
  bool frozen = false;
  MainPtr owned_source;
  const Main &source;
  mjModel *mj = nullptr;
  mjData *data = nullptr;
  Records<BodyBinding, DataKind::MuJoCoBodyBinding> ids;
  Records<ShapeBinding, DataKind::MuJoCoShapeBinding> geoms;
  std::pmr::vector<SimBodyState> states;
  std::pmr::vector<SimContact> contacts;
  std::pmr::vector<uint8_t> seen;
  mutable std::pmr::vector<mjtNum> hash_state;
  explicit World(MainPtr model)
      : owned_source(std::move(model)), source(*owned_source),
        ids(memory, this, frozen), geoms(memory, this, frozen),
        states(&memory.pool), contacts(&memory.pool), seen(&memory.pool),
        hash_state(&memory.pool) {}
  ~World() {
    if (data)
      mj_deleteData(data);
    if (mj)
      mj_deleteModel(mj);
  }
};
void xyz(double *out, SimVec3 v) {
  out[0] = v.x;
  out[1] = v.y;
  out[2] = v.z;
}
void quaternion(double *out, SimQuat q) {
  // The simulator ABI stores xyzw; MuJoCo stores wxyz. Coordinates have already
  // been converted from Blender to NED before they reach this backend.
  out[0] = q.w;
  out[1] = q.x;
  out[2] = q.y;
  out[3] = q.z;
}
void transform(double *pos, double *quat, SimPose p) {
  xyz(pos, p.position);
  quaternion(quat, p.orientation);
}
void refresh(World &w) {
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
    w.states.push_back({w.source.objects[i].id,
                        {{p[0], p[1], p[2]}, {q[1], q[2], q[3], q[0]}},
                        {c[0], c[1], c[2]},
                        {v[3], v[4], v[5]},
                        {v[0], v[1], v[2]}});
  }
  for (int i = 0; i < d->ncon; ++i) {
    auto &c = d->contact[i];
    int a = m->geom_bodyid[c.geom[0]], b = m->geom_bodyid[c.geom[1]];
    if (a == b)
      continue;
    double f[6]{};
    mj_contactForce(m, d, i, f);
    auto id = [&](int k) {
      auto it = std::find(w.ids.begin(), w.ids.end(), k);
      return it == w.ids.end() ? UINT32_MAX
                               : w.source.objects[it - w.ids.begin()].id;
    };
    w.contacts.push_back({id(a),
                          id(b),
                          {c.pos[0], c.pos[1], c.pos[2]},
                          {c.frame[0], c.frame[1], c.frame[2]},
                          f[0]});
  }
}
void apply_state(World &w, const SimBodyState *states, uint32_t count) {
  auto m = w.mj;
  auto d = w.data;
  require((!states && count == 0) || (states && count == w.ids.size()),
          "reset needs all body states");
  mj_resetData(m, d);
  for (size_t i = 0; i < w.ids.size(); ++i) {
    auto &b = w.source.objects[i];
    SimBodyState s = states ? states[i]
                            : SimBodyState{b.id,
                                           b.initial_pose,
                                           {},
                                           b.initial_linear_velocity,
                                           b.initial_angular_velocity};
    require(s.id == b.id && finite(s.linear_velocity) &&
                finite(s.angular_velocity),
            "invalid reset body");
    valid_pose(s.pose);
    int id = w.ids[i];
    if (b.motion == SIM_DYNAMIC) {
      int joint = m->body_jntadr[id], qa = m->jnt_qposadr[joint],
          va = m->jnt_dofadr[joint];
      d->qpos[qa] = s.pose.position.x;
      d->qpos[qa + 1] = s.pose.position.y;
      d->qpos[qa + 2] = s.pose.position.z;
      d->qpos[qa + 3] = s.pose.orientation.w;
      d->qpos[qa + 4] = s.pose.orientation.x;
      d->qpos[qa + 5] = s.pose.orientation.y;
      d->qpos[qa + 6] = s.pose.orientation.z;
      // MuJoCo freejoint translation is the body origin, while the ABI velocity
      // is at COM.
      auto offset = rotate(s.pose.orientation, b.center_of_mass.position);
      auto velocity = s.linear_velocity - cross(s.angular_velocity, offset);
      d->qvel[va] = velocity.x;
      d->qvel[va + 1] = velocity.y;
      d->qvel[va + 2] = velocity.z;
      auto angular = rotate(conjugate(s.pose.orientation), s.angular_velocity);
      d->qvel[va + 3] = angular.x;
      d->qvel[va + 4] = angular.y;
      d->qvel[va + 5] = angular.z;
    } else if (b.motion == SIM_KINEMATIC) {
      int k = m->body_mocapid[id];
      d->mocap_pos[3 * k] = s.pose.position.x;
      d->mocap_pos[3 * k + 1] = s.pose.position.y;
      d->mocap_pos[3 * k + 2] = s.pose.position.z;
      d->mocap_quat[4 * k] = s.pose.orientation.w;
      d->mocap_quat[4 * k + 1] = s.pose.orientation.x;
      d->mocap_quat[4 * k + 2] = s.pose.orientation.y;
      d->mocap_quat[4 * k + 3] = s.pose.orientation.z;
    } else if (states)
      require(norm(s.pose.position - b.initial_pose.position) == 0 &&
                  s.pose.orientation.x == b.initial_pose.orientation.x &&
                  s.pose.orientation.y == b.initial_pose.orientation.y &&
                  s.pose.orientation.z == b.initial_pose.orientation.z &&
                  s.pose.orientation.w == b.initial_pose.orientation.w,
              "static bodies cannot move on reset; compile a new model");
  }
  refresh(w);
}
SimResult world_create(const SimSceneView *scene, void **out, char *error,
                       uint32_t n) {
  if (!out)
    return SIM_INVALID;
  *out = nullptr;
  if (!scene)
    return SIM_INVALID;
  return guarded(error, n, [&] {
    auto world = std::make_unique<World>(Main::from_view(*scene));
    const auto &desc = world->source.header;
    require(!desc.native_hydrodynamics,
            "MuJoCo does not expose native hydrodynamics");
    // mjSpec is MuJoCo's editable model description. Fill its numeric fields
    // and asset buffers directly; mj_compile performs the model processing
    // without serializing or parsing XML. The deleter also handles failures.
    std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(mj_makeSpec(),
                                                           mj_deleteSpec);
    require(spec != nullptr, "cannot allocate MuJoCo spec");
    spec->compiler.degree = 0;
    spec->option.timestep = double(desc.dt_numerator) / desc.dt_denominator;
    xyz(spec->option.gravity, desc.gravity);
    spec->option.integrator = mjINT_IMPLICITFAST;
    spec->option.enableflags &= ~mjENBL_SLEEP;
    auto root = mjs_findBody(spec.get(), "world");
    for (size_t i = 0; i < world->source.objects.size(); ++i) {
      auto &b = world->source.objects[i];
      // A static body has no joint. A dynamic body gets six free degrees of
      // freedom. A kinematic body is positioned through MuJoCo's mocap arrays.
      // Generated names preserve the ABI's stable IDs across compilation.
      auto body = mjs_addBody(root, nullptr);
      mjs_setName(body->element, ("b" + std::to_string(b.id)).c_str());
      transform(body->pos, body->quat, b.initial_pose);
      body->mocap = b.motion == SIM_KINEMATIC;
      if (b.motion == SIM_DYNAMIC) {
        mjs_addFreeJoint(body);
        body->explicitinertial = true;
        transform(body->ipos, body->iquat, b.center_of_mass);
        body->mass = b.mass;
        // Effective calibrated inertia can violate the rigid-body triangle
        // inequality checked by the compiler. Compile with valid isotropic
        // moments, then install the requested principal moments below.
        double placeholder = std::max({b.inertia.x, b.inertia.y, b.inertia.z});
        xyz(body->inertia, {placeholder, placeholder, placeholder});
      }
      for (uint32_t k = 0; k < b.shape_count; ++k) {
        const auto &collider = world->source.objects.block(i).colliders[k];
        const auto &g = collider.data;
        require(g.restitution == 0,
                "MuJoCo adapter currently requires zero restitution");
        auto geom = mjs_addGeom(body, nullptr);
        auto suffix = std::to_string(i) + '_' + std::to_string(k);
        mjs_setName(geom->element, ("g" + suffix).c_str());
        // Geom poses are relative to the body origin. Mass and inertia come
        // from the body calibration, so collider volume contributes no mass.
        transform(geom->pos, geom->quat, g.pose);
        geom->density = 0;
        geom->contype = geom->conaffinity = g.collisions;
        xyz(geom->friction, {g.friction, 0, 0});
        if (g.kind == SIM_MESH) {
          auto mesh = mjs_addMesh(spec.get(), nullptr);
          auto name = "m" + suffix;
          mjs_setName(mesh->element, name.c_str());
          // Mesh asset buffers store float xyz triples and int index triples.
          // Copy this shape's ranges; triangle indices are already local to
          // those vertices, even when the scene contains several meshes.
          auto &vertices = *mesh->uservert;
          vertices.reserve(size_t(g.vertex_count) * 3);
          for (uint32_t v = 0; v < g.vertex_count; ++v) {
            auto &p = world->source.positions(collider)[v];
            vertices.insert(vertices.end(),
                            {float(p.x), float(p.y), float(p.z)});
          }
          auto &faces = *mesh->userface;
          faces.reserve(size_t(g.triangle_count) * 3);
          for (uint32_t t = 0; t < g.triangle_count; ++t) {
            auto &f = world->source.faces(collider)[t];
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
            geom->size[0] = g.size.x;
          else
            // MuJoCo uses [radius, half length]; the ABI keeps half length in
            // z.
            xyz(geom->size, {g.size.x, g.size.z, 0});
        }
      }
    }
    world->mj = mj_compile(spec.get(), nullptr);
    if (!world->mj)
      throw std::runtime_error(mjs_getError(spec.get()));
    // The compiled mjModel owns its arrays independently of the builder.
    spec.reset();
    for (auto &b : world->source.objects) {
      int id = mj_name2id(world->mj, mjOBJ_BODY,
                          ("b" + std::to_string(b.id)).c_str());
      world->ids.push_back(id);
      world->ids.block(world->ids.size() - 1).body =
          &world->source.objects.block(world->ids.size() - 1);
      if (b.motion == SIM_DYNAMIC) {
        auto p = world->mj->body_inertia + 3 * id;
        p[0] = b.inertia.x;
        p[1] = b.inertia.y;
        p[2] = b.inertia.z;
      }
    }
    for (size_t i = 0; i < world->source.objects.size(); ++i) {
      auto &b = world->source.objects[i];
      for (uint32_t k = 0; k < b.shape_count; ++k) {
        auto name = "g" + std::to_string(i) + '_' + std::to_string(k);
        world->geoms.push_back(mj_name2id(world->mj, mjOBJ_GEOM, name.c_str()));
        world->geoms.block(world->geoms.size() - 1).shape =
            &world->source.objects.block(i).colliders[k];
      }
    }
    world->seen.resize(world->source.objects.size());
    world->states.reserve(world->source.objects.size());
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
SimResult reset(void *p, const SimBodyState *s, uint32_t c, char *e,
                uint32_t n) {
  if (!p)
    return SIM_INVALID;
  return guarded(e, n, [&] { apply_state(*static_cast<World *>(p), s, c); });
}
SimResult step(void *p, const SimWrench *input, uint32_t count, char *e,
               uint32_t n) {
  if (!p || (!input && count))
    return SIM_INVALID;
  return guarded(e, n, [&] {
    auto &w = *static_cast<World *>(p);
    auto m = w.mj;
    // The host calculates thruster/fluid forces and torques at each body's
    // COM in world axes. Replace last tick's applied wrenches, then let MuJoCo
    // integrate those forces together with gravity and contact constraints.
    mju_zero(w.data->xfrc_applied, 6 * m->nbody);
    std::fill(w.seen.begin(), w.seen.end(), 0);
    for (uint32_t i = 0; i < count; ++i) {
      auto &f = input[i];
      require(finite(f.force) && finite(f.torque),
              "invalid or duplicate wrench");
      auto index = w.source.body_index(f.id);
      require(!w.seen[index], "duplicate wrench");
      w.seen[index] = 1;
      auto out = w.data->xfrc_applied + 6 * w.ids[index];
      out[0] = f.force.x;
      out[1] = f.force.y;
      out[2] = f.force.z;
      out[3] = f.torque.x;
      out[4] = f.torque.y;
      out[5] = f.torque.z;
    }
    mj_step(m, w.data);
    refresh(w);
  });
}
uint64_t hash(const void *p) {
  if (!p)
    return 0;
  auto &w = *static_cast<const World *>(p);
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
extern "C" SimResult sim_backend_get_api(uint32_t version, uint32_t size,
                                         SimBackendApi *api) {
  if (!api || version != SIM_BACKEND_ABI_VERSION || size != sizeof(*api))
    return SIM_UNSUPPORTED;
  *api = {};
  api->struct_size = sizeof(*api);
  api->abi_version = version;
  api->capabilities = SIM_CAP_THREADS;
  api->integrators = (1u << (SIM_INTEGRATOR_RK4 + 1)) - 1;
  std::snprintf(api->name, sizeof(api->name), "mujoco");
  Dl_info identity{};
  uint64_t digest =
      dladdr(reinterpret_cast<void *>(&mj_versionString), &identity) &&
              identity.dli_fname
          ? file_digest(identity.dli_fname)
          : 0;
  if (!digest)
    return SIM_BACKEND_ERROR;
  std::snprintf(api->version, sizeof(api->version), "%s/%016llx",
                mj_versionString(), static_cast<unsigned long long>(digest));
  api->world_create = world_create;
  api->world_destroy = [](void *p) { delete static_cast<World *>(p); };
  api->reset = reset;
  api->step = step;
  api->read_state = [](const void *p, SimBodyState *out, uint32_t c,
                       uint32_t *n) {
    return p ? copy_out(static_cast<const World *>(p)->states, out, c, n)
             : SIM_INVALID;
  };
  api->read_contacts = [](const void *p, SimContact *out, uint32_t c,
                          uint32_t *n) {
    return p ? copy_out(static_cast<const World *>(p)->contacts, out, c, n)
             : SIM_INVALID;
  };
  api->state_hash = hash;
  return SIM_OK;
}
