#include "scene.hpp"
#include <dlfcn.h>
#include <iomanip>
#include <memory>
#include <mujoco/mujoco.h>
#include <sstream>
namespace {
using namespace sim;
struct Model {
  ModelStorage source;
  mjModel *model = nullptr;
  std::vector<int> ids;
  ~Model() {
    if (model)
      mj_deleteModel(model);
  }
};
struct World {
  const Model *model;
  mjData *data;
  std::vector<SimBodyState> states;
  std::vector<SimContact> contacts;
  ~World() { mj_deleteData(data); }
};
void xyz(std::ostream &out, SimVec3 v) {
  out << v.x << ' ' << v.y << ' ' << v.z;
}
void quaternion(std::ostream &out, SimQuat q) {
  out << q.w << ' ' << q.x << ' ' << q.y << ' ' << q.z;
}
void transform(std::ostream &out, SimPose p) {
  out << " pos=\"";
  xyz(out, p.position);
  out << "\" quat=\"";
  quaternion(out, p.orientation);
  out << '"';
}
void refresh(World &w) {
  auto m = w.model->model;
  auto d = w.data;
  mj_forward(m, d);
  w.states.clear();
  w.contacts.clear();
  for (size_t i = 0; i < w.model->ids.size(); ++i) {
    int id = w.model->ids[i];
    auto p = d->xpos + 3 * id, q = d->xquat + 4 * id, c = d->xipos + 3 * id;
    double v[6];
    mj_objectVelocity(m, d, mjOBJ_BODY, id, v, 0);
    w.states.push_back({w.model->source.views[i].id,
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
      auto it = std::find(w.model->ids.begin(), w.model->ids.end(), k);
      return it == w.model->ids.end()
                 ? UINT32_MAX
                 : w.model->source.views[it - w.model->ids.begin()].id;
    };
    w.contacts.push_back({id(a),
                          id(b),
                          {c.pos[0], c.pos[1], c.pos[2]},
                          {c.frame[0], c.frame[1], c.frame[2]},
                          f[0]});
  }
}
void apply_state(World &w, const SimBodyState *states, uint32_t count) {
  auto m = w.model->model;
  auto d = w.data;
  require((!states && count == 0) || (states && count == w.model->ids.size()),
          "reset needs all body states");
  mj_resetData(m, d);
  for (size_t i = 0; i < w.model->ids.size(); ++i) {
    auto &b = w.model->source.views[i];
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
    int id = w.model->ids[i];
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
SimResult model_create(const SimModelDesc *desc, void **out, char *error,
                       uint32_t n) {
  if (!out)
    return SIM_INVALID;
  *out = nullptr;
  if (desc && desc->native_hydrodynamics)
    return fail(error, n,
                "MuJoCo native hydrodynamics is not exposed by ABI v1",
                SIM_UNSUPPORTED);
  return guarded(error, n, [&] {
    validate(desc);
    auto model = std::make_unique<Model>();
    model->source = decode(encode(*desc));
    model->source.bind();
    std::ostringstream xml;
    xml << std::setprecision(17)
        << "<mujoco><compiler angle=\"radian\"/><option timestep=\""
        << double(desc->dt_numerator) / desc->dt_denominator << "\" gravity=\"";
    xyz(xml, desc->gravity);
    xml << "\" integrator=\"implicitfast\"><flag "
           "sleep=\"disable\"/></option><asset>";
    for (uint32_t i = 0; i < desc->body_count; ++i)
      for (uint32_t k = 0; k < desc->bodies[i].shape_count; ++k) {
        auto &g = desc->bodies[i].shapes[k];
        require(g.restitution == 0,
                "MuJoCo adapter currently requires zero restitution");
        if (g.kind != SIM_MESH)
          continue;
        xml << "<mesh name=\"m" << i << '_' << k << "\" vertex=\"";
        for (uint32_t v = 0; v < g.vertex_count; ++v) {
          xyz(xml, g.vertices[v]);
          xml << ' ';
        }
        xml << "\" face=\"";
        for (uint32_t t = 0; t < g.triangle_count; ++t) {
          auto f = g.triangles[t];
          xml << f.a << ' ' << f.b << ' ' << f.c << ' ';
        }
        xml << "\"/>";
      }
    xml << "</asset><worldbody>";
    for (uint32_t i = 0; i < desc->body_count; ++i) {
      auto &b = desc->bodies[i];
      xml << "<body name=\"b" << b.id << '"';
      transform(xml, b.initial_pose);
      if (b.motion == SIM_KINEMATIC)
        xml << " mocap=\"true\"";
      xml << '>';
      if (b.motion == SIM_DYNAMIC) {
        xml << "<freejoint/><inertial";
        transform(xml, b.center_of_mass);
        double placeholder = std::max({b.inertia.x, b.inertia.y, b.inertia.z});
        xml << " mass=\"" << b.mass << "\" diaginertia=\"" << placeholder << ' '
            << placeholder << ' ' << placeholder << "\"/>";
      }
      for (uint32_t k = 0; k < b.shape_count; ++k) {
        auto &g = b.shapes[k];
        xml << "<geom name=\"g" << i << '_' << k << '"';
        transform(xml, g.pose);
        xml << " density=\"0\" contype=\"" << g.collisions
            << "\" conaffinity=\"" << g.collisions << "\" friction=\""
            << g.friction << " 0 0\"";
        if (g.kind == SIM_MESH)
          xml << " type=\"mesh\" mesh=\"m" << i << '_' << k << '"';
        else {
          xml << " type=\""
              << (g.kind == SIM_BOX      ? "box"
                  : g.kind == SIM_SPHERE ? "sphere"
                                         : "cylinder")
              << "\" size=\"";
          if (g.kind == SIM_BOX)
            xyz(xml, g.size);
          else if (g.kind == SIM_SPHERE)
            xml << g.size.x;
          else
            xml << g.size.x << ' ' << g.size.z;
          xml << '"';
        }
        xml << "/>";
      }
      xml << "</body>";
    }
    xml << "</worldbody></mujoco>";
    char message[2048]{};
    mjSpec *spec =
        mj_parseXMLString(xml.str().c_str(), nullptr, message, sizeof(message));
    if (!spec)
      throw std::runtime_error(message);
    model->model = mj_compile(spec, nullptr);
    if (!model->model) {
      std::string e = mjs_getError(spec);
      mj_deleteSpec(spec);
      throw std::runtime_error(e);
    }
    mj_deleteSpec(spec);
    for (uint32_t i = 0; i < desc->body_count; ++i) {
      auto &b = desc->bodies[i];
      int id = mj_name2id(model->model, mjOBJ_BODY,
                          ("b" + std::to_string(b.id)).c_str());
      model->ids.push_back(id);
      if (b.motion == SIM_DYNAMIC) {
        auto p = model->model->body_inertia + 3 * id;
        p[0] = b.inertia.x;
        p[1] = b.inertia.y;
        p[2] = b.inertia.z;
      }
    }
    switch (desc->integrator) {
    case SIM_INTEGRATOR_EULER:
      model->model->opt.integrator = mjINT_EULER;
      break;
    case SIM_INTEGRATOR_IMPLICIT:
      model->model->opt.integrator = mjINT_IMPLICIT;
      break;
    case SIM_INTEGRATOR_RK4:
      model->model->opt.integrator = mjINT_RK4;
      break;
    default:
      model->model->opt.integrator = mjINT_IMPLICITFAST;
      break;
    }
    auto data = mj_makeData(model->model);
    require(data, "cannot allocate MuJoCo constants");
    mj_setConst(model->model, data);
    mj_deleteData(data);
    *out = model.release();
  });
}
SimResult world_create(const void *p, void **out, char *e, uint32_t n) {
  if (!p || !out)
    return SIM_INVALID;
  *out = nullptr;
  return guarded(e, n, [&] {
    auto model = static_cast<const Model *>(p);
    auto w = std::make_unique<World>();
    w->model = model;
    w->data = mj_makeData(model->model);
    require(w->data, "cannot allocate MuJoCo world");
    apply_state(*w, nullptr, 0);
    *out = w.release();
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
    auto m = w.model->model;
    mju_zero(w.data->xfrc_applied, 6 * m->nbody);
    std::set<uint32_t> seen;
    for (uint32_t i = 0; i < count; ++i) {
      auto &f = input[i];
      require(finite(f.force) && finite(f.torque) && seen.insert(f.id).second,
              "invalid or duplicate wrench");
      auto &source = w.model->source.views;
      auto it = std::find_if(source.begin(), source.end(),
                             [&](auto &b) { return b.id == f.id; });
      require(it != source.end(), "unknown wrench body");
      auto out = w.data->xfrc_applied + 6 * w.model->ids[it - source.begin()];
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
    std::vector<mjtNum> s(mj_stateSize(w.model->model, mjSTATE_INTEGRATION));
    mj_getState(w.model->model, w.data, s.data(), mjSTATE_INTEGRATION);
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
  api->model_create = model_create;
  api->model_destroy = [](void *p) { delete static_cast<Model *>(p); };
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
