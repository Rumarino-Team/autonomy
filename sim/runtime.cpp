#include "runtime.hpp"
#include <dlfcn.h>
#include <filesystem>
#include <set>
#include <limits>
namespace sim {
static void runtime_identity() {}
Backend::Backend(const std::string &file)
    : path(std::filesystem::absolute(file).string()) {
  library_ = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!library_)
    throw std::runtime_error(dlerror());
  try {
    auto entry = reinterpret_cast<SimBackendGetApi>(
        dlsym(library_, "sim_backend_get_api"));
    require(entry, "missing sim_backend_get_api");
    require(entry(SIM_BACKEND_ABI_VERSION, sizeof(api), &api) == SIM_OK,
            "incompatible physics backend ABI");
    require(api.struct_size == sizeof(api) &&
                api.abi_version == SIM_BACKEND_ABI_VERSION && api.world_create &&
                api.world_destroy && api.reset && api.step && api.read_state &&
                api.read_contacts && api.state_hash,
            "incomplete backend API");
    Dl_info identity{};
    require(dladdr(reinterpret_cast<void *>(&runtime_identity), &identity) &&
                identity.dli_fname,
            "cannot locate simulation runtime");
    Hash build;
    auto plugin = file_digest(path.c_str()),
         runtime = file_digest(identity.dli_fname);
    require(plugin && runtime, "cannot fingerprint simulation binaries");
    build.scalar(plugin);
    build.scalar(runtime);
    build_fingerprint = build.value;
    require(api.capabilities & (SIM_CAP_THREADS | SIM_CAP_PROCESS_ISOLATED),
            "backend must support independent parallel worlds");
    require(api.name[sizeof(api.name) - 1] == 0 &&
                api.version[sizeof(api.version) - 1] == 0,
            "invalid backend identity");
  } catch (...) {
    dlclose(library_);
    library_ = nullptr;
    throw;
  }
}
Backend::~Backend() {
  if (library_)
    dlclose(library_);
}
void Backend::check(SimResult r, const char *e) const {
  if (r != SIM_OK)
    throw std::runtime_error(std::string(api.name) + ": " +
                             (e && *e ? e : "backend call failed"));
}
World::World(Scene s, std::shared_ptr<Backend> backend, ForceRegistry registry)
    : backend_(std::move(backend)), registry_(std::move(registry)),
      scene(std::move(s)) {
  require(bool(backend_), "missing backend");
  scene.validate();
  require(backend_->api.integrators & (1u << scene.header.integrator),
          "requested integrator is unsupported by backend");
  if (scene.header.native_hydrodynamics)
    require(backend_->api.capabilities & SIM_CAP_NATIVE_HYDRO,
            "backend does not support native hydrodynamics");
  for (auto &module : scene.modules)
    modules_.push_back(module.text);
  if (scene.header.native_hydrodynamics)
    std::erase(modules_, "hydrodynamics");
  std::set<std::string> names, contributions;
  if (scene.header.native_hydrodynamics)
    contributions.insert("hydrodynamics");
  for (auto &name : modules_) {
    require(names.insert(name).second, "duplicate force module");
    registry_.get(name);
    auto group = registry_.contribution(name);
    require(group.empty() || contributions.insert(group).second,
            "multiple providers for the same physical contribution");
  }
  require(contributions.contains("gravity"),
          "gravity compensation module is required");
  char error[1024]{};
  auto scenario = scene.binary();
  require(scenario.size() <= std::numeric_limits<uint32_t>::max(),
          "scene snapshot exceeds the backend argument");
  try {
    backend_->check(backend_->api.world_create(
                        reinterpret_cast<const char *>(scenario.data()),
                        uint32_t(scenario.size()), &world_, error,
                        sizeof(error)),
                    error);
    reset();
  } catch (...) {
    if (world_)
      backend_->api.world_destroy(world_);
    throw;
  }
}
World::~World() {
  if (world_)
    backend_->api.world_destroy(world_);
}
void World::read() {
  uint32_t count = 0;
  bodies.resize(scene.bodies.size());
  backend_->check(
      backend_->api.read_state(world_, bodies.data(), bodies.size(), &count),
      "");
  require(count == bodies.size(), "backend returned wrong body count");
  for (size_t i = 0; i < count; ++i)
    require(bodies[i].id == scene.bodies[i].id &&
                finite(bodies[i].pose.position) &&
                finite(bodies[i].pose.orientation) &&
                std::abs(norm2(bodies[i].pose.orientation) - 1) < 1e-5 &&
                finite(bodies[i].center_of_mass) &&
                finite(bodies[i].linear_velocity) &&
                finite(bodies[i].angular_velocity),
            "non-finite or invalid backend state");
  auto result = backend_->api.read_contacts(world_, nullptr, 0, &count);
  require(result == SIM_OK || result == SIM_BUFFER_TOO_SMALL,
          "cannot size contacts");
  contacts.resize(count);
  backend_->check(backend_->api.read_contacts(world_, contacts.data(),
                                              contacts.size(), &count),
                  "");
  for (auto &c : contacts)
    require(finite(c.position) && finite(c.normal) &&
                std::isfinite(c.normal_force),
            "non-finite contact");
}
void World::reset(const std::vector<SimBodyState> &state) {
  char error[1024]{};
  backend_->check(backend_->api.reset(world_,
                                      state.empty() ? nullptr : state.data(),
                                      state.size(), error, sizeof(error)),
                  error);
  tick = 0;
  module_state_.clear();
  rotors.assign(scene.thrusters.size(), {});
  commands.assign(scene.thrusters.size(), 0);
  input_commands = commands;
  fluid_cache_.assign(scene.bodies.size(), {});
  wrenches = fluid_cache_;
  read();
  previous_velocity_.clear();
  previous_angular_velocity_.clear();
  for (auto &b : bodies)
    previous_velocity_.push_back(b.linear_velocity);
  for (auto &b : bodies)
    previous_angular_velocity_.push_back(b.angular_velocity);
}
uint64_t World::timestamp_ns() const {
  __extension__ typedef unsigned __int128 TickProduct;
  TickProduct n = static_cast<TickProduct>(tick) *
                  scene.header.dt_numerator * 1000000000ULL;
  n /= scene.header.dt_denominator;
  require(n <= UINT64_MAX, "simulation timestamp overflow");
  return uint64_t(n);
}
void World::step(const std::vector<double> &input) {
  require(input.size() == commands.size(), "wrong thruster command count");
  require(tick < UINT64_MAX, "tick counter exhausted");
  commands = input;
  input_commands = input;
  for (double v : commands)
    require(std::isfinite(v) && v >= -1 && v <= 1,
            "commands must be finite in [-1,1]");
  for (const auto &f : scene.faults)
    if (tick >= f.start && tick < f.end) {
      if (f.kind == FaultKind::Dead || f.kind == FaultKind::Weak) {
        auto index = f.thruster;
        if (index < commands.size())
          commands[index] *= f.kind == FaultKind::Dead ? 0 : f.magnitude;
      }
    }
  for (size_t i = 0; i < wrenches.size(); ++i) {
    wrenches[i] = {scene.bodies[i].id, {}, {}};
    previous_velocity_[i] = bodies[i].linear_velocity;
    previous_angular_velocity_[i] = bodies[i].angular_velocity;
  }
  ForceContext context{scene, bodies, commands,     tick,
                       dt(),  rotors, fluid_cache_, module_state_};
  for (auto &module : modules_)
    registry_.get(module)(context, wrenches);
  for (auto &w : wrenches)
    require(finite(w.force) && finite(w.torque),
            "non-finite force model output");
  char error[1024]{};
  backend_->check(backend_->api.step(world_, wrenches.data(), wrenches.size(),
                                     error, sizeof(error)),
                  error);
  ++tick;
  read();
  for (auto &b : bodies) {
    require(norm(b.pose.position) <= scene.header.bounds.position,
            "position bound exceeded");
    require(norm(b.linear_velocity) <= scene.header.bounds.speed,
            "speed bound exceeded");
    require(norm(b.angular_velocity) <= scene.header.bounds.angular_speed,
            "angular speed bound exceeded");
  }
}
Observation World::observe() const {
  Observation o;
  const auto &c = scene.header.camera;
  const auto &b = bodies[c.body];
  o.camera = compose(b.pose, c.pose);
  o.imu = b.pose.orientation;
  o.gyro = rotate(conjugate(b.pose.orientation), b.angular_velocity);
  auto arm = b.pose.position - b.center_of_mass;
  auto angular_acceleration =
      (b.angular_velocity - previous_angular_velocity_[c.body]) / dt();
  auto origin_acceleration =
      (b.linear_velocity - previous_velocity_[c.body]) / dt() +
      cross(angular_acceleration, arm) +
      cross(b.angular_velocity, cross(b.angular_velocity, arm));
  o.accel = rotate(conjugate(b.pose.orientation),
                   origin_acceleration - scene.header.gravity);
  o.depth = o.camera.position.z;
  auto camera_inverse = conjugate(o.camera.orientation);
  double focal =
      c.width * .5 / std::tan(c.horizontal_fov * std::numbers::pi / 360);
  for (size_t i = 0; i < bodies.size(); ++i) {
    const auto &description = scene.bodies[i];
    if (!description.tracked)
      continue;
    SimVec3 lo{INFINITY, INFINITY, INFINITY},
        hi{-INFINITY, -INFINITY, -INFINITY};
    auto add = [&](SimVec3 p) {
      lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
      hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
    };
    for (uint32_t k = 0; k < description.shape_count; ++k) {
      auto &g = scene.shapes[description.shape_begin + k];
      if (g.kind == SIM_MESH) {
        for (uint32_t v = 0; v < g.vertex_count; ++v)
          add(point(g.pose, scene.vertices[g.vertex_begin + v]));
      } else {
        SimVec3 size = g.size;
        if (g.kind == SIM_SPHERE)
          size = {g.size.x, g.size.x, g.size.x};
        if (g.kind == SIM_CYLINDER)
          size.y = size.x;
        for (int k = 0; k < 8; ++k)
          add(point(g.pose, {k & 1 ? size.x : -size.x, k & 2 ? size.y : -size.y,
                             k & 4 ? size.z : -size.z}));
      }
    }
    ObjectObservation object{
        description.id,
        description.cls,
        {point(bodies[i].pose, (lo + hi) * .5), bodies[i].pose.orientation},
        hi - lo};
    double x0 = INFINITY, y0 = INFINITY, x1 = -INFINITY, y1 = -INFINITY;
    bool ahead = false;
    for (int k = 0; k < 8; ++k) {
      auto p =
          rotate(camera_inverse, point(bodies[i].pose, {k & 1 ? hi.x : lo.x,
                                                        k & 2 ? hi.y : lo.y,
                                                        k & 4 ? hi.z : lo.z}) -
                                     o.camera.position);
      if (p.z <= .05)
        continue;
      ahead = true;
      double x = c.width * .5 + focal * p.x / p.z,
             y = c.height * .5 + focal * p.y / p.z;
      x0 = std::min(x0, x);
      x1 = std::max(x1, x);
      y0 = std::min(y0, y);
      y1 = std::max(y1, y);
    }
    object.visible =
        ahead && x1 >= 0 && y1 >= 0 && x0 < c.width && y0 < c.height;
    if (object.visible) {
      object.x0 = std::clamp(x0, 0.0, double(c.width - 1));
      object.y0 = std::clamp(y0, 0.0, double(c.height - 1));
      object.x1 = std::clamp(x1, 0.0, double(c.width - 1));
      object.y1 = std::clamp(y1, 0.0, double(c.height - 1));
    }
    o.objects.push_back(object);
  }
  for (const auto &f : scene.faults)
    if (tick >= f.start && tick < f.end) {
      if (f.kind == FaultKind::Tracking) {
        o.position_ok = false;
        o.objects.clear();
      } else if (f.kind == FaultKind::Imu) {
        o.attitude_ok = false;
        o.imu_ok = false;
        o.gyro_ok = false;
      } else if (f.kind == FaultKind::Gyro)
        o.gyro.x += f.magnitude;
      else if (f.kind == FaultKind::Depth)
        o.depth += f.magnitude;
    }
  return o;
}
uint64_t World::state_hash() const {
  Hash h;
  h.scalar(tick);
  h.scalar(backend_->api.state_hash(world_));
  for (auto &b : bodies)
    h.state(b);
  for (auto &r : rotors) {
    h.scalar(r.omega);
    h.scalar(r.integral);
    h.scalar(r.torque);
    h.scalar(r.thrust);
  }
  for (double v : commands)
    h.scalar(v);
  for (auto v : previous_velocity_)
    h.vector(v);
  for (auto v : previous_angular_velocity_)
    h.vector(v);
  for (auto &f : fluid_cache_) {
    h.vector(f.force);
    h.vector(f.torque);
  }
  for (auto &[name, values] : module_state_) {
    h.bytes(name.data(), name.size());
    for (double v : values)
      h.scalar(v);
  }
  return h.value;
}
std::vector<double> generated_commands(const Scene &scene, uint64_t seed,
                                       uint64_t tick) {
  std::vector<double> out(scene.thrusters.size());
  for (size_t i = 0; i < out.size(); ++i)
    out[i] = (random(seed, 1000 + i, tick / 72) * 2 - 1) * .2;
  return out;
}
} // namespace sim
