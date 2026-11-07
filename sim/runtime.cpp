#include "runtime.hpp"
#include <dlfcn.h>
#include <filesystem>
#include <limits>
#include <set>
namespace sim {
static void runtime_identity() {}
static Main &owned_model(const MainPtr &p) {
  require(bool(p), "missing model database");
  return *p;
}
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
                api.abi_version == SIM_BACKEND_ABI_VERSION &&
                api.world_create && api.world_destroy && api.reset &&
                api.step && api.read_state && api.read_contacts &&
                api.state_hash,
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
World::World(MainPtr s, std::shared_ptr<Backend> backend,
             ForceRegistry registry, std::pmr::memory_resource *upstream)
    : model_(std::move(s)), memory_(upstream), backend_(std::move(backend)),
      registry_(std::move(registry)), modules_(&memory_.pool),
      module_state_(&memory_.pool), fluid_cache_(&memory_.pool),
      previous_velocity_(&memory_.pool),
      previous_angular_velocity_(&memory_.pool),
      observation_objects_(&memory_.pool), scene(owned_model(model_)),
      bodies(&memory_.pool), contacts(&memory_.pool),
      wrenches(&memory_.pool) {
  require(bool(backend_), "missing backend");
  if (!model_->frozen())
    model_->finalize();
  scene.validate();
  require(backend_->api.integrators & (1u << scene.header.integrator),
          "requested integrator is unsupported by backend");
  if (scene.header.native_hydrodynamics)
    require(backend_->api.capabilities & SIM_CAP_NATIVE_HYDRO,
            "backend does not support native hydrodynamics");
  for (auto &module : scene.run.modules)
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
  const auto view = scene.view();
  try {
    backend_->check(
        backend_->api.world_create(&view, &world_, error, sizeof(error)),
        error);
    reset();
  } catch (...) {
    if (world_)
      backend_->api.world_destroy(world_);
    throw;
  }
}
World::World(const Main &source, std::shared_ptr<Backend> backend,
             ForceRegistry registry, std::pmr::memory_resource *upstream)
    : World(source.clone(), std::move(backend), std::move(registry), upstream) {
}
World::~World() {
  if (world_)
    backend_->api.world_destroy(world_);
}
void World::read() {
  uint32_t count = 0;
  bodies.resize(scene.objects.size());
  backend_->check(
      backend_->api.read_state(world_, bodies.data(), bodies.size(), &count),
      "");
  require(count == bodies.size(), "backend returned wrong body count");
  for (size_t i = 0; i < count; ++i)
    require(bodies[i].id == scene.objects[i].id &&
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
void World::reset(std::span<const SimBodyState> state) {
  char error[1024]{};
  backend_->check(backend_->api.reset(world_,
                                      state.empty() ? nullptr : state.data(),
                                      state.size(), error, sizeof(error)),
                  error);
  tick = 0;
  module_state_.clear();
  rotors = std::span(rotor_storage_).first(scene.robot.motor_count);
  commands = std::span(command_storage_).first(scene.robot.motor_count);
  input_commands = std::span(input_storage_).first(scene.robot.motor_count);
  std::fill(rotors.begin(), rotors.end(), RotorState{});
  std::fill(commands.begin(), commands.end(), 0);
  std::fill(input_commands.begin(), input_commands.end(), 0);
  fluid_cache_.assign(scene.objects.size(), {});
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
  TickProduct n = static_cast<TickProduct>(tick) * scene.header.dt_numerator *
                  1000000000ULL;
  n /= scene.header.dt_denominator;
  require(n <= UINT64_MAX, "simulation timestamp overflow");
  return uint64_t(n);
}
void World::step(std::span<const double> input) {
  require(input.size() == commands.size(), "wrong thruster command count");
  require(tick < UINT64_MAX, "tick counter exhausted");
  std::copy(input.begin(), input.end(), commands.begin());
  std::copy(input.begin(), input.end(), input_commands.begin());
  for (double v : commands)
    require(std::isfinite(v) && v >= -1 && v <= 1,
            "commands must be finite in [-1,1]");
  for (const auto &f : scene.run.faults)
    if (tick >= f.start && tick < f.end) {
      if (f.kind == FaultKind::Dead || f.kind == FaultKind::Weak) {
        auto index = f.thruster;
        if (index < commands.size())
          commands[index] *= f.kind == FaultKind::Dead ? 0 : f.magnitude;
      }
    }
  for (size_t i = 0; i < wrenches.size(); ++i) {
    wrenches[i] = {scene.objects[i].id, {}, {}};
    previous_velocity_[i] = bodies[i].linear_velocity;
    previous_angular_velocity_[i] = bodies[i].angular_velocity;
  }
  ForceContext context{scene,        bodies,        commands,
                       tick,         dt(),          rotors,
                       fluid_cache_, module_state_, &memory_.scratch};
  // Common force modules use the imported body calibration, hull geometry,
  // thruster mounts and latest body state. Both backends receive the resulting
  // per-body forces/torques; the selected engine owns integration and contacts.
  for (auto &module : modules_)
    registry_.get(module)(context, wrenches);
  memory_.scratch.release();
  for (auto &w : wrenches)
    require(finite(w.force) && finite(w.torque),
            "non-finite force model output");
  char error[1024]{};
  backend_->check(backend_->api.step(world_, wrenches.data(), wrenches.size(),
                                     error, sizeof(error)),
                  error);
  ++tick;
  // Read the engine's new state before producing controller observations or
  // publishing preview poses. Blender's display does not advance physics.
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
  observation_objects_.clear();
  const auto &c = scene.robot.camera;
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
    const auto &description = scene.objects[i];
    if (!description.tracked)
      continue;
    SimVec3 lo{INFINITY, INFINITY, INFINITY},
        hi{-INFINITY, -INFINITY, -INFINITY};
    auto add = [&](SimVec3 p) {
      lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
      hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
    };
    for (const auto &collider : scene.objects.block(i).colliders) {
      const auto &g = collider.data;
      if (g.kind == SIM_MESH) {
        for (uint32_t v = 0; v < g.vertex_count; ++v)
          add(point(g.pose, scene.positions(collider)[v]));
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
    observation_objects_.push_back(object);
  }
  for (const auto &f : scene.run.faults)
    if (tick >= f.start && tick < f.end) {
      if (f.kind == FaultKind::Tracking) {
        o.position_ok = false;
        observation_objects_.clear();
      } else if (f.kind == FaultKind::Imu) {
        o.attitude_ok = false;
        o.imu_ok = false;
        o.gyro_ok = false;
      } else if (f.kind == FaultKind::Gyro)
        o.gyro.x += f.magnitude;
      else if (f.kind == FaultKind::Depth)
        o.depth += f.magnitude;
    }
  o.objects = observation_objects_;
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
std::vector<double> generated_commands(const Main &scene, uint64_t seed,
                                       uint64_t tick) {
  std::vector<double> out(scene.robot.thrusters().size());
  for (size_t i = 0; i < out.size(); ++i)
    out[i] = (random(seed, 1000 + i, tick / 72) * 2 - 1) * .2;
  return out;
}
} // namespace sim
