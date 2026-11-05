#include "runtime.hpp"
#include <dlfcn.h>
#include <limits>
#include <numbers>
namespace sim {
static void runtime_identity() {}
Backend::Backend(const std::filesystem::path &p)
    : path(std::filesystem::absolute(p)) {
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
                api.model_create && api.model_destroy && api.world_create &&
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
  scene.model.bind();
  validate(&scene.model.value);
  require(backend_->api.integrators & (1u << scene.model.value.integrator),
          "requested integrator is unsupported by backend");
  if (scene.model.value.native_hydrodynamics)
    require(backend_->api.capabilities & SIM_CAP_NATIVE_HYDRO,
            "backend does not support native hydrodynamics");
  modules_ = scene.force_modules;
  if (scene.model.value.native_hydrodynamics)
    std::erase(modules_, "hydrodynamics");
  std::set<std::string> names, contributions;
  if (scene.model.value.native_hydrodynamics)
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
  backend_->check(backend_->api.model_create(&scene.model.value, &model_, error,
                                             sizeof(error)),
                  error);
  try {
    backend_->check(
        backend_->api.world_create(model_, &world_, error, sizeof(error)),
        error);
    reset();
  } catch (...) {
    if (world_)
      backend_->api.world_destroy(world_);
    backend_->api.model_destroy(model_);
    throw;
  }
}
World::~World() {
  if (world_)
    backend_->api.world_destroy(world_);
  if (model_)
    backend_->api.model_destroy(model_);
}
void World::read() {
  uint32_t count = 0;
  bodies.resize(scene.model.bodies.size());
  backend_->check(
      backend_->api.read_state(world_, bodies.data(), bodies.size(), &count),
      "");
  require(count == bodies.size(), "backend returned wrong body count");
  for (size_t i = 0; i < count; ++i)
    require(bodies[i].id == scene.model.bodies[i].value.id &&
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
  fluid_cache_.assign(scene.model.bodies.size(), {});
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
                  scene.model.value.dt_numerator * 1000000000ULL;
  n /= scene.model.value.dt_denominator;
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
      if (f.kind == Scene::FaultKind::Dead || f.kind == Scene::FaultKind::Weak) {
        auto index = f.thruster;
        if (index < commands.size())
          commands[index] *= f.kind == Scene::FaultKind::Dead ? 0 : f.magnitude;
      }
    }
  for (size_t i = 0; i < wrenches.size(); ++i) {
    wrenches[i] = {scene.model.bodies[i].value.id, {}, {}};
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
    require(norm(b.pose.position) <= scene.bounds.position,
            "position bound exceeded");
    require(norm(b.linear_velocity) <= scene.bounds.speed,
            "speed bound exceeded");
    require(norm(b.angular_velocity) <= scene.bounds.angular_speed,
            "angular speed bound exceeded");
  }
}
Observation World::observe() const {
  Observation o;
  const auto &c = scene.camera;
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
                   origin_acceleration - scene.model.value.gravity);
  o.depth = o.camera.position.z;
  auto camera_inverse = conjugate(o.camera.orientation);
  double focal =
      c.width * .5 / std::tan(c.horizontal_fov * std::numbers::pi / 360);
  for (size_t i = 0; i < bodies.size(); ++i) {
    const auto &description = scene.model.bodies[i];
    if (!description.tracked)
      continue;
    SimVec3 lo{INFINITY, INFINITY, INFINITY},
        hi{-INFINITY, -INFINITY, -INFINITY};
    auto add = [&](SimVec3 p) {
      lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
      hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
    };
    for (auto &g : description.views)
      if (g.kind == SIM_MESH) {
        for (uint32_t v = 0; v < g.vertex_count; ++v)
          add(point(g.pose, g.vertices[v]));
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
    ObjectObservation object{
        description.value.id,
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
      if (f.kind == Scene::FaultKind::Tracking) {
        o.position_ok = false;
        o.objects.clear();
      } else if (f.kind == Scene::FaultKind::Imu) {
        o.attitude_ok = false;
        o.imu_ok = false;
        o.gyro_ok = false;
      } else if (f.kind == Scene::FaultKind::Gyro)
        o.gyro.x += f.magnitude;
      else if (f.kind == Scene::FaultKind::Depth)
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
Json World::sample() const {
  Json j = {{"t", "sample"},
            {"step", tick},
            {"timestamp_ns", timestamp_ns()},
            {"hash", state_hash()},
            {"bodies", Json::array()},
            {"thr", commands},
            {"input_commands", input_commands},
            {"rotors", Json::array()},
            {"wrenches", Json::array()}};
  for (auto &b : bodies)
    j["bodies"].push_back(json(b));
  for (auto &r : rotors)
    j["rotors"].push_back({r.omega, r.integral, r.torque, r.thrust});
  for (auto &w : wrenches)
    j["wrenches"].push_back(
        {{"id", w.id}, {"force", json(w.force)}, {"torque", json(w.torque)}});
  auto o = observe();
  j["camera"] = json(o.camera);
  j["gyro"] = json(o.gyro);
  j["accel"] = json(o.accel);
  j["depth"] = o.depth;
  j["position_ok"] = o.position_ok;
  j["attitude_ok"] = o.attitude_ok;
  return j;
}
Json draw_case(Json j, uint64_t seed) {
  auto u = [&](uint64_t channel, double lo, double hi) {
    return lo + (hi - lo) * random(seed, channel);
  };
  for (auto &b : j["bodies"]) {
    uint64_t channel = uint64_t(b.at("id").get<uint32_t>()) * 32;
    bool dynamic = b.value("motion", 0u) == SIM_DYNAMIC;
    if (!dynamic && !b.value("fuzz", false))
      continue;
    auto p = pose(b.value("pose", Json::object()));
    p.position =
        p.position + SimVec3{u(channel, -.2, .2), u(channel + 1, -.2, .2),
                             u(channel + 2, -.1, .1)};
    p.orientation = multiply(p.orientation, euler(u(channel + 3, -.05, .05),
                                                  u(channel + 4, -.05, .05),
                                                  u(channel + 5, -.15, .15)));
    b["pose"] = json(p);
    if (dynamic) {
      b["dry_mass"] =
          b.value("dry_mass", b.value("mass", 1.0)) * u(channel + 6, .97, 1.03);
      b["volume"] = b.value("volume", 0.0) * u(channel + 7, .97, 1.03);
      b["linear_velocity"] =
          json(vec(b.value("linear_velocity", Json{0, 0, 0})) +
               SimVec3{u(channel + 8, -.2, .2), u(channel + 9, -.2, .2),
                       u(channel + 10, -.2, .2)});
      b["angular_velocity"] =
          json(vec(b.value("angular_velocity", Json{0, 0, 0})) +
               SimVec3{u(channel + 11, -.2, .2), u(channel + 12, -.2, .2),
                       u(channel + 13, -.2, .2)});
    }
  }
  double speed = u(900, 0, .15),
         angle = u(901, -std::numbers::pi, std::numbers::pi);
  j["current"] = {speed * std::cos(angle), speed * std::sin(angle), 0};
  if (j.contains("hull"))
    for (auto &h : j["hull"]) {
      double scale = u(910 + h.at("body").get<uint32_t>(), .7, 1.4);
      h["drag"] = json(vec(h.at("drag")) * scale);
      h["skin"] = json(vec(h.at("skin")) * scale);
      h["volume"] =
          h.value("volume", 0.0) *
          u(uint64_t(h.at("body").get<uint32_t>()) * 32 + 7, .97, 1.03);
    }
  j["faults"] = j.value("faults", Json::array());
  if (j.contains("thrusters"))
    for (size_t i = 0; i < j["thrusters"].size(); ++i) {
      auto &thruster = j["thrusters"][i];
      if (!thruster.contains("spec"))
        thruster["spec"] = Json::object();
      auto &spec = thruster["spec"];
      uint64_t channel =
          20000 + uint64_t(thruster.at("body").get<uint32_t>()) * 4096 + i * 8;
      double scale = u(channel, .8, 1.2);
      spec["thrust_coeff"] = spec.value("thrust_coeff", .48) * scale;
      spec["torque_coeff"] = spec.value("torque_coeff", .05) * scale;
      spec["rotor_inertia"] =
          spec.value("rotor_inertia", .00146468) * u(channel + 1, .7, 1.5);
    }
  auto thrusters = j.value("thrusters", Json::array());
  if (!thrusters.empty()) {
    uint64_t start = uint64_t(u(930, 36, 360));
    j["faults"].push_back({{"kind", random(seed, 933) < .25 ? "dead" : "weak"},
                           {"start_tick", start},
                           {"end_tick", start + 180},
                           {"thruster", uint32_t(u(931, 0, thrusters.size()))},
                           {"magnitude", u(932, .3, .7)}});
  }
  const char *sensor_faults[] = {"tracking", "imu", "gyro", "depth"};
  auto sensor_kind =
      sensor_faults[std::min(3u, uint32_t(random(seed, 940) * 4))];
  j["faults"].push_back({{"kind", sensor_kind},
                         {"start_tick", 360},
                         {"end_tick", 720},
                         {"magnitude", u(941, -.05, .05)}});
  j["case_seed"] = seed;
  j["generator_version"] = 1;
  return j;
}
std::vector<double> generated_commands(const Scene &scene, uint64_t seed,
                                       uint64_t tick) {
  std::vector<double> out(scene.thrusters.size());
  for (size_t i = 0; i < out.size(); ++i)
    out[i] = (random(seed, 1000 + i, tick / 72) * 2 - 1) * .2;
  return out;
}
} // namespace sim
