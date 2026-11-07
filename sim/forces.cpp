#include "forces.hpp"
#include <numbers>
namespace sim {
void add_force(SimWrench &wrench, SimVec3 force, SimVec3 application_point,
               SimVec3 center_of_mass) {
  wrench.force = wrench.force + force;
  wrench.torque =
      wrench.torque + cross(application_point - center_of_mass, force);
}
static SimVec3 coefficients(SimVec3 force, SimQuat basis, SimVec3 cd,
                            double scale) {
  auto d = rotate(conjugate(basis), force);
  double len = norm(d);
  return force * (len < 1.4901161193847656e-8
                      ? 0
                      : scale *
                            (std::abs(d.x) * cd.x + std::abs(d.y) * cd.y +
                             std::abs(d.z) * cd.z) /
                            len);
}
static void gravity(ForceContext &c, WrenchBuffer &w) {
  for (size_t i = 0; i < w.size(); ++i)
    if (c.scene.objects[i].motion == SIM_DYNAMIC)
      w[i].force = w[i].force +
                   c.scene.header.gravity *
                       (c.scene.objects[i].dry_mass - c.scene.objects[i].mass);
}
static void hydro(ForceContext &c, WrenchBuffer &w) {
  if (c.scene.header.native_hydrodynamics)
    return;
  if (c.tick % c.scene.header.hydro_period == 0) {
    for (auto &f : c.fluid_cache) {
      f.force = {};
      f.torque = {};
    }
    std::pmr::vector<double> minz(w.size(), INFINITY, c.scratch),
        maxz(w.size(), -INFINITY, c.scratch);
    struct Wet {
      double lo = INFINITY, hi = -INFINITY;
    };
    std::array<Wet, Robot::max_hulls> wet{};
    for (size_t p = 0; p < c.scene.robot.hulls().size(); ++p) {
      auto &h = c.scene.robot.hulls()[p];
      auto transform = compose(c.bodies[h.body].pose, h.pose);
      for (uint32_t v = 0; v < h.vertex_count; ++v) {
        double z = point(transform, c.scene.positions(h)[v]).z;
        wet[p].lo = std::min(wet[p].lo, z);
        wet[p].hi = std::max(wet[p].hi, z);
      }
      minz[h.body] = std::min(minz[h.body], wet[p].lo);
      maxz[h.body] = std::max(maxz[h.body], wet[p].hi);
    }
    for (size_t i = 0; i < w.size(); ++i)
      if (minz[i] > 0 && std::isfinite(minz[i]))
        add_force(c.fluid_cache[i],
                  c.scene.header.gravity * (-c.scene.header.water_density *
                                            c.scene.objects[i].volume),
                  point(c.bodies[i].pose, c.scene.objects[i].buoyancy_center),
                  c.bodies[i].center_of_mass);
    for (size_t p = 0; p < c.scene.robot.hulls().size(); ++p) {
      auto &h = c.scene.robot.hulls()[p];
      auto &b = c.bodies[h.body];
      if (maxz[h.body] <= 0)
        continue;
      auto transform = compose(b.pose, h.pose);
      auto &out = c.fluid_cache[h.body];
      if (minz[h.body] <= 0 && wet[p].hi > 0) {
        double fraction =
            wet[p].lo > 0 ? 1 : wet[p].hi / (wet[p].hi - wet[p].lo);
        add_force(out,
                  c.scene.header.gravity *
                      (-fraction * h.volume * c.scene.header.water_density),
                  point(transform, h.centroid), b.center_of_mass);
      }
      MathVector6d pressure_drag{}, skin_drag{};
      for (uint32_t t = 0; t < h.vertex_count; t += 3) {
        auto a = point(transform, c.scene.positions(h)[t]),
             b1 = point(transform, c.scene.positions(h)[t + 1]),
             d = point(transform, c.scene.positions(h)[t + 2]);
        auto normal = cross(b1 - a, d - a);
        double twice = norm(normal);
        if (twice < 1e-6)
          continue;
        normal = normal / twice;
        auto center = (a + b1 + d) / 3;
        if (minz[h.body] <= 0 && center.z <= 0)
          continue;
        auto arm = center - b.center_of_mass;
        auto relative = c.scene.header.current -
                        (b.linear_velocity + cross(b.angular_velocity, arm));
        double vn = dot(relative, normal);
        if (vn < -1e-12) {
          auto force = relative * (norm(relative) * -vn * .5 * twice);
          pressure_drag.linear = pressure_drag.linear + force;
          pressure_drag.angular = pressure_drag.angular + cross(arm, force);
        }
        auto tangent = relative - normal * vn;
        if (dot(tangent, tangent) > 1e-9) {
          auto force = tangent * (.5 * twice);
          skin_drag.linear = skin_drag.linear + force;
          skin_drag.angular = skin_drag.angular + cross(arm, force);
        }
      }
      double rho = c.scene.header.water_density;
      out.force =
          out.force +
          coefficients(pressure_drag.linear,
                       multiply(b.pose.orientation, h.coefficient_frame), h.drag,
                       .5 * rho) +
          coefficients(skin_drag.linear,
                       multiply(b.pose.orientation, h.coefficient_frame), h.skin,
                       rho);
      out.torque =
          out.torque +
          coefficients(pressure_drag.angular,
                       multiply(b.pose.orientation, h.coefficient_frame), h.drag,
                       .5 * rho) +
          coefficients(skin_drag.angular,
                       multiply(b.pose.orientation, h.coefficient_frame), h.skin,
                       rho);
    }
  }
  for (size_t i = 0; i < w.size(); ++i) {
    w[i].force = w[i].force + c.fluid_cache[i].force;
    w[i].torque = w[i].torque + c.fluid_cache[i].torque;
  }
}
static void thrusters(ForceContext &c, WrenchBuffer &w) {
  for (size_t i = 0; i < c.scene.robot.thrusters().size(); ++i) {
    auto &t = c.scene.robot.thrusters()[i];
    auto &r = c.rotors[i];
    auto &b = c.bodies[t.body];
    auto &s = t.spec;
    double max = s.max_rpm / 60 * 2 * std::numbers::pi,
           target = s.command_sign * std::clamp(c.commands[i], -1.0, 1.0) * max;
    double error = target - r.omega;
    // Treat the proportional motor torque implicitly. Explicit Euler's pole is
    // 1 - dt*kp/inertia (about -0.90 for Hydrus at 360 Hz), which excites the
    // outer controller and becomes unstable at lower physics rates.
    double damping = r.omega > 0 ? std::abs(r.torque) : -std::abs(r.torque);
    r.omega =
        std::clamp((s.inertia * r.omega +
                    c.dt * (s.kp * target + s.ki * r.integral - damping)) /
                       (s.inertia + c.dt * s.kp),
                   -2 * max, 2 * max);
    r.integral = std::clamp(r.integral + error * c.dt, -5.0, 5.0);
    auto p = point(b.pose, t.position),
         axis = rotate(b.pose.orientation, t.direction);
    r.thrust = 0;
    if (p.z <= 0) {
      r.torque = 0;
      continue;
    }
    double inflow =
               dot(axis, b.linear_velocity +
                             cross(b.angular_velocity, p - b.center_of_mass) -
                             c.scene.header.current),
           n = r.omega / (2 * std::numbers::pi), d = s.diameter,
           rho = c.scene.header.water_density;
    r.thrust =
        rho * d * d * d * std::abs(n) * (d * s.thrust * n - s.thrust * inflow);
    r.torque = -rho * d * d * d * d * std::abs(n) *
               (d * s.torque * n - s.torque * inflow);
    add_force(w[t.body], axis * r.thrust, p, b.center_of_mass);
    w[t.body].torque = w[t.body].torque + axis * r.torque;
  }
}
ForceRegistry::ForceRegistry() {
  add("gravity_compensation", gravity, "gravity");
  add("hydrodynamics", hydro, "hydrodynamics");
  add("thrusters", thrusters, "actuation");
}
void ForceRegistry::add(std::string name, ForceFunction fn,
                        std::string contribution) {
  require(!name.empty() && bool(fn) && !functions_.contains(name),
          "invalid or duplicate force module");
  functions_.emplace(std::move(name),
                     Entry{std::move(fn), std::move(contribution)});
}
const ForceFunction &ForceRegistry::get(const std::string &name) const {
  auto it = functions_.find(name);
  require(it != functions_.end(), "unknown force module");
  return it->second.evaluate;
}
const std::string &ForceRegistry::contribution(const std::string &name) const {
  auto it = functions_.find(name);
  require(it != functions_.end(), "unknown force module");
  return it->second.contribution;
}
} // namespace sim
