#include "internal.hpp"
#include <numbers>
using namespace core;

extern "C" void sim_add_force(SimWrench *w, MathVector3f f, MathVector3f p, MathVector3f com) {
    if (w) {
        w->force = add(w->force, f);
        w->torque = add(w->torque, cross(sub(p, com), f));
    }
}

namespace {
void sim_add_force_ptr(SimWrench &w, MathVector3f f, MathVector3f p, MathVector3f com) {
    sim_add_force(&w, f, p, com);
}

static MathVector3f
coefficients(MathVector3f force, MathQuaternionf basis, MathVector3f cd, double scale) {
    auto d = rotate(conjugate(basis), force);
    double len = norm(d);
    return force * (len < 1.4901161193847656e-8
                        ? 0
                        : scale *
                              (std::abs(d.buf[0]) * cd.buf[0] + std::abs(d.buf[1]) * cd.buf[1] +
                               std::abs(d.buf[2]) * cd.buf[2]) /
                              len);
}

static void gravity(const SimForceContext &c, std::span<SimWrench> w) {
    for (size_t i = 0; i < w.size(); ++i)
        if (c.model->object_table[i]->body.motion_type == SIM_DYNAMIC)
            w[i].force =
                w[i].force + c.model->settings.gravity * (c.model->object_table[i]->body.dry_mass -
                                                          c.model->object_table[i]->body.mass);
}

static void hydro(const SimForceContext &c, std::span<SimWrench> w) {
    if (c.model->settings.native_hydrodynamics)
        return;
    if (c.tick % c.model->settings.hydro_period == 0) {
        for (auto &f : std::span(c.fluid_cache, c.body_count)) {
            f.force = {};
            f.torque = {};
        }
        auto *minz = static_cast<double *>(
            arena_alloc(c.scratch, bytes(w.size(), sizeof(double)), alignof(double))
        );
        auto *maxz = static_cast<double *>(
            arena_alloc(c.scratch, bytes(w.size(), sizeof(double)), alignof(double))
        );
        require(minz && maxz, "hydrodynamic scratch exhausted", SIM_CAPACITY);
        std::fill_n(minz, w.size(), INFINITY);
        std::fill_n(maxz, w.size(), -INFINITY);

        struct Wet {
            double lo = INFINITY, hi = -INFINITY;
        };

        std::array<Wet, SIM_MAX_HULLS> wet{};
        for (size_t p = 0; p < std::span(c.model->robot.hulls, c.model->robot.hull_count).size();
             ++p) {
            auto &h = std::span(c.model->robot.hulls, c.model->robot.hull_count)[p];
            auto transform = compose(c.bodies[h.object->id.index].pose, h.pose);
            for (uint32_t v = 0; v < h.position_count; ++v) {
                double z = point(transform, h.positions[v]).buf[2];
                wet[p].lo = std::min(wet[p].lo, z);
                wet[p].hi = std::max(wet[p].hi, z);
            }
            minz[h.object->id.index] = std::min(minz[h.object->id.index], wet[p].lo);
            maxz[h.object->id.index] = std::max(maxz[h.object->id.index], wet[p].hi);
        }
        for (size_t i = 0; i < w.size(); ++i)
            if (minz[i] > 0 && std::isfinite(minz[i]))
                sim_add_force_ptr(
                    c.fluid_cache[i],
                    c.model->settings.gravity *
                        (-c.model->settings.water_density * c.model->object_table[i]->body.volume),
                    point(c.bodies[i].pose, c.model->object_table[i]->body.buoyancy_center),
                    c.bodies[i].center_of_mass
                );
        for (size_t p = 0; p < std::span(c.model->robot.hulls, c.model->robot.hull_count).size();
             ++p) {
            auto &h = std::span(c.model->robot.hulls, c.model->robot.hull_count)[p];
            auto &b = c.bodies[h.object->id.index];
            if (maxz[h.object->id.index] <= 0)
                continue;
            auto transform = compose(b.pose, h.pose);
            auto &out = c.fluid_cache[h.object->id.index];
            if (minz[h.object->id.index] <= 0 && wet[p].hi > 0) {
                double fraction = wet[p].lo > 0 ? 1 : wet[p].hi / (wet[p].hi - wet[p].lo);
                sim_add_force_ptr(
                    out,
                    c.model->settings.gravity *
                        (-fraction * h.volume * c.model->settings.water_density),
                    point(transform, h.centroid),
                    b.center_of_mass
                );
            }

            struct Spatial {
                MathVector3f linear, angular;
            };

            Spatial pressure_drag{}, skin_drag{};
            for (uint32_t t = 0; t < h.position_count; t += 3) {
                auto a = point(transform, h.positions[t]),
                     b1 = point(transform, h.positions[t + 1]),
                     d = point(transform, h.positions[t + 2]);
                auto normal = cross(b1 - a, d - a);
                double twice = norm(normal);
                if (twice < 1e-6)
                    continue;
                normal = normal / twice;
                auto center = (a + b1 + d) / 3;
                if (minz[h.object->id.index] <= 0 && center.buf[2] <= 0)
                    continue;
                auto arm = center - b.center_of_mass;
                auto relative = c.model->settings.current -
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
            double rho = c.model->settings.water_density;
            out.force =
                out.force +
                coefficients(
                    pressure_drag.linear,
                    multiply(b.pose.quat, h.coefficient_frame),
                    h.drag,
                    .5 * rho
                ) +
                coefficients(
                    skin_drag.linear, multiply(b.pose.quat, h.coefficient_frame), h.skin, rho
                );
            out.torque =
                out.torque +
                coefficients(
                    pressure_drag.angular,
                    multiply(b.pose.quat, h.coefficient_frame),
                    h.drag,
                    .5 * rho
                ) +
                coefficients(
                    skin_drag.angular, multiply(b.pose.quat, h.coefficient_frame), h.skin, rho
                );
        }
    }
    for (size_t i = 0; i < w.size(); ++i) {
        w[i].force = w[i].force + c.fluid_cache[i].force;
        w[i].torque = w[i].torque + c.fluid_cache[i].torque;
    }
}

static void thrusters(const SimForceContext &c, std::span<SimWrench> w) {
    for (size_t i = 0;
         i < std::span(c.model->robot.thrusters, c.model->robot.thruster_count).size();
         ++i) {
        auto &t = std::span(c.model->robot.thrusters, c.model->robot.thruster_count)[i];
        auto &r = c.rotors[i];
        auto &b = c.bodies[t.object->id.index];
        auto &s = t.spec;
        double max = s.max_rpm / 60 * 2 * std::numbers::pi,
               target = s.command_sign * std::clamp(c.commands[i], -1.0, 1.0) * max;
        double error = target - r.omega;
        // Treat the proportional motor torque implicitly. Explicit Euler's pole is
        // 1 - dt*kp/inertia (about -0.90 for Hydrus at 360 Hz), which excites the
        // outer controller and becomes unstable at lower physics rates.
        double damping = r.omega > 0 ? std::abs(r.torque) : -std::abs(r.torque);
        r.omega = std::clamp(
            (s.inertia * r.omega + c.dt * (s.kp * target + s.ki * r.integral - damping)) /
                (s.inertia + c.dt * s.kp),
            -2 * max,
            2 * max
        );
        r.integral = std::clamp(r.integral + error * c.dt, -5.0, 5.0);
        auto p = point(b.pose, t.position), axis = rotate(b.pose.quat, t.direction);
        r.thrust = 0;
        if (p.buf[2] <= 0) {
            r.torque = 0;
            continue;
        }
        double inflow =
                   dot(axis,
                       b.linear_velocity + cross(b.angular_velocity, p - b.center_of_mass) -
                           c.model->settings.current),
               n = r.omega / (2 * std::numbers::pi), d = s.diameter,
               rho = c.model->settings.water_density;
        r.thrust = rho * d * d * d * std::abs(n) * (d * s.thrust * n - s.thrust * inflow);
        r.torque = -rho * d * d * d * d * std::abs(n) * (d * s.torque * n - s.torque * inflow);
        sim_add_force_ptr(w[t.object->id.index], axis * r.thrust, p, b.center_of_mass);
        w[t.object->id.index].torque = w[t.object->id.index].torque + axis * r.torque;
    }
}

SimResult evaluate_gravity(
    const SimForceContext *c, void *, SimWrench *w, uint32_t n, char *e, uint32_t cap
) {
    return guard(e, cap, [&] { gravity(*c, {w, n}); });
}

SimResult
evaluate_hydro(const SimForceContext *c, void *, SimWrench *w, uint32_t n, char *e, uint32_t cap) {
    return guard(e, cap, [&] { hydro(*c, {w, n}); });
}

SimResult evaluate_thrusters(
    const SimForceContext *c, void *, SimWrench *w, uint32_t n, char *e, uint32_t cap
) {
    return guard(e, cap, [&] { thrusters(*c, {w, n}); });
}
} // namespace

namespace core {
const ForceModule *builtin_modules(uint32_t &n) {
    static const ForceModule modules[] = {
        {"gravity_compensation", "gravity", nullptr, nullptr, nullptr, evaluate_gravity, nullptr},
        {"hydrodynamics", "hydrodynamics", nullptr, nullptr, nullptr, evaluate_hydro, nullptr},
        {"thrusters", "actuation", nullptr, nullptr, nullptr, evaluate_thrusters, nullptr}
    };
    n = 3;
    return modules;
}
} // namespace core
