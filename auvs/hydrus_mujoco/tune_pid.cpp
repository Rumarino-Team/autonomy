// Odometry PID gains on batched MuJoCo physics. The default heuristic pulses each
// axis, then sets a critically damped kp/kd from that acceleration. --es keeps the
// old 18-gain evolution search. Prints gains (does not write the json).
//
// Build: zig build -Dmujoco -DMUJOCO_PREFIX=...  →  zig-out/bin/hydrus_tune_pid

#include "auv_batch.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr int kNGains = 18;
constexpr int kPhysicsHz = 360;
constexpr double kCloseEnough = 1.0;
constexpr double kThrustorSaturate = 5.0;
constexpr double kThrustorOutputScale = 5.0;
constexpr double kSurfaceLimit = 0.2;
constexpr double kTiltLimit = 0.3;
constexpr double kYawGate = M_PI / 8.0;
constexpr double kSuccessDist = 0.3;
constexpr double kSuccessYaw = 0.2;

constexpr int kPos = 0;
constexpr int kQuat = 3;

struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
};

struct Quat {
    double x = 0, y = 0, z = 0, w = 1;
};

struct TaskConfig {
    double xy_range = 4.0;
    double z_min = 0.7;
    double z_max = 2.4;
    double goal_dist_min = 0.0;
    double goal_dist_max = 6.0;
    double init_tilt = 0.15;
    double init_speed = 0.2;
    double init_ang_speed = 0.2;
    bool randomize = false;
    double dry_mass_min = 0.97, dry_mass_max = 1.03;
    double volume_min = 0.97, volume_max = 1.03;
    double drag_min = 0.7, drag_max = 1.4;
    double thrust_min = 0.8, thrust_max = 1.2;
    double rotor_min = 0.7, rotor_max = 1.5;
    double current_speed = 0.15;
};

struct VehicleConfig {
    std::array<double, 6> kp{};
    std::array<double, 6> ki{};
    std::array<double, 6> kd{};
    std::vector<std::array<double, 6>> tam;
};

struct Scenarios {
    std::vector<double> init;
    std::vector<double> goals;
    std::vector<double> randomization;
    int n = 0;
};

struct RolloutStats {
    std::vector<bool> success;
    std::vector<bool> failed;
    std::vector<double> final_dist;
    std::vector<double> final_yaw;
    std::vector<double> hold_dist;
    std::vector<double> settle;
    std::vector<double> max_tilt;
    std::vector<double> effort;
};

struct Options {
    int episodes = 32;
    double seconds = 12.0;
    int generations = 12;
    int population = 8;
    int elite = 3;
    uint64_t seed = 1234;
    int threads = 0;
    bool randomize = false;
    bool evolution = false;
    std::string xml = "auvs/hydrus_mujoco/hydrus.xml";
    std::string config = "auvs/hydrus_mujoco/auv.json";
};

double wrapAngle(double a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0)
        a += 2.0 * M_PI;
    return a - M_PI;
}

Quat quatNormalize(Quat q) {
    const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (n < 1e-12)
        return {0, 0, 0, 1};
    return {q.x / n, q.y / n, q.z / n, q.w / n};
}

Quat quatConj(Quat q) {
    return {-q.x, -q.y, -q.z, q.w};
}

Quat quatMul(Quat a, Quat b) {
    const Vec3 av{a.x, a.y, a.z};
    const Vec3 bv{b.x, b.y, b.z};
    const Vec3 xyz{
        a.w * bv.x + b.w * av.x + av.y * bv.z - av.z * bv.y,
        a.w * bv.y + b.w * av.y + av.z * bv.x - av.x * bv.z,
        a.w * bv.z + b.w * av.z + av.x * bv.y - av.y * bv.x,
    };
    const double w = a.w * b.w - (av.x * bv.x + av.y * bv.y + av.z * bv.z);
    return {xyz.x, xyz.y, xyz.z, w};
}

Vec3 cross(Vec3 a, Vec3 b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

Vec3 quatRotate(Quat q, Vec3 v) {
    const Vec3 qv{q.x, q.y, q.z};
    const Vec3 t = cross(qv, v) * 2.0;
    return v + t * q.w + cross(qv, t);
}

Quat quatFromHeading(double psi, double roll, double pitch) {
    const double yaw = psi - M_PI / 2.0;
    const Quat qz{0, 0, std::sin(yaw / 2), std::cos(yaw / 2)};
    const Quat qy{0, std::sin(roll / 2), 0, std::cos(roll / 2)};
    const Quat qx{std::sin(pitch / 2), 0, 0, std::cos(pitch / 2)};
    return quatNormalize(quatMul(quatMul(qz, qy), qx));
}

double heading(Quat q) {
    const Vec3 fwd = quatRotate(q, {0, 1, 0});
    return std::atan2(fwd.y, fwd.x);
}

double yawError(Vec3 pos, Quat q, double goal_x, double goal_y, double goal_yaw) {
    const double dx = goal_x - pos.x;
    const double dy = goal_y - pos.y;
    const double dist = std::hypot(dx, dy);
    const double target = dist > kCloseEnough ? std::atan2(dy, dx) : goal_yaw;
    return wrapAngle(target - heading(q));
}

void quatToEuler(Quat q, double& roll, double& pitch, double& yaw) {
    const double x = q.x, y = q.y, z = q.z, w = q.w;
    roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    const double sinp = 2.0 * (w * y - z * x);
    pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
    yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

void gainBounds(std::array<double, kNGains>& lo, std::array<double, kNGains>& hi) {
    lo.fill(0);
    hi.fill(12.0);
    lo[3] = lo[4] = lo[15] = lo[16] = -12.0;
}

void packGains(const VehicleConfig& cfg, std::array<double, kNGains>& out) {
    for (int i = 0; i < 6; ++i) {
        out[i] = cfg.kp[i];
        out[6 + i] = cfg.ki[i];
        out[12 + i] = cfg.kd[i];
    }
}

void unpackGains(const std::array<double, kNGains>& g, VehicleConfig& cfg) {
    for (int i = 0; i < 6; ++i) {
        cfg.kp[i] = g[i];
        cfg.ki[i] = g[6 + i];
        cfg.kd[i] = g[12 + i];
    }
}

VehicleConfig loadConfig(const std::string& path) {
    std::ifstream in(path);
    if (!in)
        throw std::runtime_error("cannot open " + path);
    const nlohmann::json j = nlohmann::json::parse(in);
    VehicleConfig cfg;
    for (const auto& row : j.at("tam"))
        cfg.tam.push_back(row.get<std::array<double, 6>>());
    if (j.contains("odometry")) {
        const auto& odom = j.at("odometry");
        cfg.kp = odom.at("kp").get<std::array<double, 6>>();
        cfg.ki = odom.at("ki").get<std::array<double, 6>>();
        cfg.kd = odom.at("kd").get<std::array<double, 6>>();
    }
    return cfg;
}

// ES seed gains: midpoint of the search box (kp/ki/kd are not read from auv.json).
void initDefaultGains(VehicleConfig& cfg) {
    std::array<double, kNGains> lo, hi, g;
    gainBounds(lo, hi);
    for (int i = 0; i < kNGains; ++i)
        g[static_cast<size_t>(i)] = 0.5 * (lo[static_cast<size_t>(i)] + hi[static_cast<size_t>(i)]);
    unpackGains(g, cfg);
}

Scenarios sampleScenarios(std::mt19937_64& rng, int n, const TaskConfig& task) {
    std::uniform_real_distribution<double> uni01(0.0, 1.0);
    std::normal_distribution<double> normal(0.0, 1.0);

    Scenarios sc;
    sc.n = n;
    sc.init.resize(static_cast<size_t>(n) * AUV_BATCH_INIT_SIZE);
    sc.goals.resize(static_cast<size_t>(n) * 4);
    sc.randomization.resize(static_cast<size_t>(n) * AUV_BATCH_RANDOMIZATION_SIZE);

    auto uniform = [&](double lo, double hi) { return lo + (hi - lo) * uni01(rng); };

    for (int i = 0; i < n; ++i) {
        const Vec3 pos{uniform(-task.xy_range, task.xy_range), uniform(-task.xy_range, task.xy_range),
            uniform(task.z_min, task.z_max)};
        const Quat quat = quatFromHeading(uniform(-M_PI, M_PI), uniform(-task.init_tilt, task.init_tilt),
            uniform(-task.init_tilt, task.init_tilt));
        const double lin_scale = task.init_speed / std::sqrt(3.0);
        const double ang_scale = task.init_ang_speed / std::sqrt(3.0);

        double* init = sc.init.data() + i * AUV_BATCH_INIT_SIZE;
        init[0] = pos.x;
        init[1] = pos.y;
        init[2] = pos.z;
        init[3] = quat.x;
        init[4] = quat.y;
        init[5] = quat.z;
        init[6] = quat.w;
        for (int k = 0; k < 3; ++k) {
            init[7 + k] = normal(rng) * lin_scale;
            init[10 + k] = normal(rng) * ang_scale;
        }

        Vec3 dir{normal(rng), normal(rng), normal(rng)};
        const double dn = std::sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
        dir = dir * (1.0 / std::max(dn, 1e-9));
        const double gdist = uniform(task.goal_dist_min, task.goal_dist_max);
        Vec3 goal = pos + dir * gdist;
        goal.z = std::clamp(goal.z, task.z_min, task.z_max);
        double* g = sc.goals.data() + i * 4;
        g[0] = goal.x;
        g[1] = goal.y;
        g[2] = goal.z;
        g[3] = uniform(-M_PI, M_PI);

        double* r = sc.randomization.data() + i * AUV_BATCH_RANDOMIZATION_SIZE;
        if (!task.randomize) {
            for (int k = 0; k < 5; ++k)
                r[k] = 1.0;
            continue;
        }
        r[0] = uniform(task.dry_mass_min, task.dry_mass_max);
        r[1] = uniform(task.volume_min, task.volume_max);
        r[2] = uniform(task.drag_min, task.drag_max);
        r[3] = uniform(task.thrust_min, task.thrust_max);
        r[4] = uniform(task.rotor_min, task.rotor_max);
        const double theta = uniform(-M_PI, M_PI);
        const double speed = uniform(0, task.current_speed);
        r[5] = speed * std::cos(theta);
        r[6] = speed * std::sin(theta);
        r[7] = 0.0;
    }
    return sc;
}

class HostPid {
public:
    explicit HostPid(VehicleConfig cfg) : cfg_(std::move(cfg)) {}

    void reset(int n) {
        n_ = n;
        sum_error_.assign(static_cast<size_t>(n) * 6, 0.0);
        prev_error_.assign(static_cast<size_t>(n) * 6, 0.0);
        first_ = true;
    }

    void act(const double* state, const double* goals, int nthr, float* thrusters_out) {
        const double dt = 1.0 / kPhysicsHz;
        for (int i = 0; i < n_; ++i) {
            const double* s = state + i * AUV_BATCH_STATE_SIZE;
            const double* goal = goals + i * 4;
            Vec3 pos{s[kPos], s[kPos + 1], s[kPos + 2]};
            Quat quat = quatNormalize({s[kQuat], s[kQuat + 1], s[kQuat + 2], s[kQuat + 3]});

            double roll, pitch, yaw_euler;
            quatToEuler(quat, roll, pitch, yaw_euler);
            double err[6] = {goal[0] - pos.x, goal[1] - pos.y, goal[2] - pos.z, -roll, -pitch, 0.0};
            const double ye = yawError(pos, quat, goal[0], goal[1], goal[3]);
            err[5] = ye;

            double* sum = sum_error_.data() + i * 6;
            double* prev = prev_error_.data() + i * 6;
            double vel_err[6] = {0, 0, 0, 0, 0, 0};
            if (!first_) {
                for (int k = 0; k < 6; ++k) {
                    sum[k] += err[k] * dt;
                    vel_err[k] = (err[k] - prev[k]) / dt;
                }
            }
            double wrench[6];
            for (int k = 0; k < 6; ++k)
                wrench[k] = cfg_.kp[k] * err[k] + cfg_.ki[k] * sum[k] + cfg_.kd[k] * vel_err[k];
            for (int k = 0; k < 6; ++k)
                prev[k] = err[k];

            const Vec3 wlin{wrench[0], wrench[1], wrench[2]};
            const Vec3 body = quatRotate(quatConj(quat), wlin);
            const bool gated = std::abs(ye) > kYawGate;
            const double bx = gated ? 0.0 : std::max(body.x, 0.0);
            const double by = gated ? 0.0 : std::max(body.y, 0.0);
            const double tam_in[6] = {bx, by, wrench[2], -wrench[3], wrench[4], wrench[5]};

            float* cmd = thrusters_out + i * nthr;
            for (size_t t = 0; t < cfg_.tam.size(); ++t) {
                double v = 0;
                for (int k = 0; k < 6; ++k)
                    v += cfg_.tam[t][k] * tam_in[k];
                v = std::clamp(v, -kThrustorSaturate, kThrustorSaturate) / kThrustorOutputScale;
                cmd[t] = static_cast<float>(v);
            }
        }
        if (first_)
            first_ = false;
    }

private:
    VehicleConfig cfg_;
    int n_ = 0;
    std::vector<double> sum_error_;
    std::vector<double> prev_error_;
    bool first_ = true;
};

double median(std::vector<double> v) {
    if (v.empty())
        return std::numeric_limits<double>::quiet_NaN();
    const size_t mid = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid), v.end());
    if (v.size() % 2 == 1)
        return v[mid];
    const double a = v[mid];
    std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid - 1), v.end());
    return 0.5 * (a + v[mid - 1]);
}

RolloutStats runEpisodes(
    HostPid& pid,
    const Scenarios& sc,
    double seconds,
    AuvBatch* batch) {
    const int n = sc.n;
    if (auv_batch_num_envs(batch) != n)
        throw std::runtime_error("batch env count mismatch");

    std::vector<double> state(static_cast<size_t>(n) * AUV_BATCH_STATE_SIZE);
    auv_batch_reset(batch, nullptr, sc.init.data(), sc.randomization.data());
    auv_batch_get_state(batch, state.data());
    pid.reset(n);

    const int steps = static_cast<int>(std::lround(seconds * kPhysicsHz));
    const int nthr = auv_batch_num_thrusters(batch);
    std::vector<float> cmd(static_cast<size_t>(n) * static_cast<size_t>(nthr));

    std::vector<std::vector<double>> dist_hist(steps);
    std::vector<std::vector<double>> yaw_hist(steps);
    std::vector<std::vector<double>> tilt_hist(steps);
    std::vector<std::vector<double>> effort_hist(steps);
    std::vector<bool> failed(static_cast<size_t>(n), false);

    for (int k = 0; k < steps; ++k) {
        pid.act(state.data(), sc.goals.data(), nthr, cmd.data());
        effort_hist[k].resize(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i) {
            double e = 0;
            for (int t = 0; t < nthr; ++t) {
                const float v = cmd[static_cast<size_t>(i) * static_cast<size_t>(nthr) + static_cast<size_t>(t)];
                e += std::min(std::abs(static_cast<double>(v)), 1.0);
            }
            effort_hist[k][i] = e / nthr;
        }
        auv_batch_step(batch, cmd.data(), 1, state.data());

        dist_hist[k].resize(static_cast<size_t>(n));
        yaw_hist[k].resize(static_cast<size_t>(n));
        tilt_hist[k].resize(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i) {
            const double* s = state.data() + i * AUV_BATCH_STATE_SIZE;
            if (!std::isfinite(s[0])) {
                failed[static_cast<size_t>(i)] = true;
                dist_hist[k][i] = yaw_hist[k][i] = tilt_hist[k][i] = 0;
                continue;
            }
            const Vec3 pos{s[kPos], s[kPos + 1], s[kPos + 2]};
            Quat quat = quatNormalize({s[kQuat], s[kQuat + 1], s[kQuat + 2], s[kQuat + 3]});
            const Vec3 down = quatRotate(quatConj(quat), {0, 0, 1});
            const bool bad = pos.z < kSurfaceLimit || down.z < kTiltLimit;
            if (bad)
                failed[static_cast<size_t>(i)] = true;
            const double* goal = sc.goals.data() + i * 4;
            dist_hist[k][i] = std::hypot(goal[0] - pos.x, goal[1] - pos.y, goal[2] - pos.z);
            yaw_hist[k][i] = std::abs(yawError(pos, quat, goal[0], goal[1], goal[3]));
            tilt_hist[k][i] = std::acos(std::clamp(down.z, -1.0, 1.0)) * (180.0 / M_PI);
        }
    }

    RolloutStats out;
    out.success.resize(static_cast<size_t>(n));
    out.failed = failed;
    out.final_dist.resize(static_cast<size_t>(n));
    out.final_yaw.resize(static_cast<size_t>(n));
    out.hold_dist.resize(static_cast<size_t>(n));
    out.settle.resize(static_cast<size_t>(n));
    out.max_tilt.resize(static_cast<size_t>(n));
    out.effort.resize(static_cast<size_t>(n));

    for (int i = 0; i < n; ++i) {
        out.final_dist[static_cast<size_t>(i)] = dist_hist[steps - 1][static_cast<size_t>(i)];
        out.final_yaw[static_cast<size_t>(i)] = yaw_hist[steps - 1][static_cast<size_t>(i)];
        double max_t = 0;
        double eff = 0;
        double hold = 0;
        int hold_n = 0;
        for (int k = 0; k < steps; ++k) {
            max_t = std::max(max_t, tilt_hist[k][static_cast<size_t>(i)]);
            eff += effort_hist[k][static_cast<size_t>(i)];
            const double t = (k + 1) / static_cast<double>(kPhysicsHz);
            if (t > seconds - 5.0) {
                hold += dist_hist[k][static_cast<size_t>(i)];
                ++hold_n;
            }
        }
        out.max_tilt[static_cast<size_t>(i)] = max_t;
        out.effort[static_cast<size_t>(i)] = eff / steps;
        out.hold_dist[static_cast<size_t>(i)] = hold_n > 0 ? hold / hold_n : 0;

        double settle = std::numeric_limits<double>::infinity();
        for (int k = 0; k < steps; ++k) {
            const bool inside =
                dist_hist[k][static_cast<size_t>(i)] < kSuccessDist && yaw_hist[k][static_cast<size_t>(i)] < kSuccessYaw;
            if (!inside)
                continue;
            bool stays = true;
            for (int j = k; j < steps; ++j) {
                if (dist_hist[j][static_cast<size_t>(i)] >= kSuccessDist ||
                    yaw_hist[j][static_cast<size_t>(i)] >= kSuccessYaw) {
                    stays = false;
                    break;
                }
            }
            if (stays) {
                settle = (k + 1) / static_cast<double>(kPhysicsHz);
                break;
            }
        }
        out.settle[static_cast<size_t>(i)] = settle;
        const bool inside_last =
            dist_hist[steps - 1][static_cast<size_t>(i)] < kSuccessDist &&
            yaw_hist[steps - 1][static_cast<size_t>(i)] < kSuccessYaw;
        out.success[static_cast<size_t>(i)] = inside_last && !failed[static_cast<size_t>(i)];
    }
    return out;
}

double cost(const RolloutStats& r, double seconds) {
    int n = static_cast<int>(r.success.size());
    double fail = 0, succ = 0, eff = 0;
    std::vector<double> dist, yaw, settle;
    for (int i = 0; i < n; ++i) {
        fail += r.failed[static_cast<size_t>(i)] ? 1.0 : 0.0;
        succ += r.success[static_cast<size_t>(i)] ? 1.0 : 0.0;
        dist.push_back(r.final_dist[static_cast<size_t>(i)]);
        yaw.push_back(r.final_yaw[static_cast<size_t>(i)]);
        double s = r.settle[static_cast<size_t>(i)];
        if (!std::isfinite(s))
            s = seconds;
        settle.push_back(s);
        eff += r.effort[static_cast<size_t>(i)];
    }
    fail /= n;
    succ /= n;
    eff /= n;
    return 8.0 * fail + 4.0 * (1.0 - succ) + median(dist) + median(yaw) + 0.15 * median(settle) + 0.2 * eff;
}

void printSummary(const char* name, const RolloutStats& r) {
    int n = static_cast<int>(r.success.size());
    double succ = 0, fail = 0;
    std::vector<double> fd, hy, st, tilt;
    double hold_sum = 0;
    int hold_n = 0;
    for (int i = 0; i < n; ++i) {
        succ += r.success[static_cast<size_t>(i)] ? 1.0 : 0.0;
        fail += r.failed[static_cast<size_t>(i)] ? 1.0 : 0.0;
        if (!r.failed[static_cast<size_t>(i)]) {
            fd.push_back(r.final_dist[static_cast<size_t>(i)]);
            hy.push_back(r.final_yaw[static_cast<size_t>(i)]);
            hold_sum += r.hold_dist[static_cast<size_t>(i)];
            ++hold_n;
        }
        tilt.push_back(r.max_tilt[static_cast<size_t>(i)]);
        if (r.success[static_cast<size_t>(i)] && std::isfinite(r.settle[static_cast<size_t>(i)]))
            st.push_back(r.settle[static_cast<size_t>(i)]);
    }
    const double eff =
        std::accumulate(r.effort.begin(), r.effort.end(), 0.0) / std::max(1, static_cast<int>(r.effort.size()));
    std::printf(
        "%8s: success %5.1f%%  failed %5.1f%%  final dist %6.3f m (median)  hold dist %6.3f m  "
        "final yaw %5.1f deg  settle %5.2f s  max tilt %5.1f deg  effort %.3f\n",
        name,
        100.0 * succ / n,
        100.0 * fail / n,
        fd.empty() ? std::numeric_limits<double>::quiet_NaN() : median(fd),
        hold_n > 0 ? hold_sum / hold_n : std::numeric_limits<double>::quiet_NaN(),
        hy.empty() ? std::numeric_limits<double>::quiet_NaN() : median(hy) * (180.0 / M_PI),
        st.empty() ? std::numeric_limits<double>::quiet_NaN() : median(st),
        median(tilt),
        eff);
}

void printGains(const std::array<double, kNGains>& g) {
    const char* names[3] = {"kp", "ki", "kd"};
    for (int block = 0; block < 3; ++block) {
        std::printf("\"%s\": [", names[block]);
        for (int i = 0; i < 6; ++i) {
            if (i)
                std::printf(", ");
            std::printf("%.4f", g[static_cast<size_t>(block * 6 + i)]);
        }
        std::printf("]\n");
    }
}

void clipGains(std::array<double, kNGains>& g, const std::array<double, kNGains>& lo, const std::array<double, kNGains>& hi) {
    for (int i = 0; i < kNGains; ++i)
        g[static_cast<size_t>(i)] = std::clamp(g[static_cast<size_t>(i)], lo[static_cast<size_t>(i)], hi[static_cast<size_t>(i)]);
}

void mixCommand(const VehicleConfig& cfg, const double tam_in[6], int nthr, float* cmd) {
    for (int t = 0; t < nthr; ++t) {
        double v = 0;
        if (t < static_cast<int>(cfg.tam.size())) {
            for (int k = 0; k < 6; ++k)
                v += cfg.tam[static_cast<size_t>(t)][k] * tam_in[k];
        }
        cmd[t] = static_cast<float>(std::clamp(v, -kThrustorSaturate, kThrustorSaturate) / kThrustorOutputScale);
    }
}

// Wrench at which the loudest thruster on this axis hits the saturate limit.
double axisAuthority(const VehicleConfig& cfg, int axis) {
    double peak = 0;
    for (const auto& row : cfg.tam)
        peak = std::max(peak, std::abs(row[static_cast<size_t>(axis)]));
    if (peak < 1e-6)
        return kThrustorSaturate;
    return kThrustorSaturate / peak;
}

// Acceleration of each controlled channel per unit of TAM input, from a coast
// and one open-loop pulse per axis. Identity attitude, so body axes match world.
void identifyAlpha(const VehicleConfig& cfg, AuvBatch* batch, std::array<double, 6>& alpha) {
    const int n = auv_batch_num_envs(batch);
    const int nthr = auv_batch_num_thrusters(batch);
    const double dt = auv_batch_timestep(batch);
    std::vector<double> init(static_cast<size_t>(n) * AUV_BATCH_INIT_SIZE, 0.0);
    for (int i = 0; i < n; ++i) {
        double* pose = init.data() + i * AUV_BATCH_INIT_SIZE;
        pose[2] = 1.0;
        pose[6] = 1.0;
    }
    std::vector<double> state(static_cast<size_t>(n) * AUV_BATCH_STATE_SIZE);
    std::vector<float> cmd(static_cast<size_t>(n) * static_cast<size_t>(nthr), 0.f);

    auto runPulse = [&](const double tam_in[6]) {
        auv_batch_reset(batch, nullptr, init.data(), nullptr);
        const int ncmd = nthr;
        std::vector<float> one(static_cast<size_t>(ncmd));
        mixCommand(cfg, tam_in, ncmd, one.data());
        for (int i = 0; i < n; ++i)
            std::copy(one.begin(), one.end(), cmd.begin() + static_cast<std::ptrdiff_t>(i * nthr));
        const int skip = static_cast<int>(std::lround(0.35 / dt));
        const int span = static_cast<int>(std::lround(0.15 / dt));
        for (int k = 0; k < skip; ++k)
            auv_batch_step(batch, cmd.data(), 1, state.data());
        auto worldRate = [](const double* s) {
            std::array<double, 6> rate{};
            for (int k = 0; k < 3; ++k) {
                rate[static_cast<size_t>(k)] = s[7 + k];
                rate[static_cast<size_t>(3 + k)] = s[10 + k];
            }
            return rate;
        };
        const std::array<double, 6> early = worldRate(state.data());
        for (int k = 0; k < span; ++k)
            auv_batch_step(batch, cmd.data(), 1, state.data());
        const std::array<double, 6> late = worldRate(state.data());
        std::array<double, 6> accel{};
        const double elapsed = span * dt;
        for (int k = 0; k < 6; ++k)
            accel[static_cast<size_t>(k)] = (late[static_cast<size_t>(k)] - early[static_cast<size_t>(k)]) / elapsed;
        return accel;
    };

    const double coast_in[6] = {};
    const std::array<double, 6> coast = runPulse(coast_in);
    constexpr double kPulse = 1.0;
    const char* names[] = {"x", "y", "z", "roll", "pitch", "yaw"};
    std::printf("open-loop accel per unit wrench\n");
    for (int axis = 0; axis < 6; ++axis) {
        double tam_in[6] = {};
        tam_in[axis] = kPulse;
        const std::array<double, 6> pulsed = runPulse(tam_in);
        alpha[static_cast<size_t>(axis)] = (pulsed[static_cast<size_t>(axis)] - coast[static_cast<size_t>(axis)]) / kPulse;
        std::printf("  %-5s %+.4f\n", names[axis], alpha[static_cast<size_t>(axis)]);
    }
}

// kp uses a fraction of the thrust that saturates that axis at the design error.
// kd is the critical-damping partner of the measured acceleration. ki stays 0:
// the integrator wound up and pinned the thrusters in the evolution search.
std::array<double, kNGains> dampingSeed(const VehicleConfig& cfg, const std::array<double, 6>& alpha) {
    constexpr double kZeta = 1.15;
    constexpr double kFraction = 0.25;
    constexpr double kPosError = 1.0;
    constexpr double kAngError = 0.40;
    std::array<double, kNGains> g{};
    std::array<double, kNGains> lo, hi;
    gainBounds(lo, hi);
    for (int axis = 0; axis < 6; ++axis) {
        const double e_sat = axis < 3 ? kPosError : kAngError;
        const double auth = axisAuthority(cfg, axis);
        const double a = alpha[static_cast<size_t>(axis)];
        // Roll is negated once more on the way into the mixer.
        const double sign = (axis == 3 ? -1.0 : 1.0) * (a >= 0 ? 1.0 : -1.0);
        const double kp = sign * kFraction * auth / e_sat;
        const double alpha_abs = std::max(std::abs(a), 1e-4);
        const double kd = sign * 2.0 * kZeta * std::sqrt(std::abs(kp) / alpha_abs);
        g[static_cast<size_t>(axis)] = kp;
        g[static_cast<size_t>(12 + axis)] = kd;
    }
    clipGains(g, lo, hi);
    return g;
}

std::array<double, kNGains> scaleGains(const std::array<double, kNGains>& seed, double scale) {
    std::array<double, kNGains> g = seed;
    std::array<double, kNGains> lo, hi;
    gainBounds(lo, hi);
    for (int i = 0; i < 6; ++i) {
        g[static_cast<size_t>(i)] *= scale;
        g[static_cast<size_t>(12 + i)] *= scale;
    }
    clipGains(g, lo, hi);
    return g;
}

std::array<double, kNGains> heuristicSearch(
    VehicleConfig cfg,
    const Scenarios& train,
    double seconds,
    AuvBatch* batch) {
    std::array<double, 6> alpha{};
    identifyAlpha(cfg, batch, alpha);
    const std::array<double, kNGains> seed = dampingSeed(cfg, alpha);
    std::printf("damping seed\n");
    printGains(seed);

    const double scales[] = {0.50, 0.75, 1.00, 1.25, 1.50};
    double best_cost = std::numeric_limits<double>::infinity();
    std::array<double, kNGains> best = seed;
    for (double scale : scales) {
        const auto t0 = std::chrono::steady_clock::now();
        const std::array<double, kNGains> gains = scaleGains(seed, scale);
        unpackGains(gains, cfg);
        HostPid trial(cfg);
        const double c = cost(runEpisodes(trial, train, seconds, batch), seconds);
        const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        std::printf("scale %.2f  cost %.3f  %.1fs\n", scale, c, dt);
        std::fflush(stdout);
        if (c < best_cost) {
            best_cost = c;
            best = gains;
        }
    }
    std::printf("best heuristic cost %.3f\n", best_cost);
    return best;
}

std::array<double, kNGains> search(
    VehicleConfig base_cfg,
    const Scenarios& train,
    double seconds,
    AuvBatch* batch,
    int generations,
    int population,
    int elite,
    std::mt19937_64& rng) {
    std::array<double, kNGains> lo, hi, span, sigma, sigma_floor, best, center;
    gainBounds(lo, hi);
    for (int i = 0; i < kNGains; ++i) {
        span[static_cast<size_t>(i)] = hi[static_cast<size_t>(i)] - lo[static_cast<size_t>(i)];
        sigma[static_cast<size_t>(i)] = 0.2 * span[static_cast<size_t>(i)];
        sigma_floor[static_cast<size_t>(i)] = 0.02 * span[static_cast<size_t>(i)];
    }
    packGains(base_cfg, best);
    clipGains(best, lo, hi);
    center = best;

    VehicleConfig cfg = base_cfg;
    unpackGains(best, cfg);
    HostPid pid(cfg);
    double best_cost = cost(runEpisodes(pid, train, seconds, batch), seconds);
    std::printf("baseline cost %.3f\n", best_cost);
    printGains(best);

    std::normal_distribution<double> normal(0.0, 1.0);
    for (int gen = 0; gen < generations; ++gen) {
        const auto t0 = std::chrono::steady_clock::now();
        std::vector<std::pair<double, std::array<double, kNGains>>> scored;
        scored.reserve(static_cast<size_t>(population));
        for (int p = 0; p < population; ++p) {
            std::array<double, kNGains> gains = center;
            if (p > 0) {
                for (int i = 0; i < kNGains; ++i)
                    gains[static_cast<size_t>(i)] += sigma[static_cast<size_t>(i)] * normal(rng);
            } else {
                gains = best;
            }
            clipGains(gains, lo, hi);
            unpackGains(gains, cfg);
            HostPid trial(cfg);
            const double c = cost(runEpisodes(trial, train, seconds, batch), seconds);
            scored.emplace_back(c, gains);
        }
        std::sort(scored.begin(), scored.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
        if (scored[0].first < best_cost) {
            best_cost = scored[0].first;
            best = scored[0].second;
            for (int i = 0; i < kNGains; ++i)
                sigma[static_cast<size_t>(i)] = std::min(sigma[static_cast<size_t>(i)] * 1.05, 0.5 * span[static_cast<size_t>(i)]);
        } else {
            for (int i = 0; i < kNGains; ++i)
                sigma[static_cast<size_t>(i)] = std::max(sigma[static_cast<size_t>(i)] * 0.85, sigma_floor[static_cast<size_t>(i)]);
        }
        center.fill(0);
        for (int e = 0; e < elite; ++e)
            for (int i = 0; i < kNGains; ++i)
                center[static_cast<size_t>(i)] += scored[static_cast<size_t>(e)].second[static_cast<size_t>(i)];
        for (int i = 0; i < kNGains; ++i)
            center[static_cast<size_t>(i)] /= elite;
        clipGains(center, lo, hi);

        double sigma_mean = 0;
        for (int i = 0; i < kNGains; ++i)
            sigma_mean += sigma[static_cast<size_t>(i)] / std::max(span[static_cast<size_t>(i)], 1e-9);
        sigma_mean /= kNGains;
        const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        std::printf(
            "gen %3d/%d  cost %.3f  sigma %.3f  %.1fs\n",
            gen + 1,
            generations,
            best_cost,
            sigma_mean,
            dt);
    }
    return best;
}

Options parseArgs(int argc, char** argv) {
    Options opt;
    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        auto need = [&](const char* flag) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing value for %s\n", flag);
                std::exit(2);
            }
            return argv[++i];
        };
        if (std::strcmp(a, "--episodes") == 0)
            opt.episodes = std::atoi(need(a));
        else if (std::strcmp(a, "--seconds") == 0)
            opt.seconds = std::atof(need(a));
        else if (std::strcmp(a, "--generations") == 0)
            opt.generations = std::atoi(need(a));
        else if (std::strcmp(a, "--population") == 0)
            opt.population = std::atoi(need(a));
        else if (std::strcmp(a, "--elite") == 0)
            opt.elite = std::atoi(need(a));
        else if (std::strcmp(a, "--seed") == 0)
            opt.seed = static_cast<uint64_t>(std::strtoull(need(a), nullptr, 10));
        else if (std::strcmp(a, "--threads") == 0)
            opt.threads = std::atoi(need(a));
        else if (std::strcmp(a, "--config") == 0)
            opt.config = need(a);
        else if (std::strcmp(a, "--xml") == 0)
            opt.xml = need(a);
        else if (std::strcmp(a, "--randomize") == 0)
            opt.randomize = true;
        else if (std::strcmp(a, "--es") == 0)
            opt.evolution = true;
        else if (std::strcmp(a, "-h") == 0 || std::strcmp(a, "--help") == 0) {
            std::printf(
                "Usage: hydrus_tune_pid [options]\n"
                "  --episodes N       parallel scenarios (default 32)\n"
                "  --seconds T        rollout length (default 12)\n"
                "  --generations N    ES generations (default 12)\n"
                "  --population N     candidates per generation (default 8)\n"
                "  --elite N          parents kept (default 3)\n"
                "  --seed N           RNG seed (default 1234)\n"
                "  --threads N        worker threads (0 = hardware concurrency)\n"
                "  --randomize        domain randomization + current\n"
                "  --es               evolution search instead of the damping heuristic\n"
                "  --config PATH      auv.json (tam, and odometry for the held-out baseline)\n"
                "  --xml PATH         MJCF (default hydrus.xml; AUV_MJCF overrides)\n");
            std::exit(0);
        } else {
            std::fprintf(stderr, "unknown option %s\n", a);
            std::exit(2);
        }
    }
    if (opt.elite < 1 || opt.elite > opt.population) {
        std::fprintf(stderr, "--elite must be between 1 and --population\n");
        std::exit(2);
    }
    const char* env_xml = std::getenv("AUV_MJCF");
    if (env_xml != nullptr && env_xml[0] != '\0')
        opt.xml = env_xml;
    return opt;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options opt = parseArgs(argc, argv);
        VehicleConfig file_cfg = loadConfig(opt.config);
        VehicleConfig base_cfg = file_cfg;
        if (opt.evolution)
            initDefaultGains(base_cfg);

        TaskConfig task;
        task.randomize = opt.randomize;
        // BlueROV's pool floor is at 2.1 m. The default box reaches 2.4 m.
        if (opt.xml.find("bluerov") != std::string::npos)
            task.z_max = 1.85;
        std::mt19937_64 rng_train(opt.seed);
        std::mt19937_64 rng_hold(opt.seed + 1);
        const Scenarios train = sampleScenarios(rng_train, opt.episodes, task);
        const Scenarios held = sampleScenarios(rng_hold, opt.episodes, task);

        AuvBatch* batch = auv_batch_create(opt.xml.c_str(), opt.episodes, opt.threads);
        if (batch == nullptr)
            return 1;

        const std::array<double, kNGains> best = opt.evolution
            ? search(base_cfg, train, opt.seconds, batch, opt.generations, opt.population, opt.elite, rng_train)
            : heuristicSearch(base_cfg, train, opt.seconds, batch);

        std::printf("\nbest gains\n");
        printGains(best);

        VehicleConfig tuned = base_cfg;
        unpackGains(best, tuned);
        HostPid pid_base(file_cfg);
        HostPid pid_tuned(tuned);
        const RolloutStats base_stats = runEpisodes(pid_base, held, opt.seconds, batch);
        const RolloutStats tuned_stats = runEpisodes(pid_tuned, held, opt.seconds, batch);
        std::printf("\nheld-out\n");
        std::printf(
            "json cost %.3f  tuned %.3f\n",
            cost(base_stats, opt.seconds),
            cost(tuned_stats, opt.seconds));
        printSummary("json", base_stats);
        printSummary("tuned", tuned_stats);

        auv_batch_destroy(batch);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "hydrus_tune_pid: %s\n", e.what());
        return 1;
    }
    return 0;
}
