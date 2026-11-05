#include "scene.hpp"
#include <numbers>
#include <charconv>
#include <map>
#include <fcntl.h>
#include <unistd.h>
namespace sim {
std::vector<std::array<SimVec3, 3>> surface(const SimShape &s) {
  std::vector<std::array<SimVec3, 3>> out;
  auto triangle = [&](SimVec3 a, SimVec3 b, SimVec3 c) {
    out.push_back({a, b, c});
  };
  if (s.kind == SIM_MESH) {
    for (uint32_t i = 0; i < s.triangle_count; ++i) {
      auto t = s.triangles[i];
      triangle(s.vertices[t.a], s.vertices[t.b], s.vertices[t.c]);
    }
  } else if (s.kind == SIM_BOX) {
    double x = s.size.x, y = s.size.y, z = s.size.z;
    SimVec3 v[] = {{-x, -y, -z}, {-x, y, -z}, {x, y, -z}, {x, -y, -z},
                   {-x, -y, z},  {-x, y, z},  {x, y, z},  {x, -y, z}};
    int f[][4] = {{0, 1, 2, 3}, {3, 2, 6, 7}, {4, 5, 1, 0},
                  {7, 6, 5, 4}, {6, 2, 1, 5}, {3, 7, 4, 0}};
    for (auto &q : f) {
      triangle(v[q[0]], v[q[1]], v[q[2]]);
      triangle(v[q[0]], v[q[2]], v[q[3]]);
    }
    for (int level = 0; level < 3; ++level) {
      auto prev = std::move(out);
      out.clear();
      for (auto &t : prev) {
        auto a = (t[0] + t[1]) * .5, b = (t[1] + t[2]) * .5,
             c = (t[2] + t[0]) * .5;
        triangle(t[0], a, c);
        triangle(t[1], b, a);
        triangle(t[2], c, b);
        triangle(a, b, c);
      }
    }
  } else if (s.kind == SIM_CYLINDER) {
    constexpr int n = 32;
    double dp = 2 * std::numbers::pi / n,
           r = std::sqrt(dp * s.size.x * s.size.x / std::sin(dp)), z = s.size.z;
    for (int i = 0; i < n; ++i) {
      double a = i * dp, b = (i + 1) * dp;
      SimVec3 t0{r * std::cos(a), r * std::sin(a), z},
          t1{r * std::cos(b), r * std::sin(b), z}, b0{t0.x, t0.y, -z},
          b1{t1.x, t1.y, -z};
      triangle(t0, b0, t1);
      triangle(b0, b1, t1);
      triangle({0, 0, z}, t0, t1);
      triangle({0, 0, -z}, b1, b0);
    }
  } else {
    constexpr int n = 16;
    constexpr int m = 8;
    auto v = [&](int i, int k) {
      double a = 2 * std::numbers::pi * i / n, b = std::numbers::pi * k / m;
      return SimVec3{s.size.x * std::sin(b) * std::cos(a),
                     s.size.x * std::sin(b) * std::sin(a),
                     s.size.x * std::cos(b)};
    };
    for (int k = 0; k < m; ++k)
      for (int i = 0; i < n; ++i) {
        triangle(v(i, k), v(i, k + 1), v(i + 1, k + 1));
        triangle(v(i, k), v(i + 1, k + 1), v(i + 1, k));
      }
  }
  return out;
}
Scene Scene::load(const std::filesystem::path &path) {
  auto name=path.string();
  if (name=="-") return load_fd(STDIN_FILENO);
  if (name.starts_with("fd:")) {
    int fd=-1; auto [end,error]=std::from_chars(name.data()+3,name.data()+name.size(),fd);
    require(error==std::errc() && end==name.data()+name.size() && fd>=0,"invalid scene descriptor");
    return load_fd(fd);
  }
  if (path.extension()==".blend") return load_blend(path);
  require(std::filesystem::file_size(path) <= 128 * 1024 * 1024,
          "scenario exceeds 128 MiB");
  std::ifstream f(path);
  require(bool(f), "cannot open scenario");
  char prefix[8]{}; f.read(prefix,8); f.clear(); f.seekg(0);
  if (path.extension()==".asim" || std::string(prefix,8)=="ASIMSC01") {
    int fd=::open(path.c_str(),O_RDONLY | O_CLOEXEC);
    require(fd>=0,"cannot open binary scene");
    try { auto scene=load_fd(fd); ::close(fd); scene.path=std::filesystem::absolute(path); return scene; }
    catch(...) { ::close(fd); throw; }
  }
  Json document;
  f >> document;
  return parse(std::move(document), path);
}
Scene Scene::parse(Json document, const std::filesystem::path &path) {
  Scene s;
  s.path = path.empty() ? path : std::filesystem::absolute(path);
  s.document = std::move(document);
  s.blender_source=s.document.value("source",Json::object()).value("kind","")=="blender_evaluated";
  s.force_modules=s.document.value("force_modules",std::vector<std::string>{"gravity_compensation","hydrodynamics","thrusters"});
  require(s.document.at("version") == 1, "unsupported scenario version");
  s.model = decode(s.document);
  s.model.bind();
  s.current = vec(s.document.value("current", Json{0, 0, 0}));
  require(finite(s.current), "invalid current");
  s.hydro_period = s.document.value("hydro_period_ticks", 7u);
  require(s.hydro_period > 0, "hydro period must be positive");
  size_t n = s.model.bodies.size();
  s.dry_mass.resize(n);
  s.volume.resize(n);
  s.buoyancy_center.resize(n);
  for (size_t i = 0; i < n; ++i) {
    auto &b = s.model.bodies[i];
    auto &j = s.document["bodies"][i];
    s.dry_mass[i] = j.value("dry_mass", b.value.mass);
    s.volume[i] = j.value("volume", 0.0);
    s.buoyancy_center[i] = vec(j.value("buoyancy_center", Json{0, 0, 0}));
    require(std::isfinite(s.dry_mass[i]) && s.dry_mass[i] >= 0 &&
                std::isfinite(s.volume[i]) && s.volume[i] >= 0 &&
                finite(s.buoyancy_center[i]),
            "invalid body fluid properties");
  }
  auto index = [&](uint32_t id) {
    for (size_t i = 0; i < n; ++i)
      if (s.model.bodies[i].value.id == id)
        return uint32_t(i);
    throw std::invalid_argument("unknown body id");
  };
  for (auto &j : s.document.value("thrusters", Json::array())) {
    Thruster t;
    t.body = index(j.at("body"));
    require(s.model.bodies[t.body].value.motion == SIM_DYNAMIC,
            "thruster requires dynamic body");
    t.position = vec(j.at("position"));
    t.direction = vec(j.at("direction"));
    auto &p = t.spec;
    auto spec = j.value("spec", Json::object());
    p.max_rpm = spec.value("max_rpm", p.max_rpm);
    p.diameter = spec.value("diameter", p.diameter);
    p.thrust = spec.value("thrust_coeff", p.thrust);
    p.torque = spec.value("torque_coeff", p.torque);
    p.inertia = spec.value("rotor_inertia", p.inertia);
    p.kp = spec.value("kp", p.kp);
    p.ki = spec.value("ki", p.ki);
    p.command_sign = spec.value("command_sign", p.command_sign);
    require(finite(t.position) && finite(t.direction) &&
                std::abs(norm(t.direction) - 1) < 1e-6,
            "invalid thruster mount");
    require(std::isfinite(p.max_rpm) && p.max_rpm > 0 &&
                std::isfinite(p.diameter) && p.diameter > 0 &&
                std::isfinite(p.inertia) && p.inertia > 0 &&
                std::isfinite(p.thrust) && p.thrust >= 0 &&
                std::isfinite(p.torque) && p.torque >= 0 &&
                std::isfinite(p.kp) && p.kp >= 0 &&
                std::isfinite(p.ki) && p.ki >= 0 &&
                std::abs(p.command_sign) == 1,
            "invalid rotor specification");
    s.thrusters.push_back(t);
  }
  for (auto &j : s.document.value("hull", Json::array())) {
    Hull h;
    h.body = index(j.at("body"));
    h.pose = pose(j.value("pose", Json::object()));
    valid_pose(h.pose);
    h.coefficient_frame =
        quat(j.value("coefficient_orientation", json(h.pose.orientation)));
    require(finite(h.coefficient_frame) &&
                std::abs(norm2(h.coefficient_frame) - 1) < 1e-6,
            "invalid hull coefficient frame");
    h.drag = vec(j.at("drag"));
    h.skin = vec(j.at("skin"));
    h.volume = j.value("volume", 0.0);
    h.centroid = vec(j.value("centroid", Json{0, 0, 0}));
    if (j.contains("faces"))
      for (auto &t : j.at("faces")) {
        require(t.is_array() && t.size() == 3, "invalid hull triangle");
        h.faces.push_back({vec(t[0]), vec(t[1]), vec(t[2])});
      }
    else {
      auto k = j.at("shape").get<size_t>();
      require(k < s.model.bodies[h.body].views.size(), "invalid hull shape");
      const auto &g = s.model.bodies[h.body].views[k];
      h.faces = surface(g);
      h.pose = g.pose;
    }
    require(finite(h.drag) && finite(h.skin) && h.drag.x >= 0 &&
                h.drag.y >= 0 && h.drag.z >= 0 && h.skin.x >= 0 &&
                h.skin.y >= 0 && h.skin.z >= 0 && std::isfinite(h.volume) &&
                h.volume >= 0 && finite(h.centroid) && !h.faces.empty(),
            "invalid hull");
    for (auto &t : h.faces)
      for (auto v : t)
        require(finite(v), "invalid hull vertex");
    s.hull.push_back(std::move(h));
  }
  auto c = s.document.value("camera", Json::object());
  s.camera.body = index(c.value("body", s.model.bodies.front().value.id));
  s.camera.pose = pose(c.value("pose", Json::object()));
  valid_pose(s.camera.pose);
  s.camera.width = c.value("width", 800u);
  s.camera.height = c.value("height", 600u);
  s.camera.horizontal_fov = c.value("horizontal_fov", 60.0);
  require(s.camera.width && s.camera.height && s.camera.horizontal_fov > 0 &&
              s.camera.horizontal_fov < 180,
          "invalid camera");
  auto assertions = s.document.value("assertions", Json::object());
  s.bounds={assertions.value("max_position_norm",10000.0),assertions.value("max_speed",1000.0),assertions.value("max_angular_speed",1000.0)};
  for (auto key : {"max_position_norm", "max_speed", "max_angular_speed"}) {
    double value = assertions.value(key, 1000.0);
    require(std::isfinite(value) && value > 0,
            "assertion bounds must be positive and finite");
  }
  require(s.thrusters.size() <= 4096, "too many thrusters");
  for (auto &fault : s.document.value("faults", Json::array())) {
    auto kind = fault.at("kind").get<std::string>();
    require(kind == "tracking" || kind == "imu" || kind == "gyro" ||
                kind == "depth" || kind == "dead" || kind == "weak",
            "unknown fault kind");
    require(fault.at("start_tick").get<uint64_t>() <
                fault.at("end_tick").get<uint64_t>(),
            "invalid fault interval");
    double magnitude = fault.value("magnitude", kind=="weak" ? .5 : kind=="gyro" ? .02 : kind=="depth" ? .1 : 0.0);
    require(std::isfinite(magnitude), "non-finite fault magnitude");
    if (kind == "dead" || kind == "weak") {
      require(fault.at("thruster").get<size_t>() < s.thrusters.size(),
              "unknown fault thruster");
      if (kind == "weak")
        require(magnitude >= 0 && magnitude <= 1,
                "weak fault magnitude must be in [0,1]");
    }
    const std::map<std::string,FaultKind> kinds={{"tracking",FaultKind::Tracking},{"imu",FaultKind::Imu},{"gyro",FaultKind::Gyro},{"depth",FaultKind::Depth},{"dead",FaultKind::Dead},{"weak",FaultKind::Weak}};
    s.faults.push_back({kinds.at(kind),fault.at("start_tick").get<uint64_t>(),fault.at("end_tick").get<uint64_t>(),fault.value("thruster",size_t(0)),magnitude});
  }
  return s;
}
} // namespace sim
