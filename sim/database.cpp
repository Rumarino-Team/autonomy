#include "blend/robot_registry.hpp"
#include "image.hpp"
#include <cstring>
#include <unordered_map>
namespace sim {
Main::Main(std::pmr::memory_resource *upstream,
           std::pmr::memory_resource *model_upstream)
    : memory_(upstream, model_upstream), geometry_(&memory_.pool),
      geometry_lookup_(&memory_.pool), lookup_(&memory_.pool),
      objects(memory_, this, frozen_), run(memory_, this, frozen_),
      commands(&memory_.model) {
  for (const char *name :
       {"gravity_compensation", "hydrodynamics", "thrusters"}) {
    NameRec module{};
    copy_text(module.text, sizeof(module.text), name);
    run.modules.push_back(module);
  }
}
size_t Main::collider_count() const {
  size_t n = 0;
  for (size_t i = 0; i < objects.size(); ++i)
    n += objects.block(i).colliders.size();
  return n;
}
ObjectBlock &Main::add_object(const BodyRec &value,
                              std::span<const Collider> source,
                              bool fixed_robot) {
  require(!frozen_, "database is frozen");
  require(!source.empty() && source.size() <= 1024, "invalid collider count");
  if (fixed_robot)
    require(!robot.object_registered && source.size() <= robot.colliders.size(),
            "invalid fixed robot object or collider count");
  // Mesh views are produced by this database's finalizer, never borrowed from
  // a different model. Importers supply ranges into the draft geometry arrays.
  for (const auto &c : source)
    require(c.geometry.positions.empty() && c.geometry.triangles.empty(),
            "draft collider contains foreign finalized geometry");
  BodyRec body = value;
  body.shape_begin = collider_count(); // cached transport range, not ownership
  body.shape_count = source.size();
  std::span<Collider> storage;
  if (fixed_robot) {
    std::copy(source.begin(), source.end(), robot.colliders.begin());
    storage = std::span(robot.colliders).first(source.size());
    objects.borrow(robot.object, body);
    robot.object_registered = true;
    robot.shape_count = source.size();
  } else {
    storage = memory_.copy_mutable(source);
    objects.push_back(body);
  }
  auto &object = objects.block(objects.size() - 1);
  object.colliders = storage;
  for (auto &c : storage)
    c.object = &object;
  return object;
}
Collider &Main::edit_collider(size_t object, size_t collider) {
  require(!frozen_, "database is frozen");
  auto all = objects.block(object).colliders;
  require(collider < all.size(), "unknown collider");
  return const_cast<Collider &>(all[collider]);
}
std::span<const SimVec3> Main::positions(const Collider &c) const {
  require(c.object && c.object->id.owner == this,
          "collider belongs to another database");
  if (frozen_)
    return c.geometry.positions;
  const auto &s = c.data;
  require(size_t(s.vertex_begin) + s.vertex_count <= vertices.size(),
          "collider mesh range is outside the scene");
  return std::span<const SimVec3>(vertices).subspan(s.vertex_begin,
                                                    s.vertex_count);
}
std::span<const SimTriangle> Main::faces(const Collider &c) const {
  require(c.object && c.object->id.owner == this,
          "collider belongs to another database");
  if (frozen_)
    return c.geometry.triangles;
  const auto &s = c.data;
  require(size_t(s.triangle_begin) + s.triangle_count <= triangles.size(),
          "collider mesh range is outside the scene");
  return std::span<const SimTriangle>(triangles).subspan(s.triangle_begin,
                                                         s.triangle_count);
}
std::span<const SimVec3> Main::positions(const HullRec &h) const {
  size_t index = 0;
  auto hulls = robot.hulls();
  while (index < hulls.size() && &hulls[index] != &h)
    ++index;
  require(index < hulls.size(), "hull belongs to another database");
  if (frozen_)
    return robot.surface_positions[index];
  require(size_t(h.vertex_begin) + h.vertex_count <= hull_vertices.size(),
          "invalid hull triangles");
  return std::span<const SimVec3>(hull_vertices)
      .subspan(h.vertex_begin, h.vertex_count);
}
uint32_t Main::geometry_index(const Collider &c) const {
  require(c.object && c.object->id.owner == this,
          "collider belongs to another database");
  if (c.data.kind != SIM_MESH) {
    require(c.geometry.positions.empty() && c.geometry.triangles.empty(),
            "primitive has mesh geometry");
    return UINT32_MAX;
  }
  auto it = geometry_lookup_.find(c.geometry.positions.data());
  require(it != geometry_lookup_.end(), "collider has foreign geometry");
  const auto &m = geometry_[it->second];
  require(m.triangles.data() == c.geometry.triangles.data() &&
              m.positions.size() == c.geometry.positions.size() &&
              m.triangles.size() == c.geometry.triangles.size(),
          "collider has foreign geometry");
  return it->second;
}
void Main::finalize() {
  require(!frozen_, "database is already frozen");
  auto thrusters = robot.thrusters();
  auto hulls = robot.hulls();
  robot.camera = header.camera;
  validate();
  {
    // Cache values are buffer-table indices, not registered mesh objects.
    // A collider keeps spans by value, so growing the table moves no geometry.
    std::pmr::unordered_multimap<uint64_t, uint32_t> cache(&memory_.scratch);
    for (size_t i = 0; i < objects.size(); ++i) {
      auto &object = objects.block(i);
      object.id.uid = object.data.id;
      for (const auto &view : object.colliders) {
        auto &c = const_cast<Collider &>(view);
        if (c.data.kind != SIM_MESH)
          continue;
        auto p = positions(c);
        auto t = faces(c);
        Hash hash;
        hash.bytes(p.data(), p.size_bytes());
        hash.bytes(t.data(), t.size_bytes());
        auto [begin, end] = cache.equal_range(hash.value);
        uint32_t index = UINT32_MAX;
        for (auto it = begin; it != end; ++it) {
          const auto &m = geometry_[it->second];
          if (m.positions.size() == p.size() &&
              m.triangles.size() == t.size() &&
              !std::memcmp(m.positions.data(), p.data(), p.size_bytes()) &&
              !std::memcmp(m.triangles.data(), t.data(), t.size_bytes())) {
            index = it->second;
            break;
          }
        }
        if (index == UINT32_MAX) {
          index = geometry_.size();
          geometry_.push_back(
              {mesh_vertices_.append(p), mesh_triangles_.append(t)});
          geometry_lookup_.emplace(geometry_.back().positions.data(), index);
          cache.emplace(hash.value, index);
        }
        c.geometry = geometry_[index];
      }
    }
    for (size_t i = 0; i < hulls.size(); ++i)
      robot.surface_positions[i] = hull_positions_.append(positions(hulls[i]));
    for (auto &f : run.faults)
      if (f.kind == FaultKind::Dead || f.kind == FaultKind::Weak)
        decltype(run.faults)::block_of(f).thruster = &thrusters[f.thruster];
    auto register_table = [&](const auto &table) {
      for (auto *link = table.list.first; link; link = link->next) {
        auto &id = datablock(link);
        require(
            id.owner == this &&
                lookup_.emplace((uint64_t(id.kind) << 32) | id.uid, &id).second,
            "duplicate datablock ID");
      }
    };
    register_table(objects);
    register_table(run.faults);
    register_table(run.modules);
    header.body_count = objects.size();
    header.shape_count = collider_count();
    header.thruster_count = thrusters.size();
    header.hull_count = hulls.size();
    header.fault_count = run.faults.size();
    header.module_count = run.modules.size();
    header.vertex_count = header.triangle_count = header.hull_vertex_count = 0;
    for (const auto &m : geometry_) {
      header.vertex_count += m.positions.size();
      header.triangle_count += m.triangles.size();
    }
    for (const auto &h : hulls)
      header.hull_vertex_count += h.vertex_count;
    header.command_ticks =
        commands.empty() ? 0 : commands.size() / header.command_width;
    if (commands.empty())
      header.command_width = 0;
    frozen_ = true;
    robot.frozen = true;
    vertices.release();
    triangles.release();
    hull_vertices.release();
  }
  memory_.scratch.release();
  validate_graph();
}
const DataBlock *Main::find(DataKind kind, uint32_t uid) const {
  require(frozen_, "lookup requires a frozen database");
  auto it = lookup_.find((uint64_t(kind) << 32) | uid);
  return it == lookup_.end() ? nullptr : it->second;
}
size_t Main::body_index(uint32_t uid) const {
  auto *id = find(DataKind::Object, uid);
  require(id, "unknown wrench body");
  return id->index;
}
void Main::validate_graph() const {
  require(frozen_, "graph validation requires a frozen database");
  size_t cursor = 0;
  for (size_t i = 0; i < objects.size(); ++i) {
    const auto &object = objects.block(i);
    require(find(DataKind::Object, object.data.id) == &object.id &&
                object.id.kind == DataKind::Object && object.id.owner == this &&
                object.id.index == i && object.id.list == &objects.list,
            "invalid object registry");
    require(object.data.shape_begin == cursor &&
                object.data.shape_count == object.colliders.size(),
            "invalid object collider range");
    cursor += object.colliders.size();
    for (const auto &c : object.colliders) {
      require(c.object == &object, "collider has a foreign owning object");
      geometry_index(c);
    }
  }
  if (robot.object_registered)
    require(find(DataKind::Object, robot.object.data.id) == &robot.object.id &&
                !robot.object.id.pooled &&
                robot.object.colliders.data() == robot.colliders.data() &&
                robot.object.colliders.size() == robot.shape_count,
            "invalid fixed robot object");
  for (size_t i = 0; i < run.faults.size(); ++i)
    if (run.faults[i].kind == FaultKind::Dead ||
        run.faults[i].kind == FaultKind::Weak)
      require(run.faults[i].thruster < robot.thrusters().size() &&
                  run.faults.block(i).thruster ==
                      &robot.thrusters()[run.faults[i].thruster],
              "foreign fault target");
}
MainPtr Main::clone() const {
  auto copy = std::make_unique<Main>();
  copy->header = header;
  copy->path = path;
  copy->robot.definition = robot.definition;
  copy->run.modules.clear();
  for (size_t i = 0; i < objects.size(); ++i) {
    const auto &object = objects.block(i);
    std::pmr::vector<Collider> colliders(&copy->memory_.scratch);
    for (const auto &c : object.colliders) {
      Collider value{c.data};
      value.data.vertex_begin = copy->vertices.size();
      value.data.triangle_begin = copy->triangles.size();
      auto p = positions(c);
      auto t = faces(c);
      copy->vertices.append(p);
      copy->triangles.append(t);
      colliders.push_back(value);
    }
    copy->add_object(object.data, colliders, !object.id.pooled);
  }
  for (const auto &h : robot.hulls()) {
    auto value = h;
    value.vertex_begin = copy->hull_vertices.size();
    auto p = positions(h);
    copy->hull_vertices.append(p);
    copy->robot.add_hull(value);
  }
  for (const auto &t : robot.thrusters())
    copy->robot.add_thruster(t);
  for (const auto &f : run.faults)
    copy->run.faults.push_back(f);
  for (const auto &m : run.modules)
    copy->run.modules.push_back(m);
  copy->commands.assign(commands.begin(), commands.end());
  return copy;
}
SimSceneView Main::view() const {
  require(frozen_, "backend view requires a frozen database");
  if (view_.struct_size)
    return view_;
  std::vector<SimBodyDesc> b;
  std::vector<SimShapeDesc> s;
  std::vector<SimMeshDesc> m;
  for (size_t i = 0; i < objects.size(); ++i) {
    const auto &object = objects.block(i);
    const auto &v = object.data;
    b.push_back(
        {{v.id, v.motion, v.mass, v.inertia, v.center_of_mass, v.initial_pose,
          v.initial_linear_velocity, v.initial_angular_velocity},
         {},
         v.cls,
         v.tracked,
         v.fuzz,
         uint32_t(s.size()),
         uint32_t(object.colliders.size()),
         v.dry_mass,
         v.volume,
         v.buoyancy_center});
    std::memcpy(b.back().name, v.name, 64);
    for (const auto &c : object.colliders) {
      const auto &v = c.data;
      s.push_back({{v.kind, v.collisions, v.pose, v.size, v.friction,
                    v.restitution, v.drag, v.skin},
                   uint32_t(i),
                   geometry_index(c)});
    }
  }
  for (const auto &v : geometry_)
    m.push_back({v.positions.data(), uint32_t(v.positions.size()),
                 v.triangles.data(), uint32_t(v.triangles.size())});
  auto &mem = const_cast<Main *>(this)->memory_;
  auto bp = mem.copy<SimBodyDesc>(b);
  auto sp = mem.copy<SimShapeDesc>(s);
  auto mp = mem.copy<SimMeshDesc>(m);
  view_ = {sizeof(SimSceneView),
           {sizeof(SimModelDesc), header.dt_numerator, header.dt_denominator,
            header.integrator, header.gravity, header.water_density,
            header.current, header.native_hydrodynamics},
           bp.data(),
           uint32_t(bp.size()),
           sp.data(),
           uint32_t(sp.size()),
           mp.data(),
           uint32_t(mp.size())};
  return view_;
}
MainPtr Main::from_view(const SimSceneView &v) {
  require(v.struct_size == sizeof(v) && v.model.struct_size == sizeof(v.model),
          "incompatible scene view size");
  require(v.bodies && v.body_count && v.body_count <= 4096 && v.shape_count &&
              v.shape_count <= 4096 * 1024 && v.shapes &&
              v.mesh_count <= v.shape_count && (!v.mesh_count || v.meshes),
          "invalid scene view tables");
  auto out = std::make_unique<Main>();
  auto &d = out->header;
  d.dt_numerator = v.model.dt_numerator;
  d.dt_denominator = v.model.dt_denominator;
  d.integrator = v.model.integrator;
  d.gravity = v.model.gravity;
  d.water_density = v.model.water_density;
  d.current = v.model.water_current;
  d.native_hydrodynamics = v.model.native_hydrodynamics;
  size_t bytes = size_t(v.body_count) * sizeof(SimBodyDesc) +
                 size_t(v.shape_count) * sizeof(SimShapeDesc);
  require(bytes <= max_scene_bytes, "scene view is too large");
  for (uint32_t i = 0; i < v.mesh_count; ++i) {
    auto &m = v.meshes[i];
    require(m.position_count >= 4 && m.position_count <= 1000000 &&
                m.triangle_count && m.triangle_count <= 2000000 &&
                m.positions && m.triangles,
            "invalid mesh view");
    bytes += size_t(m.position_count) * sizeof(SimVec3) +
             size_t(m.triangle_count) * sizeof(SimTriangle);
    require(bytes <= max_scene_bytes, "scene view is too large");
  }
  std::vector<BodyRec> bodies;
  std::vector<bool> fixed;
  for (uint32_t i = 0; i < v.body_count; ++i) {
    auto &s = v.bodies[i];
    BodyRec b;
    auto &a = s.body;
    b.id = a.id;
    b.motion = a.motion;
    b.mass = a.mass;
    b.inertia = a.inertia;
    b.center_of_mass = a.center_of_mass;
    b.initial_pose = a.initial_pose;
    b.initial_linear_velocity = a.initial_linear_velocity;
    b.initial_angular_velocity = a.initial_angular_velocity;
    std::memcpy(b.name, s.name, 64);
    b.cls = s.cls;
    b.tracked = s.tracked;
    b.fuzz = s.fuzz;
    b.shape_begin = s.shape_begin;
    b.shape_count = s.shape_count;
    b.dry_mass = s.dry_mass;
    b.volume = s.volume;
    b.buoyancy_center = s.buoyancy_center;
    if (auto *definition = robot_definition(
            std::string_view(b.name, strnlen(b.name, sizeof(b.name))))) {
      require(!out->robot.definition,
              "only one fixed robot is supported per scene");
      out->robot.definition = definition;
      fixed.push_back(true);
    } else
      fixed.push_back(false);
    bodies.push_back(b);
  }
  std::vector<std::pair<uint32_t, uint32_t>> ranges;
  for (uint32_t i = 0; i < v.mesh_count; ++i) {
    auto &m = v.meshes[i];
    ranges.push_back({out->vertices.size(), out->triangles.size()});
    out->vertices.append({m.positions, m.position_count});
    out->triangles.append({m.triangles, m.triangle_count});
  }
  std::vector<bool> claimed(v.shape_count, false);
  for (uint32_t owner = 0; owner < v.body_count; ++owner) {
    const auto &body = bodies[owner];
    require(body.shape_count && body.shape_count <= 1024 &&
                uint64_t(body.shape_begin) + body.shape_count <= v.shape_count,
            "invalid shape owner range");
    std::vector<Collider> colliders;
    for (uint32_t k = 0; k < body.shape_count; ++k) {
      uint32_t i = body.shape_begin + k;
      const auto &s = v.shapes[i];
      const auto &g = s.shape;
      require(!claimed[i] && s.body == owner, "invalid shape owner");
      claimed[i] = true;
      Collider c;
      auto &b = c.data;
      b.kind = g.kind;
      b.collisions = g.collisions;
      b.pose = g.pose;
      b.size = g.size;
      b.friction = g.friction;
      b.restitution = g.restitution;
      b.drag = g.drag;
      b.skin = g.skin;
      if (g.kind == SIM_MESH) {
        require(s.mesh < v.mesh_count, "unknown mesh reference");
        const auto &m = v.meshes[s.mesh];
        b.vertex_begin = ranges[s.mesh].first;
        b.triangle_begin = ranges[s.mesh].second;
        b.vertex_count = m.position_count;
        b.triangle_count = m.triangle_count;
      } else
        require(s.mesh == UINT32_MAX, "primitive references a mesh");
      colliders.push_back(c);
    }
    out->add_object(body, colliders, fixed[owner]);
  }
  for (bool used : claimed)
    require(used, "shape has no owning object");
  out->finalize();
  return out;
}
} // namespace sim
