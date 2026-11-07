#include "wire.hpp"
#include <actuators/Actuator.h>
#include <core/ConsoleSimulationApp.h>
#include <core/MaterialManager.h>
#include <core/SimulationManager.h>
#include <cstring>
#include <entities/forcefields/Uniform.h>
#include <entities/solids/Box.h>
#include <entities/solids/Compound.h>
#include <entities/solids/Cylinder.h>
#include <entities/solids/Polyhedron.h>
#include <entities/solids/Sphere.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <set>
#include <sys/stat.h>
namespace {
using namespace sim;
sf::Vector3 v(SimVec3 a) { return {a.x, a.y, a.z}; }
SimVec3 v(sf::Vector3 a) { return {a.x(), a.y(), a.z()}; }
sf::Transform transform(SimPose p) {
  return sf::Transform(sf::Quaternion(p.orientation.x, p.orientation.y,
                                      p.orientation.z, p.orientation.w),
                       v(p.position));
}
SimPose pose(sf::Transform p) {
  auto q = p.getRotation();
  return {v(p.getOrigin()), {q.x(), q.y(), q.z(), q.w()}};
}
class Manager;
class WrenchActuator : public sf::Actuator {
  Manager &manager_;

public:
  explicit WrenchActuator(Manager &m)
      : sf::Actuator("portable_wrenches"), manager_(m) {}
  sf::ActuatorType getType() const override { return sf::ActuatorType::PUSH; }
  void Update(sf::Scalar) override;
};
struct SolidBinding {
  using Value = sf::SolidEntity *;
  DataBlock id;
  sf::SolidEntity *data = nullptr;
  const ObjectBlock *body = nullptr;
};
struct RigidBinding {
  using Value = btRigidBody *;
  DataBlock id;
  btRigidBody *data = nullptr;
  const ObjectBlock *body = nullptr;
};
class Manager : public sf::SimulationManager {
public:
  const Main &model;
  Memory memory;
  bool frozen = false;
  Records<SolidBinding, DataKind::StonefishSolidBinding> solids;
  Records<RigidBinding, DataKind::StonefishRigidBinding> rigid;
  std::pmr::vector<SimWrench> queued;
  std::pmr::vector<SimBodyState> states;
  std::pmr::vector<SimContact> contacts;
  std::pmr::vector<uint8_t> seen;
  WireBuffer encoded{&memory.pool};
  std::filesystem::path temporary;
  explicit Manager(const Main &m)
      : sf::SimulationManager(double(m.header.dt_denominator) /
                              m.header.dt_numerator),
        model(m), solids(memory, this, frozen), rigid(memory, this, frozen),
        queued(&memory.pool), states(&memory.pool), contacts(&memory.pool),
        seen(&memory.pool) {
    queued.reserve(m.objects.size());
    states.reserve(m.objects.size());
    seen.resize(m.objects.size());
    char path[] = "/tmp/autonomy-stonefish-XXXXXX";
    auto p = mkdtemp(path);
    if (!p)
      throw std::runtime_error("cannot create worker asset directory");
    temporary = p;
  }
  ~Manager() {
    std::error_code ignored;
    std::filesystem::remove_all(temporary, ignored);
  }
  uint64_t getSimulationClock() const override {
    return uint64_t(getSimulationTime() * 1000000);
  }
  void SimulationClockSleep(uint64_t) override {}
  void SimulationStepCompleted(sf::Scalar) override {}
  sf::SolidEntity *part(const Collider &collider, const std::string &name,
                        const std::string &material,
                        sf::PhysicsSettings physics) {
    const auto &g = collider.data;
    sf::SolidEntity *s = nullptr;
    auto origin = sf::I4();
    if (g.kind == SIM_BOX)
      s = new sf::Box(name, physics, v(g.size * 2), origin, material, "");
    else if (g.kind == SIM_SPHERE)
      s = new sf::Sphere(name, physics, g.size.x, origin, material, "");
    else if (g.kind == SIM_CYLINDER)
      s = new sf::Cylinder(name, physics, g.size.x, 2 * g.size.z, origin,
                           material, "");
    else {
      auto path = temporary / (name + ".obj");
      std::ofstream f(path);
      f << std::setprecision(17);
      for (uint32_t i = 0; i < g.vertex_count; ++i) {
        auto &p = model.positions(collider)[i];
        f << "v " << p.x << ' ' << p.y << ' ' << p.z << '\n';
      }
      for (uint32_t i = 0; i < g.triangle_count; ++i) {
        auto &t = model.faces(collider)[i];
        f << "f " << t.a + 1 << ' ' << t.b + 1 << ' ' << t.c + 1 << '\n';
      }
      f.close();
      s = new sf::Polyhedron(name, physics, path.string(), 1, origin, material,
                             "");
    }
    s->SetHydrodynamicCoefficients(v(g.drag), v(g.skin));
    return s;
  }
  void BuildScenario() override {
    setGravity(model.header.gravity.z);
    setICSolverParams(false, .001, 2, 1e6, 1e6);
    setCallSimulationStepCompleted(false);
    if (model.header.native_hydrodynamics) {
      auto mm = getMaterialManager();
      auto name = mm->CreateFluid("portable_water", model.header.water_density,
                                  1.308e-3, 1.55);
      EnableOcean(0, mm->getFluid(name));
      getOcean()->AddVelocityField(new sf::Uniform(v(model.header.current)));
    }
    std::vector<std::string> materials;
    for (size_t i = 0; i < model.objects.size(); ++i) {
      auto &b = model.objects[i];
      const auto colliders = model.objects.block(i).colliders;
      auto shape = [&](uint32_t k) -> const ShapeRec & {
        return colliders[k].data;
      };
      std::string name = "body_" + std::to_string(b.id);
      sf::PhysicsSettings physics;
      physics.mode =
          model.header.native_hydrodynamics && b.motion == SIM_DYNAMIC
              ? sf::PhysicsMode::FLOATING
              : sf::PhysicsMode::SURFACE;
      if (model.header.native_hydrodynamics && b.motion == SIM_DYNAMIC)
        for (uint32_t k = 0; k < b.shape_count; ++k)
          require(shape(k).collisions == shape(0).collisions,
                  "native hydrodynamics requires uniform collision flags "
                  "within a body");
      std::vector<uint32_t> selected;
      for (uint32_t k = 0; k < b.shape_count; ++k)
        if (shape(k).collisions)
          selected.push_back(k);
      physics.collisions = !selected.empty();
      if (selected.empty())
        for (uint32_t k = 0; k < b.shape_count; ++k)
          selected.push_back(k);
      physics.buoyancy = model.header.native_hydrodynamics;
      auto first_index = selected.front();
      for (auto k : selected)
        require(shape(k).friction == shape(first_index).friction &&
                    shape(k).restitution == shape(first_index).restitution,
                "Stonefish compound requires one contact material per body");
      auto material = getMaterialManager()->CreateMaterial(
          name + "_material", 1000, shape(first_index).restitution);
      materials.push_back(material);
      auto first = part(colliders[first_index], name + "_part0", material, physics);
      auto compound = new sf::Compound(name, physics, first,
                                       transform(shape(first_index).pose));
      for (size_t k = 1; k < selected.size(); ++k) {
        auto index = selected[k];
        compound->AddExternalPart(part(colliders[index],
                                       name + "_part" + std::to_string(k),
                                       material, physics),
                                  transform(shape(index).pose));
      }
      compound->SetArbitraryPhysicalProperties(
          b.motion == SIM_DYNAMIC ? b.mass : 1, v(b.inertia),
          transform(b.center_of_mass));
      AddSolidEntity(compound, transform(b.initial_pose));
      solids.push_back(compound);
      solids.block(i).body = &model.objects.block(i);
      btRigidBody *rb = nullptr;
      for (int c = 0; c < getDynamicsWorld()->getNumCollisionObjects(); ++c) {
        auto candidate = getDynamicsWorld()->getCollisionObjectArray()[c];
        if (candidate->getUserPointer() == compound) {
          rb = btRigidBody::upcast(candidate);
          break;
        }
      }
      require(rb, "missing Stonefish rigid body");
      rb->setDamping(0, 0);
      rb->setSleepingThresholds(0, 0);
      rb->setActivationState(DISABLE_DEACTIVATION);
      if (b.motion != SIM_DYNAMIC) {
        rb->setMassProps(0, sf::Vector3(0, 0, 0));
        rb->setCollisionFlags(rb->getCollisionFlags() |
                              (b.motion == SIM_KINEMATIC
                                   ? btCollisionObject::CF_KINEMATIC_OBJECT
                                   : btCollisionObject::CF_STATIC_OBJECT));
      }
      if (!physics.collisions) {
        // SolidEntity::AddToSimulation registers every solid in Bullet's
        // collision group, including shapes marked collisions=false. Keep
        // ocean ghost queries for native fluid bodies, but exclude contacts.
        // Re-register to remove pairs created at construction. Merely changing
        // the proxy mask leaves existing overlapping pairs in Bullet's cache.
        auto world = getDynamicsWorld();
        world->removeRigidBody(rb);
        world->addRigidBody(rb, sf::MASK_DYNAMIC,
                            model.header.native_hydrodynamics &&
                                    b.motion == SIM_DYNAMIC
                                ? sf::MASK_GHOST
                                : 0);
      }
      rigid.push_back(rb);
      rigid.block(i).body = &model.objects.block(i);
    }
    for (size_t i = 0; i < materials.size(); ++i)
      for (size_t j = i; j < materials.size(); ++j) {
        double friction =
            std::sqrt(model.objects.block(i).colliders[0].data.friction *
                      model.objects.block(j).colliders[0].data.friction);
        getMaterialManager()->SetMaterialsInteraction(
            materials[i], materials[j], friction, friction);
      }
    auto &info = getDynamicsWorld()->getSolverInfo();
    info.m_solverMode &= ~SOLVER_RANDMIZE_ORDER;
    info.m_numIterations = 30;
    AddActuator(new WrenchActuator(*this));
  }
  void initialize(const SimBodyState *initial, uint32_t count) {
    require(count == 0 || count == model.objects.size(),
            "reset requires all bodies");
    for (size_t i = 0; i < model.objects.size(); ++i) {
      auto &b = model.objects[i];
      auto s = count == 0 ? SimBodyState{b.id,
                                         b.initial_pose,
                                         {},
                                         b.initial_linear_velocity,
                                         b.initial_angular_velocity}
                          : initial[i];
      require(s.id == b.id && finite(s.linear_velocity) &&
                  finite(s.angular_velocity),
              "invalid reset body");
      valid_pose(s.pose);
      if (b.motion == SIM_STATIC && count)
        require(norm(s.pose.position - b.initial_pose.position) == 0 &&
                    s.pose.orientation.x == b.initial_pose.orientation.x &&
                    s.pose.orientation.y == b.initial_pose.orientation.y &&
                    s.pose.orientation.z == b.initial_pose.orientation.z &&
                    s.pose.orientation.w == b.initial_pose.orientation.w,
                "static body reset not supported");
      auto p = transform(compose(s.pose, b.center_of_mass));
      auto rb = rigid[i];
      rb->setWorldTransform(p);
      rb->getMotionState()->setWorldTransform(p);
      rb->setInterpolationWorldTransform(p);
      rb->setLinearVelocity(v(s.linear_velocity));
      rb->setAngularVelocity(v(s.angular_velocity));
      rb->clearForces();
      getDynamicsWorld()->updateSingleAabb(rb);
    }
  }
  std::span<const uint8_t> snapshot() {
    states.clear();
    contacts.clear();
    Hash hash;
    for (size_t i = 0; i < model.objects.size(); ++i) {
      auto &b = model.objects[i];
      auto rb = rigid[i];
      auto origin =
          rb->getWorldTransform() * transform(b.center_of_mass).inverse();
      SimBodyState s{b.id, pose(origin), v(rb->getCenterOfMassPosition()),
                     v(rb->getLinearVelocity()), v(rb->getAngularVelocity())};
      states.push_back(s);
      hash.state(s);
    }
    auto dispatcher = getDynamicsWorld()->getDispatcher();
    for (int i = 0; i < dispatcher->getNumManifolds(); ++i) {
      auto m = dispatcher->getManifoldByIndexInternal(i);
      auto id = [&](const btCollisionObject *obj) {
        auto it = std::find(rigid.begin(), rigid.end(), obj);
        return it == rigid.end() ? UINT32_MAX
                                 : model.objects[it - rigid.begin()].id;
      };
      uint32_t a = id(m->getBody0()), b = id(m->getBody1());
      for (int k = 0; k < m->getNumContacts(); ++k) {
        auto &p = m->getContactPoint(k);
        hash.scalar(p.getAppliedImpulse());
        hash.vector(v(p.m_localPointA));
        hash.vector(v(p.m_localPointB));
        if (p.getDistance() > 0)
          continue;
        contacts.push_back(
            {a, b, v(p.getPositionWorldOnB()), v(-p.m_normalWorldOnB),
             p.getAppliedImpulse() * model.header.dt_denominator /
                 model.header.dt_numerator});
      }
    }
    hash.scalar(getSimulationTime());
    worker_reply(encoded, states, contacts, hash.value);
    return encoded;
  }
};
void WrenchActuator::Update(sf::Scalar) {
  for (auto &f : manager_.queued) {
    auto &source = manager_.model.objects;
    auto it = std::find_if(source.begin(), source.end(),
                           [&](auto &b) { return b.id == f.id; });
    require(it != source.end(), "unknown force body");
    size_t index = it - source.begin();
    if (it->motion != SIM_DYNAMIC)
      continue;
    manager_.solids[index]->ApplyCentralForce(v(f.force));
    manager_.solids[index]->ApplyTorque(v(f.torque));
  }
}
class App : public sf::ConsoleSimulationApp {
public:
  explicit App(Manager *m)
      : sf::ConsoleSimulationApp("autonomy_portable", "/tmp/", m) {}
  void start(double dt) {
    autostep_ = false;
    timeStep_ = dt;
    setMaxPhysicsThreads(1);
    Init();
    StartSimulation();
  }
  void shutdown() {
    StopSimulation();
    Quit();
    CleanUp();
  }
};
class Worker {
  Memory memory;
  WorkerRequest request{&memory.pool};
  std::unique_ptr<Manager> manager;
  std::unique_ptr<App> app;

public:
  MainPtr model;
  ~Worker() { clear(); }
  void clear() {
    if (app) {
      app->shutdown();
      app.reset();
    }
    manager.reset();
  }
  std::span<const uint8_t> init_scene(MainPtr scene) {
    // Tear down the old engine while the model referenced by Manager is alive.
    clear();
    model = std::move(scene);
    return reset(nullptr, 0);
  }
  std::span<const uint8_t> reset(const SimBodyState *states, uint32_t count) {
    clear();
    manager = std::make_unique<Manager>(*model);
    app = std::make_unique<App>(manager.get());
    app->start(double(model->header.dt_numerator) /
               model->header.dt_denominator);
    manager->initialize(states, count);
    return manager->snapshot();
  }
  std::span<const uint8_t> dispatch(std::span<const uint8_t> frame) {
    read_worker_request(frame, request);
    require(bool(manager), "worker not initialized");
    if (request.op == WorkerOp::Reset)
      return reset(request.states.data(), uint32_t(request.states.size()));
    manager->queued.clear();
    std::fill(manager->seen.begin(), manager->seen.end(), 0);
    for (auto &w : request.forces) {
      require(finite(w.force) && finite(w.torque),
              "invalid or duplicate force");
      auto &b = manager->model.objects;
      auto it = std::find_if(b.begin(), b.end(),
                             [&](auto &v) { return v.id == w.id; });
      require(it != b.end(), "unknown force body");
      auto index = size_t(it - b.begin());
      require(!manager->seen[index], "duplicate force");
      manager->seen[index] = 1;
      manager->queued.push_back(w);
    }
    double dt =
        double(model->header.dt_numerator) / model->header.dt_denominator;
    manager->getDynamicsWorld()->stepSimulation(dt, 0, dt);
    return manager->snapshot();
  }
};
} // namespace
int main(int argc, char **argv) {
  if (argc != 3 || std::string(argv[1]) != "--fd")
    return 2;
  int fd = std::stoi(argv[2]);
  try {
    Worker worker;
    sim::Memory transport;
    sim::WireBuffer frame{&transport.pool}, error{&transport.pool};
    while (true) {
      sim::receive_frame(fd, frame);
      try {
        auto reply =
            frame.size() >= 8 &&
                    std::memcmp(frame.data(), sim::scene_magic, 8) == 0
                ? worker.init_scene(sim::Main::read(frame.data(), frame.size()))
                : worker.dispatch(frame);
        sim::send_frame(fd, reply.data(), reply.size());
      } catch (const std::exception &e) {
        sim::worker_error(error, e.what());
        sim::send_frame(fd, error.data(), error.size());
      }
    }
  } catch (const std::exception &e) {
    std::fprintf(stderr, "[stonefish worker] %s\n", e.what());
    return 1;
  }
}
