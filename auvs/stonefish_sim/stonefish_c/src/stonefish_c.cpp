#include "stonefish_c.h"

#include "Stonefish/StonefishCommon.h"
#include "Stonefish/actuators/ActuatorDynamics.h"
#include "Stonefish/actuators/Thruster.h"
#include "Stonefish/core/FeatherstoneRobot.h"
#include "Stonefish/core/NED.h"
#include "Stonefish/core/MaterialManager.h"
#include "Stonefish/core/Robot.h"
#include "Stonefish/core/SimulationManager.h"
#include "Stonefish/core/GraphicalSimulationApp.h"
#include "Stonefish/entities/forcefields/Atmosphere.h"
#include "Stonefish/entities/forcefields/Ocean.h"
#include "Stonefish/entities/forcefields/Uniform.h"
#include "Stonefish/entities/solids/Box.h"
#include "Stonefish/entities/solids/Compound.h"
#include "Stonefish/entities/solids/Cylinder.h"
#include "Stonefish/entities/solids/Polyhedron.h"
#include "Stonefish/entities/solids/Sphere.h"
#include "Stonefish/entities/statics/Obstacle.h"
#include "Stonefish/entities/statics/Plane.h"
#include "Stonefish/graphics/OpenGLDataStructs.h"
#include "Stonefish/graphics/OpenGLPipeline.h"
#include "Stonefish/graphics/OpenGLContent.h"
#include "Stonefish/sensors/scalar/IMU.h"
#include "Stonefish/sensors/scalar/Odometry.h"
#include "Stonefish/sensors/vision/ColorCamera.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{

struct RobotBuild
{
    sf::FeatherstoneRobot* robot = nullptr;
    sf::Compound* compound = nullptr;
    std::string robot_name;
    bool links_ready = false;
    bool added = false;
    bool self_collisions = false;
    sf::PhysicsSettings base_phy{};
    sf::Transform world = sf::I4();
};

struct WorldImpl
{
    sf::SimulationManager* sim = nullptr;
    std::string data_dir;
    std::unordered_set<std::string> materials;
    std::unordered_set<std::string> looks;
    std::vector<std::pair<std::string, std::string>> classes;
    bool environment_ready = false;
    RobotBuild robot;
};

} // namespace

struct SfWorld : WorldImpl
{
};

namespace
{

const char* Text(const char* value)
{
    return value != nullptr ? value : "";
}

bool Named(const char* value)
{
    return value != nullptr && value[0] != '\0';
}

std::string FullPath(const WorldImpl& world, const char* path)
{
    if(!Named(path))
        return {};
    const std::filesystem::path file(path);
    if(file.is_absolute() || world.data_dir.empty())
        return file.string();
    return (std::filesystem::path(world.data_dir) / file).string();
}

sf::Transform ToTransform(const SfPose& pose)
{
    const sf::Vector3 rpy(pose.rpy[0], pose.rpy[1], pose.rpy[2]);
    const sf::Vector3 xyz(pose.xyz[0], pose.xyz[1], pose.xyz[2]);
    return sf::Transform(sf::Quaternion(rpy.z(), rpy.y(), rpy.x()), xyz);
}

sf::PhysicsSettings ToPhysics(int mode, int buoyant)
{
    sf::PhysicsSettings phy;
    switch(mode)
    {
        case SF_BODY_FLOATING:
            phy.mode = sf::PhysicsMode::FLOATING;
            break;
        case SF_BODY_SUBMERGED:
            phy.mode = sf::PhysicsMode::SUBMERGED;
            break;
        default:
            phy.mode = sf::PhysicsMode::SURFACE;
            break;
    }
    phy.buoyancy = buoyant != 0;
    phy.collisions = true;
    return phy;
}

std::string Lower(std::string text)
{
    for(char& c : text)
    {
        if(c >= 'A' && c <= 'Z')
            c = static_cast<char>(c - 'A' + 'a');
    }
    return text;
}

void RememberClass(WorldImpl& world, const SfStatic& body)
{
    if(!Named(body.cls))
        return;
    const std::string cls = Lower(body.cls);
    if(cls == "scenery")
        return;
    world.classes.emplace_back(body.name, cls);
}

int Fail(const char* message)
{
    std::fprintf(stderr, "[stonefish_c] %s\n", message);
    return -1;
}

bool Ready(const SfWorld* world)
{
    return world != nullptr && world->sim != nullptr;
}

int FinalizeLinks(WorldImpl& world)
{
    RobotBuild& build = world.robot;
    if(build.links_ready)
        return 0;
    if(build.robot == nullptr || build.compound == nullptr)
        return Fail("robot has no parts");
    build.robot->DefineLinks(build.compound, {}, build.self_collisions);
    build.robot->BuildKinematicStructure();
    build.links_ready = true;
    return 0;
}

std::string PartName(const RobotBuild& build, const char* name)
{
    return build.robot_name + "/" + Text(name);
}

int AddPart(WorldImpl& world, const SfPart& part, sf::SolidEntity* solid)
{
    RobotBuild& build = world.robot;
    if(build.robot == nullptr || build.links_ready || solid == nullptr)
    {
        delete solid;
        return Fail("robot part added out of order");
    }
    if(part.mass > 0.0)
        solid->ScalePhysicalPropertiesToArbitraryMass(part.mass);
    const sf::Transform compound = ToTransform(part.compound);
    if(build.compound == nullptr)
        build.compound = new sf::Compound(build.robot_name, build.base_phy, solid, compound);
    else if(part.internal != 0)
        build.compound->AddInternalPart(solid, compound, false);
    else
        build.compound->AddExternalPart(solid, compound);
    return 0;
}

} // namespace

SfWorld* sf_world_bind(void* simulation_manager)
{
    if(simulation_manager == nullptr)
        return nullptr;
    auto* world = new SfWorld();
    world->sim = static_cast<sf::SimulationManager*>(simulation_manager);
    // A native scene may already have registered the resources needed by
    // the existing robot builder. Binding must also work in that case.
    world->environment_ready = world->sim->getOcean() != nullptr;
    for(const auto& name : world->sim->getMaterialManager()->GetMaterialsList())
        world->materials.insert(name);
    return world;
}

void sf_world_free(SfWorld* world)
{
    if(world == nullptr)
        return;
    if(!world->robot.added)
    {
        if(!world->robot.links_ready)
            delete world->robot.compound;
        delete world->robot.robot;
    }
    delete world;
}

void sf_world_set_data_dir(SfWorld* world, const char* data_dir)
{
    if(world == nullptr)
        return;
    world->data_dir = Text(data_dir);
}

int sf_material(SfWorld* world, const SfMaterial* material)
{
    if(!Ready(world) || material == nullptr || !Named(material->name))
        return Fail("material is missing a name");
    if(!world->materials.insert(material->name).second)
        return 0;
    world->sim->CreateMaterial(material->name, material->density, material->restitution);
    return 0;
}

int sf_look(SfWorld* world, const SfLook* look)
{
    if(!Ready(world) || look == nullptr || !Named(look->name))
        return Fail("look is missing a name");
    if(!world->looks.insert(look->name).second)
        return 0;
    if(sf::SimulationApp::getApp()->hasGraphics())
    {
        auto* app = static_cast<sf::GraphicalSimulationApp*>(sf::SimulationApp::getApp());
        if(app->getGLPipeline()->getContent()->getLookId(look->name) >= 0)
            return 0;
    }
    const sf::Color color(
        static_cast<float>(look->rgb[0]),
        static_cast<float>(look->rgb[1]),
        static_cast<float>(look->rgb[2]));
    world->sim->CreateLook(
        look->name,
        color,
        static_cast<float>(look->roughness),
        static_cast<float>(look->metalness),
        static_cast<float>(look->reflectivity),
        FullPath(*world, look->texture),
        FullPath(*world, look->normal_map));
    return 0;
}

int sf_friction(SfWorld* world, const SfFriction* friction)
{
    if(!Ready(world) || friction == nullptr || !Named(friction->material1) || !Named(friction->material2))
        return Fail("friction pair is missing a material");
    if(!world->sim->SetMaterialsInteraction(
           friction->material1, friction->material2, friction->static_friction, friction->dynamic_friction))
        return Fail("friction pair was not applied");
    return 0;
}

int sf_environment(SfWorld* world, const SfEnvironment* environment)
{
    if(!Ready(world) || environment == nullptr)
        return Fail("environment is missing");
    if(world->environment_ready)
        return 0;

    world->sim->getNED()->Init(environment->ned_latitude, environment->ned_longitude, sf::Scalar(0));

    const std::string water = world->sim->getMaterialManager()->CreateFluid(
        "Water", environment->water_density, 1.308e-3, 1.55);
    world->sim->EnableOcean(environment->wave_height, world->sim->getMaterialManager()->getFluid(water));
    sf::Ocean* ocean = world->sim->getOcean();
    if(ocean == nullptr)
        return Fail("ocean was not created");
    ocean->setWaterType(environment->jerlov);
    ocean->SetConditions(environment->water_temperature);
    ocean->setParticles(environment->particles != 0);
    ocean->AddVelocityField(new sf::Uniform(sf::Vector3(
        environment->current_xyz[0], environment->current_xyz[1], environment->current_xyz[2])));

    world->sim->EnableAtmosphere();
    if(sf::Atmosphere* atmosphere = world->sim->getAtmosphere())
        atmosphere->SetSunPosition(environment->sun_azimuth_deg, environment->sun_elevation_deg);

    world->environment_ready = true;
    return 0;
}

int sf_static_plane(SfWorld* world, const SfStatic* body, double uv_scale)
{
    if(!Ready(world) || body == nullptr || !Named(body->name))
        return Fail("static plane is missing a name");
    auto* plane = new sf::Plane(body->name, sf::Scalar(10000), Text(body->material), Text(body->look), static_cast<float>(uv_scale));
    world->sim->AddStaticEntity(plane, ToTransform(body->world));
    RememberClass(*world, *body);
    return 0;
}

int sf_static_box(SfWorld* world, const SfStatic* body, const double dimensions[3])
{
    if(!Ready(world) || body == nullptr || !Named(body->name) || dimensions == nullptr)
        return Fail("static box is missing a name or dimensions");
    const sf::Vector3 dims(dimensions[0], dimensions[1], dimensions[2]);
    auto* box = new sf::Obstacle(body->name, dims, ToTransform(body->origin), Text(body->material), Text(body->look));
    world->sim->AddStaticEntity(box, ToTransform(body->world));
    RememberClass(*world, *body);
    return 0;
}

int sf_static_cylinder(SfWorld* world, const SfStatic* body, double radius, double height)
{
    if(!Ready(world) || body == nullptr || !Named(body->name))
        return Fail("static cylinder is missing a name");
    auto* cylinder = new sf::Obstacle(
        body->name, radius, height, ToTransform(body->origin), Text(body->material), Text(body->look));
    world->sim->AddStaticEntity(cylinder, ToTransform(body->world));
    RememberClass(*world, *body);
    return 0;
}

int sf_static_sphere(SfWorld* world, const SfStatic* body, double radius)
{
    if(!Ready(world) || body == nullptr || !Named(body->name))
        return Fail("static sphere is missing a name");
    auto* sphere = new sf::Obstacle(body->name, radius, ToTransform(body->origin), Text(body->material), Text(body->look));
    world->sim->AddStaticEntity(sphere, ToTransform(body->world));
    RememberClass(*world, *body);
    return 0;
}

int sf_static_mesh(SfWorld* world, const SfStatic* body, const SfMesh* mesh)
{
    if(!Ready(world) || body == nullptr || !Named(body->name) || mesh == nullptr || !Named(mesh->physics_path))
        return Fail("static mesh is missing a name or physics path");
    const std::string physics = FullPath(*world, mesh->physics_path);
    const std::string visual = Named(mesh->visual_path) ? FullPath(*world, mesh->visual_path) : physics;
    const double visual_scale = Named(mesh->visual_path) ? mesh->visual_scale : mesh->physics_scale;
    const SfPose visual_origin = Named(mesh->visual_path) ? mesh->visual_origin : mesh->physics_origin;
    auto* obstacle = new sf::Obstacle(
        body->name,
        visual,
        visual_scale,
        ToTransform(visual_origin),
        physics,
        mesh->physics_scale,
        ToTransform(mesh->physics_origin),
        mesh->convex != 0,
        Text(body->material),
        Text(body->look));
    world->sim->AddStaticEntity(obstacle, ToTransform(body->world));
    RememberClass(*world, *body);
    return 0;
}

int sf_robot_begin(SfWorld* world, const SfRobot* robot)
{
    if(!Ready(world) || robot == nullptr || !Named(robot->name) || !Named(robot->base_link))
        return Fail("robot is missing a name");
    if(world->robot.robot != nullptr)
        return Fail("a robot is already being built");
    if(!world->environment_ready)
        return Fail("ocean must exist before the robot");

    RobotBuild& build = world->robot;
    build = {};
    build.robot_name = std::string(robot->name) + "/" + robot->base_link;
    build.robot = new sf::FeatherstoneRobot(robot->name, robot->fixed != 0);
    build.self_collisions = robot->self_collisions != 0;
    build.base_phy = ToPhysics(robot->physics, robot->buoyant);
    build.world = ToTransform(robot->world);
    return 0;
}

int sf_robot_part_box(SfWorld* world, const SfPartBox* part)
{
    if(!Ready(world) || part == nullptr || !Named(part->part.name))
        return Fail("robot box part is missing a name");
    const sf::Vector3 dims(part->dimensions[0], part->dimensions[1], part->dimensions[2]);
    auto* solid = new sf::Box(
        PartName(world->robot, part->part.name),
        ToPhysics(part->part.physics, part->part.buoyant),
        dims,
        ToTransform(part->part.origin),
        Text(part->part.material),
        Text(part->part.look),
        part->part.thickness);
    return AddPart(*world, part->part, solid);
}

int sf_robot_part_cylinder(SfWorld* world, const SfPartCylinder* part)
{
    if(!Ready(world) || part == nullptr || !Named(part->part.name))
        return Fail("robot cylinder part is missing a name");
    auto* solid = new sf::Cylinder(
        PartName(world->robot, part->part.name),
        ToPhysics(part->part.physics, part->part.buoyant),
        part->radius,
        part->height,
        ToTransform(part->part.origin),
        Text(part->part.material),
        Text(part->part.look),
        part->part.thickness);
    return AddPart(*world, part->part, solid);
}

int sf_robot_part_mesh(SfWorld* world, const SfPartMesh* part)
{
    if(!Ready(world) || part == nullptr || !Named(part->part.name) || !Named(part->path))
        return Fail("robot mesh part is missing a name or path");
    const sf::PhysicsSettings phy = ToPhysics(part->part.physics, part->part.buoyant);
    const std::string name = PartName(world->robot, part->part.name);
    sf::Polyhedron* solid = nullptr;
    if(Named(part->visual_path))
    {
        solid = new sf::Polyhedron(
            name,
            phy,
            FullPath(*world, part->visual_path),
            part->visual_scale,
            ToTransform(part->visual_origin),
            FullPath(*world, part->path),
            part->scale,
            ToTransform(part->part.origin),
            Text(part->part.material),
            Text(part->part.look),
            part->part.thickness);
    }
    else
    {
        solid = new sf::Polyhedron(
            name,
            phy,
            FullPath(*world, part->path),
            part->scale,
            ToTransform(part->part.origin),
            Text(part->part.material),
            Text(part->part.look),
            part->part.thickness);
    }
    return AddPart(*world, part->part, solid);
}

int sf_robot_part_sphere(SfWorld* world, const SfPartSphere* part)
{
    if(!Ready(world) || part == nullptr || !Named(part->part.name))
        return Fail("robot sphere part is missing a name");
    auto* solid = new sf::Sphere(
        PartName(world->robot, part->part.name),
        ToPhysics(part->part.physics, part->part.buoyant),
        part->radius,
        ToTransform(part->part.origin),
        Text(part->part.material),
        Text(part->part.look),
        part->part.thickness);
    return AddPart(*world, part->part, solid);
}

int sf_robot_thruster(SfWorld* world, const SfThruster* thruster)
{
    if(!Ready(world) || thruster == nullptr || !Named(thruster->name) || !Named(thruster->link))
        return Fail("thruster is missing a name or link");
    if(FinalizeLinks(*world) != 0)
        return -1;
    if(world->sim->getOcean() == nullptr)
        return Fail("thruster requires an ocean");

    sf::PhysicsSettings phy;
    phy.mode = sf::PhysicsMode::SUBMERGED;
    phy.collisions = false;
    phy.buoyancy = false;
    const std::string actuator = world->robot.robot->getName() + "/" + thruster->name;
    auto propeller = std::make_shared<sf::Polyhedron>(
        actuator + "/Propeller",
        phy,
        FullPath(*world, thruster->propeller_mesh),
        thruster->propeller_scale,
        sf::I4(),
        Text(thruster->propeller_material),
        Text(thruster->propeller_look),
        sf::Scalar(-1),
        sf::GeometryApproxType::CYLINDER);
    const sf::Scalar inertia = propeller->getInertia().getX() + propeller->getAddedInertia().getX();
    std::shared_ptr<sf::RotorDynamics> rotor;
    if(thruster->time_constant > 0.0)
        rotor = std::make_shared<sf::FirstOrder>(thruster->time_constant);
    else
        rotor = std::make_shared<sf::MechanicalPI>(inertia, thruster->kp, thruster->ki, thruster->ilimit);
    auto thrust = std::make_shared<sf::FDThrust>(
        thruster->diameter,
        thruster->thrust_forward,
        thruster->thrust_reverse,
        thruster->torque_coeff,
        thruster->right_handed != 0,
        world->sim->getOcean()->getLiquid().density);
    auto* actuator_body = new sf::Thruster(
        actuator,
        propeller,
        rotor,
        thrust,
        thruster->diameter,
        thruster->right_handed != 0,
        thruster->max_setpoint,
        thruster->inverted_setpoint != 0,
        thruster->normalized_setpoint != 0);
    const std::string link = world->robot.robot->getName() + "/" + thruster->link;
    world->robot.robot->AddLinkActuator(actuator_body, link, ToTransform(thruster->origin));
    return 0;
}

int sf_robot_odometry(SfWorld* world, const SfSensor* sensor)
{
    if(!Ready(world) || sensor == nullptr || !Named(sensor->name) || !Named(sensor->link))
        return Fail("odometry sensor is missing a name or link");
    if(FinalizeLinks(*world) != 0)
        return -1;
    const std::string name = world->robot.robot->getName() + "/" + sensor->name;
    auto* odometry = new sf::Odometry(name, sensor->rate, sensor->history);
    const std::string link = world->robot.robot->getName() + "/" + sensor->link;
    world->robot.robot->AddLinkSensor(odometry, link, ToTransform(sensor->origin));
    return 0;
}

int sf_robot_imu(SfWorld* world, const SfImu* imu)
{
    if(!Ready(world) || imu == nullptr || !Named(imu->sensor.name) || !Named(imu->sensor.link))
        return Fail("imu is missing a name or link");
    if(FinalizeLinks(*world) != 0)
        return -1;
    const std::string name = world->robot.robot->getName() + "/" + imu->sensor.name;
    auto* sensor = new sf::IMU(name, imu->sensor.rate, imu->sensor.history);
    sensor->setRange(
        sf::Vector3(imu->angular_velocity_range[0], imu->angular_velocity_range[1], imu->angular_velocity_range[2]),
        sf::Vector3(imu->linear_acceleration_range, imu->linear_acceleration_range, imu->linear_acceleration_range));
    sensor->setNoise(
        sf::Vector3(imu->angle_noise[0], imu->angle_noise[1], imu->angle_noise[2]),
        sf::Vector3(imu->angular_velocity_noise, imu->angular_velocity_noise, imu->angular_velocity_noise),
        imu->yaw_drift,
        sf::Vector3(imu->linear_acceleration_noise, imu->linear_acceleration_noise, imu->linear_acceleration_noise));
    const std::string link = world->robot.robot->getName() + "/" + imu->sensor.link;
    world->robot.robot->AddLinkSensor(sensor, link, ToTransform(imu->sensor.origin));
    return 0;
}

int sf_robot_camera(SfWorld* world, const SfCamera* camera)
{
    if(!Ready(world) || camera == nullptr || !Named(camera->sensor.name) || !Named(camera->sensor.link))
        return Fail("camera is missing a name or link");
    if(FinalizeLinks(*world) != 0)
        return -1;
    const std::string name = world->robot.robot->getName() + "/" + camera->sensor.name;
    auto* sensor = new sf::ColorCamera(
        name,
        static_cast<unsigned int>(camera->resolution_x),
        static_cast<unsigned int>(camera->resolution_y),
        camera->horizontal_fov_deg,
        camera->sensor.rate);
    const std::string link = world->robot.robot->getName() + "/" + camera->sensor.link;
    world->robot.robot->AddVisionSensor(sensor, link, ToTransform(camera->sensor.origin));
    return 0;
}

int sf_robot_end(SfWorld* world)
{
    if(!Ready(world) || world->robot.robot == nullptr)
        return Fail("no robot to finish");
    if(FinalizeLinks(*world) != 0)
        return -1;
    world->sim->AddRobot(world->robot.robot, world->robot.world);
    world->robot.added = true;
    world->robot.robot = nullptr;
    world->robot.compound = nullptr;
    world->robot.links_ready = false;
    return 0;
}

int sf_world_class_count(const SfWorld* world)
{
    if(world == nullptr)
        return 0;
    return static_cast<int>(world->classes.size());
}

int sf_world_class_at(const SfWorld* world, int index, char* name, int name_cap, char* cls, int cls_cap)
{
    if(world == nullptr || index < 0 || index >= static_cast<int>(world->classes.size()))
        return -1;
    if(name == nullptr || cls == nullptr || name_cap <= 0 || cls_cap <= 0)
        return -1;
    const auto& entry = world->classes[static_cast<size_t>(index)];
    std::snprintf(name, static_cast<size_t>(name_cap), "%s", entry.first.c_str());
    std::snprintf(cls, static_cast<size_t>(cls_cap), "%s", entry.second.c_str());
    return 0;
}

static_assert(sizeof(double) == 8, "Sf ABI uses 8-byte doubles");
static_assert(offsetof(SfPose, rpy) == 24, "SfPose.rpy");
static_assert(sizeof(SfPose) == 48, "SfPose");
static_assert(offsetof(SfMaterial, density) == 8, "SfMaterial.density");
static_assert(sizeof(SfMaterial) == 32, "SfMaterial");
static_assert(offsetof(SfLook, texture) == 56, "SfLook.texture");
static_assert(sizeof(SfLook) == 72, "SfLook");
static_assert(sizeof(SfFriction) == 32, "SfFriction");
static_assert(offsetof(SfEnvironment, particles) == 88, "SfEnvironment.particles");
static_assert(sizeof(SfEnvironment) == 96, "SfEnvironment");
static_assert(sizeof(SfStatic) == 128, "SfStatic");
static_assert(offsetof(SfMesh, convex) == 128, "SfMesh.convex");
static_assert(sizeof(SfMesh) == 136, "SfMesh");
static_assert(offsetof(SfRobot, world) == 32, "SfRobot.world");
static_assert(sizeof(SfRobot) == 80, "SfRobot");
static_assert(sizeof(SfPart) == 152, "SfPart");
static_assert(offsetof(SfPart, mass) == 136, "SfPart.mass");
static_assert(offsetof(SfPartBox, dimensions) == 152, "SfPartBox.dimensions");
static_assert(sizeof(SfPartBox) == 176, "SfPartBox");
static_assert(offsetof(SfPartCylinder, radius) == 152, "SfPartCylinder.radius");
static_assert(sizeof(SfPartCylinder) == 168, "SfPartCylinder");
static_assert(offsetof(SfPartSphere, radius) == 152, "SfPartSphere.radius");
static_assert(sizeof(SfPartSphere) == 160, "SfPartSphere");
static_assert(offsetof(SfPartMesh, path) == 152, "SfPartMesh.path");
static_assert(offsetof(SfPartMesh, visual_origin) == 184, "SfPartMesh.visual_origin");
static_assert(sizeof(SfPartMesh) == 232, "SfPartMesh");
static_assert(offsetof(SfThruster, propeller_mesh) == 96, "SfThruster.propeller_mesh");
static_assert(offsetof(SfThruster, time_constant) == 176, "SfThruster.time_constant");
static_assert(sizeof(SfThruster) == 184, "SfThruster");
static_assert(offsetof(SfSensor, history) == 72, "SfSensor.history");
static_assert(sizeof(SfSensor) == 80, "SfSensor");
static_assert(offsetof(SfImu, angular_velocity_range) == 80, "SfImu.angular_velocity_range");
static_assert(sizeof(SfImu) == 160, "SfImu");
static_assert(offsetof(SfCamera, horizontal_fov_deg) == 88, "SfCamera.horizontal_fov_deg");
static_assert(sizeof(SfCamera) == 96, "SfCamera");
