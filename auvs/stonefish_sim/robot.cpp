#include "robot.hpp"

#include <Stonefish/actuators/ActuatorDynamics.h>
#include <Stonefish/actuators/Thruster.h>
#include <Stonefish/core/FeatherstoneRobot.h>
#include <Stonefish/core/GraphicalSimulationApp.h>
#include <Stonefish/core/MaterialManager.h>
#include <Stonefish/core/Robot.h>
#include <Stonefish/core/SimulationApp.h>
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/entities/forcefields/Ocean.h>
#include <Stonefish/entities/solids/Box.h>
#include <Stonefish/entities/solids/Compound.h>
#include <Stonefish/entities/solids/Cylinder.h>
#include <Stonefish/entities/solids/Polyhedron.h>
#include <Stonefish/entities/solids/Sphere.h>
#include <Stonefish/graphics/OpenGLContent.h>
#include <Stonefish/graphics/OpenGLPipeline.h>
#include <Stonefish/sensors/scalar/IMU.h>
#include <Stonefish/sensors/scalar/Odometry.h>
#include <Stonefish/sensors/vision/ColorCamera.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_set>

namespace {

struct Pose { double xyz[3]{}; double rpy[3]{}; };
struct MaterialSpec { const char* name; double density, restitution, magnetic; };
struct LookSpec { const char* name; double rgb[3]; double roughness, metalness, reflectivity; const char* texture; const char* normal_map; };
struct RobotSpec { const char* name; const char* base_link; int fixed, self_collisions, physics, buoyant; Pose world; };
struct PartSpec { const char* name; const char* material; const char* look; int physics, buoyant; double thickness; Pose origin, compound; double mass; int internal; };
struct BoxSpec { PartSpec part; double dimensions[3]; };
struct CylinderSpec { PartSpec part; double radius, height; };
struct MeshSpec { PartSpec part; const char* path; double scale; const char* visual_path; double visual_scale; Pose visual_origin; };
struct SphereSpec { PartSpec part; double radius; };
struct ThrusterSpec {
    const char* name; const char* link; Pose origin; double diameter, max_setpoint;
    int right_handed, inverted_setpoint, normalized_setpoint; const char* propeller_mesh;
    double propeller_scale; const char* propeller_material; const char* propeller_look;
    double kp, ki, ilimit, thrust_forward, thrust_reverse, torque_coeff, time_constant;
};
struct SensorSpec { const char* name; const char* link; Pose origin; double rate; int history; };
struct ImuSpec {
    SensorSpec sensor; double angular_velocity_range[3]; double linear_acceleration_range;
    double angle_noise[3]; double angular_velocity_noise, yaw_drift, linear_acceleration_noise;
};
struct CameraSpec { SensorSpec sensor; int resolution_x, resolution_y; double horizontal_fov_deg; };

constexpr int BODY_SUBMERGED = 2;

sf::Transform ToTransform(const Pose& pose)
{
    const sf::Vector3 rpy(pose.rpy[0], pose.rpy[1], pose.rpy[2]);
    const sf::Vector3 xyz(pose.xyz[0], pose.xyz[1], pose.xyz[2]);
    return sf::Transform(sf::Quaternion(rpy.z(), rpy.y(), rpy.x()), xyz);
}

sf::PhysicsSettings ToPhysics(int mode, int buoyant)
{
    sf::PhysicsSettings physics;
    physics.mode = mode == BODY_SUBMERGED ? sf::PhysicsMode::SUBMERGED : sf::PhysicsMode::SURFACE;
    physics.buoyancy = buoyant != 0;
    physics.collisions = true;
    return physics;
}

const char* Text(const char* value) { return value != nullptr ? value : ""; }

class RobotBuilder {
public:
    RobotBuilder(sf::SimulationManager& sim, std::filesystem::path dataPath)
        : sim_(sim), dataPath_(std::move(dataPath))
    {
        for(const auto& name : sim_.getMaterialManager()->GetMaterialsList())
            materials_.insert(name);
    }

    ~RobotBuilder()
    {
        if(robot_ == nullptr)
            return;
        if(!linksReady_)
            delete compound_;
        delete robot_;
    }

    int addMaterial(const MaterialSpec& material)
    {
        if(material.name == nullptr || material.name[0] == '\0')
            return Fail("material is missing a name");
        if(materials_.insert(material.name).second)
            sim_.CreateMaterial(material.name, material.density, material.restitution);
        return 0;
    }

    int addLook(const LookSpec& look)
    {
        if(look.name == nullptr || look.name[0] == '\0')
            return Fail("look is missing a name");
        if(!looks_.insert(look.name).second)
            return 0;
        if(sf::SimulationApp::getApp()->hasGraphics())
        {
            auto* app = static_cast<sf::GraphicalSimulationApp*>(sf::SimulationApp::getApp());
            if(app->getGLPipeline()->getContent()->getLookId(look.name) >= 0)
                return 0;
        }
        sim_.CreateLook(
            look.name,
            sf::Color(float(look.rgb[0]), float(look.rgb[1]), float(look.rgb[2])),
            float(look.roughness), float(look.metalness), float(look.reflectivity),
            FullPath(look.texture), FullPath(look.normal_map));
        return 0;
    }

    int begin(const RobotSpec& spec)
    {
        if(robot_ != nullptr)
            return Fail("a robot is already being built");
        if(spec.name == nullptr || spec.base_link == nullptr || spec.name[0] == '\0' || spec.base_link[0] == '\0')
            return Fail("robot is missing a name");
        if(sim_.getOcean() == nullptr)
            return Fail("ocean must exist before the robot");
        robotName_ = spec.name;
        baseLinkName_ = robotName_ + "/" + spec.base_link;
        robot_ = new sf::FeatherstoneRobot(robotName_, spec.fixed != 0);
        selfCollisions_ = spec.self_collisions != 0;
        basePhysics_ = ToPhysics(spec.physics, spec.buoyant);
        world_ = ToTransform(spec.world);
        return 0;
    }

    int addBox(const BoxSpec& spec)
    {
        const sf::Vector3 dimensions(spec.dimensions[0], spec.dimensions[1], spec.dimensions[2]);
        auto* solid = new sf::Box(
            PartName(spec.part.name), ToPhysics(spec.part.physics, spec.part.buoyant), dimensions,
            ToTransform(spec.part.origin), Text(spec.part.material), Text(spec.part.look), spec.part.thickness);
        return addPart(spec.part, solid);
    }

    int addCylinder(const CylinderSpec& spec)
    {
        auto* solid = new sf::Cylinder(
            PartName(spec.part.name), ToPhysics(spec.part.physics, spec.part.buoyant), spec.radius, spec.height,
            ToTransform(spec.part.origin), Text(spec.part.material), Text(spec.part.look), spec.part.thickness);
        return addPart(spec.part, solid);
    }

    int addSphere(const SphereSpec& spec)
    {
        auto* solid = new sf::Sphere(
            PartName(spec.part.name), ToPhysics(spec.part.physics, spec.part.buoyant), spec.radius,
            ToTransform(spec.part.origin), Text(spec.part.material), Text(spec.part.look), spec.part.thickness);
        return addPart(spec.part, solid);
    }

    int addMesh(const MeshSpec& spec)
    {
        if(spec.path == nullptr || spec.path[0] == '\0')
            return Fail("robot mesh part is missing a path");
        sf::Polyhedron* solid = nullptr;
        if(spec.visual_path != nullptr && spec.visual_path[0] != '\0')
            solid = new sf::Polyhedron(
                PartName(spec.part.name), ToPhysics(spec.part.physics, spec.part.buoyant),
                FullPath(spec.visual_path), spec.visual_scale, ToTransform(spec.visual_origin),
                FullPath(spec.path), spec.scale, ToTransform(spec.part.origin),
                Text(spec.part.material), Text(spec.part.look), spec.part.thickness);
        else
            solid = new sf::Polyhedron(
                PartName(spec.part.name), ToPhysics(spec.part.physics, spec.part.buoyant),
                FullPath(spec.path), spec.scale, ToTransform(spec.part.origin),
                Text(spec.part.material), Text(spec.part.look), spec.part.thickness);
        return addPart(spec.part, solid);
    }

    // Detailed photo geometry is independent of the unmeasured mass budget.
    // Internal, massless parts contribute neither collision nor fluid forces.
    int addAppearance(const char* name, const char* material, const char* look, const char* mesh)
    {
        if(robot_ == nullptr || compound_ == nullptr || linksReady_)
            return Fail("appearance requires an unfinished compound body");
        const auto visual = FullPath(mesh);
        const auto proxy = FullPath("models/proteus_photo/appearance_proxy.obj");
        if(!std::filesystem::is_regular_file(visual) || !std::filesystem::is_regular_file(proxy))
            return Fail("Proteus appearance asset is missing; run create_proteus_model.py in Blender");
        sf::PhysicsSettings physics;
        physics.mode = sf::PhysicsMode::SUBMERGED;
        physics.buoyancy = false;
        physics.collisions = false;
        auto* solid = new sf::Polyhedron(PartName(name), physics,
            visual, 1.0, sf::I4(), proxy, 1.0, sf::I4(), material, look);
        solid->ScalePhysicalPropertiesToArbitraryMass(0.0);
        compound_->AddInternalPart(solid, sf::I4());
        return 0;
    }

    void showAppearance() { compound_->setDisplayInternalParts(true); }

    int addThruster(const ThrusterSpec& spec)
    {
        if(FinalizeLinks() != 0)
            return -1;
        if(sim_.getOcean() == nullptr)
            return Fail("thruster requires an ocean");
        sf::PhysicsSettings physics;
        physics.mode = sf::PhysicsMode::SUBMERGED;
        physics.collisions = false;
        physics.buoyancy = false;
        const std::string actuator = robotName_ + "/" + Text(spec.name);
        auto propeller = std::make_shared<sf::Polyhedron>(
            actuator + "/Propeller", physics, FullPath(spec.propeller_mesh), spec.propeller_scale, sf::I4(),
            Text(spec.propeller_material), Text(spec.propeller_look), sf::Scalar(-1), sf::GeometryApproxType::CYLINDER);
        const sf::Scalar inertia = propeller->getInertia().getX() + propeller->getAddedInertia().getX();
        std::shared_ptr<sf::RotorDynamics> rotor;
        if(spec.time_constant > 0.0)
            rotor = std::make_shared<sf::FirstOrder>(spec.time_constant);
        else
            rotor = std::make_shared<sf::MechanicalPI>(inertia, spec.kp, spec.ki, spec.ilimit);
        auto thrust = std::make_shared<sf::FDThrust>(
            spec.diameter, spec.thrust_forward, spec.thrust_reverse, spec.torque_coeff,
            spec.right_handed != 0, sim_.getOcean()->getLiquid().density);
        auto* actuatorBody = new sf::Thruster(
            actuator, propeller, rotor, thrust, spec.diameter, spec.right_handed != 0, spec.max_setpoint,
            spec.inverted_setpoint != 0, spec.normalized_setpoint != 0);
        robot_->AddLinkActuator(actuatorBody, robotName_ + "/" + Text(spec.link), ToTransform(spec.origin));
        return 0;
    }

    int addOdometry(const SensorSpec& spec)
    {
        if(FinalizeLinks() != 0)
            return -1;
        auto* sensor = new sf::Odometry(robotName_ + "/" + Text(spec.name), spec.rate, spec.history);
        robot_->AddLinkSensor(sensor, robotName_ + "/" + Text(spec.link), ToTransform(spec.origin));
        return 0;
    }

    int addImu(const ImuSpec& spec)
    {
        if(FinalizeLinks() != 0)
            return -1;
        auto* sensor = new sf::IMU(robotName_ + "/" + Text(spec.sensor.name), spec.sensor.rate, spec.sensor.history);
        sensor->setRange(
            sf::Vector3(spec.angular_velocity_range[0], spec.angular_velocity_range[1], spec.angular_velocity_range[2]),
            sf::Vector3(spec.linear_acceleration_range, spec.linear_acceleration_range, spec.linear_acceleration_range));
        sensor->setNoise(
            sf::Vector3(spec.angle_noise[0], spec.angle_noise[1], spec.angle_noise[2]),
            sf::Vector3(spec.angular_velocity_noise, spec.angular_velocity_noise, spec.angular_velocity_noise),
            spec.yaw_drift,
            sf::Vector3(spec.linear_acceleration_noise, spec.linear_acceleration_noise, spec.linear_acceleration_noise));
        robot_->AddLinkSensor(sensor, robotName_ + "/" + Text(spec.sensor.link), ToTransform(spec.sensor.origin));
        return 0;
    }

    int addCamera(const CameraSpec& spec)
    {
        if(FinalizeLinks() != 0)
            return -1;
        auto* sensor = new sf::ColorCamera(
            robotName_ + "/" + Text(spec.sensor.name), unsigned(spec.resolution_x), unsigned(spec.resolution_y),
            spec.horizontal_fov_deg, spec.sensor.rate);
        robot_->AddVisionSensor(sensor, robotName_ + "/" + Text(spec.sensor.link), ToTransform(spec.sensor.origin));
        return 0;
    }

    int finish()
    {
        if(robot_ == nullptr)
            return Fail("no robot to finish");
        if(FinalizeLinks() != 0)
            return -1;
        sim_.AddRobot(robot_, world_);
        robot_ = nullptr;
        compound_ = nullptr;
        linksReady_ = false;
        return 0;
    }

private:
    static int Fail(const char* message)
    {
        std::cerr << "[stonefish_sim] " << message << std::endl;
        return -1;
    }

    std::string FullPath(const char* value) const
    {
        if(value == nullptr || value[0] == '\0')
            return {};
        const std::filesystem::path path(value);
        return path.is_absolute() ? path.string() : (dataPath_ / path).string();
    }

    std::string PartName(const char* name) const { return baseLinkName_ + "/" + Text(name); }

    int addPart(const PartSpec& spec, sf::SolidEntity* solid)
    {
        if(robot_ == nullptr || linksReady_ || solid == nullptr)
        {
            delete solid;
            return Fail("robot part added out of order");
        }
        if(spec.mass > 0.0)
            solid->ScalePhysicalPropertiesToArbitraryMass(spec.mass);
        const sf::Transform compound = ToTransform(spec.compound);
        if(compound_ == nullptr)
            compound_ = new sf::Compound(baseLinkName_, basePhysics_, solid, compound);
        else if(spec.internal != 0)
            compound_->AddInternalPart(solid, compound, false);
        else
            compound_->AddExternalPart(solid, compound);
        return 0;
    }

    int FinalizeLinks()
    {
        if(linksReady_)
            return 0;
        if(robot_ == nullptr || compound_ == nullptr)
            return Fail("robot has no parts");
        robot_->DefineLinks(compound_, {}, selfCollisions_);
        robot_->BuildKinematicStructure();
        linksReady_ = true;
        return 0;
    }

    sf::SimulationManager& sim_;
    std::filesystem::path dataPath_;
    std::unordered_set<std::string> materials_;
    std::unordered_set<std::string> looks_;
    std::string robotName_, baseLinkName_;
    sf::FeatherstoneRobot* robot_ = nullptr;
    sf::Compound* compound_ = nullptr;
    sf::Transform world_ = sf::I4();
    sf::PhysicsSettings basePhysics_{};
    bool selfCollisions_ = false;
    bool linksReady_ = false;
};

} // namespace

Pose PoseAt(double x, double y, double z, double roll, double pitch, double yaw)
{
    Pose pose{};
    pose.xyz[0] = x;
    pose.xyz[1] = y;
    pose.xyz[2] = z;
    pose.rpy[0] = roll;
    pose.rpy[1] = pitch;
    pose.rpy[2] = yaw;
    return pose;
}

// The controller drives body +Y as forward. BlueROV's nose is +X, so yaw the
// body contents a quarter turn and the nose lies on +Y.
Pose YawQuarter(Pose pose)
{
    constexpr double kQuarterTurn = 1.5707963267948966;
    const double x = pose.xyz[0];
    pose.xyz[0] = -pose.xyz[1];
    pose.xyz[1] = x;
    pose.rpy[2] += kQuarterTurn;
    return pose;
}

PartSpec SubmergedPart(const char* name, const char* material, const char* look, double thickness, Pose origin, Pose compound)
{
    PartSpec part{};
    part.name = name;
    part.material = material;
    part.look = look;
    part.physics = BODY_SUBMERGED;
    part.buoyant = 1;
    part.thickness = thickness;
    part.origin = origin;
    part.compound = compound;
    return part;
}

PartSpec BodyPart(
    const char* name,
    const char* material,
    const char* look,
    double thickness,
    double mass,
    int buoyant,
    int internal,
    Pose origin,
    Pose compound)
{
    PartSpec part = SubmergedPart(name, material, look, thickness, origin, compound);
    part.mass = mass;
    part.buoyant = buoyant;
    part.internal = internal;
    return part;
}

bool CallOk(int status, const char* what);

bool AddMaterial(RobotBuilder& world, const char* name, double density, double restitution)
{
    const MaterialSpec material{name, density, restitution, 0.0};
    return CallOk(world.addMaterial(material), name);
}

bool AddLook(
    RobotBuilder& world,
    const char* name,
    double red,
    double green,
    double blue,
    double roughness,
    double metalness,
    double reflectivity,
    const char* texture)
{
    const LookSpec look{name, {red, green, blue}, roughness, metalness, reflectivity, texture, nullptr};
    return CallOk(world.addLook(look), name);
}

bool AddThruster(
    RobotBuilder& world,
    const char* name,
    const char* link,
    Pose origin,
    double diameter,
    double max_setpoint,
    int right_handed,
    int inverted,
    const char* propeller,
    double propeller_scale,
    const char* propeller_material,
    const char* propeller_look,
    double kp,
    double ki,
    double ilimit,
    double thrust_forward,
    double thrust_reverse,
    double torque,
    double time_constant)
{
    ThrusterSpec thruster{};
    thruster.name = name;
    thruster.link = link;
    thruster.origin = origin;
    thruster.diameter = diameter;
    thruster.max_setpoint = max_setpoint;
    thruster.right_handed = right_handed;
    thruster.inverted_setpoint = inverted;
    thruster.normalized_setpoint = 1;
    thruster.propeller_mesh = propeller;
    thruster.propeller_scale = propeller_scale;
    thruster.propeller_material = propeller_material;
    thruster.propeller_look = propeller_look;
    thruster.kp = kp;
    thruster.ki = ki;
    thruster.ilimit = ilimit;
    thruster.thrust_forward = thrust_forward;
    thruster.thrust_reverse = thrust_reverse;
    thruster.torque_coeff = torque;
    thruster.time_constant = time_constant;
    return CallOk(world.addThruster(thruster), name);
}

bool AddBox(
    RobotBuilder& world,
    const char* name,
    const char* material,
    const char* look,
    double x,
    double y,
    double z,
    double mass,
    int buoyant,
    int internal,
    Pose compound)
{
    BoxSpec box{};
    box.part = BodyPart(name, material, look, -1.0, mass, buoyant, internal, PoseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), compound);
    box.dimensions[0] = x;
    box.dimensions[1] = y;
    box.dimensions[2] = z;
    return CallOk(world.addBox(box), name);
}

bool AddCylinder(
    RobotBuilder& world,
    const char* name,
    const char* material,
    const char* look,
    double radius,
    double height,
    double thickness,
    double mass,
    int buoyant,
    int internal,
    Pose compound)
{
    CylinderSpec cylinder{};
    cylinder.part = BodyPart(
        name, material, look, thickness, mass, buoyant, internal, PoseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), compound);
    cylinder.radius = radius;
    cylinder.height = height;
    return CallOk(world.addCylinder(cylinder), name);
}

bool AddSphere(
    RobotBuilder& world,
    const char* name,
    const char* material,
    const char* look,
    double radius,
    double mass,
    int buoyant,
    int internal,
    Pose compound)
{
    SphereSpec sphere{};
    sphere.part = BodyPart(
        name, material, look, -1.0, mass, buoyant, internal, PoseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), compound);
    sphere.radius = radius;
    return CallOk(world.addSphere(sphere), name);
}

bool AddMesh(
    RobotBuilder& world,
    const char* name,
    const char* material,
    const char* look,
    const char* physics,
    double physics_scale,
    Pose physics_origin,
    const char* visual,
    double visual_scale,
    Pose visual_origin,
    double thickness,
    int buoyant,
    Pose compound)
{
    MeshSpec mesh{};
    mesh.part = BodyPart(name, material, look, thickness, 0.0, buoyant, 0, physics_origin, compound);
    mesh.path = physics;
    mesh.scale = physics_scale;
    mesh.visual_path = visual;
    mesh.visual_scale = visual_scale;
    mesh.visual_origin = visual_origin;
    return CallOk(world.addMesh(mesh), name);
}

bool CallOk(int status, const char* what)
{
    if(status == 0)
        return true;
    std::cerr << "[stonefish_sim] " << what << " failed" << std::endl;
    return false;
}

bool BuildHydrusRobot(RobotBuilder& world)
{
    const MaterialSpec materials[] = {
        {"acrylic", 1200.0, 0.3, 0.0},
        {"aluminium", 2700.0, 0.8, 0.0},
        {"abs", 1040.0, 0.4, 0.0},
        {"hdpe", 950.0, 0.4, 0.0},
        {"pvc", 1500.0, 0.4, 0.0},
    };
    for(const MaterialSpec& material : materials)
    {
        if(!CallOk(world.addMaterial(material), "hydrus material"))
            return false;
    }
    const LookSpec looks[] = {
        {"clear", {0.9, 0.9, 0.9}, 0.1, 0.0, 0.5, nullptr, nullptr},
        {"black", {0.0, 0.0, 0.0}, 0.1, 0.0, 0.5, nullptr, nullptr},
        {"green", {0.0, 0.2, 0.0}, 0.1, 0.1, 0.5, nullptr, nullptr},
        {"propeller", {1.0, 1.0, 1.0}, 0.3, 0.0, 0.5, nullptr, nullptr},
    };
    for(const LookSpec& look : looks)
    {
        if(!CallOk(world.addLook(look), "hydrus look"))
            return false;
    }

    RobotSpec robot{};
    robot.name = "HydrusAUV";
    robot.base_link = "Hydrus";
    robot.fixed = 0;
    robot.self_collisions = 0;
    robot.physics = BODY_SUBMERGED;
    robot.buoyant = 1;
    robot.world = PoseAt(0.0, -1.0, 2.0, 0.0, 0.0, 3.14);
    if(!CallOk(world.begin(robot), "hydrus robot"))
        return false;

    const Pose mesh_origin = PoseAt(0.0, 0.0, 0.0, -1.5708, 0.0, 0.0);
    CylinderSpec cabin{};
    cabin.part = SubmergedPart("Cabin", "acrylic", "clear", 0.005, mesh_origin, PoseAt(0.0, 0.0, -0.0775, 0.0, 0.0, 0.0));
    cabin.radius = 0.0825;
    cabin.height = 0.75;
    BoxSpec electronics{};
    electronics.part = SubmergedPart("Box", "aluminium", "black", 0.003, mesh_origin, PoseAt(0.0, 0.0, -0.080, 0.0, 0.0, 0.0));
    electronics.dimensions[0] = 0.22;
    electronics.dimensions[1] = 0.22;
    electronics.dimensions[2] = 0.12;
    BoxSpec dvl{};
    dvl.part = SubmergedPart("DVL", "abs", "green", -1.0, mesh_origin, PoseAt(0.0, 0.0, 0.07, 0.0, 0.0, 0.0));
    dvl.dimensions[0] = 0.1;
    dvl.dimensions[1] = 0.1;
    dvl.dimensions[2] = 0.1;
    MeshSpec legs_left{};
    legs_left.part = SubmergedPart(
        "LowLegsLeft", "hdpe", "black", -1.0, mesh_origin, PoseAt(0.0, -0.375, -0.08, 0.0, 0.0, 0.0));
    legs_left.path = "models/hydrus_lowlegs.obj";
    legs_left.scale = 0.01;
    MeshSpec legs_right{};
    legs_right.part = SubmergedPart(
        "LowLegsRight", "hdpe", "black", -1.0, mesh_origin, PoseAt(0.228, -0.375, -0.08, 0.0, 0.0, 0.0));
    legs_right.path = "models/hydrus_lowlegs.obj";
    legs_right.scale = 0.01;
    if(!CallOk(world.addCylinder(cabin), "cabin")
       || !CallOk(world.addBox(electronics), "electronics box")
       || !CallOk(world.addBox(dvl), "dvl")
       || !CallOk(world.addMesh(legs_left), "left legs")
       || !CallOk(world.addMesh(legs_right), "right legs"))
        return false;

    const double max_setpoint = 1000.0 / 60.0 * 2.0 * 3.14159265358979323846;
    const Pose mounts[] = {
        PoseAt(0.198, 0.407, 0.0, 0.0, 0.0, -1.1781),
        PoseAt(-0.198, 0.407, 0.0, 0.0, 0.0, -1.9635),
        PoseAt(0.198, -0.408, 0.0, 0.0, 0.0, 1.1781),
        PoseAt(-0.198, -0.408, 0.0, 0.0, 0.0, 1.9635),
        PoseAt(0.211, 0.169, 0.0, 0.0, -1.571, 0.0),
        PoseAt(-0.211, 0.169, 0.0, 0.0, -1.571, 0.0),
        PoseAt(0.211, -0.169, 0.0, 0.0, -1.571, 0.0),
        PoseAt(-0.211, -0.169, 0.0, 0.0, -1.571, 0.0),
    };
    const char* thruster_names[] = {
        "thruster_0_front_left",
        "thruster_1_front_right",
        "thruster_2_back_left",
        "thruster_3_back_right",
        "thruster_4_depth_front_left",
        "thruster_5_front_right",
        "thruster_6_depth_back_left",
        "thruster_7_back_right",
    };
    for(int i = 0; i < 8; ++i)
    {
        ThrusterSpec thruster{};
        thruster.name = thruster_names[i];
        thruster.link = "Hydrus";
        thruster.origin = mounts[i];
        thruster.diameter = 0.18;
        thruster.max_setpoint = max_setpoint;
        thruster.right_handed = 1;
        thruster.inverted_setpoint = 1;
        thruster.normalized_setpoint = 1;
        thruster.propeller_mesh = "models/propeller.obj";
        thruster.propeller_scale = 0.75;
        thruster.propeller_material = "pvc";
        thruster.propeller_look = "propeller";
        thruster.kp = 1.0;
        thruster.ki = 10.0;
        thruster.ilimit = 5.0;
        thruster.thrust_forward = 0.48;
        thruster.thrust_reverse = 0.48;
        thruster.torque_coeff = 0.05;
        if(!CallOk(world.addThruster(thruster), thruster.name))
            return false;
    }

    SensorSpec odometry{};
    odometry.name = "Odometry";
    odometry.link = "Hydrus";
    odometry.origin = PoseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    odometry.rate = 120.0;
    odometry.history = -1;
    ImuSpec imu{};
    imu.sensor.name = "HydrusCameraIMU";
    imu.sensor.link = "Hydrus";
    imu.sensor.origin = PoseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    imu.sensor.rate = 30.0;
    imu.sensor.history = 1;
    imu.angular_velocity_range[0] = 20.0;
    imu.angular_velocity_range[1] = 20.0;
    imu.angular_velocity_range[2] = 20.0;
    imu.linear_acceleration_range = 30.0;
    imu.angle_noise[0] = 0.001;
    imu.angle_noise[1] = 0.001;
    imu.angle_noise[2] = 0.001;
    imu.angular_velocity_noise = 0.0005;
    imu.yaw_drift = 0.0;
    imu.linear_acceleration_noise = 0.005;
    CameraSpec camera{};
    camera.sensor.name = "RGBCamera";
    camera.sensor.link = "Hydrus";
    camera.sensor.origin = PoseAt(0.0, 0.3, 0.0, 1.5708, 0.0, 3.14);
    camera.sensor.rate = 20.0;
    camera.sensor.history = -1;
    camera.resolution_x = 800;
    camera.resolution_y = 600;
    camera.horizontal_fov_deg = 60.0;
    if(!CallOk(world.addOdometry(odometry), "odometry")
       || !CallOk(world.addImu(imu), "imu")
       || !CallOk(world.addCamera(camera), "camera")
       || !CallOk(world.finish(), "hydrus robot end"))
        return false;
    return true;
}

#include "proteus_appearance.inc"

bool BuildProteusRobot(RobotBuilder& world)
{
    const Pose identity = PoseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    if(!AddLook(world, "blue", 0.0, 0.5, 1.0, 0.3, 0.0, 0.5, nullptr))
        return false;

    RobotSpec robot{};
    robot.name = "ProteusAUV";
    robot.base_link = "Proteus";
    robot.physics = BODY_SUBMERGED;
    robot.buoyant = 1;
    robot.world = PoseAt(0.0, -1.0, 2.0, 0.0, 0.0, 3.14);
    if(!CallOk(world.begin(robot), "proteus robot"))
        return false;
    if(!AddCylinder(world, "TopCabin", "acrylic", "clear", 0.1, 0.3, 0.005, 0.0, 1, 0, PoseAt(0.0, 0.0, -0.05, 1.5708, 0.0, 0.0))
       || !AddCylinder(world, "BottomCabin", "acrylic", "clear", 0.05, 0.31, 0.005, 0.0, 1, 0, PoseAt(0.0, 0.0, 0.13, 1.5708, 0.0, 0.0))
       || !AddBox(world, "LeftRail", "abs", "blue", 0.04, 0.45, 0.04, 0.0, 1, 0, PoseAt(0.17, 0.0, -0.03, 0.0, 0.0, 0.0))
       || !AddBox(world, "RightRail", "abs", "blue", 0.04, 0.45, 0.04, 0.0, 1, 0, PoseAt(-0.17, 0.0, -0.03, 0.0, 0.0, 0.0))
       || !AddMesh(
           world, "ProtySupport1", "abs", "blue", "models/protysupport.obj", 0.001, identity, nullptr, 1.0, identity, -1.0, 1,
           PoseAt(0.0, 0.135, -0.012, 1.5708, 0.0, 0.0))
       || !AddMesh(
           world, "ProtySupport2", "abs", "blue", "models/protysupport.obj", 0.001, identity, nullptr, 1.0, identity, -1.0, 1,
           PoseAt(0.0, -0.135, -0.012, 1.5708, 0.0, 0.0)))
        return false;

    if(!AddProteusAppearance(world))
        return false;

    const double max_setpoint = 1000.0 / 60.0 * 2.0 * 3.14159265358979323846;
    const Pose mounts[] = {
        // Side pods sit below the blue rails in the photographs. Positions
        // are provisional photo estimates; retain the existing thrust curve.
        PoseAt(0.205, 0.0, 0.13, 0.0, 0.0, 4.7123),
        PoseAt(-0.205, 0.0, 0.13, 0.0, 0.0, 4.7123),
        PoseAt(0.165, 0.265, 0.0, 0.0, -1.571, 0.0),
        PoseAt(-0.165, 0.265, 0.0, 0.0, -1.571, 0.0),
        PoseAt(0.165, -0.265, 0.0, 0.0, -1.571, 0.0),
        PoseAt(-0.165, -0.265, 0.0, 0.0, -1.571, 0.0),
    };
    const char* names[] = {
        "thruster_0_left",
        "thruster_1_right",
        "thruster_2_depth_front_left",
        "thruster_3_depth_front_right",
        "thruster_4_depth_back_left",
        "thruster_5_depth_back_right",
    };
    for(int i = 0; i < 6; ++i)
    {
        if(!AddThruster(
               world, names[i], "Proteus", mounts[i], 0.18, max_setpoint, 1, 1, "models/proteus_photo/propeller.obj", 1.0, "pvc",
               "proteus_thrusters", 0.0, 0.0, 0.0, 0.48, 0.48, 0.05, 0.2))
            return false;
    }

    SensorSpec odometry{};
    odometry.name = "Odometry";
    odometry.link = "Proteus";
    odometry.origin = identity;
    odometry.rate = 30.0;
    odometry.history = -1;
    ImuSpec imu{};
    imu.sensor.name = "ProteusCameraIMU";
    imu.sensor.link = "Proteus";
    imu.sensor.origin = identity;
    imu.sensor.rate = 200.0;
    imu.sensor.history = 1;
    imu.angular_velocity_range[0] = 20.0;
    imu.angular_velocity_range[1] = 20.0;
    imu.angular_velocity_range[2] = 20.0;
    imu.linear_acceleration_range = 30.0;
    imu.angle_noise[0] = 0.00016968;
    imu.angle_noise[1] = 0.00016968;
    imu.angle_noise[2] = 0.00016968;
    imu.angular_velocity_noise = 0.00016968;
    imu.yaw_drift = 0.000019393;
    imu.linear_acceleration_noise = 0.002;
    CameraSpec camera{};
    camera.sensor.name = "Camera";
    camera.sensor.link = "Proteus";
    camera.sensor.origin = PoseAt(0.0, 0.3, 0.0, 1.5708, 0.0, 3.14);
    camera.sensor.rate = 21.0;
    camera.sensor.history = -1;
    camera.resolution_x = 800;
    camera.resolution_y = 600;
    camera.horizontal_fov_deg = 60.0;
    return CallOk(world.addOdometry(odometry), "proteus odometry")
           && CallOk(world.addImu(imu), "proteus imu")
           && CallOk(world.addCamera(camera), "proteus camera")
           && CallOk(world.finish(), "proteus robot end");
}

bool BuildBluerov2Robot(RobotBuilder& world)
{
    const Pose identity = PoseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    if(!AddMaterial(world, "Fiberglass", 1800.0, 0.3)
       || !AddMaterial(world, "Neutral", 1000.0, 0.5)
       || !AddMaterial(world, "Steel", 7810.0, 0.9)
       || !AddLook(world, "None", 0.2, 0.2, 0.2, 0.5, 0.0, 0.5, nullptr)
       || !AddLook(world, "blue", 0.0, 0.5, 1.0, 0.3, 0.0, 0.5, nullptr)
       || !AddLook(world, "br2", 1.0, 1.0, 1.0, 0.4, 0.5, 0.5, "bluerov2/br2.png"))
        return false;

    RobotSpec robot{};
    robot.name = "bluerov2";
    robot.base_link = "base_link";
    robot.physics = BODY_SUBMERGED;
    robot.buoyant = 1;
    robot.world = PoseAt(0.0, -1.0, 2.0, 0.0, 0.0, 3.14);
    if(!CallOk(world.begin(robot), "bluerov2 robot"))
        return false;
    if(!AddMesh(
           world, "HullBottom", "Fiberglass", "br2", "bluerov2/bluerov2_phy.obj", 1.0, YawQuarter(identity),
           "bluerov2/bluerov2.obj", 1.0, YawQuarter(identity), 0.005, 0, identity)
       || !AddMesh(
           world, "HeavyFit", "Fiberglass", "black", "bluerov2/bluerov2_ring.obj", 1.0,
           YawQuarter(PoseAt(0.0, 0.0, 0.03, 0.0, 0.0, 0.0)), "bluerov2/bluerov2_wings.obj", 1.0, YawQuarter(identity),
           0.005, 0, identity)
       || !AddBox(
           world, "BackLeft", "Neutral", "None", 0.2, 0.15, 0.091, 0.025, 1, 1,
           YawQuarter(PoseAt(-0.1, -0.1, 0.0, 0.0, 0.0, 0.0)))
       || !AddBox(
           world, "BackRight", "Neutral", "None", 0.2, 0.15, 0.091, 0.025, 1, 1,
           YawQuarter(PoseAt(-0.1, 0.1, 0.0, 0.0, 0.0, 0.0)))
       || !AddBox(
           world, "FrontLeft", "Neutral", "None", 0.2, 0.15, 0.091, 0.025, 1, 1,
           YawQuarter(PoseAt(0.09, -0.1, 0.0, 0.0, 0.0, 0.0)))
       || !AddBox(
           world, "FrontRight", "Neutral", "None", 0.2, 0.15, 0.091, 0.025, 1, 1,
           YawQuarter(PoseAt(0.09, 0.1, 0.0, 0.0, 0.0, 0.0)))
       || !AddSphere(
           world, "WeightCenter", "Steel", "black", 0.01, 2.0, 0, 1, YawQuarter(PoseAt(0.0, 0.0, 0.1, 0.0, 0.0, 0.0)))
       || !AddSphere(
           world, "WeightLeft", "Steel", "black", 0.01, 1.0, 0, 1, YawQuarter(PoseAt(0.0, -0.075, 0.1, 0.0, 0.0, 0.0)))
       || !AddSphere(
           world, "WeightRight", "Steel", "black", 0.01, 1.0, 0, 1,
           YawQuarter(PoseAt(0.0, 0.075, 0.1, 0.0, 0.0, 0.0))))
        return false;

    const double max_setpoint = 4000.0 / 60.0 * 2.0 * 3.14159265358979323846;
    struct Mount
    {
        const char* name;
        Pose origin;
        int right_handed;
        int inverted;
        const char* propeller;
    };
    const Mount mounts[] = {
        {"FrontRight", YawQuarter(PoseAt(0.1355, 0.1, 0.0725, 0.0, 0.0, -0.7853981634)), 1, 1, "bluerov2/ccw.obj"},
        {"FrontLeft", YawQuarter(PoseAt(0.1355, -0.1, 0.0725, 0.0, 0.0, 0.7853981634)), 1, 1, "bluerov2/ccw.obj"},
        {"BackRight", YawQuarter(PoseAt(-0.1475, 0.1, 0.0725, 0.0, 0.0, -2.3561944902)), 0, 0, "bluerov2/cw.obj"},
        {"BackLeft", YawQuarter(PoseAt(-0.1475, -0.1, 0.0725, 0.0, 0.0, 2.3561944902)), 0, 0, "bluerov2/cw.obj"},
        {"DiveFrontRight", YawQuarter(PoseAt(0.12, 0.218, 0.0, 0.0, -1.5707963268, 0.0)), 1, 0, "bluerov2/cw.obj"},
        {"DiveFrontLeft", YawQuarter(PoseAt(0.12, -0.218, 0.0, 0.0, -1.5707963268, 0.0)), 0, 1, "bluerov2/ccw.obj"},
        {"DiveBackRight", YawQuarter(PoseAt(-0.12, 0.218, 0.0, 0.0, -1.5707963268, 0.0)), 0, 1, "bluerov2/ccw.obj"},
        {"DiveBackLeft", YawQuarter(PoseAt(-0.12, -0.218, 0.0, 0.0, -1.5707963268, 0.0)), 1, 0, "bluerov2/cw.obj"},
    };
    for(const Mount& mount : mounts)
    {
        if(!AddThruster(
               world, mount.name, "base_link", mount.origin, 0.076, max_setpoint, mount.right_handed, mount.inverted,
               mount.propeller, 1.0, "Neutral", "blue", 0.0, 0.0, 0.0, 0.334, 0.334, 0.032, 0.2))
            return false;
    }

    SensorSpec odometry{};
    odometry.name = "odometry";
    odometry.link = "base_link";
    odometry.origin = identity;
    odometry.rate = 100.0;
    odometry.history = -1;
    ImuSpec imu{};
    imu.sensor.name = "imu_filter";
    imu.sensor.link = "base_link";
    imu.sensor.origin = identity;
    imu.sensor.rate = 20.0;
    imu.sensor.history = -1;
    imu.angular_velocity_range[0] = 20.0;
    imu.angular_velocity_range[1] = 20.0;
    imu.angular_velocity_range[2] = 20.0;
    imu.linear_acceleration_range = 30.0;
    imu.angle_noise[0] = 0.000001745;
    imu.angle_noise[1] = 0.000001745;
    imu.angle_noise[2] = 0.000001745;
    imu.angular_velocity_noise = 0.00001745;
    CameraSpec left{};
    left.sensor.name = "camera_left";
    left.sensor.link = "base_link";
    left.sensor.origin = YawQuarter(PoseAt(0.16, -0.0725, 0.15, 1.571, 0.0, 1.571));
    left.sensor.rate = 30.0;
    left.sensor.history = -1;
    left.resolution_x = 640;
    left.resolution_y = 480;
    left.horizontal_fov_deg = 75.0;
    CameraSpec right{};
    right.sensor.name = "camera_right";
    right.sensor.link = "base_link";
    right.sensor.origin = YawQuarter(PoseAt(0.16, 0.0725, 0.15, 1.571, 0.0, 1.571));
    right.sensor.rate = 30.0;
    right.sensor.history = -1;
    right.resolution_x = 640;
    right.resolution_y = 480;
    right.horizontal_fov_deg = 75.0;
    return CallOk(world.addOdometry(odometry), "bluerov2 odometry")
           && CallOk(world.addImu(imu), "bluerov2 imu")
           && CallOk(world.addCamera(left), "bluerov2 left camera")
           && CallOk(world.addCamera(right), "bluerov2 right camera")
           && CallOk(world.finish(), "bluerov2 robot end");
}

bool BuildGirona500Robot(RobotBuilder& world)
{
    const Pose identity = PoseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    if(!AddMaterial(world, "Fiberglass", 1800.0, 0.3)
       || !AddMaterial(world, "Neutral", 1000.0, 0.5)
       || !AddMaterial(world, "Aluminium", 2700.0, 0.8))
        return false;

    RobotSpec robot{};
    robot.name = "Girona500AUV";
    robot.base_link = "Vehicle";
    robot.physics = BODY_SUBMERGED;
    robot.buoyant = 1;
    robot.world = PoseAt(0.0, -1.0, 2.0, 0.0, 0.0, -1.5708);
    if(!CallOk(world.begin(robot), "girona500 robot"))
        return false;
    if(!AddMesh(
           world, "HullBottom", "Fiberglass", "yellow", "models/hull_hydro.obj", 1.0, identity, nullptr, 1.0, identity, 0.003, 0,
           PoseAt(-0.049, 0.0, 0.361, 0.0, 0.0, 0.0))
       || !AddMesh(
           world, "HullPort", "Fiberglass", "yellow", "models/hull_hydro.obj", 1.0, identity, nullptr, 1.0, identity, 0.003, 0,
           PoseAt(-0.049, -0.35, -0.339, 0.0, 0.0, 0.0))
       || !AddMesh(
           world, "HullStarboard", "Fiberglass", "yellow", "models/hull_hydro.obj", 1.0, identity, nullptr, 1.0, identity, 0.003, 0,
           PoseAt(-0.049, 0.35, -0.339, 0.0, 0.0, 0.0))
       || !AddMesh(
           world, "VBarStern", "Aluminium", "gray", "models/vbar_hydro.obj", 1.0, identity, nullptr, 1.0, identity, 0.003, 0,
           PoseAt(-0.299, 0.0, 0.211, 0.0, 0.0, 0.0))
       || !AddMesh(
           world, "VBarBow", "Aluminium", "gray", "models/vbar_hydro.obj", 1.0, identity, nullptr, 1.0, identity, 0.003, 0,
           PoseAt(0.251, 0.0, 0.211, 0.0, 0.0, 0.0))
       || !AddMesh(
           world, "DuctSway", "Neutral", "gray", "models/duct_hydro.obj", 1.0, identity, nullptr, 1.0, identity, -1.0, 1,
           PoseAt(-0.0627, 0.0307, -0.019, 0.0, 3.14, 1.57))
       || !AddMesh(
           world, "DuctSurgePort", "Neutral", "gray", "models/duct_hydro.obj", 1.0, identity, nullptr, 1.0, identity, -1.0, 1,
           PoseAt(-0.3297, -0.2587, -0.019, 3.14, 0.0, 0.0))
       || !AddMesh(
           world, "DuctSurgeStarboard", "Neutral", "gray", "models/duct_hydro.obj", 1.0, identity, nullptr, 1.0, identity, -1.0, 1,
           PoseAt(-0.3297, 0.2587, -0.019, 0.0, 0.0, 0.0))
       || !AddMesh(
           world, "DuctHeaveStern", "Neutral", "gray", "models/duct_hydro.obj", 1.0, identity, nullptr, 1.0, identity, -1.0, 1,
           PoseAt(-0.5827, 0.0, -0.3137, 0.0, -1.57, 1.57))
       || !AddMesh(
           world, "DuctHeaveBow", "Neutral", "gray", "models/duct_hydro.obj", 1.0, identity, nullptr, 1.0, identity, -1.0, 1,
           PoseAt(0.5347, 0.0, -0.3137, 0.0, -1.57, -1.57))
       || !AddCylinder(
           world, "BatteryCylinder", "Neutral", "gray", 0.13, 0.6, -1.0, 70.0, 1, 1, PoseAt(-0.099, 0.0, 0.361, 0.0, 1.571, 0.0))
       || !AddCylinder(
           world, "PortCylinder", "Neutral", "gray", 0.13, 1.0, -1.0, 20.0, 1, 1, PoseAt(-0.049, -0.35, -0.339, 0.0, 1.571, 0.0))
       || !AddCylinder(
           world, "StarboardCylinder", "Neutral", "gray", 0.13, 1.0, -1.0, 20.0, 1, 1,
           PoseAt(-0.049, 0.35, -0.339, 0.0, 1.571, 0.0)))
        return false;

    const double max_setpoint = 1000.0 / 60.0 * 2.0 * 3.14159265358979323846;
    const Pose mounts[] = {
        PoseAt(-0.3297, -0.2587, -0.021, 3.1416, 0.0, 0.0),
        PoseAt(-0.3297, 0.2587, -0.021, 0.0, 0.0, 0.0),
        PoseAt(0.5347, 0.0, -0.3137, 0.0, -1.571, -1.571),
        PoseAt(-0.5827, 0.0, -0.3137, 0.0, -1.571, 1.571),
        PoseAt(-0.0627, 0.0307, -0.021, 0.0, 3.1416, 1.571),
    };
    const char* names[] = {
        "ThrusterSurgePort",
        "ThrusterSurgeStarboard",
        "ThrusterHeaveBow",
        "ThrusterHeaveStern",
        "ThrusterSway",
    };
    for(int i = 0; i < 5; ++i)
    {
        if(!AddThruster(
               world, names[i], "Vehicle", mounts[i], 0.18, max_setpoint, 1, 1, "models/propeller.obj", 1.0, "Neutral",
               "propeller", 1.0, 0.5, 2.0, 0.48, 0.48, 0.05, 0.0))
            return false;
    }

    SensorSpec odometry{};
    odometry.name = "Odometry";
    odometry.link = "Vehicle";
    odometry.origin = identity;
    odometry.rate = 30.0;
    odometry.history = -1;
    CameraSpec camera{};
    camera.sensor.name = "Camera";
    camera.sensor.link = "Vehicle";
    camera.sensor.origin = PoseAt(0.5, -0.12, 0.5, 0.0, 0.0, 1.571);
    camera.sensor.rate = 10.0;
    camera.sensor.history = -1;
    camera.resolution_x = 800;
    camera.resolution_y = 600;
    camera.horizontal_fov_deg = 60.0;
    return CallOk(world.addOdometry(odometry), "girona500 odometry")
           && CallOk(world.addCamera(camera), "girona500 camera")
           && CallOk(world.finish(), "girona500 robot end");
}

bool BuildRobotWithBuilder(RobotBuilder& world, const std::string& robot)
{
    if(robot.empty() || robot == "hydrus")
        return BuildHydrusRobot(world);
    if(robot == "proteus")
        return BuildProteusRobot(world);
    if(robot == "bluerov2")
        return BuildBluerov2Robot(world);
    if(robot == "girona500")
        return BuildGirona500Robot(world);
    std::cerr << "[stonefish_sim] unknown robot '" << robot << "'" << std::endl;
    return false;
}

bool BuildRobot(sf::SimulationManager& sim, const std::filesystem::path& dataPath, const std::string& robot)
{
    RobotBuilder builder(sim, dataPath);
    return BuildRobotWithBuilder(builder, robot);
}
