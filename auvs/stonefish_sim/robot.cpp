#include "stonefish_c/include/stonefish_c.h"

#include <iostream>
#include <string>

SfPose PoseAt(double x, double y, double z, double roll, double pitch, double yaw)
{
    SfPose pose{};
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
SfPose YawQuarter(SfPose pose)
{
    constexpr double kQuarterTurn = 1.5707963267948966;
    const double x = pose.xyz[0];
    pose.xyz[0] = -pose.xyz[1];
    pose.xyz[1] = x;
    pose.rpy[2] += kQuarterTurn;
    return pose;
}

SfPart SubmergedPart(const char* name, const char* material, const char* look, double thickness, SfPose origin, SfPose compound)
{
    SfPart part{};
    part.name = name;
    part.material = material;
    part.look = look;
    part.physics = SF_BODY_SUBMERGED;
    part.buoyant = 1;
    part.thickness = thickness;
    part.origin = origin;
    part.compound = compound;
    return part;
}

SfPart BodyPart(
    const char* name,
    const char* material,
    const char* look,
    double thickness,
    double mass,
    int buoyant,
    int internal,
    SfPose origin,
    SfPose compound)
{
    SfPart part = SubmergedPart(name, material, look, thickness, origin, compound);
    part.mass = mass;
    part.buoyant = buoyant;
    part.internal = internal;
    return part;
}

bool CallOk(int status, const char* what);

bool AddMaterial(SfWorld* world, const char* name, double density, double restitution)
{
    const SfMaterial material{name, density, restitution, 0.0};
    return CallOk(sf_material(world, &material), name);
}

bool AddLook(
    SfWorld* world,
    const char* name,
    double red,
    double green,
    double blue,
    double roughness,
    double metalness,
    double reflectivity,
    const char* texture)
{
    const SfLook look{name, {red, green, blue}, roughness, metalness, reflectivity, texture, nullptr};
    return CallOk(sf_look(world, &look), name);
}

bool AddThruster(
    SfWorld* world,
    const char* name,
    const char* link,
    SfPose origin,
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
    SfThruster thruster{};
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
    return CallOk(sf_robot_thruster(world, &thruster), name);
}

bool AddBox(
    SfWorld* world,
    const char* name,
    const char* material,
    const char* look,
    double x,
    double y,
    double z,
    double mass,
    int buoyant,
    int internal,
    SfPose compound)
{
    SfPartBox box{};
    box.part = BodyPart(name, material, look, -1.0, mass, buoyant, internal, PoseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), compound);
    box.dimensions[0] = x;
    box.dimensions[1] = y;
    box.dimensions[2] = z;
    return CallOk(sf_robot_part_box(world, &box), name);
}

bool AddCylinder(
    SfWorld* world,
    const char* name,
    const char* material,
    const char* look,
    double radius,
    double height,
    double thickness,
    double mass,
    int buoyant,
    int internal,
    SfPose compound)
{
    SfPartCylinder cylinder{};
    cylinder.part = BodyPart(
        name, material, look, thickness, mass, buoyant, internal, PoseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), compound);
    cylinder.radius = radius;
    cylinder.height = height;
    return CallOk(sf_robot_part_cylinder(world, &cylinder), name);
}

bool AddSphere(
    SfWorld* world,
    const char* name,
    const char* material,
    const char* look,
    double radius,
    double mass,
    int buoyant,
    int internal,
    SfPose compound)
{
    SfPartSphere sphere{};
    sphere.part = BodyPart(
        name, material, look, -1.0, mass, buoyant, internal, PoseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), compound);
    sphere.radius = radius;
    return CallOk(sf_robot_part_sphere(world, &sphere), name);
}

bool AddMesh(
    SfWorld* world,
    const char* name,
    const char* material,
    const char* look,
    const char* physics,
    double physics_scale,
    SfPose physics_origin,
    const char* visual,
    double visual_scale,
    SfPose visual_origin,
    double thickness,
    int buoyant,
    SfPose compound)
{
    SfPartMesh mesh{};
    mesh.part = BodyPart(name, material, look, thickness, 0.0, buoyant, 0, physics_origin, compound);
    mesh.path = physics;
    mesh.scale = physics_scale;
    mesh.visual_path = visual;
    mesh.visual_scale = visual_scale;
    mesh.visual_origin = visual_origin;
    return CallOk(sf_robot_part_mesh(world, &mesh), name);
}

bool CallOk(int status, const char* what)
{
    if(status == 0)
        return true;
    std::cerr << "[stonefish_sim] " << what << " failed" << std::endl;
    return false;
}

bool BuildHydrusRobot(SfWorld* world)
{
    const SfMaterial materials[] = {
        {"acrylic", 1200.0, 0.3, 0.0},
        {"aluminium", 2700.0, 0.8, 0.0},
        {"abs", 1040.0, 0.4, 0.0},
        {"hdpe", 950.0, 0.4, 0.0},
        {"pvc", 1500.0, 0.4, 0.0},
    };
    for(const SfMaterial& material : materials)
    {
        if(!CallOk(sf_material(world, &material), "hydrus material"))
            return false;
    }
    const SfLook looks[] = {
        {"clear", {0.9, 0.9, 0.9}, 0.1, 0.0, 0.5, nullptr, nullptr},
        {"black", {0.0, 0.0, 0.0}, 0.1, 0.0, 0.5, nullptr, nullptr},
        {"green", {0.0, 0.2, 0.0}, 0.1, 0.1, 0.5, nullptr, nullptr},
        {"propeller", {1.0, 1.0, 1.0}, 0.3, 0.0, 0.5, nullptr, nullptr},
    };
    for(const SfLook& look : looks)
    {
        if(!CallOk(sf_look(world, &look), "hydrus look"))
            return false;
    }

    SfRobot robot{};
    robot.name = "HydrusAUV";
    robot.base_link = "Hydrus";
    robot.fixed = 0;
    robot.self_collisions = 0;
    robot.physics = SF_BODY_SUBMERGED;
    robot.buoyant = 1;
    robot.world = PoseAt(0.0, -1.0, 2.0, 0.0, 0.0, 3.14);
    if(!CallOk(sf_robot_begin(world, &robot), "hydrus robot"))
        return false;

    const SfPose mesh_origin = PoseAt(0.0, 0.0, 0.0, -1.5708, 0.0, 0.0);
    SfPartCylinder cabin{};
    cabin.part = SubmergedPart("Cabin", "acrylic", "clear", 0.005, mesh_origin, PoseAt(0.0, 0.0, -0.0775, 0.0, 0.0, 0.0));
    cabin.radius = 0.0825;
    cabin.height = 0.75;
    SfPartBox electronics{};
    electronics.part = SubmergedPart("Box", "aluminium", "black", 0.003, mesh_origin, PoseAt(0.0, 0.0, -0.080, 0.0, 0.0, 0.0));
    electronics.dimensions[0] = 0.22;
    electronics.dimensions[1] = 0.22;
    electronics.dimensions[2] = 0.12;
    SfPartBox dvl{};
    dvl.part = SubmergedPart("DVL", "abs", "green", -1.0, mesh_origin, PoseAt(0.0, 0.0, 0.07, 0.0, 0.0, 0.0));
    dvl.dimensions[0] = 0.1;
    dvl.dimensions[1] = 0.1;
    dvl.dimensions[2] = 0.1;
    SfPartMesh legs_left{};
    legs_left.part = SubmergedPart(
        "LowLegsLeft", "hdpe", "black", -1.0, mesh_origin, PoseAt(0.0, -0.375, -0.08, 0.0, 0.0, 0.0));
    legs_left.path = "models/hydrus_lowlegs.obj";
    legs_left.scale = 0.01;
    SfPartMesh legs_right{};
    legs_right.part = SubmergedPart(
        "LowLegsRight", "hdpe", "black", -1.0, mesh_origin, PoseAt(0.228, -0.375, -0.08, 0.0, 0.0, 0.0));
    legs_right.path = "models/hydrus_lowlegs.obj";
    legs_right.scale = 0.01;
    if(!CallOk(sf_robot_part_cylinder(world, &cabin), "cabin")
       || !CallOk(sf_robot_part_box(world, &electronics), "electronics box")
       || !CallOk(sf_robot_part_box(world, &dvl), "dvl")
       || !CallOk(sf_robot_part_mesh(world, &legs_left), "left legs")
       || !CallOk(sf_robot_part_mesh(world, &legs_right), "right legs"))
        return false;

    const double max_setpoint = 1000.0 / 60.0 * 2.0 * 3.14159265358979323846;
    const SfPose mounts[] = {
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
        SfThruster thruster{};
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
        if(!CallOk(sf_robot_thruster(world, &thruster), thruster.name))
            return false;
    }

    SfSensor odometry{};
    odometry.name = "Odometry";
    odometry.link = "Hydrus";
    odometry.origin = PoseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    odometry.rate = 120.0;
    odometry.history = -1;
    SfImu imu{};
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
    SfCamera camera{};
    camera.sensor.name = "RGBCamera";
    camera.sensor.link = "Hydrus";
    camera.sensor.origin = PoseAt(0.0, 0.3, 0.0, 1.5708, 0.0, 3.14);
    camera.sensor.rate = 20.0;
    camera.sensor.history = -1;
    camera.resolution_x = 800;
    camera.resolution_y = 600;
    camera.horizontal_fov_deg = 60.0;
    if(!CallOk(sf_robot_odometry(world, &odometry), "odometry")
       || !CallOk(sf_robot_imu(world, &imu), "imu")
       || !CallOk(sf_robot_camera(world, &camera), "camera")
       || !CallOk(sf_robot_end(world), "hydrus robot end"))
        return false;
    return true;
}

bool BuildProteusRobot(SfWorld* world)
{
    const SfPose identity = PoseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    if(!AddLook(world, "blue", 0.0, 0.5, 1.0, 0.3, 0.0, 0.5, nullptr))
        return false;

    SfRobot robot{};
    robot.name = "ProteusAUV";
    robot.base_link = "Proteus";
    robot.physics = SF_BODY_SUBMERGED;
    robot.buoyant = 1;
    robot.world = PoseAt(0.0, -1.0, 2.0, 0.0, 0.0, 3.14);
    if(!CallOk(sf_robot_begin(world, &robot), "proteus robot"))
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

    const double max_setpoint = 1000.0 / 60.0 * 2.0 * 3.14159265358979323846;
    const SfPose mounts[] = {
        PoseAt(0.246, 0.0, -0.03, 0.0, 0.0, 4.7123),
        PoseAt(-0.246, 0.0, -0.03, 0.0, 0.0, 4.7123),
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
               world, names[i], "Proteus", mounts[i], 0.18, max_setpoint, 1, 1, "models/propeller.obj", 0.5, "pvc",
               "propeller", 0.0, 0.0, 0.0, 0.48, 0.48, 0.05, 0.2))
            return false;
    }

    SfSensor odometry{};
    odometry.name = "Odometry";
    odometry.link = "Proteus";
    odometry.origin = identity;
    odometry.rate = 30.0;
    odometry.history = -1;
    SfImu imu{};
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
    SfCamera camera{};
    camera.sensor.name = "Camera";
    camera.sensor.link = "Proteus";
    camera.sensor.origin = PoseAt(0.0, 0.3, 0.0, 1.5708, 0.0, 3.14);
    camera.sensor.rate = 21.0;
    camera.sensor.history = -1;
    camera.resolution_x = 800;
    camera.resolution_y = 600;
    camera.horizontal_fov_deg = 60.0;
    return CallOk(sf_robot_odometry(world, &odometry), "proteus odometry")
           && CallOk(sf_robot_imu(world, &imu), "proteus imu")
           && CallOk(sf_robot_camera(world, &camera), "proteus camera")
           && CallOk(sf_robot_end(world), "proteus robot end");
}

bool BuildBluerov2Robot(SfWorld* world)
{
    const SfPose identity = PoseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    if(!AddMaterial(world, "Fiberglass", 1800.0, 0.3)
       || !AddMaterial(world, "Neutral", 1000.0, 0.5)
       || !AddMaterial(world, "Steel", 7810.0, 0.9)
       || !AddLook(world, "None", 0.2, 0.2, 0.2, 0.5, 0.0, 0.5, nullptr)
       || !AddLook(world, "blue", 0.0, 0.5, 1.0, 0.3, 0.0, 0.5, nullptr)
       || !AddLook(world, "br2", 1.0, 1.0, 1.0, 0.4, 0.5, 0.5, "bluerov2/br2.png"))
        return false;

    SfRobot robot{};
    robot.name = "bluerov2";
    robot.base_link = "base_link";
    robot.physics = SF_BODY_SUBMERGED;
    robot.buoyant = 1;
    robot.world = PoseAt(0.0, -1.0, 2.0, 0.0, 0.0, 3.14);
    if(!CallOk(sf_robot_begin(world, &robot), "bluerov2 robot"))
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
        SfPose origin;
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

    SfSensor odometry{};
    odometry.name = "odometry";
    odometry.link = "base_link";
    odometry.origin = identity;
    odometry.rate = 100.0;
    odometry.history = -1;
    SfImu imu{};
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
    SfCamera left{};
    left.sensor.name = "camera_left";
    left.sensor.link = "base_link";
    left.sensor.origin = YawQuarter(PoseAt(0.16, -0.0725, 0.15, 1.571, 0.0, 1.571));
    left.sensor.rate = 30.0;
    left.sensor.history = -1;
    left.resolution_x = 640;
    left.resolution_y = 480;
    left.horizontal_fov_deg = 75.0;
    SfCamera right{};
    right.sensor.name = "camera_right";
    right.sensor.link = "base_link";
    right.sensor.origin = YawQuarter(PoseAt(0.16, 0.0725, 0.15, 1.571, 0.0, 1.571));
    right.sensor.rate = 30.0;
    right.sensor.history = -1;
    right.resolution_x = 640;
    right.resolution_y = 480;
    right.horizontal_fov_deg = 75.0;
    return CallOk(sf_robot_odometry(world, &odometry), "bluerov2 odometry")
           && CallOk(sf_robot_imu(world, &imu), "bluerov2 imu")
           && CallOk(sf_robot_camera(world, &left), "bluerov2 left camera")
           && CallOk(sf_robot_camera(world, &right), "bluerov2 right camera")
           && CallOk(sf_robot_end(world), "bluerov2 robot end");
}

bool BuildGirona500Robot(SfWorld* world)
{
    const SfPose identity = PoseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    if(!AddMaterial(world, "Fiberglass", 1800.0, 0.3)
       || !AddMaterial(world, "Neutral", 1000.0, 0.5)
       || !AddMaterial(world, "Aluminium", 2700.0, 0.8))
        return false;

    SfRobot robot{};
    robot.name = "Girona500AUV";
    robot.base_link = "Vehicle";
    robot.physics = SF_BODY_SUBMERGED;
    robot.buoyant = 1;
    robot.world = PoseAt(0.0, -1.0, 2.0, 0.0, 0.0, -1.5708);
    if(!CallOk(sf_robot_begin(world, &robot), "girona500 robot"))
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
    const SfPose mounts[] = {
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

    SfSensor odometry{};
    odometry.name = "Odometry";
    odometry.link = "Vehicle";
    odometry.origin = identity;
    odometry.rate = 30.0;
    odometry.history = -1;
    SfCamera camera{};
    camera.sensor.name = "Camera";
    camera.sensor.link = "Vehicle";
    camera.sensor.origin = PoseAt(0.5, -0.12, 0.5, 0.0, 0.0, 1.571);
    camera.sensor.rate = 10.0;
    camera.sensor.history = -1;
    camera.resolution_x = 800;
    camera.resolution_y = 600;
    camera.horizontal_fov_deg = 60.0;
    return CallOk(sf_robot_odometry(world, &odometry), "girona500 odometry")
           && CallOk(sf_robot_camera(world, &camera), "girona500 camera")
           && CallOk(sf_robot_end(world), "girona500 robot end");
}

bool BuildRobot(SfWorld* world, const std::string& robot)
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
