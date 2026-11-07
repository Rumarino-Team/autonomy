#ifndef AUTONOMY_SIMULATION_TYPES_H
#define AUTONOMY_SIMULATION_TYPES_H

#include "arena_allocator.h"
#include "math.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SIM_MODEL_ABI_VERSION 5u
#define SIM_MAX_THRUSTERS 8u
#define SIM_MAX_ROBOT_COLLIDERS 5u
#define SIM_MAX_HULLS 5u
#define SIM_MAX_SCENE_BYTES (128u * 1024u * 1024u)

typedef int32_t SimResult;

enum {
    SIM_OK,
    SIM_INVALID,
    SIM_UNSUPPORTED,
    SIM_BUFFER_TOO_SMALL,
    SIM_BACKEND_ERROR,
    SIM_NO_MEMORY,
    SIM_CAPACITY
};

enum {
    SIM_STATIC,
    SIM_DYNAMIC,
    SIM_KINEMATIC
};

enum {
    SIM_BOX,
    SIM_SPHERE,
    SIM_CYLINDER,
    SIM_MESH
};

enum {
    SIM_INTEGRATOR_DEFAULT,
    SIM_INTEGRATOR_EULER,
    SIM_INTEGRATOR_IMPLICIT,
    SIM_INTEGRATOR_IMPLICITFAST,
    SIM_INTEGRATOR_RK4
};

enum {
    SIM_CAP_THREADS = 1u,
    SIM_CAP_PROCESS_ISOLATED = 2u,
    SIM_CAP_NATIVE_HYDRO = 4u
};

typedef uint32_t DataKind;

enum {
    SIM_DATA_OBJECT,
    SIM_DATA_FAULT,
    SIM_DATA_MODULE,
    SIM_DATA_MUJOCO_BODY,
    SIM_DATA_MUJOCO_SHAPE,
    SIM_DATA_STONEFISH_SOLID,
    SIM_DATA_STONEFISH_RIGID
};

typedef uint32_t FaultKind;

enum {
    SIM_FAULT_TRACKING,
    SIM_FAULT_IMU,
    SIM_FAULT_GYRO,
    SIM_FAULT_DEPTH,
    SIM_FAULT_DEAD,
    SIM_FAULT_WEAK,
    SIM_FAULT_CAMERA
};

typedef struct Link {
    struct Link *next;
    struct Link *prev;
} Link;

typedef struct ListBase {
    Link *first;
    Link *last;
} ListBase;

typedef struct IntrusiveDataNode {
    Link link;
    DataKind kind;
    uint32_t uid;
    const void *owner;
    ListBase *list;
    uint32_t index;
    uint32_t pooled;
} IntrusiveDataNode;

typedef struct SimTriangle {
    uint32_t a;
    uint32_t b;
    uint32_t c;
} SimTriangle;

typedef struct MeshData {
    const MathVector3f *positions;
    uint32_t position_count;
    const SimTriangle *triangles;
    uint32_t triangle_count;
} MeshData;

typedef struct ShapeRec {
    uint32_t kind;
    uint32_t collisions;
    MathPose pose; /* Local to the rigid body origin. */
    /* Box half extents; sphere radius x; cylinder radius x/half-height z. */
    MathVector3f size;
    MathVector3f drag;
    MathVector3f skin;
    double friction;
    double restitution;
} ShapeRec;

typedef struct UnderwaterBody {
    uint32_t id;
    char name[64];
    uint32_t cls;
    uint32_t motion_type;
    uint32_t tracked;
    uint32_t fuzz;
    double mass;
    MathVector3f inertia;
    MathPose center_of_mass;
    MathPose initial_pose;
    /* World-frame COM velocity. */
    MathVector3f initial_velocity;
    MathVector3f initial_angular_velocity;
    double dry_mass;
    double volume;
    MathVector3f buoyancy_center;
} UnderwaterBody;

struct ObjectBlock;

typedef struct Collider {
    ShapeRec data;
    MeshData geometry;
    const struct ObjectBlock *object;
} Collider;

typedef struct ObjectBlock {
    IntrusiveDataNode id; /* Must remain first: Link -> node -> typed record. */
    UnderwaterBody body;
    const Collider *colliders;
    uint32_t collider_count;
} ObjectBlock;

typedef struct RotorSpec {
    double max_rpm;
    double diameter;
    double thrust;
    double torque;
    double inertia;
    double kp;
    double ki;
    double command_sign;
} RotorSpec;

typedef struct Thruster {
    const ObjectBlock *object;
    MathVector3f position;
    MathVector3f direction;
    RotorSpec spec;
} Thruster;

typedef struct HullRec {
    const ObjectBlock *object;
    MathPose pose;
    const MathVector3f *positions; /* Triangle soup, three positions per face. */
    uint32_t position_count;
    MathVector3f drag;
    MathVector3f skin;
    MathVector3f centroid;
    MathQuaternionf coefficient_frame;
    double volume;
} HullRec;

/* Sensor records are plain immutable scene data, never intrusive datablocks.
 * Pose/IMU mounts map sensor axes into body axes; camera +Z is optical forward. */
typedef uint32_t SensorKind;

enum {
    SIM_SENSOR_POSE,
    SIM_SENSOR_IMU,
    SIM_SENSOR_PRESSURE,
    SIM_SENSOR_CAMERA
};

typedef struct CameraSettings {
    uint32_t width, height;
    double horizontal_fov, near_range, far_range;
} CameraSettings;

typedef struct SensorRec {
    uint32_t id;
    SensorKind kind;
    const ObjectBlock *object;
    MathPose pose;
    uint32_t enabled, period_ticks;

    union {
        struct {
            uint32_t position_axes, attitude_axes;
        } tracking;

        CameraSettings camera;
    } settings;
} SensorRec;

typedef struct SensorRouting {
    const SensorRec *tracking, *imu, *pressure, *camera;
} SensorRouting;

typedef struct FaultBlock {
    IntrusiveDataNode id;
    FaultKind kind;
    uint32_t thruster;
    const SensorRec *sensor; /* Required for sensor faults; NULL for actuator faults. */
    uint64_t start;
    uint64_t end;
    double magnitude;
} FaultBlock;

typedef struct ModuleBlock {
    IntrusiveDataNode id;
    char name[64];
} ModuleBlock;

typedef struct Robot {
    char name[64];
    char asset_file[64];
    uint8_t asset_hash[32];
    ObjectBlock object;
    Collider colliders[SIM_MAX_ROBOT_COLLIDERS];
    Thruster thrusters[SIM_MAX_THRUSTERS];
    HullRec hulls[SIM_MAX_HULLS];
    uint32_t registered;
    uint32_t thruster_count;
    uint32_t hull_count;
    const SensorRec *sensors;
    uint32_t sensor_count;
} Robot;

typedef struct BoundsRec {
    double position;
    double speed;
    double angular_speed;
} BoundsRec;

typedef struct SimulationSettings {
    uint32_t dt_numerator;
    uint32_t dt_denominator;
    uint32_t integrator;
    uint32_t native_hydrodynamics;
    uint32_t hydro_period;
    MathVector3f gravity;
    MathVector3f current;
    double water_density;
    BoundsRec bounds;
} SimulationSettings;

/* Callbacks must be paired, thread-safe across independent worlds, and return
 * aligned memory or NULL. */
typedef struct SimAllocator {
    void *user;
    void *(*allocate)(void *user, size_t bytes, size_t alignment);
    void (*deallocate)(void *user, void *memory, size_t bytes, size_t alignment);
} SimAllocator;

typedef struct SimulationConfig {
    uint32_t object_capacity;
    uint32_t fault_capacity;
    uint32_t module_capacity;
    uint32_t collider_capacity;
    uint32_t vertex_capacity;
    uint32_t triangle_capacity;
    uint32_t hull_vertex_capacity;
    uint32_t sensor_capacity;
    size_t scratch_bytes;
    SimAllocator allocator;
} SimulationConfig;
typedef struct SimulationStorage SimulationStorage;

typedef struct Simulation {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t initialized;
    uint32_t frozen;
    SimulationSettings settings;
    Robot robot;
    const SensorRec *sensors;
    uint32_t sensor_count;
    SensorRouting routing;
    ListBase objects;
    ListBase faults;
    ListBase modules;
    const ObjectBlock *const *object_table;
    const FaultBlock *const *fault_table;
    const ModuleBlock *const *module_table;
    uint32_t object_count;
    uint32_t fault_count;
    uint32_t module_count;
    const MeshData *geometry;
    uint32_t geometry_count;
    ArenaAllocator vertices;
    ArenaAllocator triangles;
    ArenaAllocator hull_vertices;
    PoolAllocator object_pool;
    PoolAllocator fault_pool;
    PoolAllocator module_pool;
    /* Allocation ownership only; never interpreted by a backend. */
    SimulationStorage *storage;
} Simulation;

typedef struct SimBodyState {
    uint32_t id;
    MathPose pose;
    MathVector3f center_of_mass;
    MathVector3f linear_velocity;
    MathVector3f angular_velocity;
} SimBodyState;

typedef struct SimWrench {
    uint32_t id;
    MathVector3f force;
    MathVector3f torque;
} SimWrench;

typedef struct SimContact {
    uint32_t a;
    uint32_t b;
    MathVector3f position;
    MathVector3f normal;
    double normal_force;
} SimContact;

typedef struct RotorState {
    double omega;
    double integral;
    double torque;
    double thrust;
} RotorState;

typedef struct ObjectObservation {
    uint32_t id;
    uint32_t cls;
    MathPose pose;
    MathVector3f size;
    uint32_t x0;
    uint32_t y0;
    uint32_t x1;
    uint32_t y1;
    uint32_t visible;
} ObjectObservation;

/* Sample validity bits refer to payload channels, not body routing. */
enum {
    SIM_SAMPLE_POSE = 1u,
    SIM_SAMPLE_IMU = 2u,
    SIM_SAMPLE_DEPTH = 4u,
    SIM_SAMPLE_CAMERA = 8u
};

typedef struct SensorSample {
    uint32_t sensor_id, kind;
    uint64_t tick;
    uint32_t valid, position_axes, attitude_axes;

    union {
        MathPose pose;

        struct {
            MathQuaternionf orientation;
            MathVector3f gyro, accel;
        } imu;

        double depth;

        struct {
            MathPose pose;
            const ObjectObservation *objects;
            uint32_t count;
        } camera;
    } value;
} SensorSample;

typedef struct Observation {
    MathPose body; /* Routed body-origin estimate; camera remains optical pose. */
    uint32_t position_axes, attitude_axes; /* X/Y/Z or roll/pitch/yaw bit masks. */
    uint32_t image_width, image_height;
    MathPose camera;
    MathQuaternionf imu;
    MathVector3f gyro;
    MathVector3f accel;
    double depth;
    uint32_t position_ok;
    uint32_t attitude_ok;
    uint32_t gyro_ok;
    uint32_t imu_ok;
    uint32_t depth_ok;
    const ObjectObservation *objects;
    uint32_t object_count;
} Observation;

#ifdef __cplusplus
}
#endif

#endif
