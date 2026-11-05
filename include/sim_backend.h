#ifndef AUTONOMY_SIM_BACKEND_H
#define AUTONOMY_SIM_BACKEND_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define SIM_BACKEND_ABI_VERSION 1u
typedef int32_t SimResult;
enum {
  SIM_OK,
  SIM_INVALID,
  SIM_UNSUPPORTED,
  SIM_BUFFER_TOO_SMALL,
  SIM_BACKEND_ERROR
};
enum { SIM_STATIC, SIM_DYNAMIC, SIM_KINEMATIC };
enum { SIM_BOX, SIM_SPHERE, SIM_CYLINDER, SIM_MESH };
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
typedef struct {
  double x, y, z;
} SimVec3;
/* Unit xyzw quaternion, rotates body-local vectors into the parent frame. */
typedef struct {
  double x, y, z, w;
} SimQuat;
typedef struct {
  SimVec3 position;
  SimQuat orientation;
} SimPose;
typedef struct {
  uint32_t a, b, c;
} SimTriangle;
typedef struct {
  uint32_t kind, collisions;
  SimPose pose; /* relative to the body origin */
  SimVec3 size; /* box half extents; sphere radius x; cylinder radius x, half
                   length z */
  const SimVec3 *vertices;
  uint32_t vertex_count;
  const SimTriangle *triangles;
  uint32_t triangle_count;
  double friction, restitution;
  SimVec3 drag,
      skin; /* coefficients for explicitly selected native hydrodynamics */
} SimShape;
typedef struct {
  uint32_t id, motion;
  double mass; /* effective inertial mass; gravity compensation belongs to the
                  host */
  SimVec3 inertia; /* positive principal moments, including configured added
                      inertia */
  SimPose center_of_mass; /* principal inertial frame relative to body origin */
  SimPose initial_pose;
  SimVec3 initial_linear_velocity,
      initial_angular_velocity; /* world frame, at COM */
  const SimShape *shapes;
  uint32_t shape_count;
} SimBody;
typedef struct {
  uint32_t struct_size;
  uint32_t dt_numerator, dt_denominator;
  uint32_t integrator;
  SimVec3 gravity;
  double water_density;
  SimVec3 water_current; /* world frame */
  uint32_t native_hydrodynamics;
  const SimBody *bodies;
  uint32_t body_count;
} SimModelDesc;
typedef struct {
  uint32_t id;
  SimPose pose;
  SimVec3 center_of_mass, linear_velocity,
      angular_velocity; /* world frame, at COM */
} SimBodyState;
typedef struct {
  uint32_t id;
  SimVec3 force, torque;
} SimWrench; /* world frame, at COM */
typedef struct {
  uint32_t a, b;
  SimVec3 position, normal;
  double normal_force;
} SimContact;
typedef struct {
  uint32_t struct_size, abi_version, capabilities;
  uint32_t integrators; /* bitmask: 1u << SIM_INTEGRATOR_* */
  char name[32], version[64];
  SimResult (*model_create)(const SimModelDesc *, void **, char *, uint32_t);
  void (*model_destroy)(void *);
  SimResult (*world_create)(const void *, void **, char *, uint32_t);
  void (*world_destroy)(void *);
  /* Reset is a cold reset, including solver history. NULL uses the model
   * initial state. */
  SimResult (*reset)(void *, const SimBodyState *, uint32_t, char *, uint32_t);
  SimResult (*step)(void *, const SimWrench *, uint32_t, char *, uint32_t);
  SimResult (*read_state)(const void *, SimBodyState *, uint32_t, uint32_t *);
  SimResult (*read_contacts)(const void *, SimContact *, uint32_t, uint32_t *);
  /* Diagnostic digest only. Checkpoints are intentionally not part of ABI v1.
   */
  uint64_t (*state_hash)(const void *);
} SimBackendApi;
/* Returns an independently compiled backend's API. Never throws across this
   boundary. Inputs are borrowed for the duration of a call; model_create
   deep-copies them. Different world handles may be used concurrently. Serialize
   calls on the same world. Keep a model alive until all its worlds are
   destroyed; unload the DSO last. */
typedef SimResult (*SimBackendGetApi)(uint32_t version, uint32_t size,
                                      SimBackendApi *);
SimResult sim_backend_get_api(uint32_t version, uint32_t size, SimBackendApi *);
#ifdef __cplusplus
}
#endif
#endif
