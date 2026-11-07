#include "sim_backend.h"
#include <dlfcn.h>
#include <stdio.h>
#include <string.h>
_Static_assert(sizeof(SimVec3) == 3 * sizeof(double), "vector layout");
_Static_assert(_Alignof(MathVector3d) == _Alignof(double), "double alignment");
_Static_assert(sizeof(MathVector6d) == 6 * sizeof(double), "spatial vector layout");
_Static_assert(offsetof(MathVector6d, angular) == 3 * sizeof(double),
               "linear then angular components");
_Static_assert(sizeof(MathVector3f) == 16 && _Alignof(MathVector3f) == 16,
               "float vector ABI unchanged");
_Static_assert(sizeof(MathVector6f) == 32 && _Alignof(MathVector6f) == 32,
               "float spatial vector ABI unchanged");
_Static_assert(sizeof(SimQuat) == 4 * sizeof(double), "quaternion layout");
_Static_assert(offsetof(SimQuat, w) == 3 * sizeof(double), "xyzw quaternion");
_Static_assert(sizeof(SimTriangle) == 3 * sizeof(uint32_t), "triangle layout");
_Static_assert(sizeof(SimWrench) == 56 && offsetof(SimWrench, force) == 8 &&
                   offsetof(SimWrench, torque) == 32,
               "backend wrench ABI unchanged");
#define CHECK(x)                                                               \
  do {                                                                         \
    if (!(x)) {                                                                \
      fprintf(stderr, "C ABI failed: %s (%s)\n", #x, error);                   \
      return 1;                                                                \
    }                                                                          \
  } while (0)
int main(int argc, char **argv) {
  if (argc == 1)
    return SIM_BACKEND_ABI_VERSION != 3;
  char error[1024] = {0};
  void *library = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
  CHECK(library);
  void *symbol = dlsym(library, "sim_backend_get_api");
  SimBackendGetApi get;
  _Static_assert(sizeof(get) == sizeof(symbol), "POSIX function pointer size");
  memcpy(&get, &symbol, sizeof(get));
  CHECK(get);
  SimBackendApi api = {0};
  CHECK(get(2, sizeof(api), &api) == SIM_UNSUPPORTED);
  CHECK(get(3, sizeof(api), &api) == SIM_OK);
  SimBodyDesc body = {0};
  body.body.id = 17;
  body.body.motion = SIM_DYNAMIC;
  body.body.mass = 2;
  body.body.inertia = (SimVec3){1, 1, 1};
  body.body.center_of_mass.orientation.w = 1;
  body.body.initial_pose.orientation.w = 1;
  body.body.initial_pose.position.z = 2;
  body.dry_mass = 2;
  body.shape_count = 1;
  strcpy(body.name, "C cube");
  SimShapeDesc shape = {0};
  shape.shape.kind = SIM_BOX;
  shape.shape.pose.orientation.w = 1;
  shape.shape.size = (SimVec3){.1, .1, .1};
  shape.shape.friction = .5;
  shape.mesh = UINT32_MAX;
  SimSceneView view = {0};
  view.struct_size = sizeof(view);
  view.model.struct_size = sizeof(view.model);
  view.model.dt_numerator = 1;
  view.model.dt_denominator = 360;
  view.model.water_density = 1025;
  view.bodies = &body;
  view.body_count = 1;
  view.shapes = &shape;
  view.shape_count = 1;
  void *world = NULL;
  CHECK(api.world_create(&view, &world, error, sizeof(error)) == SIM_OK);
  /* The adapter must have copied both descriptor arrays before returning. */
  memset(&body, 0, sizeof(body));
  memset(&shape, 0, sizeof(shape));
  memset(&view, 0, sizeof(view));
  SimWrench force = {17, {1, 0, 0}, {0, 0, 0}};
  CHECK(api.step(world, &force, 1, error, sizeof(error)) == SIM_OK);
  uint32_t count = 0;
  SimBodyState state;
  CHECK(api.read_state(world, &state, 1, &count) == SIM_OK && count == 1 &&
        state.id == 17);
  CHECK(state.linear_velocity.x > 1. / 720. - 1e-10 &&
        state.linear_velocity.x < 1. / 720. + 1e-10);
  CHECK(api.reset(world, NULL, 0, error, sizeof(error)) == SIM_OK);
  CHECK(api.read_state(world, &state, 1, &count) == SIM_OK &&
        state.pose.position.z == 2);
  api.world_destroy(world);
  dlclose(library);
  return 0;
}
