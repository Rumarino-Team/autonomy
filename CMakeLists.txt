cmake_minimum_required(VERSION 3.20)

project(autonomy LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(ExternalProject)

set(STONEFISH_DIR "${CMAKE_BINARY_DIR}/stonefish")
set(STONEFISH_BUILD_DIR "${CMAKE_BINARY_DIR}/stonefish-build")

ExternalProject_Add(stonefish
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor/stonefish"
  BINARY_DIR "${STONEFISH_BUILD_DIR}"

  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${STONEFISH_DIR}
    -DCMAKE_BUILD_TYPE=Release

  BUILD_BYPRODUCTS
    "${STONEFISH_DIR}/lib/libStonefish.so"
)

add_library(auv_stonefish_sim SHARED
    auvs/stonefish_sim/auv.cpp
)

add_dependencies(auv_stonefish_sim stonefish)

target_include_directories(auv_stonefish_sim PRIVATE
  "${STONEFISH_DIR}/include"
  "${STONEFISH_DIR}/include/Stonefish"
)

target_compile_definitions(auv_stonefish_sim PRIVATE
  _GNU_SOURCE
  BT_EULER_DEFAULT_ZYX
  BT_USE_DOUBLE_PRECISION
  SHADER_DIR_PATH="${STONEFISH_DIR}/share/Stonefish/shaders/"
)

target_compile_options(auv_stonefish_sim PRIVATE
  -fopenmp
)

target_link_directories(auv_stonefish_sim PRIVATE
  "${STONEFISH_DIR}/lib"
)

target_link_libraries(auv_stonefish_sim PRIVATE
  Stonefish
  SDL2
  freetype
  OpenGL
  GLX
  GLU
  gomp
)

set_target_properties(auv_stonefish_sim PROPERTIES
  BUILD_RPATH "${STONEFISH_DIR}/lib"
)

install(
  TARGETS auv_stonefish_sim
)

set(MUJOCO_PREFIX "" CACHE PATH "MuJoCo install prefix or source tree")

if(MUJOCO_PREFIX)
  if(EXISTS "${MUJOCO_PREFIX}/lib/libmujoco.so")
    set(MUJOCO_LIB_DIR "${MUJOCO_PREFIX}/lib")
  elseif(EXISTS "${MUJOCO_PREFIX}/build/lib/libmujoco.so")
    set(MUJOCO_LIB_DIR "${MUJOCO_PREFIX}/build/lib")
  else()
    message(FATAL_ERROR
      "libmujoco.so not found under ${MUJOCO_PREFIX}/lib or ${MUJOCO_PREFIX}/build/lib")
  endif()
  set(MUJOCO_INCLUDE_DIR "${MUJOCO_PREFIX}/include")
  add_subdirectory(auvs/blender_scene_cpp)
  add_library(autonomy_mujoco_compose STATIC
    auvs/hydrus_mujoco/mujoco_scene_builder.cpp
    auvs/hydrus_mujoco/mujoco_platform.cpp
  )
  set_target_properties(autonomy_mujoco_compose PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_include_directories(autonomy_mujoco_compose PUBLIC
    "${CMAKE_SOURCE_DIR}/auvs/hydrus_mujoco"
    "${CMAKE_SOURCE_DIR}/auvs/blender_scene_cpp"
    "${MUJOCO_INCLUDE_DIR}"
    "${CMAKE_SOURCE_DIR}/vendor/nlohmann_json/single_include"
  )
  target_link_directories(autonomy_mujoco_compose PUBLIC "${MUJOCO_LIB_DIR}")
  target_link_libraries(autonomy_mujoco_compose PUBLIC autonomy_blend_reader mujoco)
  enable_language(C)

  add_library(auv_mujoco SHARED
    auvs/hydrus_mujoco/auv.cpp
    auvs/hydrus_mujoco/auv_core.cpp
    auvs/hydrus_mujoco/auv_batch.cpp
    auvs/hydrus_mujoco/auv_camera.cpp
    auvs/hydrus_mujoco/blend_watcher.cpp
    auvs/hydrus_mujoco/blender_link.cpp
    auvs/hydrus_mujoco/recorder.cpp
    auvs/hydrus_mujoco/hydrus_drag_mesh_data.c
  )
  target_include_directories(auv_mujoco PRIVATE
    "${CMAKE_SOURCE_DIR}/include"
    "${CMAKE_SOURCE_DIR}/auvs/hydrus_mujoco"
    "${MUJOCO_INCLUDE_DIR}"
  )
  target_link_directories(auv_mujoco PRIVATE "${MUJOCO_LIB_DIR}")
  target_link_libraries(auv_mujoco PRIVATE autonomy_mujoco_compose mujoco pthread)
  set_target_properties(auv_mujoco PROPERTIES BUILD_RPATH "${MUJOCO_LIB_DIR}")

  add_library(auv_mujoco_rl SHARED
    auvs/hydrus_mujoco/auv_batch.cpp
    auvs/hydrus_mujoco/auv_camera.cpp
    auvs/hydrus_mujoco/auv_core.cpp
    auvs/hydrus_mujoco/hydrus_drag_mesh_data.c
  )
  target_include_directories(auv_mujoco_rl PRIVATE
    "${CMAKE_SOURCE_DIR}/auvs/hydrus_mujoco"
    "${MUJOCO_INCLUDE_DIR}"
  )
  target_link_directories(auv_mujoco_rl PRIVATE "${MUJOCO_LIB_DIR}")
  target_link_libraries(auv_mujoco_rl PRIVATE autonomy_mujoco_compose mujoco pthread)
  set_target_properties(auv_mujoco_rl PROPERTIES BUILD_RPATH "${MUJOCO_LIB_DIR}")

  add_executable(hydrus_tune_pid auvs/hydrus_mujoco/tune_pid.cpp)
  target_include_directories(hydrus_tune_pid PRIVATE
    "${CMAKE_SOURCE_DIR}/auvs/hydrus_mujoco"
    "${MUJOCO_INCLUDE_DIR}"
    "${CMAKE_SOURCE_DIR}/vendor/nlohmann_json/single_include"
  )
  target_link_directories(hydrus_tune_pid PRIVATE "${MUJOCO_LIB_DIR}")
  target_link_libraries(hydrus_tune_pid PRIVATE auv_mujoco_rl)
  set_target_properties(hydrus_tune_pid PROPERTIES
    BUILD_RPATH "${MUJOCO_LIB_DIR};${CMAKE_BINARY_DIR}"
    INSTALL_RPATH "$ORIGIN/../lib"
  )
  install(TARGETS hydrus_tune_pid)
  add_executable(mujoco_compose_smoke auvs/hydrus_mujoco/mujoco_compose_smoke.cpp)
  target_link_libraries(mujoco_compose_smoke PRIVATE autonomy_mujoco_compose)
endif()
