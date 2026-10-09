cmake_minimum_required(VERSION 3.20)

project(autonomy LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(ExternalProject)

add_subdirectory(auvs/stonefish_sim/blender_stonefish_cpp)
option(AUTONOMY_BLEND_CPP "Also build the alternate native Blender target" OFF)

# OFF builds the upstream vendor/stonefish submodule; ON builds the vendor/stonefish_rtx
# fork with the OptiX path tracer and DLSS.
option(AUTONOMY_STONEFISH_RTX "Build against vendor/stonefish_rtx (StonefishRTX)" OFF)
# Empty uses the toolkit's nvcc. Set an older one when the toolkit is newer than the driver.
set(AUTONOMY_STONEFISH_RTX_NVCC "" CACHE FILEPATH "nvcc for the StonefishRTX OptiX programs")

if(AUTONOMY_STONEFISH_RTX)
  set(STONEFISH_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor/stonefish_rtx")
  set(STONEFISH_DIR "${CMAKE_BINARY_DIR}/stonefish_rtx")
  set(STONEFISH_BUILD_DIR "${CMAKE_BINARY_DIR}/stonefish_rtx-build")
  set(STONEFISH_LIB StonefishRTX)
  set(STONEFISH_EXTRA_ARGS -DSTONEFISH_RTX=ON -DSTONEFISH_DLSS=ON)
  if(AUTONOMY_STONEFISH_RTX_NVCC)
    list(APPEND STONEFISH_EXTRA_ARGS -DSTONEFISH_RTX_PROGRAMS_NVCC=${AUTONOMY_STONEFISH_RTX_NVCC})
  endif()
  set(STONEFISH_EXTRA_DEFS STONEFISH_HAS_RTX STONEFISH_HAS_DLSS)
else()
  set(STONEFISH_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor/stonefish")
  set(STONEFISH_DIR "${CMAKE_BINARY_DIR}/stonefish")
  set(STONEFISH_BUILD_DIR "${CMAKE_BINARY_DIR}/stonefish-build")
  set(STONEFISH_LIB Stonefish)
  set(STONEFISH_EXTRA_ARGS)
  set(STONEFISH_EXTRA_DEFS)
endif()
# Installed headers live in include/Stonefish/. The wrappers include them as
# "Stonefish/...", so include/ must be on the path or those includes fall
# through to a system Stonefish under /usr/local/include. Stonefish's own
# headers include siblings as "StonefishCommon.h" and <version.h>, which
# need include/Stonefish as well.
set(AUTONOMY_STONEFISH_PREFIX "" CACHE PATH "Reuse an existing Stonefish install instead of rebuilding it")
if(AUTONOMY_STONEFISH_PREFIX)
  set(STONEFISH_DIR "${AUTONOMY_STONEFISH_PREFIX}")
  if(NOT EXISTS "${STONEFISH_DIR}/lib/lib${STONEFISH_LIB}.so")
    message(FATAL_ERROR "Requested Stonefish library not found under ${STONEFISH_DIR}/lib")
  endif()
endif()
set(STONEFISH_INCLUDE_ROOT "${STONEFISH_DIR}/include")
set(STONEFISH_INCLUDE_DIR "${STONEFISH_INCLUDE_ROOT}/${STONEFISH_LIB}")

if(AUTONOMY_STONEFISH_PREFIX)
  add_custom_target(stonefish)
else()
ExternalProject_Add(stonefish
  SOURCE_DIR "${STONEFISH_SOURCE_DIR}"
  BINARY_DIR "${STONEFISH_BUILD_DIR}"

  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${STONEFISH_DIR}
    -DCMAKE_BUILD_TYPE=Release
    ${STONEFISH_EXTRA_ARGS}

  # Local engine edits must rebuild in both raster and RTX configurations.
  BUILD_ALWAYS TRUE

  BUILD_BYPRODUCTS
    "${STONEFISH_DIR}/lib/lib${STONEFISH_LIB}.so"
)
endif()

include(CheckIncludeFileCXX)
set(CMAKE_REQUIRED_FLAGS "-std=c++23")
check_include_file_cxx(print AUTONOMY_HAS_STD_PRINT)
unset(CMAKE_REQUIRED_FLAGS)

function(add_auv_sim target platform_config)
  add_library(${target} SHARED auvs/stonefish_sim/auv.cpp auvs/stonefish_sim/robot.cpp)
  target_sources(${target} PRIVATE auvs/stonefish_sim/blender_stonefish_cpp/build_scene.cpp)
  target_link_libraries(${target} PRIVATE stonefish_blend_reader)
  add_dependencies(${target} stonefish)

  target_include_directories(${target} PRIVATE
    "${STONEFISH_INCLUDE_ROOT}"
    "${STONEFISH_INCLUDE_DIR}"
  )
  # GCC 13 and earlier ship C++23 without <print>. auvs/stonefish_sim/compat fills that gap.
  if(NOT AUTONOMY_HAS_STD_PRINT)
    target_include_directories(${target} BEFORE PRIVATE
      "${CMAKE_SOURCE_DIR}/auvs/stonefish_sim/compat"
    )
  endif()

  target_compile_definitions(${target} PRIVATE
    _GNU_SOURCE
    BT_EULER_DEFAULT_ZYX
    BT_USE_DOUBLE_PRECISION
    ${STONEFISH_EXTRA_DEFS}
    SHADER_DIR_PATH="${STONEFISH_DIR}/share/${STONEFISH_LIB}/shaders/"
    AUV_PLATFORM_CONFIG="${platform_config}"
    BLENDER_STONEFISH_CONFIG="${CMAKE_SOURCE_DIR}/auvs/stonefish_sim/stonefish_config.yaml"
  )

  target_compile_options(${target} PRIVATE
    -fopenmp
  )

  target_link_directories(${target} PRIVATE
    "${STONEFISH_DIR}/lib"
  )

  target_link_libraries(${target} PRIVATE
    ${STONEFISH_LIB}
    SDL2
    freetype
    OpenGL
    GLX
    GLU
    gomp
  )

  set_target_properties(${target} PROPERTIES
    BUILD_RPATH "${STONEFISH_DIR}/lib"
  )

  install(TARGETS ${target})
endfunction()

add_auv_sim(auv_stonefish_sim "auvs/stonefish_sim/platform.json")
if(AUTONOMY_BLEND_CPP)
  add_auv_sim(auv_stonefish_cpp_sim "auvs/stonefish_sim/platform.json")
  add_executable(stonefish_blend_smoke
    auvs/stonefish_sim/blender_stonefish_cpp/smoke.cpp
    auvs/stonefish_sim/blender_stonefish_cpp/build_scene.cpp
    auvs/stonefish_sim/robot.cpp
  )
  add_dependencies(stonefish_blend_smoke stonefish)
  target_include_directories(stonefish_blend_smoke PRIVATE "${STONEFISH_INCLUDE_ROOT}" "${STONEFISH_INCLUDE_DIR}")
  target_compile_definitions(stonefish_blend_smoke PRIVATE BT_EULER_DEFAULT_ZYX BT_USE_DOUBLE_PRECISION
    SHADER_DIR_PATH="${STONEFISH_DIR}/share/${STONEFISH_LIB}/shaders/" ${STONEFISH_EXTRA_DEFS})
  target_link_directories(stonefish_blend_smoke PRIVATE "${STONEFISH_DIR}/lib")
  target_link_libraries(stonefish_blend_smoke PRIVATE stonefish_blend_reader ${STONEFISH_LIB} SDL2 freetype OpenGL GLX GLU gomp)
  set_target_properties(stonefish_blend_smoke PROPERTIES BUILD_RPATH "${STONEFISH_DIR}/lib")
  add_executable(stonefish_blend_plugin_smoke auvs/stonefish_sim/blender_stonefish_cpp/tests/plugin_smoke.cpp)
  target_include_directories(stonefish_blend_plugin_smoke PRIVATE "${CMAKE_SOURCE_DIR}/include")
  target_link_libraries(stonefish_blend_plugin_smoke PRIVATE dl)
endif()

# MuJoCo prefix is an install tree (include/ and lib/libmujoco.so) or a source
# tree whose library lives at build/lib/libmujoco.so. Unset, the Stonefish
# targets still configure.
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

  # Physics plugin. It does not link Stonefish: both libraries export TinyXML.
  # The window is libauv_mujoco_view.so, loaded with RTLD_DEEPBIND.
  add_library(auv_mujoco SHARED
    auvs/hydrus_mujoco/auv.cpp
    auvs/hydrus_mujoco/auv_core.cpp
  )
  target_include_directories(auv_mujoco PRIVATE
    "${CMAKE_SOURCE_DIR}/include"
    "${CMAKE_SOURCE_DIR}/auvs/hydrus_mujoco"
    "${MUJOCO_INCLUDE_DIR}"
  )
  target_link_directories(auv_mujoco PRIVATE "${MUJOCO_LIB_DIR}")
  target_link_libraries(auv_mujoco PRIVATE mujoco dl pthread)
  set_target_properties(auv_mujoco PROPERTIES
    BUILD_RPATH "${MUJOCO_LIB_DIR}"
  )

  add_library(auv_mujoco_view SHARED
    auvs/hydrus_mujoco/auv_view.cpp
  )
  add_dependencies(auv_mujoco_view stonefish)
  target_include_directories(auv_mujoco_view PRIVATE
    "${CMAKE_SOURCE_DIR}/auvs/hydrus_mujoco"
    "${STONEFISH_INCLUDE_ROOT}"
    "${STONEFISH_INCLUDE_DIR}"
  )
  target_compile_definitions(auv_mujoco_view PRIVATE
    _GNU_SOURCE
    BT_EULER_DEFAULT_ZYX
    BT_USE_DOUBLE_PRECISION
    ${STONEFISH_EXTRA_DEFS}
    SHADER_DIR_PATH="${STONEFISH_DIR}/share/${STONEFISH_LIB}/shaders/"
  )
  target_compile_options(auv_mujoco_view PRIVATE -fopenmp)
  target_link_directories(auv_mujoco_view PRIVATE "${STONEFISH_DIR}/lib")
  target_link_libraries(auv_mujoco_view PRIVATE
    ${STONEFISH_LIB}
    SDL2
    freetype
    OpenGL
    GLX
    GLU
    gomp
  )
  set_target_properties(auv_mujoco_view PROPERTIES
    BUILD_RPATH "${STONEFISH_DIR}/lib"
  )

  add_library(auv_mujoco_rl SHARED
    auvs/hydrus_mujoco/auv_batch.cpp
    auvs/hydrus_mujoco/auv_camera.cpp
    auvs/hydrus_mujoco/auv_core.cpp
  )
  target_include_directories(auv_mujoco_rl PRIVATE
    "${CMAKE_SOURCE_DIR}/auvs/hydrus_mujoco"
    "${MUJOCO_INCLUDE_DIR}"
  )
  target_link_directories(auv_mujoco_rl PRIVATE "${MUJOCO_LIB_DIR}")
  target_link_libraries(auv_mujoco_rl PRIVATE mujoco pthread)
  set_target_properties(auv_mujoco_rl PROPERTIES
    BUILD_RPATH "${MUJOCO_LIB_DIR}"
  )

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
endif()
