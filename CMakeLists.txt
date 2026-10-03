cmake_minimum_required(VERSION 3.20)

project(autonomy LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(ExternalProject)

find_package(Python3 COMPONENTS Development REQUIRED)

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
set(STONEFISH_INCLUDE_ROOT "${STONEFISH_DIR}/include")
set(STONEFISH_INCLUDE_DIR "${STONEFISH_INCLUDE_ROOT}/${STONEFISH_LIB}")

ExternalProject_Add(stonefish
  SOURCE_DIR "${STONEFISH_SOURCE_DIR}"
  BINARY_DIR "${STONEFISH_BUILD_DIR}"

  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${STONEFISH_DIR}
    -DCMAKE_BUILD_TYPE=Release
    ${STONEFISH_EXTRA_ARGS}

  BUILD_ALWAYS ${AUTONOMY_STONEFISH_RTX}

  BUILD_BYPRODUCTS
    "${STONEFISH_DIR}/lib/lib${STONEFISH_LIB}.so"
)

add_library(stonefish_c STATIC stonefish_c/src/stonefish_c.cpp)
add_dependencies(stonefish_c stonefish)
set_target_properties(stonefish_c PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(stonefish_c PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/stonefish_c/include"
  PRIVATE
    "${STONEFISH_INCLUDE_ROOT}"
    "${STONEFISH_INCLUDE_DIR}"
)
target_compile_definitions(stonefish_c PRIVATE
  BT_EULER_DEFAULT_ZYX
  BT_USE_DOUBLE_PRECISION
  ${STONEFISH_EXTRA_DEFS}
)

include(CheckIncludeFileCXX)
set(CMAKE_REQUIRED_FLAGS "-std=c++23")
check_include_file_cxx(print AUTONOMY_HAS_STD_PRINT)
unset(CMAKE_REQUIRED_FLAGS)

function(add_auv_sim target platform_config)
  add_library(${target} SHARED auvs/hydrus_sim/auv.cpp)
  add_dependencies(${target} stonefish)

  target_include_directories(${target} PRIVATE
    "${STONEFISH_INCLUDE_ROOT}"
    "${STONEFISH_INCLUDE_DIR}"
    ${Python3_INCLUDE_DIRS}
  )
  # GCC 13 and earlier ship C++23 without <print>. auvs/hydrus_sim/compat fills that gap.
  if(NOT AUTONOMY_HAS_STD_PRINT)
    target_include_directories(${target} BEFORE PRIVATE
      "${CMAKE_SOURCE_DIR}/auvs/hydrus_sim/compat"
    )
  endif()

  target_compile_definitions(${target} PRIVATE
    _GNU_SOURCE
    BT_EULER_DEFAULT_ZYX
    BT_USE_DOUBLE_PRECISION
    ${STONEFISH_EXTRA_DEFS}
    SHADER_DIR_PATH="${STONEFISH_DIR}/share/${STONEFISH_LIB}/shaders/"
    AUV_PLATFORM_CONFIG="${platform_config}"
    BLENDER_STONEFISH_ROOT="${CMAKE_SOURCE_DIR}"
    BLENDER_STONEFISH_CONFIG="${CMAKE_SOURCE_DIR}/blender_stonefish/config.yaml"
  )

  target_compile_options(${target} PRIVATE
    -fopenmp
  )

  target_link_directories(${target} PRIVATE
    "${STONEFISH_DIR}/lib"
  )

  target_link_libraries(${target} PRIVATE
    stonefish_c
    ${STONEFISH_LIB}
    SDL2
    freetype
    OpenGL
    GLX
    GLU
    gomp
    Python3::Python
  )

  set_target_properties(${target} PROPERTIES
    BUILD_RPATH "${STONEFISH_DIR}/lib"
  )

  install(TARGETS ${target})
endfunction()

add_auv_sim(auv_hydrus_sim "auvs/hydrus_sim/platform.json")

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
  if(EXISTS "${MUJOCO_INCLUDE_DIR}/GLFW/glfw3.h")
    set(GLFW_INCLUDE_DIR "${MUJOCO_INCLUDE_DIR}")
  elseif(EXISTS "${MUJOCO_PREFIX}/build/_deps/glfw3-src/include/GLFW/glfw3.h")
    set(GLFW_INCLUDE_DIR "${MUJOCO_PREFIX}/build/_deps/glfw3-src/include")
  else()
    set(GLFW_INCLUDE_DIR "/usr/include")
  endif()

  add_library(auv_hydrus_mujoco SHARED
    auvs/hydrus_mujoco/auv.cpp
    auvs/hydrus_mujoco/hydrus_core.cpp
  )
  target_include_directories(auv_hydrus_mujoco PRIVATE
    "${CMAKE_SOURCE_DIR}/include"
    "${CMAKE_SOURCE_DIR}/auvs/hydrus_mujoco"
    "${MUJOCO_INCLUDE_DIR}"
    "${GLFW_INCLUDE_DIR}"
  )
  target_link_directories(auv_hydrus_mujoco PRIVATE "${MUJOCO_LIB_DIR}")
  target_link_libraries(auv_hydrus_mujoco PRIVATE mujoco)
  if(EXISTS "${MUJOCO_LIB_DIR}/libglfw.so" OR
      EXISTS "/usr/lib/x86_64-linux-gnu/libglfw.so" OR
      EXISTS "/usr/lib/libglfw.so")
    target_link_libraries(auv_hydrus_mujoco PRIVATE glfw)
  elseif(EXISTS "${MUJOCO_LIB_DIR}/libglfw3.a")
    target_link_libraries(auv_hydrus_mujoco PRIVATE
      "${MUJOCO_LIB_DIR}/libglfw3.a"
      X11
      Xrandr
      Xi
      Xinerama
      Xcursor
      GL
      dl
      m
    )
  else()
    target_link_libraries(auv_hydrus_mujoco PRIVATE glfw)
  endif()
  target_link_libraries(auv_hydrus_mujoco PRIVATE pthread)
  set_target_properties(auv_hydrus_mujoco PROPERTIES
    BUILD_RPATH "${MUJOCO_LIB_DIR}"
  )

  add_library(hydrus_mujoco_rl SHARED
    auvs/hydrus_mujoco/hydrus_batch.cpp
    auvs/hydrus_mujoco/hydrus_camera.cpp
    auvs/hydrus_mujoco/hydrus_core.cpp
  )
  target_include_directories(hydrus_mujoco_rl PRIVATE
    "${CMAKE_SOURCE_DIR}/auvs/hydrus_mujoco"
    "${MUJOCO_INCLUDE_DIR}"
  )
  target_link_directories(hydrus_mujoco_rl PRIVATE "${MUJOCO_LIB_DIR}")
  target_link_libraries(hydrus_mujoco_rl PRIVATE mujoco pthread)
  set_target_properties(hydrus_mujoco_rl PROPERTIES
    BUILD_RPATH "${MUJOCO_LIB_DIR}"
  )
endif()
