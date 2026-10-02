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

function(add_auv_sim target platform_config)
  add_library(${target} SHARED auvs/hydrus_sim/auv.cpp)
  add_dependencies(${target} stonefish)

  target_include_directories(${target} PRIVATE
    "${STONEFISH_DIR}/include"
    "${STONEFISH_DIR}/include/Stonefish"
  )

  target_compile_definitions(${target} PRIVATE
    _GNU_SOURCE
    BT_EULER_DEFAULT_ZYX
    BT_USE_DOUBLE_PRECISION
    SHADER_DIR_PATH="${STONEFISH_DIR}/share/Stonefish/shaders/"
    AUV_PLATFORM_CONFIG="${platform_config}"
  )

  target_compile_options(${target} PRIVATE
    -fopenmp
  )

  target_link_directories(${target} PRIVATE
    "${STONEFISH_DIR}/lib"
  )

  target_link_libraries(${target} PRIVATE
    Stonefish
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

add_auv_sim(auv_hydrus_sim "auvs/hydrus_sim/platform.json")
add_auv_sim(auv_proteus_sim "auvs/proteus_sim/platform.json")
add_auv_sim(auv_bluerov2_sim "auvs/bluerov2_sim/platform.json")
add_auv_sim(auv_girona500_sim "auvs/girona500_sim/platform.json")
