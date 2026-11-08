cmake_minimum_required(VERSION 3.20)

project(autonomy LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(SIMULATOR_ENABLE_STONEFISH "Build the simulator Stonefish backend" OFF)
if(SIMULATOR_ENABLE_STONEFISH)
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
endif()

option(AUTONOMY_BUILD_SIMULATOR "Build the data-oriented C simulator core and backends" OFF)
if(AUTONOMY_BUILD_SIMULATOR)
  enable_testing()
  add_subdirectory(auvs/simulator)
endif()
