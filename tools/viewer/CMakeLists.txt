cmake_minimum_required(VERSION 3.16)
project(viewer C CXX)

include(FetchContent)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(raylib URL https://github.com/raysan5/raylib/archive/refs/tags/5.5.tar.gz)
FetchContent_MakeAvailable(raylib)

option(VIEWER_ENABLE_ZED "Build camera support with the ZED SDK and CUDA" OFF)
if(VIEWER_ENABLE_ZED)
  find_package(ZED REQUIRED)
  find_package(CUDAToolkit REQUIRED)
endif()

add_executable(viewer main.cpp net.cpp video.cpp)
target_compile_features(viewer PRIVATE cxx_std_17)
target_link_libraries(viewer PRIVATE raylib)

if(VIEWER_ENABLE_ZED)
  target_compile_definitions(viewer PRIVATE VIEWER_HAS_ZED=1)
  target_include_directories(viewer PRIVATE ${ZED_INCLUDE_DIRS})
  target_link_libraries(viewer PRIVATE ${ZED_LIBRARIES} CUDA::cuda_driver CUDA::cudart)
else()
  target_compile_definitions(viewer PRIVATE VIEWER_HAS_ZED=0)
endif()

if(WIN32)
  target_link_libraries(viewer PRIVATE ws2_32)
endif()
