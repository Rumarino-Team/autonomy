include(FetchContent)
find_path(SIM_JSON_INCLUDE nlohmann/json.hpp HINTS "${PROJECT_SOURCE_DIR}/vendor/nlohmann_json/single_include")
if(NOT SIM_JSON_INCLUDE)
  FetchContent_Declare(sim_json GIT_REPOSITORY https://github.com/nlohmann/json.git GIT_TAG v3.12.0 GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(sim_json)
  set(SIM_JSON_INCLUDE "${sim_json_SOURCE_DIR}/single_include")
endif()
add_library(sim_headers INTERFACE)
target_include_directories(sim_headers INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}" "${SIM_JSON_INCLUDE}")
# -iquote keeps the repository's include/math.h out of <cmath>'s system lookup.
target_compile_options(sim_headers INTERFACE "-iquote${PROJECT_SOURCE_DIR}/include" -Wall -Wextra -Wpedantic -ffp-contract=off -fno-fast-math)
add_library(sim_scene STATIC database.cpp scene_codec.cpp image.cpp blend_loader.cpp)
set_target_properties(sim_scene PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_subdirectory(blend)
target_link_libraries(sim_scene PUBLIC sim_headers autonomy_blend_reader)
add_library(autonomy_sim SHARED json_io.cpp forces.cpp runtime.cpp)
target_link_libraries(autonomy_sim PUBLIC sim_scene PRIVATE ${CMAKE_DL_LIBS})
set_target_properties(autonomy_sim PROPERTIES BUILD_RPATH "$ORIGIN" INSTALL_RPATH "$ORIGIN")

set(SIM_MUJOCO_PREFIX "" CACHE PATH "Existing pinned MuJoCo install; empty builds vendor/mujoco")
if(SIM_MUJOCO_PREFIX)
  find_path(SIM_MUJOCO_INCLUDE mujoco/mujoco.h PATHS "${SIM_MUJOCO_PREFIX}/include" NO_DEFAULT_PATH REQUIRED)
  find_library(SIM_MUJOCO_LIBRARY mujoco PATHS "${SIM_MUJOCO_PREFIX}/lib" NO_DEFAULT_PATH REQUIRED)
else()
  set(SIM_MUJOCO_INCLUDE "${CMAKE_BINARY_DIR}/mujoco/include")
  set(SIM_MUJOCO_LIBRARY "${CMAKE_BINARY_DIR}/mujoco/lib/libmujoco.so")
  file(MAKE_DIRECTORY "${SIM_MUJOCO_INCLUDE}")
  ExternalProject_Add(sim_mujoco_vendor SOURCE_DIR "${PROJECT_SOURCE_DIR}/vendor/mujoco"
    BINARY_DIR "${CMAKE_BINARY_DIR}/mujoco-build"
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/mujoco -DCMAKE_BUILD_TYPE=Release
      -DMUJOCO_BUILD_EXAMPLES=OFF -DMUJOCO_BUILD_SIMULATE=OFF -DMUJOCO_BUILD_TESTS=OFF -DMUJOCO_TEST_PYTHON_UTIL=OFF -DBUILD_TESTING=OFF
    BUILD_BYPRODUCTS "${SIM_MUJOCO_LIBRARY}")
endif()
add_library(sim_backend_mujoco SHARED backends/mujoco.cpp)
target_include_directories(sim_backend_mujoco SYSTEM PRIVATE "${SIM_MUJOCO_INCLUDE}")
target_link_libraries(sim_backend_mujoco PRIVATE sim_scene "${SIM_MUJOCO_LIBRARY}")
if(TARGET sim_mujoco_vendor)
  add_dependencies(sim_backend_mujoco sim_mujoco_vendor)
endif()

set(SIM_STONEFISH_PREFIX "" CACHE PATH "Existing Stonefish install; empty builds the existing vendor target")
if(SIM_STONEFISH_PREFIX)
  find_path(SIM_STONEFISH_INCLUDE StonefishCommon.h PATHS "${SIM_STONEFISH_PREFIX}/include/Stonefish" NO_DEFAULT_PATH REQUIRED)
  find_library(SIM_STONEFISH_LIBRARY Stonefish PATHS "${SIM_STONEFISH_PREFIX}/lib" NO_DEFAULT_PATH REQUIRED)
else()
  set(SIM_STONEFISH_INCLUDE "${STONEFISH_DIR}/include/Stonefish")
  set(SIM_STONEFISH_LIBRARY "${STONEFISH_DIR}/lib/libStonefish.so")
  file(MAKE_DIRECTORY "${SIM_STONEFISH_INCLUDE}")
endif()
find_package(SDL2 REQUIRED)
add_executable(sim_stonefish_worker backends/stonefish_worker.cpp backends/wire.cpp)
target_include_directories(sim_stonefish_worker SYSTEM PRIVATE "${SIM_STONEFISH_INCLUDE}" ${SDL2_INCLUDE_DIRS})
target_compile_definitions(sim_stonefish_worker PRIVATE BT_USE_DOUBLE_PRECISION BT_EULER_DEFAULT_ZYX)
target_link_libraries(sim_stonefish_worker PRIVATE sim_scene "${SIM_STONEFISH_LIBRARY}" ${SDL2_LIBRARIES} ${CMAKE_DL_LIBS})
if(NOT SIM_STONEFISH_PREFIX)
  add_dependencies(sim_stonefish_worker stonefish)
endif()
add_library(sim_backend_stonefish SHARED backends/stonefish.cpp backends/wire.cpp)
target_link_libraries(sim_backend_stonefish PRIVATE sim_scene ${CMAKE_DL_LIBS})
target_compile_definitions(sim_backend_stonefish PRIVATE SIM_ENGINE_LIBRARY_PATH="${SIM_STONEFISH_LIBRARY}")
add_dependencies(sim_backend_stonefish sim_stonefish_worker)

add_executable(sim_conformance tests/conformance.cpp)
target_link_libraries(sim_conformance PRIVATE autonomy_sim)
add_dependencies(sim_conformance sim_backend_mujoco sim_backend_stonefish)
add_test(NAME sim_mujoco_contract COMMAND sim_conformance $<TARGET_FILE:sim_backend_mujoco>)
add_test(NAME sim_stonefish_contract COMMAND sim_conformance $<TARGET_FILE:sim_backend_stonefish>)
set_tests_properties(sim_mujoco_contract sim_stonefish_contract PROPERTIES TIMEOUT 180)
install(TARGETS autonomy_sim sim_backend_mujoco sim_backend_stonefish LIBRARY DESTINATION lib COMPONENT simulation)
install(TARGETS sim_stonefish_worker RUNTIME DESTINATION lib COMPONENT simulation)
install(FILES "${PROJECT_SOURCE_DIR}/include/sim_backend.h"
              "${PROJECT_SOURCE_DIR}/include/math.h" DESTINATION include COMPONENT simulation)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/tools/blender/autonomy_sim" DESTINATION share/autonomy/blender COMPONENT simulation
  FILES_MATCHING PATTERN "*.py" PATTERN "test_*" EXCLUDE PATTERN "__pycache__" EXCLUDE)

add_library(auv_sim SHARED auv_plugin.cpp blender_link.cpp)
target_link_libraries(auv_sim PRIVATE autonomy_sim)
set_target_properties(auv_sim PROPERTIES BUILD_RPATH "$ORIGIN" INSTALL_RPATH "$ORIGIN")
install(TARGETS auv_sim LIBRARY DESTINATION lib COMPONENT simulation)

add_executable(simctl cli.cpp)
target_link_libraries(simctl PRIVATE autonomy_sim)
install(TARGETS simctl RUNTIME DESTINATION bin COMPONENT simulation)
set_target_properties(simctl PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")

add_executable(sim_abi_c tests/abi.c)
target_link_libraries(sim_abi_c PRIVATE sim_headers ${CMAKE_DL_LIBS})
add_test(NAME sim_c_abi COMMAND sim_abi_c)
add_executable(sim_scenarios tests/scenarios.cpp)
target_link_libraries(sim_scenarios PRIVATE autonomy_sim)
add_executable(sim_forces tests/forces.cpp)
target_link_libraries(sim_forces PRIVATE autonomy_sim)
add_executable(sim_blender_source tests/blender_source.cpp)
target_link_libraries(sim_blender_source PRIVATE autonomy_sim)
add_dependencies(sim_scenarios sim_backend_mujoco sim_backend_stonefish)

# Saved-file physics tests run headlessly; Blender is needed only to author
# mutation fixtures and exercise its presentation add-on.
set(SIM_POOL_SCENE "${PROJECT_SOURCE_DIR}/auvs/stonefish_sim/data/pool_scene.blend")
add_test(NAME sim_rotor_response COMMAND sim_forces "${SIM_POOL_SCENE}")
set_tests_properties(sim_rotor_response PROPERTIES TIMEOUT 180)
find_program(SIM_BLENDER_EXECUTABLE NAMES blender)
if(SIM_BLENDER_EXECUTABLE)
  add_test(NAME sim_blender_source_export COMMAND "${SIM_BLENDER_EXECUTABLE}"
    --background --factory-startup "${SIM_POOL_SCENE}"
    --python-exit-code 1 --python "${PROJECT_SOURCE_DIR}/tools/blender/test_source.py" --
    $<TARGET_FILE:sim_blender_source> $<TARGET_FILE:sim_backend_mujoco> $<TARGET_FILE:sim_backend_stonefish>)
  set_tests_properties(sim_blender_source_export PROPERTIES TIMEOUT 300 RUN_SERIAL TRUE)
  foreach(engine mujoco stonefish)
    add_test(NAME sim_${engine}_blender_integration COMMAND "${SIM_BLENDER_EXECUTABLE}"
      --background --factory-startup
      --python-exit-code 1 --python "${PROJECT_SOURCE_DIR}/tools/blender/test_blender.py" -- ${engine})
    set_tests_properties(sim_${engine}_blender_integration PROPERTIES TIMEOUT 180 SKIP_RETURN_CODE 77 RUN_SERIAL TRUE)
  endforeach()
  add_test(NAME sim_blender_rotor_animation COMMAND "${SIM_BLENDER_EXECUTABLE}"
    --background --factory-startup "${PROJECT_SOURCE_DIR}/auvs/stonefish_sim/data/pool_scene.blend"
    --python-exit-code 1 --python "${PROJECT_SOURCE_DIR}/tools/blender/autonomy_sim/test_rotor_animation.py")
  set_tests_properties(sim_blender_rotor_animation PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE)
endif()
add_custom_target(sim_all DEPENDS simctl auv_sim sim_backend_mujoco sim_backend_stonefish sim_conformance sim_scenarios sim_abi_c sim_forces sim_blender_source)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_test(NAME sim_blender_direct COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/tools/blender/test_direct.py"
  $<TARGET_FILE:simctl> $<TARGET_FILE:sim_backend_mujoco> $<TARGET_FILE:sim_backend_stonefish>)
set_tests_properties(sim_blender_direct PROPERTIES TIMEOUT 300 RUN_SERIAL TRUE)
add_test(NAME sim_controller_allocation COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/controller.py")
set_tests_properties(sim_controller_allocation PROPERTIES TIMEOUT 180)
foreach(engine mujoco stonefish)
  add_test(NAME sim_${engine}_harness COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/harness.py" $<TARGET_FILE:simctl> $<TARGET_FILE:sim_backend_${engine}>)
  set_tests_properties(sim_${engine}_harness PROPERTIES TIMEOUT 180)
endforeach()

set_target_properties(sim_backend_mujoco sim_stonefish_worker PROPERTIES INSTALL_RPATH "$ORIGIN")
get_filename_component(SIM_MUJOCO_LIBDIR "${SIM_MUJOCO_LIBRARY}" DIRECTORY)
get_filename_component(SIM_STONEFISH_LIBDIR "${SIM_STONEFISH_LIBRARY}" DIRECTORY)
install(DIRECTORY "${SIM_MUJOCO_LIBDIR}/" DESTINATION lib COMPONENT simulation FILES_MATCHING PATTERN "libmujoco.so*")
install(DIRECTORY "${SIM_STONEFISH_LIBDIR}/" DESTINATION lib COMPONENT simulation FILES_MATCHING PATTERN "libStonefish.so*")
foreach(engine mujoco stonefish)
  add_test(NAME sim_${engine}_mission COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/mission.py"
    "${PROJECT_SOURCE_DIR}/zig-out/bin/src_3" $<TARGET_FILE:auv_sim> $<TARGET_FILE:sim_backend_${engine}>
    "${SIM_POOL_SCENE}" "${CMAKE_CURRENT_SOURCE_DIR}/scenarios/controller.json" $<TARGET_FILE:simctl>)
  set_tests_properties(sim_${engine}_mission PROPERTIES TIMEOUT 180 SKIP_RETURN_CODE 77)
  add_test(NAME sim_${engine}_mission_budget COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/mission.py"
    "${PROJECT_SOURCE_DIR}/zig-out/bin/src_3" $<TARGET_FILE:auv_sim> $<TARGET_FILE:sim_backend_${engine}>
    "${SIM_POOL_SCENE}" "${CMAKE_CURRENT_SOURCE_DIR}/scenarios/controller.json" $<TARGET_FILE:simctl> --bounded)
  set_tests_properties(sim_${engine}_mission_budget PROPERTIES TIMEOUT 180 SKIP_RETURN_CODE 77)
  add_test(NAME sim_${engine}_blender_link COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/blender_link.py"
    "${PROJECT_SOURCE_DIR}/zig-out/bin/src_3" $<TARGET_FILE:auv_sim> $<TARGET_FILE:sim_backend_${engine}>
    "${SIM_POOL_SCENE}" "${CMAKE_CURRENT_SOURCE_DIR}/scenarios/controller.json")
  set_tests_properties(sim_${engine}_blender_link PROPERTIES TIMEOUT 90 SKIP_RETURN_CODE 77)
endforeach()

# Storage/transport tests exercise the real adapters as well as isolated pools.
add_executable(sim_database tests/database.cpp backends/wire.cpp)
target_link_libraries(sim_database PRIVATE autonomy_sim)
add_dependencies(sim_database sim_backend_mujoco sim_backend_stonefish)
foreach(engine mujoco stonefish)
  add_test(NAME sim_${engine}_database COMMAND sim_database $<TARGET_FILE:sim_backend_${engine}>)
  add_test(NAME sim_${engine}_c_view COMMAND sim_abi_c $<TARGET_FILE:sim_backend_${engine}>)
  set_tests_properties(sim_${engine}_database sim_${engine}_c_view PROPERTIES TIMEOUT 180)
endforeach()
add_dependencies(sim_all sim_database)

# Each fixed asset is a complete one-robot scene. Qualify both adapters using
# the existing replay, cold reset and concurrent-world scenario checks.
set(SIM_ROBOT_ASSETS "${PROJECT_SOURCE_DIR}/auvs/stonefish_sim/data/robots")
foreach(robot hydrus proteus bluerov girona500)
  foreach(engine mujoco stonefish)
    add_test(NAME sim_${engine}_${robot}_asset COMMAND sim_scenarios
      $<TARGET_FILE:sim_backend_${engine}> "${SIM_ROBOT_ASSETS}/${robot}.blend")
    set_tests_properties(sim_${engine}_${robot}_asset PROPERTIES TIMEOUT 180)
  endforeach()
endforeach()
if(SIM_BLENDER_EXECUTABLE)
  add_test(NAME sim_fixed_robot_assets COMMAND "${Python3_EXECUTABLE}"
    "${PROJECT_SOURCE_DIR}/tools/blender/test_robot_assets.py"
    $<TARGET_FILE:simctl> "${SIM_ROBOT_ASSETS}" "${SIM_BLENDER_EXECUTABLE}")
  set_tests_properties(sim_fixed_robot_assets PROPERTIES TIMEOUT 180 RUN_SERIAL TRUE)
endif()
install(DIRECTORY "${SIM_ROBOT_ASSETS}/" DESTINATION share/autonomy/robots
  COMPONENT simulation FILES_MATCHING PATTERN "*.blend" PATTERN "manifest.json"
  PATTERN "README.md")
