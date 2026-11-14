enable_language(C)

include(CTest)

find_program(SIMULATOR_ZIG_EXECUTABLE zig REQUIRED)
set(simulator_zig_prefix "${CMAKE_CURRENT_BINARY_DIR}/zig-core")
set(simulator_zig_cache "${CMAKE_CURRENT_BINARY_DIR}/zig-core-cache")
if(WIN32)
    set(simulator_core_staged "${simulator_zig_prefix}/bin/simulator_core.dll")
    set(auv_simulator_staged "${simulator_zig_prefix}/bin/auv_simulator.dll")
    set(simulator_core_library "${CMAKE_CURRENT_BINARY_DIR}/simulator_core.dll")
    set(auv_simulator_library "${CMAKE_CURRENT_BINARY_DIR}/auv_simulator.dll")
else()
    set(simulator_core_staged "${simulator_zig_prefix}/lib/libsimulator_core.so")
    set(auv_simulator_staged "${simulator_zig_prefix}/lib/libauv_simulator.so")
    set(simulator_core_library "${CMAKE_CURRENT_BINARY_DIR}/libsimulator_core.so")
    set(auv_simulator_library "${CMAKE_CURRENT_BINARY_DIR}/libauv_simulator.so")
endif()
file(GLOB simulator_zig_sources CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/zig/*.zig"
    "${PROJECT_SOURCE_DIR}/src/BinaryData.zig"
    "${PROJECT_SOURCE_DIR}/build.zig"
)
add_custom_command(
    OUTPUT "${simulator_core_library}" "${auv_simulator_library}"
    COMMAND ${SIMULATOR_ZIG_EXECUTABLE} build simulator-core
            -Doptimize=ReleaseSafe
            --prefix "${simulator_zig_prefix}"
            --cache-dir "${simulator_zig_cache}"
            --summary line
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${simulator_core_staged}" "${simulator_core_library}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${auv_simulator_staged}" "${auv_simulator_library}"
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
    DEPENDS ${simulator_zig_sources}
    COMMENT "Building Zig simulator core"
    VERBATIM
)
add_custom_target(simulator_zig_libraries DEPENDS "${simulator_core_library}" "${auv_simulator_library}")

add_library(simulator_core SHARED IMPORTED GLOBAL)
set_target_properties(simulator_core PROPERTIES IMPORTED_LOCATION "${simulator_core_library}")
add_dependencies(simulator_core simulator_zig_libraries)

target_include_directories(
    simulator_core
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include/autonomy/simulator>
)

# Keep our math.h out of the system <cmath> search path.
target_compile_options(
    simulator_core
    INTERFACE
        "-iquote${PROJECT_SOURCE_DIR}/include"
)

option(SIMULATOR_SANITIZE "Enable address/undefined sanitizers" OFF)
option(SIMULATOR_THREAD_SANITIZE "Enable thread sanitizer for native concurrency checks" OFF)
if(SIMULATOR_SANITIZE AND SIMULATOR_THREAD_SANITIZE)
    message(FATAL_ERROR "ASan and TSan need separate builds")
endif()
if(SIMULATOR_THREAD_SANITIZE)
    target_compile_options(simulator_core INTERFACE -fsanitize=thread -fno-omit-frame-pointer -g)
    target_link_options(simulator_core INTERFACE -fsanitize=thread)
endif()

if(SIMULATOR_SANITIZE)
    target_compile_options(
        simulator_core
        INTERFACE
            -fsanitize=address,undefined
            -fno-omit-frame-pointer
    )

    target_link_options(simulator_core INTERFACE -fsanitize=address,undefined)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        # The Zig host preloads ASan. Use the same shared runtime in native
        # clients/workers so exec never mixes static and shared ASan runtimes.
        execute_process(COMMAND ${CMAKE_CXX_COMPILER} -print-file-name=libclang_rt.asan-${CMAKE_SYSTEM_PROCESSOR}.so
            OUTPUT_VARIABLE simulator_asan_runtime OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ERROR_IS_FATAL ANY)
        get_filename_component(simulator_asan_runtime_dir "${simulator_asan_runtime}" DIRECTORY)
        target_link_options(simulator_core INTERFACE -shared-libasan "-Wl,-rpath,${simulator_asan_runtime_dir}")
    endif()
endif()

set(SIMULATOR_MUJOCO_PREFIX "" CACHE PATH "Existing pinned MuJoCo installation")

if(SIMULATOR_MUJOCO_PREFIX)
    find_path(
        SIMULATOR_MUJOCO_INCLUDE
        mujoco/mujoco.h
        PATHS "${SIMULATOR_MUJOCO_PREFIX}/include"
        NO_DEFAULT_PATH
        REQUIRED
    )

    find_library(
        SIMULATOR_MUJOCO_LIB
        mujoco
        PATHS "${SIMULATOR_MUJOCO_PREFIX}/lib"
        NO_DEFAULT_PATH
        REQUIRED
    )
else()
    include(FetchContent)

    set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(MUJOCO_BUILD_SIMULATE OFF CACHE BOOL "" FORCE)
    set(MUJOCO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(MUJOCO_TEST_PYTHON_UTIL OFF CACHE BOOL "" FORCE)

    FetchContent_Declare(
        simulator_mujoco
        GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
        GIT_TAG 237c17e48539b6c90bf90d3161547cbdcbfaa1e0
    )

    FetchContent_MakeAvailable(simulator_mujoco)

    if(WIN32 AND TARGET mujoco)
        # Zig's thin LTO links a compiler-rt that does not resolve MinGW libm.
        # Zig also rejects lld-link's /STACK and /OPT forms.
        set_target_properties(mujoco PROPERTIES INTERPROCEDURAL_OPTIMIZATION OFF)
        get_target_property(mujoco_link_options mujoco LINK_OPTIONS)
        if(mujoco_link_options)
            set(mujoco_gnu_link_options)
            foreach(opt IN LISTS mujoco_link_options)
                if(opt MATCHES "STACK|/OPT:|fuse-ld=lld-link")
                    continue()
                endif()
                list(APPEND mujoco_gnu_link_options "${opt}")
            endforeach()
            list(APPEND mujoco_gnu_link_options "-Wl,--stack,16777216")
            set_target_properties(mujoco PROPERTIES LINK_OPTIONS "${mujoco_gnu_link_options}")
        endif()
    endif()

    set(SIMULATOR_MUJOCO_LIB mujoco)

    set(SIMULATOR_MUJOCO_INCLUDE "${simulator_mujoco_SOURCE_DIR}/include")
endif()

if(TARGET mujoco)
    set(simulator_mujoco_binary "$<TARGET_FILE:mujoco>")
else()
    set(simulator_mujoco_binary "${SIMULATOR_MUJOCO_LIB}")
endif()
if(WIN32)
    set(simulator_mujoco_staged "${simulator_zig_prefix}/bin/simulator_backend_mujoco.dll")
    set(simulator_mujoco_library "${CMAKE_CURRENT_BINARY_DIR}/simulator_backend_mujoco.dll")
else()
    set(simulator_mujoco_staged "${simulator_zig_prefix}/lib/libsimulator_backend_mujoco.so")
    set(simulator_mujoco_library "${CMAKE_CURRENT_BINARY_DIR}/libsimulator_backend_mujoco.so")
endif()
add_custom_command(
    OUTPUT "${simulator_mujoco_library}"
    COMMAND ${SIMULATOR_ZIG_EXECUTABLE} build simulator-mujoco
            -Doptimize=ReleaseSafe
            "-Dmujoco-include=${SIMULATOR_MUJOCO_INCLUDE}"
            "-Dmujoco-library=${simulator_mujoco_binary}"
            --prefix "${simulator_zig_prefix}"
            --cache-dir "${simulator_zig_cache}"
            --summary line
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${simulator_mujoco_staged}" "${simulator_mujoco_library}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${simulator_mujoco_binary}" "${CMAKE_CURRENT_BINARY_DIR}"
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
    DEPENDS ${simulator_zig_sources} "${simulator_core_library}" "${simulator_mujoco_binary}"
    COMMENT "Building Zig MuJoCo backend"
    VERBATIM
)
add_custom_target(simulator_zig_mujoco DEPENDS "${simulator_mujoco_library}")
add_library(simulator_backend_mujoco SHARED IMPORTED GLOBAL)
set_target_properties(simulator_backend_mujoco PROPERTIES IMPORTED_LOCATION "${simulator_mujoco_library}")
add_dependencies(simulator_backend_mujoco simulator_zig_mujoco)

if(SIMULATOR_ENABLE_STONEFISH)
set(SIMULATOR_STONEFISH_PREFIX "" CACHE PATH "Existing Stonefish installation")

if(SIMULATOR_STONEFISH_PREFIX)
    find_path(
        SIMULATOR_STONEFISH_INCLUDE
        StonefishCommon.h
        PATHS "${SIMULATOR_STONEFISH_PREFIX}/include/Stonefish"
        NO_DEFAULT_PATH
        REQUIRED
    )

    find_library(
        SIMULATOR_STONEFISH_LIB
        Stonefish
        PATHS "${SIMULATOR_STONEFISH_PREFIX}/lib"
        NO_DEFAULT_PATH
        REQUIRED
    )
else()
    set(SIMULATOR_STONEFISH_INCLUDE "${STONEFISH_DIR}/include/Stonefish")

    set(SIMULATOR_STONEFISH_LIB "${STONEFISH_DIR}/lib/libStonefish.so")

    file(MAKE_DIRECTORY "${SIMULATOR_STONEFISH_INCLUDE}")
endif()

find_package(SDL2 REQUIRED)

endif()

add_library(simulator_wire STATIC backends/wire.cpp backends/scene_wire.cpp)

set_target_properties(
    simulator_wire
    PROPERTIES
        POSITION_INDEPENDENT_CODE YES
)

target_link_libraries(simulator_wire PUBLIC simulator_core)

if(SIMULATOR_ENABLE_STONEFISH)
add_executable(simulator_stonefish_worker backends/stonefish_worker.cpp)

target_include_directories(
    simulator_stonefish_worker
    SYSTEM
    PRIVATE
        "${SIMULATOR_STONEFISH_INCLUDE}"
        ${SDL2_INCLUDE_DIRS}
)

target_compile_definitions(
    simulator_stonefish_worker
    PRIVATE
        BT_USE_DOUBLE_PRECISION
        BT_EULER_DEFAULT_ZYX
)

target_link_libraries(
    simulator_stonefish_worker
    PRIVATE
        simulator_wire
        "${SIMULATOR_STONEFISH_LIB}"
        ${SDL2_LIBRARIES}
)

if(NOT SIMULATOR_STONEFISH_PREFIX)
    add_dependencies(simulator_stonefish_worker stonefish)
endif()

add_library(simulator_backend_stonefish SHARED backends/stonefish.cpp)

target_link_libraries(simulator_backend_stonefish PRIVATE simulator_wire ${CMAKE_DL_LIBS})

add_dependencies(simulator_backend_stonefish simulator_stonefish_worker)

endif()

set(simulator_engines mujoco)
set(simulator_plugins simulator_backend_mujoco)
if(SIMULATOR_ENABLE_STONEFISH)
    list(APPEND simulator_engines stonefish)
    list(APPEND simulator_plugins simulator_backend_stonefish)
endif()

add_custom_target(
    simulator_all
    DEPENDS
    simulator_zig_libraries
    simulator_zig_mujoco
    ${simulator_plugins}
)

add_executable(simulator_c_example examples/simulate.c)

target_include_directories(simulator_c_example PRIVATE examples)

target_compile_options(simulator_c_example PRIVATE -std=c11 -pedantic-errors)

target_link_libraries(simulator_c_example PRIVATE simulator_core)

add_executable(simulator_c_abi tests/abi.c tests/abi_other.c)

target_compile_options(simulator_c_abi PRIVATE -std=c11 -pedantic-errors)

target_link_libraries(simulator_c_abi PRIVATE simulator_core)

add_executable(simulator_core_tests tests/core.cpp)

target_link_libraries(simulator_core_tests PRIVATE simulator_wire)

add_executable(simulator_snapshot_tests tests/snapshot.cpp)

target_link_libraries(simulator_snapshot_tests PRIVATE simulator_core)

add_executable(simulator_engine_tests tests/engines.cpp)

target_link_libraries(simulator_engine_tests PRIVATE simulator_core)

add_dependencies(
    simulator_all
    simulator_c_example
    simulator_c_abi
    simulator_core_tests
    simulator_engine_tests
)

add_test(NAME simulator_c_abi COMMAND simulator_c_abi)

add_test(NAME simulator_database COMMAND simulator_core_tests)

foreach(engine IN LISTS simulator_engines)
    add_test(
        NAME
        simulator_${engine}_c
        COMMAND
        simulator_c_example
        $<TARGET_FILE:simulator_backend_${engine}>
    )

    add_test(
        NAME
        simulator_${engine}_contract
        COMMAND
        simulator_engine_tests
        $<TARGET_FILE:simulator_backend_${engine}>
    )

    set_tests_properties(simulator_${engine}_c simulator_${engine}_contract PROPERTIES TIMEOUT 120)
endforeach()

foreach(target IN LISTS simulator_plugins)
    get_target_property(simulator_target_imported ${target} IMPORTED)
    if(simulator_target_imported)
        continue()
    endif()
    set_target_properties(
        ${target}
        PROPERTIES
            C_VISIBILITY_PRESET hidden
            CXX_VISIBILITY_PRESET hidden
            VISIBILITY_INLINES_HIDDEN YES
    )
    if(UNIX)
        set_target_properties(${target} PROPERTIES BUILD_RPATH "$ORIGIN" INSTALL_RPATH "$ORIGIN")
    endif()
endforeach()

if(SIMULATOR_ENABLE_STONEFISH)
set_target_properties(
    simulator_stonefish_worker
    PROPERTIES
        INSTALL_RPATH "$ORIGIN"
)

endif()

install(
    FILES
    "${simulator_core_library}"
    "${auv_simulator_library}"
    "${simulator_mujoco_library}"
    DESTINATION lib
    COMPONENT simulator
)
set(simulator_native_plugins ${simulator_plugins})
list(REMOVE_ITEM simulator_native_plugins simulator_backend_mujoco)
if(simulator_native_plugins)
install(
    TARGETS
    ${simulator_native_plugins}
    RUNTIME DESTINATION lib
    LIBRARY DESTINATION lib
    COMPONENT simulator
)
endif()

if(SIMULATOR_ENABLE_STONEFISH)
install(TARGETS simulator_stonefish_worker RUNTIME DESTINATION lib COMPONENT simulator)
endif()

install(TARGETS simulator_c_example RUNTIME DESTINATION bin COMPONENT simulator)

if(UNIX)
    set_target_properties(simulator_c_example PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
endif()

install(
    FILES
    sim.h
    robots.h
    simulation_types.h
    sim_backend.h
    arena_allocator.h
    "${PROJECT_SOURCE_DIR}/include/math.h"
    DESTINATION include/autonomy/simulator
    COMPONENT simulator
)

add_executable(simulator_dynamics_tests tests/dynamics.cpp)

target_link_libraries(simulator_dynamics_tests PRIVATE simulator_core)

add_dependencies(simulator_all simulator_dynamics_tests)

foreach(engine IN LISTS simulator_engines)
    add_test(
        NAME
        simulator_${engine}_dynamics
        COMMAND
        simulator_dynamics_tests
        $<TARGET_FILE:simulator_backend_${engine}>
    )

    set_tests_properties(simulator_${engine}_dynamics PROPERTIES TIMEOUT 120)
endforeach()

add_executable(simulator_cpp_example examples/simulate.cpp)

target_compile_options(simulator_cpp_example PRIVATE -std=c++23 -pedantic-errors)

target_link_libraries(simulator_cpp_example PRIVATE simulator_core)

add_dependencies(simulator_all simulator_cpp_example)

foreach(engine IN LISTS simulator_engines)
    add_test(
        NAME
        simulator_${engine}_cpp
        COMMAND
        simulator_cpp_example
        $<TARGET_FILE:simulator_backend_${engine}>
    )
endforeach()

foreach(header arena_allocator.h simulation_types.h sim_backend.h sim.h robots.h scene_presets.h auv_simulator.h auv.h)
    string(REPLACE "." "_" stem "${header}")

    foreach(language c cpp)
        set(source "${CMAKE_CURRENT_BINARY_DIR}/header_${stem}.${language}")

        file(WRITE "${source}" "#include \"${header}\"\nint main(void) { return 0; }\n")

        add_executable(simulator_header_${stem}_${language} "${source}")

        target_compile_options(simulator_header_${stem}_${language} PRIVATE -pedantic-errors)

        set_target_properties(
            simulator_header_${stem}_${language}
            PROPERTIES
                C_STANDARD 11
                C_STANDARD_REQUIRED YES
                C_EXTENSIONS NO
                CXX_STANDARD 23
                CXX_EXTENSIONS NO
        )

        target_link_libraries(simulator_header_${stem}_${language} PRIVATE simulator_core)

        if(SIMULATOR_SANITIZE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND language STREQUAL "c")
            set_target_properties(simulator_header_${stem}_${language} PROPERTIES LINKER_LANGUAGE CXX)
        endif()
        add_dependencies(simulator_all simulator_header_${stem}_${language})
    endforeach()
endforeach()

add_executable(simulator_trace tests/trace.c)

target_compile_options(simulator_trace PRIVATE -std=c11 -pedantic-errors)

target_link_libraries(simulator_trace PRIVATE simulator_core)

add_dependencies(simulator_all simulator_trace)

# Package the native shared libraries required by the adapters alongside them.
if(TARGET mujoco)
    install(TARGETS mujoco RUNTIME DESTINATION lib LIBRARY DESTINATION lib COMPONENT simulator)
else()
    get_filename_component(mujoco_libdir "${SIMULATOR_MUJOCO_LIB}" DIRECTORY)

    if(WIN32)
        install(
            DIRECTORY
            "${mujoco_libdir}/"
            DESTINATION lib
            COMPONENT simulator
            FILES_MATCHING
            PATTERN "mujoco.dll"
            PATTERN "libmujoco.dll"
        )
    else()
        install(
            DIRECTORY
            "${mujoco_libdir}/"
            DESTINATION lib
            COMPONENT simulator
            FILES_MATCHING
            PATTERN "libmujoco.so*"
        )
    endif()
endif()

if(SIMULATOR_ENABLE_STONEFISH)
get_filename_component(stonefish_libdir "${SIMULATOR_STONEFISH_LIB}" DIRECTORY)

install(
    DIRECTORY
    "${stonefish_libdir}/"
    DESTINATION lib
    COMPONENT simulator
    FILES_MATCHING
    PATTERN "libStonefish.so*"
)

endif()

if(SIMULATOR_SANITIZE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # Compile the examples as C, but link the C++ UBSan support used by the shared core.
    foreach(
        target
        simulator_c_example
        simulator_c_abi
        simulator_trace
        simulator_header_arena_allocator_h_c
        simulator_header_simulation_types_h_c
        simulator_header_sim_backend_h_c
        simulator_header_sim_h_c
        simulator_header_robots_h_c
    )
        set_target_properties(
            ${target}
            PROPERTIES
                LINKER_LANGUAGE CXX
        )
    endforeach()
endif()

add_executable(simulator_backend_c_tests tests/backend_c.c)

target_compile_options(simulator_backend_c_tests PRIVATE -std=c11 -pedantic-errors)

target_link_libraries(simulator_backend_c_tests PRIVATE simulator_core)

if(SIMULATOR_SANITIZE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set_target_properties(
        simulator_backend_c_tests
        PROPERTIES
            LINKER_LANGUAGE CXX
    )
endif()

add_dependencies(simulator_all simulator_backend_c_tests)

foreach(engine IN LISTS simulator_engines)
    add_test(
        NAME
        simulator_${engine}_c_mesh
        COMMAND
        simulator_backend_c_tests
        $<TARGET_FILE:simulator_backend_${engine}>
    )
endforeach()

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tmp")

get_directory_property(simulator_tests TESTS)

set_tests_properties(
    ${simulator_tests}
    PROPERTIES
        ENVIRONMENT "TMPDIR=${CMAKE_CURRENT_BINARY_DIR}/tmp"
)

add_executable(simulator_cpp_abi tests/abi_cpp.cpp tests/abi_other.c)

set_target_properties(
    simulator_cpp_abi
    PROPERTIES
        C_STANDARD 11
        C_EXTENSIONS NO
        CXX_STANDARD 23
        CXX_EXTENSIONS NO
)

target_compile_options(simulator_cpp_abi PRIVATE -pedantic-errors)

target_link_libraries(simulator_cpp_abi PRIVATE simulator_core)

add_dependencies(simulator_all simulator_cpp_abi)

add_test(NAME simulator_cpp_abi COMMAND simulator_cpp_abi)

install(
    FILES
    README.md
    VALIDATION.md
    BASE_COMMIT.txt
    DESTINATION share/autonomy/simulator
    COMPONENT simulator
)

add_executable(simulator_robot_example examples/robot.c)
target_compile_options(simulator_robot_example PRIVATE -std=c11 -pedantic-errors)
target_link_libraries(simulator_robot_example PRIVATE simulator_core)

add_executable(simulator_robot_tests tests/robots.cpp)
target_link_libraries(simulator_robot_tests PRIVATE simulator_core)
add_dependencies(simulator_all simulator_robot_example simulator_robot_tests)

if(SIMULATOR_SANITIZE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set_target_properties(simulator_robot_example PROPERTIES LINKER_LANGUAGE CXX)
endif()

add_test(NAME simulator_robot_database COMMAND simulator_robot_tests)
foreach(engine IN LISTS simulator_engines)
    add_test(
        NAME simulator_${engine}_robots
        COMMAND simulator_robot_tests $<TARGET_FILE:simulator_backend_${engine}>
    )
    add_test(
        NAME simulator_${engine}_robot_c
        COMMAND simulator_robot_example $<TARGET_FILE:simulator_backend_${engine}> hydrus
    )
    set_tests_properties(
        simulator_${engine}_robots simulator_${engine}_robot_c
        PROPERTIES TIMEOUT 120 ENVIRONMENT "TMPDIR=${CMAKE_CURRENT_BINARY_DIR}/tmp"
    )
endforeach()

find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(
    NAME simulator_robot_assets
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/robot_assets.py
            ${CMAKE_CURRENT_SOURCE_DIR}
)

if(UNIX)
    set_target_properties(simulator_robot_example PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
endif()
install(TARGETS simulator_robot_example RUNTIME DESTINATION bin COMPONENT simulator)
install(DIRECTORY data/robots DESTINATION share/autonomy/simulator/data COMPONENT simulator)
install(FILES data/pool_scene.blend DESTINATION share/autonomy/simulator/data COMPONENT simulator)
install(DIRECTORY data/textures DESTINATION share/autonomy/simulator/data COMPONENT simulator)

add_library(auv_simulator SHARED IMPORTED GLOBAL)
set_target_properties(auv_simulator PROPERTIES IMPORTED_LOCATION "${auv_simulator_library}")
add_dependencies(auv_simulator simulator_zig_libraries)
target_link_libraries(auv_simulator INTERFACE simulator_core)
add_dependencies(simulator_all auv_simulator)
install(FILES scene_presets.h auv_simulator.h "${PROJECT_SOURCE_DIR}/include/auv.h" "${PROJECT_SOURCE_DIR}/include/object_classes.h"
        DESTINATION include/autonomy/simulator COMPONENT simulator)

add_executable(simulator_controller_config controller_config.cpp)
target_link_libraries(simulator_controller_config PRIVATE simulator_core)
add_dependencies(simulator_all simulator_controller_config)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/config/" DESTINATION share/autonomy/config COMPONENT simulator)

add_executable(simulator_sensor_tests tests/sensors.cpp)
target_link_libraries(simulator_sensor_tests PRIVATE simulator_core)
add_dependencies(simulator_all simulator_sensor_tests)
add_test(NAME simulator_sensors COMMAND simulator_sensor_tests)

add_executable(simulator_auv_bridge_tests tests/auv_bridge.c)
target_compile_options(simulator_auv_bridge_tests PRIVATE -std=c11 -pedantic-errors)
target_link_libraries(simulator_auv_bridge_tests PRIVATE simulator_core ${CMAKE_DL_LIBS})
if(UNIX)
    target_link_libraries(simulator_auv_bridge_tests PRIVATE m)
endif()
if(SIMULATOR_SANITIZE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set_target_properties(simulator_auv_bridge_tests PROPERTIES LINKER_LANGUAGE CXX)
endif()
add_dependencies(simulator_all simulator_auv_bridge_tests)

find_program(SIMULATOR_ZIG_EXECUTABLE zig REQUIRED)
set(simulator_zig_build_args)
if(SIMULATOR_SANITIZE)
    list(APPEND simulator_zig_build_args -Dexternal-asan=true)
endif()
set(simulator_zig_target_args)
if(WIN32)
    list(APPEND simulator_zig_target_args -Dtarget=x86_64-windows-gnu)
endif()
add_custom_target(simulator_zig_controller
    COMMAND ${SIMULATOR_ZIG_EXECUTABLE} build fuzz-startup-test ${simulator_zig_build_args} ${simulator_zig_target_args} --prefix "${CMAKE_CURRENT_BINARY_DIR}/zig"
    COMMAND ${SIMULATOR_ZIG_EXECUTABLE} build ${simulator_zig_build_args} ${simulator_zig_target_args} --prefix "${CMAKE_CURRENT_BINARY_DIR}/zig"
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
    BYPRODUCTS "${CMAKE_CURRENT_BINARY_DIR}/zig/bin/src_3"
    COMMENT "Building Zig mission controller"
)
add_dependencies(simulator_all simulator_zig_controller)

set(simulator_mission_preload)
if(SIMULATOR_SANITIZE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    if(TARGET mujoco)
        set(simulator_mission_preload "${simulator_asan_runtime}:$<TARGET_FILE:mujoco>")
    else()
        set(simulator_mission_preload "${simulator_asan_runtime}:${SIMULATOR_MUJOCO_LIB}")
    endif()
endif()
foreach(engine IN LISTS simulator_engines)
    add_test(NAME simulator_${engine}_auv_bridge
        COMMAND simulator_auv_bridge_tests $<TARGET_FILE:auv_simulator> $<TARGET_FILE:simulator_backend_${engine}>)
    add_test(NAME simulator_${engine}_mission
        COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/tests/mission.py"
                "${CMAKE_CURRENT_BINARY_DIR}/zig/bin/src_3" $<TARGET_FILE:auv_simulator>
                $<TARGET_FILE:simulator_backend_${engine}> "${PROJECT_SOURCE_DIR}/config/hydrus.zig"
                ${simulator_mission_preload})
    add_test(NAME simulator_${engine}_lifecycle
        COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/tests/lifecycle.py"
                "${CMAKE_CURRENT_BINARY_DIR}/zig/bin/src_3" $<TARGET_FILE:auv_simulator>
                $<TARGET_FILE:simulator_backend_${engine}> "${PROJECT_SOURCE_DIR}/config/hydrus.zig"
                "${CMAKE_CURRENT_BINARY_DIR}/zig/lib/libauv_stationary_sim.so"
                "${PROJECT_SOURCE_DIR}/config/stationary.zig" ${simulator_mission_preload})
    set_tests_properties(simulator_${engine}_auv_bridge simulator_${engine}_mission simulator_${engine}_lifecycle
        PROPERTIES TIMEOUT 180 ENVIRONMENT "TMPDIR=${CMAKE_CURRENT_BINARY_DIR}/tmp")
endforeach()
add_test(NAME simulator_controller_calibration
    COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/tests/controller_config.py"
            $<TARGET_FILE:simulator_controller_config> "${PROJECT_SOURCE_DIR}/config/hydrus.zig" ${simulator_mission_preload})
add_test(NAME simulator_zig_tests
    COMMAND ${SIMULATOR_ZIG_EXECUTABLE} build test)
set_tests_properties(simulator_zig_tests PROPERTIES WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}" TIMEOUT 120)

install(DIRECTORY "${PROJECT_SOURCE_DIR}/tools/blender/" DESTINATION share/autonomy/tools/blender COMPONENT simulator FILES_MATCHING PATTERN "*.py")
install(FILES "${PROJECT_SOURCE_DIR}/tools/data_binary.py" DESTINATION share/autonomy/tools COMPONENT simulator)
install(FILES "${PROJECT_SOURCE_DIR}/tools/data_binary.py" DESTINATION share/autonomy/tools/blender/autonomy_sim COMPONENT simulator)
install(FILES SENSORS.md FUZZING.md DESTINATION share/autonomy/simulator COMPONENT simulator)

if(SIMULATOR_SANITIZE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # Keep native MuJoCo registration roots mapped until leak checking. Python
    # harnesses preload ASan only into their instrumented subprocesses.
    if(TARGET mujoco)
        set(simulator_native_preload "$<TARGET_FILE:mujoco>")
    else()
        set(simulator_native_preload "${SIMULATOR_MUJOCO_LIB}")
    endif()
    get_directory_property(simulator_all_tests TESTS)
    foreach(test IN LISTS simulator_all_tests)
        if(NOT test MATCHES "(_mission|_lifecycle|_assets|_calibration|_zig_tests)$")
            set_property(TEST ${test} APPEND PROPERTY ENVIRONMENT
                "LD_PRELOAD=${simulator_asan_runtime}:${simulator_native_preload}")
        endif()
    endforeach()
endif()

find_package(Threads REQUIRED)
add_executable(simulator_concurrency_tests tests/concurrency.cpp)
target_link_libraries(simulator_concurrency_tests PRIVATE auv_simulator simulator_core Threads::Threads)
target_compile_options(simulator_concurrency_tests PRIVATE -UNDEBUG)
add_dependencies(simulator_all simulator_concurrency_tests)
add_test(NAME simulator_mujoco_concurrency COMMAND simulator_concurrency_tests $<TARGET_FILE:simulator_backend_mujoco>)
add_test(NAME simulator_mujoco_fuzz
    COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/tests/fuzz.py"
            "${CMAKE_CURRENT_BINARY_DIR}/zig/bin/src_3" $<TARGET_FILE:auv_simulator>
            $<TARGET_FILE:simulator_backend_mujoco> "${PROJECT_SOURCE_DIR}/config/hydrus.zig"
            ${simulator_mission_preload})
set_tests_properties(simulator_mujoco_concurrency simulator_mujoco_fuzz PROPERTIES TIMEOUT 240)
if(SIMULATOR_SANITIZE)
    set_tests_properties(simulator_mujoco_fuzz PROPERTIES TIMEOUT 900)
endif()
if(SIMULATOR_SANITIZE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set_property(TEST simulator_mujoco_concurrency APPEND PROPERTY ENVIRONMENT
        "LD_PRELOAD=${simulator_asan_runtime}:${simulator_native_preload}")
endif()

add_test(NAME simulator_mujoco_partial_startup
    COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/tests/fuzz_startup.py"
            "${CMAKE_CURRENT_BINARY_DIR}/zig/bin/fuzz_startup_test" $<TARGET_FILE:auv_simulator>
            $<TARGET_FILE:simulator_backend_mujoco> "${PROJECT_SOURCE_DIR}/config/hydrus.zig"
            ${simulator_mission_preload})
set_tests_properties(simulator_mujoco_partial_startup PROPERTIES TIMEOUT 60)

# The new host/application contract is checked independently in C and C++.
add_executable(simulator_application_reload_tests tests/application.cpp)
target_include_directories(simulator_application_reload_tests PRIVATE "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(simulator_application_reload_tests PRIVATE auv_simulator simulator_core)
add_dependencies(simulator_application_reload_tests simulator_zig_controller)
add_dependencies(simulator_all simulator_application_reload_tests simulator_snapshot_tests)
if(WIN32)
    set(simulator_application_library "${CMAKE_CURRENT_BINARY_DIR}/zig/bin/autonomy_application.dll")
else()
    set(simulator_application_library "${CMAKE_CURRENT_BINARY_DIR}/zig/lib/libautonomy_application.so")
endif()
add_test(NAME simulator_application_reload
    COMMAND simulator_application_reload_tests "${simulator_application_library}" $<TARGET_FILE:simulator_backend_mujoco>)
add_test(NAME simulator_binary_snapshot COMMAND simulator_snapshot_tests)
set_tests_properties(simulator_application_reload PROPERTIES TIMEOUT 120)
if(SIMULATOR_SANITIZE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set_property(TEST simulator_application_reload simulator_binary_snapshot APPEND PROPERTY ENVIRONMENT "LD_PRELOAD=${simulator_asan_runtime}:${simulator_native_preload}")
endif()
add_executable(simulator_application_c_abi_tests tests/application_abi.c tests/application_abi_other.c)
target_include_directories(simulator_application_c_abi_tests PRIVATE "${PROJECT_SOURCE_DIR}/include")
target_compile_options(simulator_application_c_abi_tests PRIVATE -std=c11 -pedantic-errors)
add_dependencies(simulator_all simulator_application_c_abi_tests)
add_test(NAME simulator_application_c_abi COMMAND simulator_application_c_abi_tests)
add_test(NAME simulator_binary_codec COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/tests/binary_codec.py")
add_test(NAME simulator_python_snapshot COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/tests/binary_snapshot.py" $<TARGET_FILE:simulator_snapshot_tests>)

install(FILES "${PROJECT_SOURCE_DIR}/docs/APPLICATION_RELOAD.md" DESTINATION share/autonomy COMPONENT simulator)
