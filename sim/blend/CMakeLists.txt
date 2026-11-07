include(FetchContent)
FetchContent_Declare(cblend
  GIT_REPOSITORY https://github.com/SK83RJOSH/cblend.git
  GIT_TAG a80b57666faf001a04ee00972a4a84871894099e
  GIT_SUBMODULES third_party/expected third_party/optional third_party/range-v3
  SOURCE_SUBDIR autonomy-do-not-configure-upstream
)
FetchContent_MakeAvailable(cblend)
file(READ ${cblend_SOURCE_DIR}/source/cblend.cpp cblend_original)
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/cblend_process_field.inc cblend_field_fix)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/cblend_process_field.inc)
string(FIND "${cblend_original}" "Result<Option<AggregateType::Field>, ReflectionError> ProcessField(" cblend_fix_start)
string(FIND "${cblend_original}" "Result<TypeDatabase, ReflectionError> CreateTypeDatabase(" cblend_fix_end)
if(cblend_fix_start LESS 0 OR cblend_fix_end LESS 0)
  message(FATAL_ERROR "Pinned CBlend source does not match the array field patch")
endif()
string(SUBSTRING "${cblend_original}" 0 ${cblend_fix_start} cblend_before)
string(SUBSTRING "${cblend_original}" ${cblend_fix_end} -1 cblend_after)
file(CONFIGURE OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/cblend_fixed.cpp CONTENT "${cblend_before}${cblend_field_fix}${cblend_after}" @ONLY)
add_library(autonomy_cblend STATIC
  ${CMAKE_CURRENT_BINARY_DIR}/cblend_fixed.cpp
  ${cblend_SOURCE_DIR}/source/cblend_format.cpp
  ${cblend_SOURCE_DIR}/source/cblend_query.cpp
  ${cblend_SOURCE_DIR}/source/cblend_reflection.cpp
  ${cblend_SOURCE_DIR}/source/cblend_stream.cpp
)
set_target_properties(autonomy_cblend PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(autonomy_cblend PUBLIC cxx_std_20)
target_include_directories(autonomy_cblend SYSTEM PUBLIC
  ${cblend_SOURCE_DIR}/include
  ${cblend_SOURCE_DIR}/third_party/expected/include
  ${cblend_SOURCE_DIR}/third_party/optional/include
  ${cblend_SOURCE_DIR}/third_party/range-v3/include
)
find_package(ZLIB REQUIRED)
find_package(OpenSSL REQUIRED COMPONENTS Crypto)
find_package(glm REQUIRED)
find_library(AUTONOMY_ZSTD_LIBRARY NAMES zstd REQUIRED)
find_library(AUTONOMY_QHULL_LIBRARY NAMES qhull_r REQUIRED)
add_library(autonomy_blend_reader STATIC native_scene.cpp robot_registry.cpp
  ../robots/hydrus.cpp ../robots/proteus.cpp ../robots/bluerov.cpp ../robots/girona500.cpp)
set_target_properties(autonomy_blend_reader PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(autonomy_blend_reader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(autonomy_blend_reader PRIVATE SIM_BLENDER_PROFILES="${PROJECT_SOURCE_DIR}/sim/profiles")
target_link_libraries(autonomy_blend_reader PUBLIC sim_headers autonomy_cblend glm::glm OpenSSL::Crypto ZLIB::ZLIB ${AUTONOMY_ZSTD_LIBRARY} ${AUTONOMY_QHULL_LIBRARY})
