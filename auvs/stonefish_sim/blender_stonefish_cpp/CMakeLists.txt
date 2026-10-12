# Build only the CBlend reader and its header dependencies. Upstream's
# top-level CMake also enables unrelated packaging and analysis tools.
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
target_precompile_headers(autonomy_cblend PRIVATE
  <algorithm> <array> <charconv> <concepts> <deque> <filesystem>
  <fstream> <functional> <map> <memory> <set> <span> <string>
  <string_view> <tuple> <unordered_map> <unordered_set> <variant> <vector>
)
find_package(yaml-cpp REQUIRED)
find_package(ZLIB REQUIRED)
find_library(AUTONOMY_ZSTD_LIBRARY NAMES zstd REQUIRED)
add_library(stonefish_blend_reader STATIC blend_scene.cpp)
set_target_properties(stonefish_blend_reader PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(stonefish_blend_reader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(stonefish_blend_reader PUBLIC autonomy_cblend yaml-cpp ZLIB::ZLIB ${AUTONOMY_ZSTD_LIBRARY})
add_executable(stonefish_blend_inspect inspect.cpp)
target_include_directories(stonefish_blend_inspect PRIVATE ${CMAKE_SOURCE_DIR}/vendor/nlohmann_json/single_include)
target_link_libraries(stonefish_blend_inspect PRIVATE stonefish_blend_reader)
