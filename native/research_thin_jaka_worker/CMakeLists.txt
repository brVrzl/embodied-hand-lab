cmake_minimum_required(VERSION 3.20)
project(research_thin_jaka_worker LANGUAGES CXX)
enable_testing()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

get_filename_component(REPOSITORY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(JAKA_SDK_ROOT "${REPOSITORY_ROOT}/third_party/jaka_sdk/v2.2.7/linux")
set(JAKA_INCLUDE_DIR "${JAKA_SDK_ROOT}/c_cpp/inc_of_c++")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(JAKA_LIBRARY_DIR "${JAKA_SDK_ROOT}/python3/aarch64-linux-gnu")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64)$")
  set(JAKA_LIBRARY_DIR "${JAKA_SDK_ROOT}/python3/x86_64-linux-gnu")
else()
  message(FATAL_ERROR "Unsupported JAKA SDK architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()
find_library(JAKA_API_LIBRARY NAMES jakaAPI PATHS "${JAKA_LIBRARY_DIR}" NO_DEFAULT_PATH REQUIRED)

add_subdirectory("${REPOSITORY_ROOT}/native/teleop_shaping"
                 "${CMAKE_CURRENT_BINARY_DIR}/teleop_shaping")
add_executable(research_thin_jaka_worker main.cpp jaka_sdk_translation.cpp)
target_include_directories(research_thin_jaka_worker PRIVATE
  "${JAKA_INCLUDE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(research_thin_jaka_worker PRIVATE
  teleop_shaping_core "${JAKA_API_LIBRARY}" pthread)
target_compile_options(research_thin_jaka_worker PRIVATE
  -Wall -Wextra -Wpedantic -Werror)
set_target_properties(research_thin_jaka_worker PROPERTIES
  BUILD_RPATH "${JAKA_LIBRARY_DIR}")
