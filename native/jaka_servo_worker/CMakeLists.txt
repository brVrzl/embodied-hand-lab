cmake_minimum_required(VERSION 3.20)
project(jaka_servo_worker LANGUAGES CXX)

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

add_executable(jaka_servo_worker main.cpp)
target_include_directories(jaka_servo_worker PRIVATE "${JAKA_INCLUDE_DIR}")
target_link_libraries(jaka_servo_worker PRIVATE "${JAKA_API_LIBRARY}" pthread)
target_compile_options(jaka_servo_worker PRIVATE -Wall -Wextra -Wpedantic -Werror)
set_target_properties(jaka_servo_worker PROPERTIES BUILD_RPATH "${JAKA_LIBRARY_DIR}")
