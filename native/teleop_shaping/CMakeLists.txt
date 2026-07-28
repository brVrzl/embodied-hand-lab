cmake_minimum_required(VERSION 3.20)
project(teleop_shaping LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(teleop_shaping_core STATIC
  src/abi_v1.cpp
  src/clutch_recovery_transport.cpp
  src/engagement_joint_shaper.cpp
  src/fake_jaka_lifecycle.cpp
  src/fake_consumer.cpp
  src/reference_joint_shaper.cpp
  src/thin_jaka_transport_adapter.cpp
)
set_target_properties(teleop_shaping_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(teleop_shaping_core PUBLIC include)
target_compile_options(teleop_shaping_core PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_library(teleop_shaping_c_api SHARED src/c_api.cpp)
target_link_libraries(teleop_shaping_c_api PRIVATE teleop_shaping_core)
target_include_directories(teleop_shaping_c_api PUBLIC include)
target_compile_options(teleop_shaping_c_api PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(teleop_shaping_tests tests/test_main.cpp)
target_link_libraries(teleop_shaping_tests PRIVATE teleop_shaping_core)
target_compile_options(teleop_shaping_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(thin_jaka_transport_tests tests/test_thin_jaka_transport.cpp)
target_link_libraries(thin_jaka_transport_tests PRIVATE teleop_shaping_core)
target_compile_options(thin_jaka_transport_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(engagement_joint_shaper_tests
  tests/test_engagement_joint_shaper.cpp)
target_link_libraries(engagement_joint_shaper_tests PRIVATE teleop_shaping_core)
target_compile_options(engagement_joint_shaper_tests PRIVATE
  -Wall -Wextra -Wpedantic -Werror)

enable_testing()
add_test(NAME teleop_shaping_tests COMMAND teleop_shaping_tests)
add_test(NAME thin_jaka_transport_tests COMMAND thin_jaka_transport_tests)
add_test(NAME engagement_joint_shaper_tests COMMAND engagement_joint_shaper_tests)
