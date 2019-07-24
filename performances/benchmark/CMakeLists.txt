cmake_minimum_required(VERSION 3.5)
project(benchmark)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(performance_test REQUIRED)
find_package(performance_test_factory REQUIRED)

add_executable(benchmark src/benchmark.cpp)
ament_target_dependencies(benchmark rclcpp performance_test performance_test_factory)

install(TARGETS
  benchmark
  DESTINATION lib/${PROJECT_NAME}
)

set(TOPOLOGY_FILES
  topology/sierra_nevada.json
  topology/mont_blanc.json
  topology/white_mountain.json
  topology/debug_sierra_nevada_reliable.json
  topology/debug_sierra_nevada_best_effort.json
  topology/debug_mont_blanc_reliable.json
  topology/debug_mont_blanc_best_effort.json
)

install(FILES
  ${TOPOLOGY_FILES}
  DESTINATION lib/${PROJECT_NAME}/topology)

ament_package()

