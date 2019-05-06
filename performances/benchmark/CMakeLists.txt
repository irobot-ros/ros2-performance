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
find_package(benchmark_msgs REQUIRED)
find_package(performance_test_msgs REQUIRED)
find_package(performance_test REQUIRED)


include_directories(
  include
)

add_executable(benchmark src/benchmark.cpp)
ament_target_dependencies(benchmark rclcpp performance_test performance_test_msgs benchmark_msgs)

install(TARGETS
  benchmark
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  topology/sierra_nevada.json
  DESTINATION lib/${PROJECT_NAME}/topology)

ament_package()

