cmake_minimum_required(VERSION 3.5)
project(irobot_benchmark)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(performance_test REQUIRED)
find_package(performance_test_factory REQUIRED)

add_executable(irobot_benchmark src/irobot_benchmark.cpp)
ament_target_dependencies(irobot_benchmark rclcpp rclcpp_lifecycle performance_test performance_test_factory)

install(TARGETS
  irobot_benchmark
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    profiles
    topology
  TYPE
    DATA
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

