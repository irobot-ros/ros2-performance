cmake_minimum_required(VERSION 3.5)
project(performance_composition)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(irobot_interfaces_plugin REQUIRED)
find_package(performance_test REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(composable_node SHARED
  src/composable_node.cpp
  src/global_factory.cpp
)
ament_target_dependencies(composable_node rclcpp rclcpp_components performance_test)
target_include_directories(composable_node PUBLIC include)

add_executable(performance_composition
  src/main.cpp
)
ament_target_dependencies(performance_composition performance_test irobot_interfaces_plugin)
target_link_libraries(performance_composition composable_node)

install(TARGETS
  composable_node
  performance_composition
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(composable_node)
ament_export_dependencies(rclcpp rclcpp_components performance_test)

ament_package()
