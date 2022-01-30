cmake_minimum_required(VERSION 3.5)
project(composition_benchmark)

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

add_library(composable_node SHARED
  src/composable_node.cpp
)
ament_target_dependencies(composable_node rclcpp rclcpp_components performance_test)
target_include_directories(composable_node PUBLIC include)
rclcpp_components_register_nodes(composable_node "ComposableNode")

add_library(composable_publisher SHARED
  src/composable_publisher.cpp
)
ament_target_dependencies(composable_publisher rclcpp rclcpp_components performance_test irobot_interfaces_plugin)
target_include_directories(composable_publisher PUBLIC include)
rclcpp_components_register_nodes(composable_publisher "ComposablePublisher")

add_library(composable_subscriber SHARED
  src/composable_subscriber.cpp
)
ament_target_dependencies(composable_subscriber rclcpp rclcpp_components performance_test irobot_interfaces_plugin)
target_include_directories(composable_subscriber PUBLIC include)
rclcpp_components_register_nodes(composable_subscriber "ComposableSubscriber")

add_library(base_node SHARED
  src/base_node.cpp
)
ament_target_dependencies(base_node rclcpp rclcpp_components performance_test)
target_include_directories(base_node PUBLIC include)

add_subdirectory(apps)

install(TARGETS
  base_node
  composable_node
  composable_publisher
  composable_subscriber
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(composable_node)
ament_export_dependencies(rclcpp rclcpp_components performance_test)

ament_package()
