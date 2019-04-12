cmake_minimum_required(VERSION 3.5)
project(performance_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set( CUSTOM_INTERFACES
  "interfaces/msg/Stamped10b.msg"
  "interfaces/msg/Stamped100b.msg"
  "interfaces/msg/Stamped250b.msg"
  "interfaces/msg/Stamped1kb.msg"
  "interfaces/msg/Stamped10kb.msg"
  "interfaces/msg/Stamped100kb.msg"
  "interfaces/msg/Stamped250kb.msg"
  "interfaces/msg/Stamped1mb.msg"
  "interfaces/msg/Stamped4mb.msg"
  "interfaces/msg/Stamped8mb.msg"
  "interfaces/msg/StampedVector.msg"
  "interfaces/srv/GetHeader.srv"
)

set(UTILITIES_SRC
  src/ros2/utilities/names_utilities.cpp
  src/ros2/utilities/options.cpp
  src/ros2/utilities/print_utilities.cpp
  src/ros2/utilities/node_stats.cpp
  src/ros2/utilities/template_factory.cpp
)

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${CUSTOM_INTERFACES}
  DEPENDENCIES std_msgs
)

add_executable(publisher_nodes_main src/ros2/publisher_nodes_main.cpp ${UTILITIES_SRC})
ament_target_dependencies(publisher_nodes_main rclcpp)
rosidl_target_interfaces(publisher_nodes_main ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(subscriber_nodes_main src/ros2/subscriber_nodes_main.cpp ${UTILITIES_SRC})
ament_target_dependencies(subscriber_nodes_main rclcpp)
rosidl_target_interfaces(subscriber_nodes_main ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(client_nodes_main src/ros2/client_nodes_main.cpp ${UTILITIES_SRC})
ament_target_dependencies(client_nodes_main rclcpp)
rosidl_target_interfaces(client_nodes_main ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(server_nodes_main src/ros2/server_nodes_main.cpp ${UTILITIES_SRC})
ament_target_dependencies(server_nodes_main rclcpp)
rosidl_target_interfaces(server_nodes_main ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(simple_pub_sub_main src/ros2/simple_pub_sub_main.cpp ${UTILITIES_SRC})
ament_target_dependencies(simple_pub_sub_main rclcpp)
rosidl_target_interfaces(simple_pub_sub_main ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(simple_client_service_main src/ros2/simple_client_service_main.cpp ${UTILITIES_SRC})
ament_target_dependencies(simple_client_service_main rclcpp)
rosidl_target_interfaces(simple_client_service_main ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(discovery_time_main src/ros2/discovery_time_main.cpp ${UTILITIES_SRC})
ament_target_dependencies(discovery_time_main rclcpp)
rosidl_target_interfaces(discovery_time_main ${PROJECT_NAME} "rosidl_typesupport_cpp")


install(TARGETS
  publisher_nodes_main
  subscriber_nodes_main
  client_nodes_main
  server_nodes_main
  simple_pub_sub_main
  simple_client_service_main
  discovery_time_main
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()

