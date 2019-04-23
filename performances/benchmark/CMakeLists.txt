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
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(performance_test REQUIRED)
find_package(performance_test_msgs REQUIRED)


include_directories(
  include
)

set(msg_files
  "interfaces/msg/Amazon.msg"
  "interfaces/msg/Ganges.msg"
  "interfaces/msg/Nile.msg"
  "interfaces/msg/Danube.msg"
  "interfaces/msg/Volga.msg"
  "interfaces/msg/Tigris.msg"
  "interfaces/msg/Salween.msg"
  "interfaces/msg/Arkansas.msg"
  "interfaces/msg/Parana.msg"
  "interfaces/msg/Congo.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs performance_test_msgs
)

add_executable(sierra_nevada src/sierra_nevada.cpp)
ament_target_dependencies(sierra_nevada rclcpp performance_test performance_test_msgs)
rosidl_target_interfaces(sierra_nevada ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  sierra_nevada
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

