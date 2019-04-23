cmake_minimum_required(VERSION 3.5)
project(performance_test_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set( CUSTOM_MSGS
  "msg/PerformanceHeader.msg"
  "msg/Stamped10b.msg"
  "msg/Stamped100b.msg"
  "msg/Stamped250b.msg"
  "msg/Stamped1kb.msg"
  "msg/Stamped10kb.msg"
  "msg/Stamped100kb.msg"
  "msg/Stamped250kb.msg"
  "msg/Stamped1mb.msg"
  "msg/Stamped4mb.msg"
  "msg/Stamped8mb.msg"
  "msg/StampedVector.msg"
)

set ( CUSTOM_SRVS
  "srv/Stamped10b.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${CUSTOM_MSGS}
  ${CUSTOM_SRVS}
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

