cmake_minimum_required(VERSION 3.5)
project(benchmark_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(performance_test_msgs REQUIRED)

set( CUSTOM_MSGS
  "msg/Stamped3Float32.msg"
  "msg/Stamped4Float32.msg"
  "msg/Stamped4Int32.msg"
  "msg/Stamped9Float32.msg"
  "msg/Stamped12Float32.msg"
  "msg/StampedInt64.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${CUSTOM_MSGS}
  DEPENDENCIES builtin_interfaces std_msgs performance_test_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()









