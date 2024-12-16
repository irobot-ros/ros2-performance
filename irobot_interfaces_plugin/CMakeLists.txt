cmake_minimum_required(VERSION 3.5)
project(irobot_interfaces_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(performance_test_plugin_cmake REQUIRED)
find_package(performance_test_msgs REQUIRED)

find_package(std_msgs REQUIRED)

set( CUSTOM_MSGS
  "msg/StampedInt64.msg"
  "msg/Stamped3Float32.msg"
  "msg/Stamped4Float32.msg"
  "msg/Stamped4Int32.msg"
  "msg/Stamped9Float32.msg"
  "msg/Stamped12Float32.msg"
  "msg/Stamped10b.msg"
  "msg/Stamped50b.msg"
  "msg/Stamped100b.msg"
  "msg/Stamped250b.msg"
  "msg/Stamped1kb.msg"
  "msg/Stamped5kb.msg"
  "msg/Stamped10kb.msg"
  "msg/Stamped25kb.msg"
  "msg/Stamped50kb.msg"
  "msg/Stamped100kb.msg"
  "msg/Stamped250kb.msg"
  "msg/Stamped500kb.msg"
  "msg/Stamped600kb.msg"
  "msg/Stamped1mb.msg"
  "msg/Stamped4mb.msg"
  "msg/Stamped5mb.msg"
  "msg/Stamped8mb.msg"
  "msg/StampedVector.msg"
)

set ( CUSTOM_SRVS
  "srv/Stamped10b.srv"
  "srv/Stamped100kb.srv"
  "srv/Stamped1mb.srv"
  "srv/Stamped4mb.srv"
)

set( CUSTOM_ACTIONS
  "action/Stamped10b.action"
  "action/Stamped100kb.action"
  "action/Stamped1mb.action"
  "action/Stamped4mb.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${CUSTOM_MSGS}
  ${CUSTOM_SRVS}
  ${CUSTOM_ACTIONS}
  DEPENDENCIES std_msgs performance_test_msgs
)

generate_factory_plugin("${CUSTOM_MSGS}" "${CUSTOM_SRVS}" "${CUSTOM_ACTIONS}")

ament_export_dependencies(rosidl_default_runtime)

ament_package()
