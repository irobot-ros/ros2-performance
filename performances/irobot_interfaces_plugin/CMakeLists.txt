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

set(INTERFACES_NAME ${PROJECT_NAME})
rosidl_generate_interfaces(${INTERFACES_NAME}
  ${CUSTOM_MSGS}
  ${CUSTOM_SRVS}
  DEPENDENCIES std_msgs performance_test_msgs
)


##### MACRO START

find_package(rclcpp REQUIRED)
find_package(performance_test REQUIRED)

set (LIBRARY_DEPENDENCIES
  rclcpp
  performance_test
)

set (CUSTOM_TARGET_NAME ${PROJECT_NAME}_implementation.cpp)
set (CUSTOM_TARGET_PATH ${PROJECT_BINARY_DIR}/generated/${CUSTOM_TARGET_NAME})
add_custom_command(
    OUTPUT ${CUSTOM_TARGET_PATH}
    COMMAND python3
      ${CMAKE_CURRENT_LIST_DIR}/generate_script.py
      ${CUSTOM_TARGET_PATH}
      --package ${PROJECT_NAME}
      --msg ${CUSTOM_MSGS}
      --srv ${CUSTOM_SRVS}
    DEPENDS ${INTERFACES_NAME}
    COMMENT "Generate (custom command)"
)

set (LIBRARY_NAME ${PROJECT_NAME}_implementation)
add_library(${LIBRARY_NAME} SHARED
  ${CUSTOM_TARGET_PATH}
)
ament_target_dependencies(${LIBRARY_NAME} ${LIBRARY_DEPENDENCIES})
rosidl_target_interfaces(${LIBRARY_NAME} ${INTERFACES_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  ${LIBRARY_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(${LIBRARY_NAME})
ament_export_dependencies(${LIBRARY_DEPENDENCIES})

##### MACRO END


ament_export_dependencies(rosidl_default_runtime)

ament_package()
