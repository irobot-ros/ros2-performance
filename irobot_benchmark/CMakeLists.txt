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
find_package(irobot_events_executor REQUIRED)
find_package(performance_test_factory REQUIRED)

add_executable(irobot_benchmark src/irobot_benchmark.cpp)
ament_target_dependencies(irobot_benchmark rclcpp rclcpp_lifecycle performance_test performance_test_factory irobot_events_executor)

install(TARGETS
  irobot_benchmark
  DESTINATION lib/${PROJECT_NAME}
)

set(TOPOLOGY_FILES
  topology/sierra_nevada.json
  topology/cedar.json
  topology/mont_blanc.json
  topology/white_mountain.json
  topology/debug_sierra_nevada_reliable.json
  topology/debug_sierra_nevada_best_effort.json
  topology/debug_mont_blanc_reliable.json
  topology/debug_mont_blanc_best_effort.json
)

install(FILES
  ${TOPOLOGY_FILES}
  DESTINATION lib/${PROJECT_NAME}/topology)

set(TOPOLOGY_FILES_SINGLE_PROCESS
  topology/tests/single_process/loaned_10b.json
  topology/tests/single_process/loaned_1mb.json
  topology/tests/single_process/loaned_4mb.json
  topology/tests/single_process/pub_sub_10b_ipc_off.json
  topology/tests/single_process/pub_sub_10b_ipc_on.json
  topology/tests/single_process/pub_sub_4mb_ipc_off.json
  topology/tests/single_process/pub_sub_4mb_ipc_on.json
)

install(FILES
  ${TOPOLOGY_FILES_SINGLE_PROCESS}
  DESTINATION lib/${PROJECT_NAME}/topology/tests/single_process)

set(TOPOLOGY_FILES_MULTI_PROCESS
  topology/tests/multi_process/pub_ipc_off_10b.json
  topology/tests/multi_process/pub_ipc_off_4mb.json
  topology/tests/multi_process/pub_loaned_10b.json
  topology/tests/multi_process/pub_loaned_1mb.json
  topology/tests/multi_process/pub_loaned_4mb.json
  topology/tests/multi_process/sub_ipc_off_10b.json
  topology/tests/multi_process/sub_ipc_off_4mb.json
  topology/tests/multi_process/sub_loaned_10b.json
  topology/tests/multi_process/sub_loaned_1mb.json
  topology/tests/multi_process/sub_loaned_4mb.json
)

install(FILES
  ${TOPOLOGY_FILES_MULTI_PROCESS}
  DESTINATION lib/${PROJECT_NAME}/topology/tests/multi_process)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

