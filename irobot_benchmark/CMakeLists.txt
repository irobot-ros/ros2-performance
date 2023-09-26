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

set(FT topology/full_topologies)

set(TEST_TOPOLOGY_FILES
  ${FT}/cedar.json
  ${FT}/debug_mont_blanc_best_effort.json
  ${FT}/debug_mont_blanc_reliable.json
  ${FT}/debug_sierra_nevada_best_effort.json
  ${FT}/debug_sierra_nevada_reliable.json
  ${FT}/debug_white_mountain_best_effort.json
  ${FT}/mont_blanc.json
  ${FT}/sierra_nevada.json
  ${FT}/sierra_nevada_fixed_size.json
  ${FT}/debug_sierra_nevada_fixed_size_best_effort.json
  ${FT}/sierra_nevada_fixed_size_loaned.json
  ${FT}/sierra_nevada_loaned.json
  ${FT}/white_mountain.json
  ${FT}/white_mountain_fixed_size.json
  ${FT}/debug_white_mountain_fixed_size_best_effort.json
  ${FT}/white_mountain_fixed_size_loaned.json
  ${FT}/white_mountain_loaned.json
)

install(FILES
  ${TEST_TOPOLOGY_FILES}
  DESTINATION lib/${PROJECT_NAME}/${FT})

set(SP topology/single_process)

set(TOPOLOGY_FILES_SINGLE_PROCESS
  ${SP}/pub_sub_loaned_10b.json
  ${SP}/pub_sub_loaned_100kb.json
  ${SP}/pub_sub_loaned_1mb.json
  ${SP}/pub_sub_loaned_4mb.json

  ${SP}/pub_sub_10b_ipc_on.json
  ${SP}/pub_sub_100kb_ipc_on.json
  ${SP}/pub_sub_1mb_ipc_on.json
  ${SP}/pub_sub_4mb_ipc_on.json

  ${SP}/pub_sub_10b_ipc_off.json
  ${SP}/pub_sub_100kb_ipc_off.json
  ${SP}/pub_sub_1mb_ipc_off.json
  ${SP}/pub_sub_4mb_ipc_off.json
)

install(FILES
  ${TOPOLOGY_FILES_SINGLE_PROCESS}
  DESTINATION lib/${PROJECT_NAME}/${SP})

set(MP topology/multi_process)

set(TOPOLOGY_FILES_MULTI_PROCESS
  ${MP}/pub_ipc_off_10b.json
  ${MP}/pub_ipc_off_100kb.json
  ${MP}/pub_ipc_off_1mb.json
  ${MP}/pub_ipc_off_4mb.json

  ${MP}/sub_ipc_off_10b.json
  ${MP}/sub_ipc_off_100kb.json
  ${MP}/sub_ipc_off_1mb.json
  ${MP}/sub_ipc_off_4mb.json

  ${MP}/pub_loaned_10b.json
  ${MP}/pub_loaned_100kb.json
  ${MP}/pub_loaned_1mb.json
  ${MP}/pub_loaned_4mb.json

  ${MP}/sub_loaned_10b.json
  ${MP}/sub_loaned_100kb.json
  ${MP}/sub_loaned_1mb.json
  ${MP}/sub_loaned_4mb.json
)

install(FILES
  ${TOPOLOGY_FILES_MULTI_PROCESS}
  DESTINATION lib/${PROJECT_NAME}/${MP})

set(PROFILES
  profiles/ignore_local_endpoints.xml
  profiles/shared_memory_fastdds_config.xml)

install(FILES
  ${PROFILES}
  DESTINATION lib/${PROJECT_NAME}/profiles)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

