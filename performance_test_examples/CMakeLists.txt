cmake_minimum_required(VERSION 3.5)
project(performance_test_examples)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(performance_test REQUIRED)
find_package(performance_test_factory REQUIRED)

##
## Examples
##

add_executable(publisher_nodes_main src/publisher_nodes_main.cpp)
ament_target_dependencies(publisher_nodes_main rclcpp performance_test performance_test_factory)

add_executable(subscriber_nodes_main src/subscriber_nodes_main.cpp)
ament_target_dependencies(subscriber_nodes_main rclcpp performance_test performance_test_factory)

add_executable(client_nodes_main src/client_nodes_main.cpp)
ament_target_dependencies(client_nodes_main rclcpp performance_test performance_test_factory)

add_executable(server_nodes_main src/server_nodes_main.cpp)
ament_target_dependencies(server_nodes_main rclcpp performance_test performance_test_factory)

add_executable(simple_pub_sub_main src/simple_pub_sub_main.cpp)
ament_target_dependencies(simple_pub_sub_main rclcpp performance_test performance_test_factory)

add_executable(simple_client_service_main src/simple_client_service_main.cpp)
ament_target_dependencies(simple_client_service_main rclcpp performance_test performance_test_factory)

add_executable(json_system_main src/json_system_main.cpp)
ament_target_dependencies(json_system_main rclcpp performance_test performance_test_factory)

install(TARGETS
  publisher_nodes_main
  subscriber_nodes_main
  client_nodes_main
  server_nodes_main
  simple_pub_sub_main
  simple_client_service_main
  json_system_main
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


##
## build the unit tests
##
if(BUILD_TESTING)
  # Linters
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_cppcheck(
    LANGUAGE C++
  )
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify(
    LANGUAGE C++
  )
  ament_xmllint()
endif()

ament_package()
