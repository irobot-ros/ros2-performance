
find_package(ament_lint_auto REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

ament_lint_auto_find_test_dependencies()

find_package(rclcpp_lifecycle REQUIRED)
ament_add_gtest(test_node test_node.cpp)
target_link_libraries(test_node
  ${LIBRARY_NAME}
  ${performance_test_msgs_TARGETS}
  ${performance_metrics_LIBRARIES}
  rclcpp_lifecycle::rclcpp_lifecycle
)

ament_add_gtest(test_system test_system.cpp)
target_link_libraries(test_system
  ${LIBRARY_NAME}
  ${performance_test_msgs_TARGETS}
  ${performance_metrics_LIBRARIES}
)

ament_add_gtest(test_executors test_executors.cpp)
target_link_libraries(test_executors ${LIBRARY_NAME})
