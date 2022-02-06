
find_package(ament_lint_auto REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

ament_lint_auto_find_test_dependencies()

find_package(rclcpp_lifecycle REQUIRED)
ament_add_gtest(test_node test_node.cpp)
ament_target_dependencies(test_node performance_metrics performance_test_msgs rclcpp_lifecycle)
target_link_libraries(test_node ${LIBRARY_NAME})

ament_add_gtest(test_system test_system.cpp)
ament_target_dependencies(test_system performance_metrics performance_test_msgs)
target_link_libraries(test_system ${LIBRARY_NAME})

ament_add_gtest(test_executors test_executors.cpp)
target_link_libraries(test_executors ${LIBRARY_NAME})
