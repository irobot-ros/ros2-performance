
find_package(ament_lint_auto REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

ament_lint_auto_find_test_dependencies()

ament_add_gtest(test_stat test_stat.cpp)
ament_target_dependencies(test_stat ${LIBRARY_DEPENDENCIES})

ament_add_gtest(test_tracker test_tracker.cpp)
ament_target_dependencies(test_tracker performance_test_msgs ${LIBRARY_NAME})
target_link_libraries(test_tracker ${LIBRARY_NAME})

ament_add_gtest(test_node test_node.cpp)
ament_target_dependencies(test_node performance_test_msgs ${LIBRARY_NAME})
target_link_libraries(test_node ${LIBRARY_NAME})

ament_add_gtest(test_system test_system.cpp)
ament_target_dependencies(test_system performance_test_msgs ${LIBRARY_NAME})
target_link_libraries(test_system ${LIBRARY_NAME})
