
find_package(ament_lint_auto REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(performance_test REQUIRED)

ament_lint_auto_find_test_dependencies()

ament_add_gtest(stat_test stat_test.cpp)
ament_target_dependencies(stat_test ${LIBRARY_DEPENDENCIES})

ament_add_gtest(tracker_test tracker_test.cpp)
ament_target_dependencies(tracker_test performance_test_msgs ${LIBRARY_NAME})

ament_add_gtest(node_test node_test.cpp)
ament_target_dependencies(node_test performance_test_msgs ${LIBRARY_NAME})

ament_add_gtest(system_test system_test.cpp)
ament_target_dependencies(system_test performance_test_msgs ${LIBRARY_NAME})

ament_add_gtest(factory_test factory_test.cpp)
ament_target_dependencies(factory_test performance_test_msgs ${LIBRARY_NAME})
