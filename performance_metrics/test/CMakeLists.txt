
find_package(ament_lint_auto REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

ament_lint_auto_find_test_dependencies()

ament_add_gtest(test_stat test_stat.cpp)
target_link_libraries(test_stat ${LIBRARY_NAME})

ament_add_gtest(test_tracker test_tracker.cpp)
ament_target_dependencies(test_tracker performance_test_msgs)
target_link_libraries(test_tracker ${LIBRARY_NAME})

ament_add_gtest(test_dummy_work test_dummy_work.cpp)
ament_target_dependencies(test_dummy_work)
target_link_libraries(test_dummy_work ${LIBRARY_NAME})
