
find_package(ament_lint_auto REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

ament_lint_auto_find_test_dependencies()

ament_add_gtest(test_factory test_factory.cpp)
ament_target_dependencies(test_factory performance_test_msgs)
target_link_libraries(test_factory ${LIBRARY_NAME})
