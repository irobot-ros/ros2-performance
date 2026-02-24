
find_package(ament_lint_auto REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

ament_lint_auto_find_test_dependencies()

ament_add_gtest(test_factory test_factory.cpp)
target_link_libraries(test_factory
  ${LIBRARY_NAME}
  ${performance_test_msgs_TARGETS}
)
