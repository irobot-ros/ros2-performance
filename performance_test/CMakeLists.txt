cmake_minimum_required(VERSION 3.5)
project(performance_test)

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
find_package(performance_test_msgs REQUIRED)

set(LIBRARY_DEPENDENCIES
  rclcpp
  rclcpp_lifecycle
  performance_test_msgs
)

set(LIBRARY_SRC
  src/events_logger.cpp
  src/performance_node_base.cpp
  src/resource_usage_logger.cpp
  src/system.cpp
  src/tracker.cpp
  src/utils/fork_process.cpp
  src/utils/names_utilities.cpp
  src/utils/node_options.cpp
  src/utils/stat_logger.cpp
)

set(LIBRARY_NAME ${PROJECT_NAME})
add_library(${LIBRARY_NAME} SHARED ${LIBRARY_SRC})

target_include_directories(${LIBRARY_NAME} PUBLIC
  include
  external/cxxopts-2.1.2/include
)
ament_target_dependencies(${LIBRARY_NAME} ${LIBRARY_DEPENDENCIES})

install(TARGETS
  ${LIBRARY_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  external/cxxopts-2.1.2/include/
  DESTINATION include/
)

##
## build the unit tests
##
if(BUILD_TESTING)
  add_subdirectory(test)

  # Linters
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  set(_linter_excludes
    external
    external/cxxopts-2.1.2/include/cxxopts.hpp
  )

  ament_cppcheck(
    EXCLUDE ${_linter_excludes}
    LANGUAGE C++
  )
  ament_cpplint(EXCLUDE ${_linter_excludes})
  ament_lint_cmake()
  ament_uncrustify(
    EXCLUDE ${_linter_excludes}
    LANGUAGE C++
  )
  ament_xmllint()
endif()

ament_export_include_directories(include)
ament_export_libraries(${LIBRARY_NAME})
ament_export_dependencies(${LIBRARY_DEPENDENCIES})

ament_package()
