cmake_minimum_required(VERSION 3.5)
project(performance_test_factory)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)

find_package(rcpputils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(performance_test REQUIRED)

## Build external dependencies
set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(external/json)

set(LIBRARY_DEPENDENCIES
  rcpputils
  rclcpp
  rclcpp_lifecycle
  performance_test
)

set(LIBRARY_SRC
  src/cli_options.cpp
  src/factory.cpp
  src/load_plugins.cpp
)

set(LIBRARY_NAME ${PROJECT_NAME})
add_library(${LIBRARY_NAME} SHARED ${LIBRARY_SRC})
target_include_directories(
  ${LIBRARY_NAME}
  PUBLIC
    external/cxxopts/include
    include
)

ament_target_dependencies(${LIBRARY_NAME} ${LIBRARY_DEPENDENCIES})
target_link_libraries(${LIBRARY_NAME} nlohmann_json::nlohmann_json)

install(TARGETS
  ${LIBRARY_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  external/json/single_include/
  external/cxxopts/include/
  DESTINATION include/
)

##
## build the performance_test_factory examples
##
add_subdirectory(examples)

##
## build the unit tests
##
if(BUILD_TESTING)
  add_subdirectory(test)

  # Linters
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  set(_linter_excludes_dir external)
  file(GLOB_RECURSE _linter_excludes_files FOLLOW_SYMLINKS
    "${CMAKE_CURRENT_SOURCE_DIR}/external/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/external/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/external/*.hh"
    "${CMAKE_CURRENT_SOURCE_DIR}/external/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/external/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/external/*.c"
  )

  ament_cppcheck(
    EXCLUDE
      external/cxxopts/test/catch.hpp
      external/cxxopts/test/options.cpp
      external/json/test/src/unit-iterators1.cpp
      external/json/test/src/unit-iterators2.cpp
      external/json/benchmarks/thirdparty/benchmark/src/benchmark_register.cc
      external/json/benchmarks/src/benchmarks.cpp
      external/json/single_include/nlohmann/json.hpp
      external/json/test/thirdparty/catch/catch.hpp
      external/json/test/thirdparty/Fuzzer/FuzzerLoop.cpp
      external/json/test/thirdparty/Fuzzer/afl/afl_driver.cpp
    LANGUAGE C++
  )
  ament_cpplint(EXCLUDE ${_linter_excludes_files})
  ament_uncrustify(
    EXCLUDE ${_linter_excludes_files}
    LANGUAGE C++
  )
  ament_xmllint()
endif()

ament_export_include_directories(include)
ament_export_libraries(${LIBRARY_NAME})
ament_export_dependencies(${LIBRARY_DEPENDENCIES})

ament_package()
