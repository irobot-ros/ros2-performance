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

find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(performance_test REQUIRED)

include_directories(
  include
  external/json/single_include
)

set (LIBRARY_DEPENDENCIES
  rclcpp
  performance_test
)

set(LIBRARY_SRC
  src/factory.cpp
  src/load_plugins.cpp
)

set(LIBRARY_NAME ${PROJECT_NAME})
add_library(${LIBRARY_NAME} SHARED ${LIBRARY_SRC})
ament_target_dependencies(${LIBRARY_NAME} ${LIBRARY_DEPENDENCIES})

install(TARGETS
  ${LIBRARY_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  external/json/single_include/
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
endif()

ament_export_include_directories(include)
ament_export_libraries(${LIBRARY_NAME})
ament_export_dependencies(${LIBRARY_DEPENDENCIES})

ament_package()