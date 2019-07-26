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
find_package(performance_test_msgs REQUIRED)

include_directories(
  include
  external/cxxopts-2.1.2/include
)

set (LIBRARY_DEPENDENCIES
  rclcpp
  performance_test_msgs
)

set(LIBRARY_SRC
  src/ros2/names_utilities.cpp
  src/ros2/system.cpp
  src/ros2/tracker.cpp
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
  external/cxxopts-2.1.2/include/
  DESTINATION include/
)

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
