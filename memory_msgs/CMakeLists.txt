cmake_minimum_required(VERSION 3.8)
project(memory_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
 "msg/M0MB.msg"
 "msg/M1.msg"
 "msg/M2.msg"
 "msg/M3.msg"
 "msg/M4.msg"
 "msg/M5.msg"
 "msg/M6.msg"
 "msg/M7.msg"
 "msg/M8.msg"
 "msg/M9.msg"
 "msg/M10.msg"
 "msg/M11.msg"
 "msg/M12.msg"
 "msg/M13.msg"
 "msg/M14.msg"
 "msg/M15.msg"
 "msg/M16.msg"
 "msg/M17.msg"
 "msg/M18.msg"
 "msg/M19.msg"
 "msg/M20.msg"
 "msg/M21.msg"
 "msg/M22.msg"
 "msg/M23.msg"
 "msg/M24.msg"
 "msg/M25.msg"
 "msg/M26.msg"
 "msg/M27.msg"
 "msg/M28.msg"
 "msg/M29.msg"
 "msg/M30.msg"
 "msg/M31.msg"
 "msg/M32.msg"
 "msg/M33.msg"
 "msg/M34.msg"
 "msg/M35.msg"
 "msg/M36.msg"
 "msg/M37.msg"
 "msg/M38.msg"
 "msg/M39.msg"
 "msg/M40.msg"
 "msg/M41.msg"
 "msg/M42.msg"
 "msg/M43.msg"
 "msg/M44.msg"
 "msg/M45.msg"
 "msg/M46.msg"
 "msg/M47.msg"
 "msg/M48.msg"
 "msg/M49.msg"
 "msg/M50.msg"
 )

ament_package()
