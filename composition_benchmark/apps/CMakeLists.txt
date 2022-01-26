add_executable(simple_composable_main simple_composable_main.cpp
)
ament_target_dependencies(simple_composable_main performance_test)
target_link_libraries(simple_composable_main composable_node)

add_executable(simple_base_main simple_base_main.cpp
)
ament_target_dependencies(simple_base_main performance_test)
target_link_libraries(simple_base_main base_node)

find_package(irobot_interfaces_plugin REQUIRED)
add_executable(publisher_main publisher_main.cpp
)
ament_target_dependencies(publisher_main performance_test irobot_interfaces_plugin)
target_link_libraries(publisher_main base_node)

add_executable(subscriber_main subscriber_main.cpp
)
ament_target_dependencies(subscriber_main performance_test irobot_interfaces_plugin)
target_link_libraries(subscriber_main base_node)

install(TARGETS
  simple_base_main
  simple_composable_main
  publisher_main
  subscriber_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
