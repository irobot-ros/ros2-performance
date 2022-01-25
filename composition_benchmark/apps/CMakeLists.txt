add_executable(simple_composable_main simple_composable_main.cpp
)
ament_target_dependencies(simple_composable_main performance_test)
target_link_libraries(simple_composable_main composable_node)

add_executable(simple_base_main simple_base_main.cpp
)
ament_target_dependencies(simple_base_main performance_test)
target_link_libraries(simple_base_main base_node)

install(TARGETS
  simple_base_main
  simple_composable_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
