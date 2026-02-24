find_package(irobot_interfaces_plugin REQUIRED)

add_executable(base_simple_main base_simple_main.cpp)
target_link_libraries(base_simple_main
  base_node
  ${performance_test_LIBRARIES}
)

add_executable(base_publisher_main base_publisher_main.cpp)
target_link_libraries(base_publisher_main
  ${irobot_interfaces_plugin_TARGETS}
  ${performance_test_LIBRARIES}
  ${performance_test_factory_LIBRARIES}
  base_node
)

add_executable(base_subscriber_main base_subscriber_main.cpp)
target_link_libraries(base_subscriber_main
  ${irobot_interfaces_plugin_TARGETS}
  ${performance_test_LIBRARIES}
  ${performance_test_factory_LIBRARIES}
  base_node
)

add_executable(composable_simple_main composable_simple_main.cpp)
target_link_libraries(composable_simple_main composable_node)

add_executable(composable_pub_sub_main composable_pub_sub_main.cpp)
target_link_libraries(composable_pub_sub_main
 ${irobot_interfaces_plugin_TARGETS}
 ${performance_test_LIBRARIES}
 composable_publisher
 composable_subscriber
)

add_executable(multi_producer multi-producer-node.cpp)
target_link_libraries(multi_producer
  ${irobot_interfaces_plugin_TARGETS}
  ${performance_test_LIBRARIES}
  composable_publisher
  composable_subscriber
)

install(TARGETS
  base_simple_main
  base_publisher_main
  base_subscriber_main
  composable_simple_main
  composable_pub_sub_main
  multi_producer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
