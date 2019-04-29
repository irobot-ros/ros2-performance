



add_executable(publisher_nodes_main publisher_nodes_main.cpp)
ament_target_dependencies(publisher_nodes_main rclcpp performance_test_msgs ${LIBRARY_NAME})
target_link_libraries(publisher_nodes_main ${LIBRARY_NAME})

add_executable(subscriber_nodes_main subscriber_nodes_main.cpp)
ament_target_dependencies(subscriber_nodes_main rclcpp performance_test_msgs ${LIBRARY_NAME})
target_link_libraries(subscriber_nodes_main ${LIBRARY_NAME})

add_executable(client_nodes_main client_nodes_main.cpp)
ament_target_dependencies(client_nodes_main rclcpp performance_test_msgs ${LIBRARY_NAME})
target_link_libraries(client_nodes_main ${LIBRARY_NAME})

add_executable(server_nodes_main server_nodes_main.cpp)
ament_target_dependencies(server_nodes_main rclcpp performance_test_msgs ${LIBRARY_NAME})
target_link_libraries(server_nodes_main ${LIBRARY_NAME})

add_executable(simple_pub_sub_main simple_pub_sub_main.cpp)
ament_target_dependencies(simple_pub_sub_main rclcpp performance_test_msgs ${LIBRARY_NAME})
target_link_libraries(simple_pub_sub_main ${LIBRARY_NAME})

add_executable(simple_client_service_main simple_client_service_main.cpp)
ament_target_dependencies(simple_client_service_main rclcpp performance_test_msgs ${LIBRARY_NAME})
target_link_libraries(simple_client_service_main ${LIBRARY_NAME})

add_executable(json_system_main json_system_main.cpp)
ament_target_dependencies(json_system_main rclcpp performance_test_msgs ${LIBRARY_NAME})
target_link_libraries(json_system_main ${LIBRARY_NAME})

install(TARGETS
  publisher_nodes_main
  subscriber_nodes_main
  client_nodes_main
  server_nodes_main
  simple_pub_sub_main
  simple_client_service_main
  json_system_main
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
