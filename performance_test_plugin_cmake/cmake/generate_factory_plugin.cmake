# This is the path where this file is located
set(GENERATE_FACTORY_PLUGIN__INTERNAL_DIR__ ${CMAKE_CURRENT_LIST_DIR} CACHE INTERNAL "")

function(generate_factory_plugin argMSGS argSRVS argACTIONS)

  find_package(rclcpp REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(performance_test REQUIRED)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)

  set(LIBRARY_DEPENDENCIES
    rclcpp
    rclcpp_action
    performance_test
  )

  set(CUSTOM_TARGET_NAME ${PROJECT_NAME}_implementation.cpp)
  set(CUSTOM_TARGET_PATH ${PROJECT_BINARY_DIR}/generated/${CUSTOM_TARGET_NAME})
  set(GENERATOR_SCRIPT ${GENERATE_FACTORY_PLUGIN__INTERNAL_DIR__}/generate_script.py)

  add_custom_command(
    OUTPUT ${CUSTOM_TARGET_PATH}
    COMMAND ${Python3_EXECUTABLE}
      ${GENERATOR_SCRIPT}
      ${CUSTOM_TARGET_PATH}
      --package ${PROJECT_NAME}
      --msg ${argMSGS}
      --srv ${argSRVS}
      --action ${argACTIONS}
    DEPENDS ${PROJECT_NAME} ${GENERATOR_SCRIPT}
  )

  set(LIBRARY_NAME ${PROJECT_NAME}_implementation)
  add_library(${LIBRARY_NAME} SHARED
    ${CUSTOM_TARGET_PATH}
  )
  ament_target_dependencies(${LIBRARY_NAME} ${LIBRARY_DEPENDENCIES})

  # First try >= Humble, fall back to <= Galactic
  if(COMMAND rosidl_get_typesupport_target)
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  else()
    rosidl_target_interfaces(${LIBRARY_NAME} ${PROJECT_NAME} "rosidl_typesupport_cpp")
  endif()
  target_link_libraries(${LIBRARY_NAME} ${cpp_typesupport_target})

  install(TARGETS
    ${LIBRARY_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  ament_export_libraries(${LIBRARY_NAME})
  ament_export_dependencies(${LIBRARY_DEPENDENCIES})

endfunction()
