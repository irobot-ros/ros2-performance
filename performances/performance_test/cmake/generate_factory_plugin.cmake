# This is the path where this file is located
set(GENERATE_FACTORY_PLUGIN__INTERNAL_DIR__ ${CMAKE_CURRENT_LIST_DIR} CACHE INTERNAL "")


function(generate_factory_plugin)

  find_package(rclcpp REQUIRED)
  find_package(performance_test REQUIRED)

  set (LIBRARY_DEPENDENCIES
    rclcpp
    performance_test
  )

  set (CUSTOM_TARGET_NAME ${PROJECT_NAME}_implementation.cpp)
  set (CUSTOM_TARGET_PATH ${PROJECT_BINARY_DIR}/generated/${CUSTOM_TARGET_NAME})
  add_custom_command(
      OUTPUT ${CUSTOM_TARGET_PATH}
      COMMAND python3
        ${GENERATE_FACTORY_PLUGIN__INTERNAL_DIR__}/generate_script.py
        ${CUSTOM_TARGET_PATH}
        --package ${PROJECT_NAME}
        --msg ${CUSTOM_MSGS}
        --srv ${CUSTOM_SRVS}
      DEPENDS ${PROJECT_NAME}
      COMMENT "Generate (custom command)"
  )

  set (LIBRARY_NAME ${PROJECT_NAME}_implementation)
  add_library(${LIBRARY_NAME} SHARED
    ${CUSTOM_TARGET_PATH}
  )
  ament_target_dependencies(${LIBRARY_NAME} ${LIBRARY_DEPENDENCIES})
  rosidl_target_interfaces(${LIBRARY_NAME} ${PROJECT_NAME} "rosidl_typesupport_cpp")

  install(TARGETS
    ${LIBRARY_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  ament_export_libraries(${LIBRARY_NAME})
  ament_export_dependencies(${LIBRARY_DEPENDENCIES})

endfunction()