cmake_minimum_required(VERSION 3.16)
project(controller_interface LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  rclcpp_lifecycle
  realtime_tools
  fmt
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gen_version_h REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(controller_interface SHARED
  src/controller_interface_base.cpp
  src/controller_interface.cpp
  src/chainable_controller_interface.cpp
)
target_compile_features(controller_interface PUBLIC cxx_std_17)
target_include_directories(controller_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/controller_interface>
)
target_link_libraries(controller_interface PUBLIC
                      hardware_interface::hardware_interface
                      rclcpp_lifecycle::rclcpp_lifecycle
                      realtime_tools::realtime_tools
                      fmt::fmt)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)

  ament_add_gmock(test_controller_interface test/test_controller_interface.cpp)
  target_link_libraries(test_controller_interface
    controller_interface
  )

  ament_add_gmock(test_controller_with_options test/test_controller_with_options.cpp)
  target_link_libraries(test_controller_with_options
    controller_interface
  )
  target_compile_definitions(
    test_controller_with_options
    PRIVATE PARAMETERS_FILE_PATH="${CMAKE_CURRENT_LIST_DIR}/test/")

  ament_add_gmock(test_chainable_controller_interface test/test_chainable_controller_interface.cpp)
  target_link_libraries(test_chainable_controller_interface
    controller_interface
    hardware_interface::hardware_interface
  )

  ament_add_gmock(test_semantic_component_interface test/test_semantic_component_interface.cpp)
  target_link_libraries(test_semantic_component_interface
    controller_interface
    hardware_interface::hardware_interface
  )

  ament_add_gmock(test_force_torque_sensor test/test_force_torque_sensor.cpp)
  target_link_libraries(test_force_torque_sensor
    controller_interface
    hardware_interface::hardware_interface
  )

  ament_add_gmock(test_imu_sensor test/test_imu_sensor.cpp)
  target_link_libraries(test_imu_sensor
    controller_interface
    hardware_interface::hardware_interface
  )
  target_link_libraries(test_imu_sensor ${sensor_msgs_TARGETS})

  ament_add_gmock(test_pose_sensor test/test_pose_sensor.cpp)
  target_link_libraries(test_pose_sensor
    controller_interface
    hardware_interface::hardware_interface
  )
  target_link_libraries(test_pose_sensor ${geometry_msgs_TARGETS})
  ament_add_gmock(test_gps_sensor test/test_gps_sensor.cpp)
  target_link_libraries(test_gps_sensor
    controller_interface
    hardware_interface::hardware_interface
  )

  # Semantic component command interface tests

  ament_add_gmock(test_semantic_component_command_interface
    test/test_semantic_component_command_interface.cpp
  )
  target_link_libraries(test_semantic_component_command_interface
    controller_interface
    hardware_interface::hardware_interface
  )

  ament_add_gmock(test_led_rgb_device test/test_led_rgb_device.cpp)
  target_link_libraries(test_led_rgb_device
    controller_interface
    hardware_interface::hardware_interface
  )
  target_link_libraries(test_led_rgb_device ${std_msgs_TARGETS})
endif()

install(
  DIRECTORY include/
  DESTINATION include/controller_interface
)
install(TARGETS controller_interface
  EXPORT export_controller_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_controller_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
ament_generate_version_header(${PROJECT_NAME})
