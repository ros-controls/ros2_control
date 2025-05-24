cmake_minimum_required(VERSION 3.16)
project(hardware_interface LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  lifecycle_msgs
  pluginlib
  rclcpp_lifecycle
  rcpputils
  rcutils
  realtime_tools
  TinyXML2
  tinyxml2_vendor
  joint_limits
  urdf
  pal_statistics
  fmt
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gen_version_h REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(hardware_interface SHARED
  src/actuator.cpp
  src/component_parser.cpp
  src/resource_manager.cpp
  src/sensor.cpp
  src/system.cpp
  src/lexical_casts.cpp
)
target_compile_features(hardware_interface PUBLIC cxx_std_17)
target_include_directories(hardware_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/hardware_interface>
  ${pal_statistics_INCLUDE_DIRS}
)
target_link_libraries(hardware_interface PUBLIC
                      rclcpp::rclcpp
                      rclcpp_lifecycle::rclcpp_lifecycle
                      pluginlib::pluginlib
                      urdf::urdf
                      realtime_tools::realtime_tools
                      rcutils::rcutils
                      rcpputils::rcpputils
                      ${joint_limits_TARGETS}
                      ${TinyXML2_LIBRARIES}
                      ${tinyxml2_vendor_LIBRARIES}
                      ${pal_statistics_LIBRARIES}
                      ${control_msgs_TARGETS}
                      ${lifecycle_msgs_TARGETS}
                      fmt::fmt)

add_library(mock_components SHARED
  src/mock_components/generic_system.cpp
)
target_compile_features(mock_components PUBLIC cxx_std_17)
target_include_directories(mock_components PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/hardware_interface>
)
target_link_libraries(mock_components PUBLIC hardware_interface)

pluginlib_export_plugin_description_file(
  hardware_interface mock_components_plugin_description.xml)

if(BUILD_TESTING)

  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_macros test/test_macros.cpp)
  target_include_directories(test_macros PRIVATE include)
  target_link_libraries(test_macros rcpputils::rcpputils)

  ament_add_gmock(test_inst_hardwares test/test_inst_hardwares.cpp)
  target_link_libraries(test_inst_hardwares hardware_interface rcpputils::rcpputils)

  ament_add_gmock(test_joint_handle test/test_handle.cpp)
  target_link_libraries(test_joint_handle hardware_interface rcpputils::rcpputils)

  # Test helper methods
  ament_add_gmock(test_helpers test/test_helpers.cpp)
  target_link_libraries(test_helpers hardware_interface)

  ament_add_gmock(test_component_interfaces test/test_component_interfaces.cpp)
  target_link_libraries(test_component_interfaces hardware_interface ros2_control_test_assets::ros2_control_test_assets)

  ament_add_gmock(test_component_interfaces_custom_export test/test_component_interfaces_custom_export.cpp)
  target_link_libraries(test_component_interfaces_custom_export hardware_interface ros2_control_test_assets::ros2_control_test_assets)

  ament_add_gmock(test_component_parser test/test_component_parser.cpp)
  target_link_libraries(test_component_parser hardware_interface ros2_control_test_assets::ros2_control_test_assets)

  add_library(test_hardware_components SHARED
  test/test_hardware_components/test_single_joint_actuator.cpp
  test/test_hardware_components/test_force_torque_sensor.cpp
  test/test_hardware_components/test_imu_sensor.cpp
  test/test_hardware_components/test_two_joint_system.cpp
  test/test_hardware_components/test_system_with_command_modes.cpp
  )
  target_link_libraries(test_hardware_components hardware_interface)
  install(TARGETS test_hardware_components
    DESTINATION lib
  )
  pluginlib_export_plugin_description_file(
    hardware_interface test/test_hardware_components/test_hardware_components.xml
  )

  ament_add_gmock(test_generic_system test/mock_components/test_generic_system.cpp)
  target_include_directories(test_generic_system PRIVATE include)
  target_link_libraries(test_generic_system hardware_interface ros2_control_test_assets::ros2_control_test_assets)
endif()

install(
  DIRECTORY include/
  DESTINATION include/hardware_interface
)
install(
  TARGETS
    mock_components
    hardware_interface
  EXPORT export_hardware_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
ament_generate_version_header(${PROJECT_NAME})
