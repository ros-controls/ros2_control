cmake_minimum_required(VERSION 3.16)
project(hardware_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow
          -Werror=missing-braces)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  lifecycle_msgs
  pluginlib
  rclcpp_lifecycle
  rcpputils
  rcutils
  TinyXML2
  tinyxml2_vendor
  joint_limits
  urdf
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gen_version_h REQUIRED)
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
)
ament_target_dependencies(hardware_interface PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(hardware_interface PRIVATE "HARDWARE_INTERFACE_BUILDING_DLL")

add_library(mock_components SHARED
  src/mock_components/generic_system.cpp
)
target_compile_features(mock_components PUBLIC cxx_std_17)
target_include_directories(mock_components PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/hardware_interface>
)
ament_target_dependencies(mock_components PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(mock_components PRIVATE "HARDWARE_INTERFACE_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  hardware_interface mock_components_plugin_description.xml)

if(BUILD_TESTING)

  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_macros test/test_macros.cpp)
  target_include_directories(test_macros PRIVATE include)
  ament_target_dependencies(test_macros rcpputils)

  ament_add_gmock(test_inst_hardwares test/test_inst_hardwares.cpp)
  target_link_libraries(test_inst_hardwares hardware_interface)
  ament_target_dependencies(test_inst_hardwares rcpputils)

  ament_add_gmock(test_joint_handle test/test_handle.cpp)
  target_link_libraries(test_joint_handle hardware_interface)
  ament_target_dependencies(test_joint_handle rcpputils)

  ament_add_gmock(test_component_interfaces test/test_component_interfaces.cpp)
  target_link_libraries(test_component_interfaces hardware_interface)
  ament_target_dependencies(test_component_interfaces ros2_control_test_assets)

  ament_add_gmock(test_component_interfaces_custom_export test/test_component_interfaces_custom_export.cpp)
  target_link_libraries(test_component_interfaces_custom_export hardware_interface)
  ament_target_dependencies(test_component_interfaces_custom_export ros2_control_test_assets)

  ament_add_gmock(test_component_parser test/test_component_parser.cpp)
  target_link_libraries(test_component_parser hardware_interface)
  ament_target_dependencies(test_component_parser ros2_control_test_assets)

  add_library(test_hardware_components SHARED
  test/test_hardware_components/test_single_joint_actuator.cpp
  test/test_hardware_components/test_force_torque_sensor.cpp
  test/test_hardware_components/test_imu_sensor.cpp
  test/test_hardware_components/test_two_joint_system.cpp
  test/test_hardware_components/test_system_with_command_modes.cpp
  )
  target_link_libraries(test_hardware_components hardware_interface)
  ament_target_dependencies(test_hardware_components
    pluginlib)
  install(TARGETS test_hardware_components
    DESTINATION lib
  )
  pluginlib_export_plugin_description_file(
    hardware_interface test/test_hardware_components/test_hardware_components.xml
  )

  ament_add_gmock(test_generic_system test/mock_components/test_generic_system.cpp)
  target_include_directories(test_generic_system PRIVATE include)
  target_link_libraries(test_generic_system hardware_interface)
  ament_target_dependencies(test_generic_system
    pluginlib
    ros2_control_test_assets
  )
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
