cmake_minimum_required(VERSION 3.16)
project(hardware_interface_testing LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow
          -Werror=missing-braces)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  hardware_interface
  lifecycle_msgs
  pluginlib
  rclcpp_lifecycle
  ros2_control_test_assets
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(test_components SHARED
test/test_components/test_actuator.cpp
test/test_components/test_sensor.cpp
test/test_components/test_system.cpp)
ament_target_dependencies(test_components hardware_interface pluginlib ros2_control_test_assets)
install(TARGETS test_components
DESTINATION lib
)
pluginlib_export_plugin_description_file(
hardware_interface test/test_components/test_components.xml)

if(BUILD_TESTING)

  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_resource_manager test/test_resource_manager.cpp)
  ament_target_dependencies(test_resource_manager hardware_interface ros2_control_test_assets)

  ament_add_gmock(test_resource_manager_prepare_perform_switch test/test_resource_manager_prepare_perform_switch.cpp)
  ament_target_dependencies(test_resource_manager_prepare_perform_switch hardware_interface ros2_control_test_assets)

endif()

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
