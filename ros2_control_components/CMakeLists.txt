cmake_minimum_required(VERSION 3.5)
project(ros2_control_components)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(
  ros2_control_components
  SHARED
  # joints
  src/position_joint.cpp
  # sensors
)
ament_target_dependencies(
  ros2_control_components
  hardware_interface
  pluginlib
)
# prevent pluginlib from using boost
target_compile_definitions(ros2_control_components PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface ros2_control_components_plugins.xml)

install(TARGETS
  ros2_control_components
  DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_joints test/test_joints.cpp)
  ament_target_dependencies(test_joints hardware_interface pluginlib)
endif()

ament_export_libraries(
  ros2_control_components
)
ament_export_dependencies(
  control_components
  hardware_interface
  pluginlib
)
ament_package()
