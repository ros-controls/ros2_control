cmake_minimum_required(VERSION 3.5)
project(ros2_control_test_assets)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES urdf/test_hardware_components.urdf
  DESTINATION share/ros2_control_test_assets/urdf
)

ament_export_include_directories(
  include
)
ament_package()
