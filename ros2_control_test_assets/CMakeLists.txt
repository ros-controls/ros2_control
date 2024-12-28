cmake_minimum_required(VERSION 3.5)
project(ros2_control_test_assets)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY include/
<<<<<<< HEAD
  DESTINATION include
=======
  DESTINATION include/ros2_control_test_assets
)
install(
  FILES urdf/test_hardware_components.urdf
  DESTINATION share/ros2_control_test_assets/urdf
)
install(TARGETS ros2_control_test_assets
  EXPORT export_ros2_control_test_assets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
>>>>>>> efd4c99 (Move test_utils module from demos repo (#1955))
)

ament_export_include_directories(
  include
)
ament_package()
