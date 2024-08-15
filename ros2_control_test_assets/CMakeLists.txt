cmake_minimum_required(VERSION 3.16)
project(ros2_control_test_assets LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

add_library(ros2_control_test_assets INTERFACE)
target_compile_features(ros2_control_test_assets INTERFACE cxx_std_17)
target_include_directories(ros2_control_test_assets INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/ros2_control_test_assets>
)

install(
  DIRECTORY include/
  DESTINATION include/ros2_control_test_assets
)
install(TARGETS ros2_control_test_assets
  EXPORT export_ros2_control_test_assets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_ros2_control_test_assets HAS_LIBRARY_TARGET)
ament_package()
