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
install(
  DIRECTORY schema urdf
  DESTINATION share/ros2_control_test_assets
)
install(TARGETS ros2_control_test_assets
  EXPORT export_ros2_control_test_assets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_ros2_control_test_assets HAS_LIBRARY_TARGET)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(ament_lint_common REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Use pkg-config to find libxml2
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBXML2 REQUIRED IMPORTED_TARGET libxml-2.0)


# Unit test for XML schema validation

ament_add_gmock(
  test_validate_xml_schema
  test/validate_xml_schema.cpp
)
target_link_libraries(test_validate_xml_schema
  PkgConfig::LIBXML2
  ament_index_cpp::ament_index_cpp
)


ament_package()
