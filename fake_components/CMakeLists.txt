cmake_minimum_required(VERSION 3.5)
project(fake_components)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(
  fake_components
  SHARED
  src/generic_system.cpp
)
target_include_directories(
  fake_components
  PUBLIC
  include
)
ament_target_dependencies(
  fake_components
  hardware_interface
  rclcpp
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(fake_components PRIVATE "HARDWARE_INTERFACE_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(fake_components PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface fake_components_plugin_description.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  fake_components
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_generic_system test/test_generic_system.cpp)
  target_include_directories(test_generic_system PRIVATE include)
  ament_target_dependencies(test_generic_system
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )

endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  fake_components
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)
ament_package()
