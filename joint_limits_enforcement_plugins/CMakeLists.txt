cmake_minimum_required(VERSION 3.5)
project(joint_limits_enforcement_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(joint_limits REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(ruckig REQUIRED)

add_library(
  ruckig_joint_limiter
  SHARED
  src/ruckig_joint_limiter.cpp
)
target_include_directories(
  ruckig_joint_limiter
  PRIVATE
  include
)
target_link_libraries(
  ruckig_joint_limiter
  ruckig::ruckig
)
ament_target_dependencies(
  ruckig_joint_limiter
  joint_limits
  rclcpp
  rcutils
  ruckig
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ruckig_joint_limiter PRIVATE "CONTROLLER_INTERFACE_BUILDING_DLL")

pluginlib_export_plugin_description_file(joint_limits joint_limits_enforcement_plugins.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ruckig_joint_limiter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(joint_limits REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)

  ament_add_gmock(test_ruckig_joint_limiter test/test_ruckig_joint_limiter.cpp)
  target_include_directories(test_ruckig_joint_limiter PRIVATE include)
  ament_target_dependencies(
    test_ruckig_joint_limiter
    joint_limits
    pluginlib
    rclcpp
  )
endif()

ament_export_dependencies(
  joint_limits
  rclcpp
  rcutils
  ruckig
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ruckig_joint_limiter
)

ament_package()
