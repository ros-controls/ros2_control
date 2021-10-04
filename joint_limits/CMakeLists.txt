cmake_minimum_required(VERSION 3.5)
project(joint_limits)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)

add_library(joint_limiter_interface SHARED src/joint_limiter_interface.cpp)
target_include_directories(
  joint_limiter_interface
  PRIVATE
  include)
ament_target_dependencies(
  joint_limiter_interface
  rclcpp
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(joint_limiter_interface PRIVATE "CONTROLLER_INTERFACE_BUILDING_DLL")

add_library(
  simple_joint_limiter
  SHARED
  src/simple_joint_limiter.cpp
)
target_include_directories(
  simple_joint_limiter
  PRIVATE
  include
)
target_link_libraries(
  simple_joint_limiter
  joint_limiter_interface
)
ament_target_dependencies(
  simple_joint_limiter
  rclcpp
  rcutils
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(simple_joint_limiter PRIVATE "CONTROLLER_INTERFACE_BUILDING_DLL")

pluginlib_export_plugin_description_file(joint_limits joint_limiters.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    joint_limiter_interface
    simple_joint_limiter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)

  #ament_add_gmock(joint_limits_test test/joint_limits_test.cpp)
  #target_include_directories(joint_limits_test PUBLIC include)
  #target_link_libraries(joint_limits_test joint_limits)

  ament_add_gmock(test_simple_joint_limiter test/test_simple_joint_limiter.cpp)
  target_include_directories(test_simple_joint_limiter PRIVATE include)
  target_link_libraries(test_simple_joint_limiter joint_limiter_interface)
  ament_target_dependencies(
    test_simple_joint_limiter
    pluginlib
    rclcpp
  )
endif()

ament_export_dependencies(
  rclcpp
  rcutils
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  joint_limiter_interface
  simple_joint_limiter
)

ament_package()
