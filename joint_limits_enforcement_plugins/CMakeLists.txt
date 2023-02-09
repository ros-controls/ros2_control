cmake_minimum_required(VERSION 3.16)
project(joint_limits_enforcement_plugins LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  joint_limits
  pluginlib
  rclcpp
  rcutils
  ruckig
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(ruckig_joint_limiter SHARED
  src/ruckig_joint_limiter.cpp
)
target_compile_features(ruckig_joint_limiter PUBLIC cxx_std_17)
target_include_directories(ruckig_joint_limiter PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/joint_limits_enforcement_plugins>
)
ament_target_dependencies(ruckig_joint_limiter PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ruckig_joint_limiter PRIVATE "JOINT_LIMITS_ENFORCEMENT_PLUGINS_BUILDING_DLL")

pluginlib_export_plugin_description_file(joint_limits joint_limits_enforcement_plugins.xml)

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

install(DIRECTORY include/
  DESTINATION include/joint_limits_enforcement_plugins
)
install(
  TARGETS
    ruckig_joint_limiter
  EXPORT export_joint_limits_enforcement_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_joint_limits_enforcement_plugins HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
