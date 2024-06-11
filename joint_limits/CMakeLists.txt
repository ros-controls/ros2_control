cmake_minimum_required(VERSION 3.16)
project(joint_limits LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow
          -Werror=missing-braces)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  pluginlib
  realtime_tools
  rclcpp
  rclcpp_lifecycle
  trajectory_msgs
  urdf
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(ament_cmake_gen_version_h REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(joint_limits INTERFACE)
target_compile_features(joint_limits INTERFACE cxx_std_17)
target_include_directories(joint_limits INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/joint_limits>
)
ament_target_dependencies(joint_limits INTERFACE ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_library(joint_limiter_interface SHARED
  src/joint_limiter_interface.cpp
)
target_compile_features(joint_limiter_interface PUBLIC cxx_std_17)
target_include_directories(joint_limiter_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/joint_limits>
)
ament_target_dependencies(joint_limiter_interface PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(joint_limiter_interface PRIVATE "JOINT_LIMITS_BUILDING_DLL")


add_library(joint_saturation_limiter SHARED
  src/joint_saturation_limiter.cpp
)
target_compile_features(joint_saturation_limiter PUBLIC cxx_std_17)
target_include_directories(joint_saturation_limiter PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/joint_limits>
)
ament_target_dependencies(joint_saturation_limiter PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(joint_saturation_limiter PRIVATE "JOINT_LIMITS_BUILDING_DLL")

pluginlib_export_plugin_description_file(joint_limits joint_limiters.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(generate_parameter_library REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)

  ament_add_gtest_executable(joint_limits_rosparam_test test/joint_limits_rosparam_test.cpp)
  target_link_libraries(joint_limits_rosparam_test joint_limits)

  ament_add_gtest(joint_limits_urdf_test test/joint_limits_urdf_test.cpp)
  target_link_libraries(joint_limits_urdf_test joint_limits)

  add_launch_test(test/joint_limits_rosparam.launch.py)
  install(
    TARGETS joint_limits_rosparam_test
    DESTINATION lib/joint_limits
  )
  install(
    FILES test/joint_limits_rosparam.yaml test/joint_saturation_limiter_param.yaml
    DESTINATION share/joint_limits/test
  )

  add_rostest_with_parameters_gmock(test_joint_saturation_limiter test/test_joint_saturation_limiter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/joint_saturation_limiter_param.yaml
  )
  target_include_directories(test_joint_saturation_limiter PRIVATE include)
  target_link_libraries(test_joint_saturation_limiter joint_limiter_interface)
  ament_target_dependencies(
    test_joint_saturation_limiter
    pluginlib
    rclcpp
  )

endif()

install(
  DIRECTORY include/
  DESTINATION include/joint_limits
)
install(TARGETS
    joint_limits
    joint_limiter_interface
    joint_saturation_limiter
  EXPORT export_joint_limits
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_joint_limits HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
ament_generate_version_header(${PROJECT_NAME})
