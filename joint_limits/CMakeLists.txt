cmake_minimum_required(VERSION 3.16)
project(joint_limits LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  urdf
)

find_package(ament_cmake REQUIRED)
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


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

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
    FILES test/joint_limits_rosparam.yaml
    DESTINATION share/joint_limits/test
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/joint_limits
)
install(TARGETS joint_limits
  EXPORT export_joint_limits
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_joint_limits HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
