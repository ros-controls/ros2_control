cmake_minimum_required(VERSION 3.5)
project(joint_limits)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra  -Wconversion)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(joint_limits INTERFACE)
target_compile_features(joint_limits INTERFACE cxx_std_17)
target_include_directories(joint_limits INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/joint_limits>
)
ament_target_dependencies(joint_limits INTERFACE ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake)
  find_package(rclcpp)
  find_package(rclcpp_lifecycle)

  ament_add_gtest_executable(joint_limits_rosparam_test test/joint_limits_rosparam_test.cpp)
  target_include_directories(joint_limits_rosparam_test PRIVATE include)
  ament_target_dependencies(joint_limits_rosparam_test rclcpp rclcpp_lifecycle)

  add_launch_test(test/joint_limits_rosparam.launch.py)
  install(
    TARGETS
    joint_limits_rosparam_test
    DESTINATION lib/${PROJECT_NAME}
  )
  install(
    FILES
    test/joint_limits_rosparam.yaml
    DESTINATION share/${PROJECT_NAME}/test
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
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
)

ament_export_include_directories(
  include
)

ament_package()
ament_generate_version_header(${PROJECT_NAME})
