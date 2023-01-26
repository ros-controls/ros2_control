cmake_minimum_required(VERSION 3.16)
project(joint_limits LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
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
target_link_libraries(joint_limits INTERFACE
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest_executable(joint_limits_rosparam_test test/joint_limits_rosparam_test.cpp)
  target_link_libraries(joint_limits_rosparam_test joint_limits)

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
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
)
ament_package()
