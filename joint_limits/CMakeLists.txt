cmake_minimum_required(VERSION 3.5)
project(joint_limits)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake)
  find_package(rclcpp)

  add_executable(joint_limits_rosparam_test test/joint_limits_rosparam_test.cpp)
  target_include_directories(joint_limits_rosparam_test PUBLIC include ${GTEST_INCLUDE_DIRS})
  target_link_libraries(joint_limits_rosparam_test ${GTEST_LIBRARIES})
  ament_target_dependencies(joint_limits_rosparam_test rclcpp ament_cmake_gtest)
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

ament_export_dependencies(
  rclcpp
)

ament_export_include_directories(
  include
)

ament_package()
