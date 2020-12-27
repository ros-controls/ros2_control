cmake_minimum_required(VERSION 3.5)
project(test_robot_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_library(test_robot_hardware_components SHARED
  src/test_single_joint_actuator.cpp
  src/test_force_torque_sensor.cpp
  src/test_two_joint_system.cpp
)
ament_target_dependencies(test_robot_hardware_components
  hardware_interface
  pluginlib
)
pluginlib_export_plugin_description_file(
  hardware_interface test_robot_components.xml
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS test_robot_hardware_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES test_robot_hardware.urdf
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  hardware_interface
  rclcpp
)
ament_export_libraries(test_robot_hardware_components)
ament_export_include_directories(include)
ament_package()
