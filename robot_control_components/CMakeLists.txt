cmake_minimum_required(VERSION 3.5)
project(robot_control_components)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(${PROJECT_NAME} SHARED
  # actuators
  src/effort_actuator.cpp
  src/position_actuator.cpp
  src/velocity_actuator.cpp
  # sensors
  src/position_sensor.cpp
  src/velocity_sensor.cpp
)
target_include_directories(${PROJECT_NAME}
  PRIVATE include
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(
  robot_control_components robot_control_components_plugins.xml)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  hardware_interface
  pluginlib
)
ament_package()
