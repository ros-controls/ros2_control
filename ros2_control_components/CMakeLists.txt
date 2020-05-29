cmake_minimum_required(VERSION 3.5)
project(ros2_control_components)

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

find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_control_core REQUIRED)
# # find_package(ros2_control_hardware REQUIRED)


## COMPILE
#Actuators
add_library(${PROJECT_NAME}_actuators SHARED
  src/actuators/position_actuator.cpp
  src/actuators/velocity_actuator.cpp
)

target_include_directories(${PROJECT_NAME}_actuators
  PRIVATE include
)

ament_target_dependencies(${PROJECT_NAME}_actuators
  pluginlib
  rclcpp
  ros2_control_core
)

#Sensors
add_library(${PROJECT_NAME}_sensors SHARED
  src/sensors/position_sensor.cpp
  src/sensors/velocity_sensor.cpp
)

target_include_directories(${PROJECT_NAME}_sensors
  PRIVATE include
)

ament_target_dependencies(${PROJECT_NAME}_sensors
  pluginlib
  rclcpp
  ros2_control_core
)


pluginlib_export_plugin_description_file(ros2_control_components ros2_control_components.xml)

## INSTALL
install(TARGETS ${PROJECT_NAME}_actuators ${PROJECT_NAME}_sensors
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

## TESTS
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_dummy test/test_dummy.cpp)
  target_include_directories(test_dummy PRIVATE include)
#   ament_target_dependencies(test_dummy rcpputils)


  # Uncomment this before creating a new PR to check code quality
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_dependencies(
  pluginlib
  rclcpp
  ros2_control_core
)
ament_export_libraries(
  ${PROJECT_NAME}_actuators
  ${PROJECT_NAME}_sensors
)
ament_export_include_directories(
  include
)
ament_package()

