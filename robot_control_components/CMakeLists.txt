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

find_package(control_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

add_library(component_parser SHARED
  src/utils/component_parser.cpp
)

target_include_directories(component_parser
  PRIVATE include
)

ament_target_dependencies(component_parser
  TinyXML2
)

add_library(${PROJECT_NAME} SHARED
  src/actuator.cpp
  src/robot.cpp
  src/sensor.cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE include
)

target_link_libraries(${PROJECT_NAME}
  component_parser
)

ament_target_dependencies(${PROJECT_NAME}
  control_msgs
  pluginlib
  rclcpp
)

add_library(${PROJECT_NAME}_actuators SHARED
  src/impl/effort_actuator.cpp
  src/impl/position_actuator.cpp
  src/impl/velocity_actuator.cpp
)

target_include_directories(${PROJECT_NAME}_actuators
  PRIVATE include
)

ament_target_dependencies(${PROJECT_NAME}_actuators
  control_msgs
  pluginlib
  rclcpp
)

add_library(${PROJECT_NAME}_sensors SHARED
  src/impl/position_sensor.cpp
  src/impl/velocity_sensor.cpp
)

target_include_directories(${PROJECT_NAME}_sensors
  PRIVATE include
)

ament_target_dependencies(${PROJECT_NAME}_sensors
  control_msgs
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(robot_control_components robot_control_components_plugins.xml)

install(TARGETS
  component_parser
  ${PROJECT_NAME}
  ${PROJECT_NAME}_actuators
  ${PROJECT_NAME}_sensors
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(test_component_parser test/test_component_parser.cpp)
  target_include_directories(test_component_parser PRIVATE include)
  target_link_libraries(test_component_parser component_parser)
endif()

ament_export_dependencies(
  control_msgs
  pluginlib
  rclcpp
)
ament_export_libraries(
  component_parser
  ${PROJECT_NAME}
  ${PROJECT_NAME}_actuators
  ${PROJECT_NAME}_sensors
)
ament_export_include_directories(
include
)
ament_package()

