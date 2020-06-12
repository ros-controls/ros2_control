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


## COMPILE
add_library(${PROJECT_NAME} SHARED
  src/actuator.cpp
  src/robot.cpp
  src/sensor.cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE include
)

ament_target_dependencies(${PROJECT_NAME}
  control_msgs
  pluginlib
  rclcpp
)

## INSTALL
install(TARGETS ${PROJECT_NAME}
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
  control_msgs
  pluginlib
  rclcpp
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_include_directories(
include
)
ament_package()

