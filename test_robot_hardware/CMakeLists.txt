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

add_library(test_robot_hardware SHARED src/test_robot_hardware.cpp)
target_include_directories(test_robot_hardware PRIVATE include)
ament_target_dependencies(
  test_robot_hardware
  hardware_interface
  rclcpp
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(test_robot_hardware PRIVATE "TEST_ROBOT_HARDWARE_BUILDING_DLL")

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS test_robot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_robot_hardware_interface test/test_robot_hardware_interface.cpp)
  if(TARGET test_robot_hardware_interface)
    target_include_directories(test_robot_hardware_interface PRIVATE include)
    target_link_libraries(
      test_robot_hardware_interface
      test_robot_hardware
    )
  endif()
endif()

ament_export_dependencies(
  hardware_interface
  rclcpp
)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_package()
