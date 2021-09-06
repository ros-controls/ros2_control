cmake_minimum_required(VERSION 3.5)
project(controller_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(controller_interface SHARED src/controller_interface.cpp)
target_include_directories(
  controller_interface
  PRIVATE
  include)
ament_target_dependencies(
  controller_interface
  hardware_interface
  rclcpp_lifecycle
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(controller_interface PRIVATE "CONTROLLER_INTERFACE_BUILDING_DLL")

install(DIRECTORY include/
  DESTINATION include
)
install(TARGETS controller_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  find_package(hardware_interface REQUIRED)
  find_package(sensor_msgs REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(test_controller_with_options test/test_controller_with_options.cpp)
  target_link_libraries(test_controller_with_options controller_interface)
  target_include_directories(test_controller_with_options PRIVATE include)

  ament_add_gmock(
    test_semantic_component_interface
    test/test_semantic_component_interface.cpp
  )
  target_include_directories(test_semantic_component_interface PRIVATE include)
  ament_target_dependencies(
    test_semantic_component_interface
    hardware_interface
  )

  ament_add_gmock(
    test_force_torque_sensor
    test/test_force_torque_sensor.cpp
  )
  target_include_directories(test_force_torque_sensor PRIVATE include)
  ament_target_dependencies(
    test_force_torque_sensor
    hardware_interface
  )

  ament_add_gmock(
    test_imu_sensor
    test/test_imu_sensor.cpp
  )
  target_include_directories(test_imu_sensor PRIVATE include)
  ament_target_dependencies(
    test_imu_sensor
    hardware_interface
    sensor_msgs
  )
endif()

ament_export_dependencies(
  hardware_interface
  rclcpp_lifecycle
  sensor_msgs
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  controller_interface
)
ament_package()
