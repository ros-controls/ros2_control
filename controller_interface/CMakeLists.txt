cmake_minimum_required(VERSION 3.5)
project(controller_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/controller_interface_base.cpp
  src/controller_interface.cpp
  src/chainable_controller_interface.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  rclcpp_lifecycle
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "CONTROLLER_INTERFACE_BUILDING_DLL")

install(DIRECTORY include/
  DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  find_package(hardware_interface REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)

  ament_add_gmock(test_controller_interface test/test_controller_interface.cpp)
  target_link_libraries(test_controller_interface ${PROJECT_NAME})
  target_include_directories(test_controller_interface PRIVATE include)

  ament_add_gmock(test_controller_with_options test/test_controller_with_options.cpp)
  target_link_libraries(test_controller_with_options ${PROJECT_NAME})
  target_include_directories(test_controller_with_options PRIVATE include)

  ament_add_gmock(test_chainable_controller_interface test/test_chainable_controller_interface.cpp)
  target_link_libraries(test_chainable_controller_interface ${PROJECT_NAME})
  target_include_directories(test_chainable_controller_interface PRIVATE include)
  ament_target_dependencies(test_chainable_controller_interface hardware_interface)

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

  ament_add_gmock(test_pose_sensor test/test_pose_sensor.cpp)
  target_include_directories(test_pose_sensor PRIVATE include)
  ament_target_dependencies(test_pose_sensor
    hardware_interface
    geometry_msgs
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
  ${PROJECT_NAME}
)
ament_package()
