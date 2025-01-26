cmake_minimum_required(VERSION 3.16)
project(controller_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow
          -Werror=missing-braces)
endif()

# using this instead of visibility macros
# S1 from https://github.com/ros-controls/ros2_controllers/issues/1053
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  rclcpp_lifecycle
  realtime_tools
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gen_version_h REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(controller_interface SHARED
  src/controller_interface_base.cpp
  src/controller_interface.cpp
  src/chainable_controller_interface.cpp
)
target_compile_features(controller_interface PUBLIC cxx_std_17)
target_include_directories(controller_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/controller_interface>
)
ament_target_dependencies(controller_interface PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)

  ament_add_gmock(test_controller_interface test/test_controller_interface.cpp)
  target_link_libraries(test_controller_interface
    controller_interface
  )

  ament_add_gmock(test_controller_with_options test/test_controller_with_options.cpp)
  target_link_libraries(test_controller_with_options
    controller_interface
  )
  target_compile_definitions(
    test_controller_with_options
    PRIVATE PARAMETERS_FILE_PATH="${CMAKE_CURRENT_LIST_DIR}/test/")

  ament_add_gmock(test_chainable_controller_interface test/test_chainable_controller_interface.cpp)
  target_link_libraries(test_chainable_controller_interface
    controller_interface
    hardware_interface::hardware_interface
  )

  ament_add_gmock(test_semantic_component_interface test/test_semantic_component_interface.cpp)
  target_link_libraries(test_semantic_component_interface
    controller_interface
    hardware_interface::hardware_interface
  )

  ament_add_gmock(test_force_torque_sensor test/test_force_torque_sensor.cpp)
  target_link_libraries(test_force_torque_sensor
    controller_interface
    hardware_interface::hardware_interface
  )

  ament_add_gmock(test_imu_sensor test/test_imu_sensor.cpp)
  target_link_libraries(test_imu_sensor
    controller_interface
    hardware_interface::hardware_interface
  )
  ament_target_dependencies(test_imu_sensor
    sensor_msgs
  )

  ament_add_gmock(test_pose_sensor test/test_pose_sensor.cpp)
  target_link_libraries(test_pose_sensor
    controller_interface
    hardware_interface::hardware_interface
  )
  ament_target_dependencies(test_pose_sensor
    geometry_msgs
  )

  ament_add_gmock(test_gps_sensor test/test_gps_sensor.cpp)
  target_link_libraries(test_gps_sensor
    controller_interface
    hardware_interface::hardware_interface
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/controller_interface
)
install(TARGETS controller_interface
  EXPORT export_controller_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_controller_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
ament_generate_version_header(${PROJECT_NAME})
