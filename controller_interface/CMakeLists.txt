cmake_minimum_required(VERSION 3.16)
project(controller_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow
          -Werror=missing-braces)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  rclcpp_lifecycle
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
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(controller_interface PRIVATE "CONTROLLER_INTERFACE_BUILDING_DLL")

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)

  ament_add_gmock(test_controller_interface test/test_controller_interface.cpp)
  target_link_libraries(test_controller_interface
    controller_interface
  )

  ament_add_gmock(test_controller_with_options test/test_controller_with_options.cpp)
  install(FILES test/test_controller_node_options.yaml
      DESTINATION test)
  target_link_libraries(test_controller_with_options
    controller_interface
  )

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
