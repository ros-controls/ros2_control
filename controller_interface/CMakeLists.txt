cmake_minimum_required(VERSION 3.16)
project(controller_interface LANGUAGES CXX)

<<<<<<< HEAD
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
=======
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
>>>>>>> 727297e (Fix CMake install so overriding works (#926))
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
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
  find_package(sensor_msgs REQUIRED)

  ament_add_gmock(test_controller_interface test/test_controller_interface.cpp)
  target_link_libraries(test_controller_interface
    controller_interface
  )

  ament_add_gmock(test_controller_with_options test/test_controller_with_options.cpp)
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

<<<<<<< HEAD
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
=======
ament_export_targets(export_controller_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
>>>>>>> 727297e (Fix CMake install so overriding works (#926))
ament_package()
ament_generate_version_header(${PROJECT_NAME})
