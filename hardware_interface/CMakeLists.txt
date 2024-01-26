cmake_minimum_required(VERSION 3.5)
project(hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    control_msgs
    lifecycle_msgs
    pluginlib
    rclcpp_lifecycle
    rcpputils
    rcutils
    tinyxml2_vendor
    TinyXML2
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  ${PROJECT_NAME}
  SHARED
  src/actuator.cpp
  src/component_parser.cpp
  src/resource_manager.cpp
  src/sensor.cpp
  src/system.cpp
  src/lexical_casts.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  include
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "HARDWARE_INTERFACE_BUILDING_DLL")

# Mock components
add_library(
  mock_components
  SHARED
  src/mock_components/generic_system.cpp
)
target_include_directories(
  mock_components
  PUBLIC
  include
)
target_link_libraries(
  mock_components
  ${PROJECT_NAME}
)
ament_target_dependencies(
  mock_components
  pluginlib
  rcpputils
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(mock_components PRIVATE "HARDWARE_INTERFACE_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  ${PROJECT_NAME} mock_components_plugin_description.xml)

# Fake components
add_library(
  fake_components
  SHARED
  src/mock_components/generic_system.cpp
  src/mock_components/fake_generic_system.cpp
)
target_include_directories(
  fake_components
  PUBLIC
  include
)
target_link_libraries(
  fake_components
  ${PROJECT_NAME}
)
ament_target_dependencies(
  fake_components
  pluginlib
  rcpputils
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(fake_components PRIVATE "HARDWARE_INTERFACE_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  ${PROJECT_NAME} fake_components_plugin_description.xml)


install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  fake_components
  mock_components
  ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)

  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_macros test/test_macros.cpp)
  target_include_directories(test_macros PRIVATE include)
  ament_target_dependencies(test_macros rcpputils)

  ament_add_gmock(test_inst_hardwares test/test_inst_hardwares.cpp)
  target_link_libraries(test_inst_hardwares hardware_interface)
  ament_target_dependencies(test_inst_hardwares rcpputils)

  ament_add_gmock(test_joint_handle test/test_handle.cpp)
  target_link_libraries(test_joint_handle ${PROJECT_NAME})
  ament_target_dependencies(test_joint_handle rcpputils)

  ament_add_gmock(test_component_interfaces test/test_component_interfaces.cpp)
  target_link_libraries(test_component_interfaces ${PROJECT_NAME})

  ament_add_gmock(test_component_parser test/test_component_parser.cpp)
  target_link_libraries(test_component_parser ${PROJECT_NAME})
  ament_target_dependencies(test_component_parser ros2_control_test_assets)

  add_library(test_hardware_components SHARED
  test/test_hardware_components/test_single_joint_actuator.cpp
  test/test_hardware_components/test_force_torque_sensor.cpp
  test/test_hardware_components/test_two_joint_system.cpp
  test/test_hardware_components/test_system_with_command_modes.cpp
  )
  target_link_libraries(test_hardware_components ${PROJECT_NAME})
  ament_target_dependencies(test_hardware_components
    pluginlib)
  install(TARGETS test_hardware_components
    DESTINATION lib
  )
  pluginlib_export_plugin_description_file(
    ${PROJECT_NAME} test/test_hardware_components/test_hardware_components.xml
  )
  ament_add_gmock(test_generic_system test/mock_components/test_generic_system.cpp)
  target_include_directories(test_generic_system PRIVATE include)
  target_link_libraries(test_generic_system ${PROJECT_NAME})
  ament_target_dependencies(test_generic_system
    pluginlib
    ros2_control_test_assets
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  fake_components
  mock_components
  ${PROJECT_NAME}
)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
