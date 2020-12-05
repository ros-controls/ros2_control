cmake_minimum_required(VERSION 3.5)
project(hardware_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

add_library(
  hardware_interface
  SHARED
  src/actuator.cpp
  src/component_parser.cpp
  src/resource_manager.cpp
  src/sensor.cpp
  src/system.cpp
)
target_include_directories(
  hardware_interface
  PUBLIC
  include
)
ament_target_dependencies(
  hardware_interface
  control_msgs
  pluginlib
  rcutils
  rcpputils
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(hardware_interface PRIVATE "HARDWARE_INTERFACE_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(hardware_interface PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  hardware_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_macros test/test_macros.cpp)
  target_include_directories(test_macros PRIVATE include)
  ament_target_dependencies(test_macros rcpputils)

  ament_add_gmock(test_joint_handle test/test_handle.cpp)
  target_link_libraries(test_joint_handle hardware_interface)
  ament_target_dependencies(test_joint_handle rcpputils)

  ament_add_gmock(test_component_interfaces test/test_component_interfaces.cpp)
  target_link_libraries(test_component_interfaces hardware_interface)

  ament_add_gmock(test_component_parser test/test_component_parser.cpp)
  target_link_libraries(test_component_parser hardware_interface)

  add_library(test_components SHARED
    test/test_components/test_actuator.cpp
    test/test_components/test_sensor.cpp
    test/test_components/test_system.cpp)
  target_link_libraries(test_components hardware_interface)
  ament_target_dependencies(test_components
    pluginlib)
  install(TARGETS test_components
    DESTINATION lib
  )
  pluginlib_export_plugin_description_file(
    hardware_interface test/test_components/test_components.xml)

  ament_add_gmock(
    test_resource_manager
    test/test_resource_manager.cpp
  )
  target_link_libraries(test_resource_manager hardware_interface)
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  hardware_interface
)
ament_export_dependencies(
  control_msgs
  pluginlib
  rcpputils
  tinyxml2_vendor
  TinyXML2
)
ament_package()
