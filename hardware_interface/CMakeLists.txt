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
find_package(urdf REQUIRED)

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
  urdf
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(hardware_interface PRIVATE "HARDWARE_INTERFACE_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(hardware_interface PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Fake components
add_library(
  fake_components
  SHARED
  src/fake_components/generic_system.cpp
)
target_include_directories(
  fake_components
  PUBLIC
  include
)
target_link_libraries(
  fake_components
  hardware_interface
)
ament_target_dependencies(
  fake_components
  pluginlib
  rcpputils
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(fake_components PRIVATE "HARDWARE_INTERFACE_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(fake_components PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface fake_components_plugin_description.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  fake_components
  hardware_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)

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
  ament_target_dependencies(test_component_parser ros2_control_test_assets)

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

  add_library(test_hardware_components SHARED
  test/test_hardware_components/test_single_joint_actuator.cpp
  test/test_hardware_components/test_force_torque_sensor.cpp
  test/test_hardware_components/test_two_joint_system.cpp
  )
  target_link_libraries(test_hardware_components hardware_interface)
  ament_target_dependencies(test_hardware_components
    pluginlib)
  install(TARGETS test_hardware_components
    DESTINATION lib
  )
  pluginlib_export_plugin_description_file(
    hardware_interface test/test_hardware_components/test_hardware_components.xml
  )

  ament_add_gmock(test_resource_manager test/test_resource_manager.cpp)
  target_link_libraries(test_resource_manager hardware_interface)
  ament_target_dependencies(test_resource_manager ros2_control_test_assets)

  ament_add_gmock(test_generic_system test/fake_components/test_generic_system.cpp)
  target_include_directories(test_generic_system PRIVATE include)
  target_link_libraries(test_generic_system hardware_interface)
  ament_target_dependencies(test_generic_system
    pluginlib
    ros2_control_test_assets
  )

  # joint_limit_interface tests
  ament_add_gmock(joint_limits_interface_test test/joint_limits/joint_limits_interface_test.cpp)
  target_include_directories(joint_limits_interface_test PUBLIC include)
  ament_target_dependencies(joint_limits_interface_test rclcpp)

  add_executable(joint_limits_rosparam_test test/joint_limits/joint_limits_rosparam_test.cpp)
  target_include_directories(joint_limits_rosparam_test PUBLIC include ${GTEST_INCLUDE_DIRS})
  target_link_libraries(joint_limits_rosparam_test ${GTEST_LIBRARIES})
  ament_target_dependencies(joint_limits_rosparam_test rclcpp)
  add_launch_test(test/joint_limits/joint_limits_rosparam.launch.py)
  install(
          TARGETS
          joint_limits_rosparam_test
          DESTINATION lib/${PROJECT_NAME}
  )
  install(
          FILES
          test/joint_limits/joint_limits_rosparam.yaml
          DESTINATION share/${PROJECT_NAME}/test
  )

  ament_add_gmock(joint_limits_urdf_test test/joint_limits/joint_limits_urdf_test.cpp)
  target_include_directories(joint_limits_urdf_test PUBLIC include)
  ament_target_dependencies(joint_limits_urdf_test rclcpp urdf)
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  fake_components
  hardware_interface
)
ament_export_dependencies(
  control_msgs
  pluginlib
  rcpputils
  tinyxml2_vendor
  TinyXML2
  urdf
)
ament_package()
