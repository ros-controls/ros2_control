cmake_minimum_required(VERSION 3.5)
project(controller_manager)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake)
find_package(ament_cmake_core REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(controller_manager SHARED
  src/controller_manager.cpp
  src/resource_manager.cpp
)
target_include_directories(controller_manager PRIVATE include)
ament_target_dependencies(controller_manager
  ament_index_cpp
  controller_interface
  controller_manager_msgs
  hardware_interface
  pluginlib
  rclcpp
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(controller_manager PRIVATE "CONTROLLER_MANAGER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(controller_manager PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(TARGETS controller_manager
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_index_get_prefix_path(ament_index_build_path SKIP_AMENT_PREFIX_PATH)
  # Get the first item (it will be the build space version of the build path).
  list(GET ament_index_build_path 0 ament_index_build_path)
  if(WIN32)
    # On Windows prevent CMake errors and prevent it being evaluated as a list.
    string(REPLACE "\\" "/" ament_index_build_path "${ament_index_build_path}")
  endif()

  add_library(test_controller SHARED test/test_controller/test_controller.cpp)
  target_include_directories(test_controller PRIVATE include)
  target_link_libraries(test_controller controller_manager)
  target_compile_definitions(test_controller PRIVATE "CONTROLLER_MANAGER_BUILDING_DLL")
  pluginlib_export_plugin_description_file(
    controller_interface test/test_controller/test_controller.xml)
  install(TARGETS test_controller
    DESTINATION lib
  )

  ament_add_gmock(
    test_controller_manager
    test/test_controller_manager.cpp
  )
  target_include_directories(test_controller_manager PRIVATE include)
  target_link_libraries(test_controller_manager controller_manager test_controller)
  ament_target_dependencies(
    test_controller_manager
    test_robot_hardware
  )

  ament_add_gmock(
    test_load_controller
    test/test_load_controller.cpp
    APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}_$<CONFIG>
  )
  target_include_directories(test_load_controller PRIVATE include)
  target_link_libraries(test_load_controller controller_manager)
  ament_target_dependencies(
    test_load_controller
    test_robot_hardware
  )

  ament_add_gmock(
    test_controller_manager_srvs
    test/test_controller_manager_srvs.cpp
    APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}_$<CONFIG>
  )
  target_include_directories(test_controller_manager_srvs PRIVATE include)
  target_link_libraries(test_controller_manager_srvs controller_manager test_controller)
  ament_target_dependencies(
    test_controller_manager_srvs
    test_robot_hardware
  )

  add_library(test_components SHARED
    test/test_components/test_actuator.cpp
    test/test_components/test_sensor.cpp
    test/test_components/test_system.cpp)
  ament_target_dependencies(test_components
    hardware_interface
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
  target_include_directories(test_resource_manager PRIVATE include src)
  target_link_libraries(test_resource_manager controller_manager)
endif()

ament_export_libraries(
  controller_manager
)
ament_export_include_directories(
  include
)
ament_export_dependencies(
  controller_interface
  controller_manager_msgs
  pluginlib
)
ament_package()
