cmake_minimum_required(VERSION 3.5)
project(controller_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(controller_manager SHARED
  src/controller_manager.cpp
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

add_executable(ros2_control_node src/ros2_control_node.cpp)
target_include_directories(ros2_control_node PRIVATE include)
target_link_libraries(ros2_control_node controller_manager)
ament_target_dependencies(ros2_control_node
  controller_interface
  hardware_interface
  rclcpp
)

install(TARGETS controller_manager
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(TARGETS ros2_control_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/spawner
  scripts/spawner.py  # Deprecated
  scripts/unspawner
  scripts/unspawner.py  # Deprecated
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
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

  add_library(test_controller_failed_init SHARED test/test_controller_failed_init/test_controller_failed_init.cpp)
  target_include_directories(test_controller_failed_init PRIVATE include)
  target_link_libraries(test_controller_failed_init controller_manager)
  target_compile_definitions(test_controller_failed_init PRIVATE "CONTROLLER_MANAGER_BUILDING_DLL")
  pluginlib_export_plugin_description_file(
    controller_interface test/test_controller_failed_init/test_controller_failed_init.xml)
  install(TARGETS test_controller_failed_init
    DESTINATION lib
  )

  ament_add_gmock(
    test_controller_manager
    test/test_controller_manager.cpp
  )
  target_include_directories(test_controller_manager PRIVATE include)
  target_link_libraries(test_controller_manager controller_manager test_controller)
  ament_target_dependencies(test_controller_manager ros2_control_test_assets)

  ament_add_gmock(
    test_load_controller
    test/test_load_controller.cpp
    APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}_$<CONFIG>
  )
  target_include_directories(test_load_controller PRIVATE include)
  target_link_libraries(test_load_controller controller_manager test_controller test_controller_failed_init)
  ament_target_dependencies(test_load_controller ros2_control_test_assets)

  ament_add_gmock(
    test_controller_manager_srvs
    test/test_controller_manager_srvs.cpp
    APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}_$<CONFIG>
  )
  target_include_directories(test_controller_manager_srvs PRIVATE include)
  target_link_libraries(test_controller_manager_srvs controller_manager test_controller)
  ament_target_dependencies(
    test_controller_manager_srvs
    controller_manager_msgs
    ros2_control_test_assets
  )

  add_library(test_controller_with_interfaces SHARED test/test_controller_with_interfaces/test_controller_with_interfaces.cpp)
  target_include_directories(test_controller_with_interfaces PRIVATE include)
  target_link_libraries(test_controller_with_interfaces controller_manager)
  target_compile_definitions(test_controller_with_interfaces PRIVATE "CONTROLLER_MANAGER_BUILDING_DLL")
  pluginlib_export_plugin_description_file(
    controller_interface test/test_controller_with_interfaces/test_controller_with_interfaces.xml)
  install(TARGETS test_controller_with_interfaces
    DESTINATION lib
  )

  ament_add_gmock(
    test_release_interfaces
    test/test_release_interfaces.cpp
    APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}_$<CONFIG>
  )
  target_include_directories(test_release_interfaces PRIVATE include)
  target_link_libraries(test_release_interfaces controller_manager test_controller_with_interfaces)
  ament_target_dependencies(test_release_interfaces ros2_control_test_assets)

  ament_add_gmock(
    test_spawner_unspawner
    test/test_spawner_unspawner.cpp
  )
  target_include_directories(test_spawner_unspawner PRIVATE include)
  target_link_libraries(test_spawner_unspawner controller_manager test_controller)
  ament_target_dependencies(test_spawner_unspawner ros2_control_test_assets)
endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_export_libraries(
  controller_manager
)
ament_export_include_directories(
  include
)
ament_export_dependencies(
  controller_interface
  controller_manager_msgs
  hardware_interface
  pluginlib
  rclcpp
)
ament_package()
