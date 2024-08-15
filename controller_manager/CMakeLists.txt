cmake_minimum_required(VERSION 3.5)
project(controller_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_index_cpp
    controller_interface
    controller_manager_msgs
    hardware_interface
    pluginlib
    rclcpp
    realtime_tools
    std_msgs
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/controller_manager.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "CONTROLLER_MANAGER_BUILDING_DLL")

add_executable(ros2_control_node src/ros2_control_node.cpp)
target_include_directories(ros2_control_node PRIVATE include)
target_link_libraries(ros2_control_node
  ${PROJECT_NAME}
)
ament_target_dependencies(ros2_control_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS ${PROJECT_NAME}
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

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  add_library(test_controller SHARED test/test_controller/test_controller.cpp)
  target_include_directories(test_controller PRIVATE include)
  target_link_libraries(test_controller ${PROJECT_NAME})
  target_compile_definitions(test_controller PRIVATE "CONTROLLER_MANAGER_BUILDING_DLL")
  pluginlib_export_plugin_description_file(
    controller_interface test/test_controller/test_controller.xml)
  install(TARGETS test_controller
    DESTINATION lib
  )

  add_library(test_controller_failed_init SHARED test/test_controller_failed_init/test_controller_failed_init.cpp)
  target_include_directories(test_controller_failed_init PRIVATE include)
  target_link_libraries(test_controller_failed_init ${PROJECT_NAME})
  target_compile_definitions(test_controller_failed_init PRIVATE "CONTROLLER_MANAGER_BUILDING_DLL")
  pluginlib_export_plugin_description_file(
    controller_interface test/test_controller_failed_init/test_controller_failed_init.xml)
  install(TARGETS test_controller_failed_init
    DESTINATION lib
  )

  add_library(test_chainable_controller SHARED test/test_chainable_controller/test_chainable_controller.cpp)
  ament_target_dependencies(test_chainable_controller realtime_tools)
  target_include_directories(test_chainable_controller PRIVATE include)
  target_link_libraries(test_chainable_controller controller_manager)
  target_compile_definitions(test_chainable_controller PRIVATE "CONTROLLER_MANAGER_BUILDING_DLL")
  pluginlib_export_plugin_description_file(
    controller_interface test/test_chainable_controller/test_chainable_controller.xml)
  install(TARGETS test_chainable_controller
    DESTINATION lib
  )

  ament_add_gmock(
    test_controller_manager
    test/test_controller_manager.cpp
  )
  target_include_directories(test_controller_manager PRIVATE include)
  target_link_libraries(test_controller_manager ${PROJECT_NAME} test_controller)
  ament_target_dependencies(test_controller_manager ros2_control_test_assets)

  ament_add_gmock(
    test_controller_manager_with_namespace.cpp
    test/test_controller_manager_with_namespace.cpp
  )
  target_include_directories(test_controller_manager_with_namespace.cpp PRIVATE include)
  target_link_libraries(test_controller_manager_with_namespace.cpp ${PROJECT_NAME} test_controller)
  ament_target_dependencies(test_controller_manager_with_namespace.cpp ros2_control_test_assets)

  ament_add_gmock(
    test_load_controller
    test/test_load_controller.cpp
    APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}_$<CONFIG>
  )
  target_include_directories(test_load_controller PRIVATE include)
  target_link_libraries(test_load_controller ${PROJECT_NAME} test_controller test_controller_failed_init)
  ament_target_dependencies(test_load_controller ros2_control_test_assets)

  ament_add_gmock(
    test_controllers_chaining_with_controller_manager
    test/test_controllers_chaining_with_controller_manager.cpp
  )
  target_include_directories(test_controllers_chaining_with_controller_manager PRIVATE include)
  target_link_libraries(test_controllers_chaining_with_controller_manager controller_manager test_chainable_controller test_controller)
  ament_target_dependencies(test_controllers_chaining_with_controller_manager ros2_control_test_assets)

  ament_add_gmock(
    test_controller_manager_srvs
    test/test_controller_manager_srvs.cpp
    APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}_$<CONFIG>
  )
  target_include_directories(test_controller_manager_srvs PRIVATE include)
  target_link_libraries(test_controller_manager_srvs ${PROJECT_NAME} test_controller test_chainable_controller)
  ament_target_dependencies(
    test_controller_manager_srvs
    controller_manager_msgs
    ros2_control_test_assets
  )
  set_tests_properties(test_controller_manager_srvs PROPERTIES TIMEOUT 120)
  ament_add_gmock(test_controller_manager_urdf_passing
    test/test_controller_manager_urdf_passing.cpp
  )
  target_include_directories(test_controller_manager_urdf_passing PRIVATE include)
  target_link_libraries(test_controller_manager_urdf_passing
    controller_manager
  )
  ament_target_dependencies(test_controller_manager_urdf_passing
    controller_manager_msgs
    ros2_control_test_assets
  )

  add_library(test_controller_with_interfaces SHARED test/test_controller_with_interfaces/test_controller_with_interfaces.cpp)
  target_include_directories(test_controller_with_interfaces PRIVATE include)
  target_link_libraries(test_controller_with_interfaces ${PROJECT_NAME})
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
  target_link_libraries(test_release_interfaces ${PROJECT_NAME} test_controller_with_interfaces)
  ament_target_dependencies(test_release_interfaces ros2_control_test_assets)

  ament_add_gmock(
    test_spawner_unspawner
    test/test_spawner_unspawner.cpp
    TIMEOUT 120
  )
  target_include_directories(test_spawner_unspawner PRIVATE include)
  target_link_libraries(test_spawner_unspawner ${PROJECT_NAME} test_controller)
  ament_target_dependencies(test_spawner_unspawner ros2_control_test_assets)

  install(FILES test/test_controller_spawner_with_type.yaml
  DESTINATION test)

  ament_add_gmock(
    test_hardware_management_srvs
    test/test_hardware_management_srvs.cpp
  )
  target_include_directories(test_hardware_management_srvs PRIVATE include)
  target_link_libraries(test_hardware_management_srvs ${PROJECT_NAME} test_controller)
  ament_target_dependencies(
    test_hardware_management_srvs
    controller_manager_msgs
    ros2_control_test_assets
  )

  find_package(ament_cmake_pytest REQUIRED)
  install(FILES test/test_ros2_control_node.yaml
    DESTINATION test)
  ament_add_pytest_test(test_ros2_control_node test/test_ros2_control_node_launch.py)
endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_package()
