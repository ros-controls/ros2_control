cmake_minimum_required(VERSION 3.5)
project(controller_manager_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/ControllerState.msg
  msg/HardwareInterface.msg
)
set(srv_files
  srv/ConfigureController.srv
  srv/ConfigureStartController.srv
  srv/ListControllers.srv
  srv/ListControllerTypes.srv
  srv/ListHardwareInterfaces.srv
  srv/LoadConfigureController.srv
  srv/LoadController.srv
  srv/LoadStartController.srv
  srv/ReloadControllerLibraries.srv
  srv/SwitchController.srv
  srv/UnloadController.srv
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
