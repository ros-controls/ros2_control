cmake_minimum_required(VERSION 3.16)
project(controller_manager_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/ControllerState.msg
  msg/ChainConnection.msg
  msg/HardwareComponentState.msg
  msg/HardwareInterface.msg
)
set(srv_files
  srv/ConfigureController.srv
  srv/ListControllers.srv
  srv/ListControllerTypes.srv
  srv/ListHardwareComponents.srv
  srv/ListHardwareInterfaces.srv
  srv/LoadController.srv
  srv/ReloadControllerLibraries.srv
  srv/SetHardwareComponentState.srv
  srv/SwitchController.srv
  srv/UnloadController.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces lifecycle_msgs
  ADD_LINTER_TESTS
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
