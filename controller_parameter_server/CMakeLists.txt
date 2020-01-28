cmake_minimum_required(VERSION 3.5)
project(controller_parameter_server)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_library(yaml_parser SHARED src/yaml_parser.cpp)
target_include_directories(yaml_parser PUBLIC include)
target_compile_definitions(yaml_parser PRIVATE "CONTROLLER_PARAMETER_SERVER_BUILDING_DLL")
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_link_libraries(yaml_parser "stdc++fs")
endif()
ament_target_dependencies(yaml_parser yaml_cpp_vendor)

add_library(parameter_server SHARED src/parameter_server.cpp)
target_include_directories(parameter_server PUBLIC include)
target_link_libraries(parameter_server yaml_parser)
target_compile_definitions(parameter_server PRIVATE "CONTROLLER_PARAMETER_SERVER_BUILDING_DLL")
ament_target_dependencies(
  parameter_server
  rclcpp)

add_executable(controller_parameter_server src/controller_parameter_server.cpp)
target_include_directories(controller_parameter_server PRIVATE include)
target_link_libraries(controller_parameter_server parameter_server)

install(
  TARGETS parameter_server yaml_parser
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS controller_parameter_server
  DESTINATION lib/${PROJECT_NAME}/
)
install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(test_config_file ${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_controller_config.yaml)

  ament_add_gtest(
    test_yaml_parser test/test_yaml_parser.cpp
    ENV config_file=${test_config_file})
  target_link_libraries(test_yaml_parser yaml_parser)
  ament_target_dependencies(test_yaml_parser rcutils)

  ament_add_gtest(
    test_parameter_server test/test_parameter_server.cpp
    ENV config_file=${test_config_file})
  target_link_libraries(test_parameter_server parameter_server)
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  parameter_server
  yaml_parser
)
ament_export_dependencies(
  yaml-cpp
)
ament_package()
