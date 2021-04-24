cmake_minimum_required(VERSION 3.5)
project(ros2_control_test_assets)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
endif()

ament_export_include_directories(
  include
)
ament_package()
