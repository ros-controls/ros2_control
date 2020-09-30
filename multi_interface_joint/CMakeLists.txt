cmake_minimum_required(VERSION 3.5)
project(multi_interface_joint)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(hardware_interface REQUIRED)

add_library(multi_interface_joint src/multi_interface_joint.cpp)
target_include_directories(multi_interface_joint PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  multi_interface_joint
  "hardware_interface"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(multi_interface_joint PRIVATE "MULTI_INTERFACE_JOINT_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS multi_interface_joint
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(test_multi_interface_joint
    test/test_multi_interface_joint.cpp
  )
  target_link_libraries(test_multi_interface_joint multi_interface_joint)
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  multi_interface_joint
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
