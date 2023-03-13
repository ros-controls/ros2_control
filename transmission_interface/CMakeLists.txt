cmake_minimum_required(VERSION 3.5)
project(transmission_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    hardware_interface
    pluginlib
    rclcpp
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

install(
  DIRECTORY include/
  DESTINATION include
)

add_library(${PROJECT_NAME} SHARED
  src/simple_transmission_loader.cpp
  src/four_bar_linkage_transmission_loader.cpp
  src/differential_transmission_loader.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(${PROJECT_NAME} ros2_control_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(
    test_simple_transmission
    test/simple_transmission_test.cpp
  )
  target_include_directories(test_simple_transmission PUBLIC include hardware_interface)
  ament_target_dependencies(test_simple_transmission hardware_interface)

  ament_add_gmock(
    test_differential_transmission
    test/differential_transmission_test.cpp
  )
  target_include_directories(test_differential_transmission PUBLIC include hardware_interface)
  ament_target_dependencies(test_differential_transmission hardware_interface)

  ament_add_gmock(
    test_four_bar_linkage_transmission
    test/four_bar_linkage_transmission_test.cpp
  )
  target_include_directories(test_four_bar_linkage_transmission PUBLIC include hardware_interface)
  ament_target_dependencies(test_four_bar_linkage_transmission hardware_interface)

  ament_add_gmock(
    test_simple_transmission_loader
    test/simple_transmission_loader_test.cpp
  )
  target_include_directories(test_simple_transmission_loader PUBLIC include hardware_interface)
  ament_target_dependencies(test_simple_transmission_loader hardware_interface)

  ament_add_gmock(
    test_four_bar_linkage_transmission_loader
    test/four_bar_linkage_transmission_loader_test.cpp
  )
  target_include_directories(test_four_bar_linkage_transmission_loader PUBLIC include hardware_interface)
  ament_target_dependencies(test_four_bar_linkage_transmission_loader hardware_interface)

  ament_add_gmock(
    test_differential_transmission_loader
    test/differential_transmission_loader_test.cpp
  )
  target_include_directories(test_differential_transmission_loader PUBLIC include hardware_interface)
  ament_target_dependencies(test_differential_transmission_loader hardware_interface)

  ament_add_gmock(
    test_utils
    test/utils_test.cpp
  )
  target_include_directories(test_utils PUBLIC include hardware_interface)
  ament_target_dependencies(test_utils hardware_interface)
endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  hardware_interface
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
