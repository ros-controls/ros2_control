cmake_minimum_required(VERSION 3.5)
project(transmission_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

add_library(transmission_interface SHARED
  include/transmission_interface/transmission_loader.hpp
  include/transmission_interface/simple_transmission_loader.hpp
  src/simple_transmission_loader.cpp
)

target_include_directories(transmission_interface PUBLIC include)

ament_target_dependencies(
  transmission_interface
  pluginlib
  hardware_interface
  rclcpp
)

pluginlib_export_plugin_description_file(transmission_interface ros2_control_plugins.xml)

install(TARGETS transmission_interface
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

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
endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  hardware_interface
)

ament_export_libraries(
  transmission_interface
)

ament_package()
