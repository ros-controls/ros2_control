cmake_minimum_required(VERSION 3.16)
project(transmission_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(transmission_interface SHARED
  src/simple_transmission_loader.cpp
  src/four_bar_linkage_transmission_loader.cpp
  src/differential_transmission_loader.cpp
)
target_include_directories(transmission_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/transmission_interface>
)
target_link_libraries(transmission_interface PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
)
pluginlib_export_plugin_description_file(transmission_interface ros2_control_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_simple_transmission
    test/simple_transmission_test.cpp
  )
  target_link_libraries(test_simple_transmission transmission_interface)

  ament_add_gmock(test_differential_transmission
    test/differential_transmission_test.cpp
  )
  target_link_libraries(test_differential_transmission transmission_interface)

  ament_add_gmock(test_four_bar_linkage_transmission
    test/four_bar_linkage_transmission_test.cpp
  )
  target_link_libraries(test_four_bar_linkage_transmission transmission_interface)

  ament_add_gmock(test_simple_transmission_loader
    test/simple_transmission_loader_test.cpp
  )
  target_link_libraries(test_simple_transmission_loader transmission_interface)

  ament_add_gmock(test_four_bar_linkage_transmission_loader
    test/four_bar_linkage_transmission_loader_test.cpp
  )
  target_link_libraries(test_four_bar_linkage_transmission_loader transmission_interface)

  ament_add_gmock(test_differential_transmission_loader
    test/differential_transmission_loader_test.cpp
  )
  target_link_libraries(test_differential_transmission_loader transmission_interface)
endif()

install(
  DIRECTORY include/
  DESTINATION include/transmission_interface
)
install(TARGETS transmission_interface
  EXPORT export_transmission_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_transmission_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)
ament_package()
