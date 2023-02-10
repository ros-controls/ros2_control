cmake_minimum_required(VERSION 3.16)
project(transmission_interface LANGUAGES CXX)

<<<<<<< HEAD
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type
  # deactivating this, because of issue in upstream class_loader_imp.hpp
  # -Werror=shadow
)
=======
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
>>>>>>> 727297e (Fix CMake install so overriding works (#926))
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

add_library(transmission_interface SHARED
  src/simple_transmission_loader.cpp
  src/four_bar_linkage_transmission_loader.cpp
  src/differential_transmission_loader.cpp
)
target_include_directories(transmission_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/transmission_interface>
)
ament_target_dependencies(transmission_interface PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
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

<<<<<<< HEAD
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

=======
ament_export_targets(export_transmission_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
>>>>>>> 727297e (Fix CMake install so overriding works (#926))
ament_package()
ament_generate_version_header(${PROJECT_NAME})
