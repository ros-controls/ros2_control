cmake_minimum_required(VERSION 3.5)
project(transmission_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
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
endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  hardware_interface
)
ament_package()
