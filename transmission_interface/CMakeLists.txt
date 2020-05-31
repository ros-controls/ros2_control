cmake_minimum_required(VERSION 3.5)
project(transmission_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

add_library(transmission_parser SHARED src/transmission_parser.cpp)
target_include_directories(transmission_parser PUBLIC include)
ament_target_dependencies(transmission_parser
  hardware_interface
  TinyXML2
)
target_compile_definitions(transmission_parser PRIVATE "TRANSMISSION_INTERFACE_BUILDING_DLL")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS transmission_parser
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(
    test_transmission_parser
    test/test_transmission_parser.cpp
  )
  target_link_libraries(test_transmission_parser transmission_parser)
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  transmission_parser
)
ament_package()
