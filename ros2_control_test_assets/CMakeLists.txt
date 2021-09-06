cmake_minimum_required(VERSION 3.5)
project(ros2_control_test_assets)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_package()
