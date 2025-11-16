# This subdirectory handles pytest-based launch tests for controller_manager

find_package(ament_cmake_pytest REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install YAML test files
#
install(
  FILES
    test_controller_load.yaml
    test_ros2_control_node_combined.yaml
  DESTINATION test
)

# Make sure test files get installed into the test tree
install(
  FILES
    test_launch_utils_unit.py
    test_launch_utils_integration_list.py
    test_launch_utils_integration_dict.py
    test_launch_utils_integration_load.py
  DESTINATION test
)

# Register each test with ament
ament_add_pytest_test(test_launch_utils_unit test_launch_utils_unit.py
    APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}
    PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}:${CMAKE_CURRENT_SOURCE_DIR}:${PYTHONPATH}
)
ament_add_pytest_test(test_launch_utils_integration_list
  test_launch_utils_integration_list.py
  TIMEOUT 60
)
ament_add_pytest_test(test_launch_utils_integration_dict
  test_launch_utils_integration_dict.py
  TIMEOUT 60
)
ament_add_pytest_test(test_launch_utils_integration_load
  test_launch_utils_integration_load.py
  TIMEOUT 60
)
