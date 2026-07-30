# This subdirectory handles pytest-based launch tests for controller_manager

find_package(ament_cmake_pytest REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install YAML test files
#
install(
  FILES
    test_ros2_control_node_combined.yaml
  DESTINATION share/${PROJECT_NAME}/test/test_launch_utils
)

# Register each test with ament
ament_add_pytest_test(test_launch_utils_unit
    test_launch_utils_unit.py
)
ament_add_pytest_test(test_launch_utils_integration_list
  test_launch_utils_integration_list.py
    APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}_$<CONFIG>
)
ament_add_pytest_test(test_launch_utils_integration_dict
  test_launch_utils_integration_dict.py
    APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}_$<CONFIG>
)
ament_add_pytest_test(test_launch_utils_integration_load
  test_launch_utils_integration_load.py
    APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}_$<CONFIG>
)
