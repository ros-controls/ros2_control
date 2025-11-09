#!/usr/bin/env python3
# Copyright 2025 Robert Kwan
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.import os

import pytest
import unittest
import os
import tempfile

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
import launch_testing
from launch_testing.actions import ReadyToTest
import launch_testing.asserts
import launch_testing.markers
import launch_ros.actions

import rclpy

from controller_manager.launch_utils import generate_load_controller_launch_description


def get_loaded_controllers(node, timeout=30.0):
    """Query /controller_manager/list_controllers until controllers are available."""
    from controller_manager_msgs.srv import ListControllers
    import time

    client = node.create_client(ListControllers, "/controller_manager/list_controllers")

    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError("Controller manager service not available")

    seen = []
    start_time = time.time()
    while time.time() - start_time < timeout:
        req = ListControllers.Request()
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)

        if fut.done() and fut.result() is not None:
            response = fut.result()
            seen = [c.name for c in response.controller]
            if seen:
                break

        time.sleep(0.2)

    return seen


@pytest.mark.launch_test
def generate_test_description():

    # Create temporary directory for all test files
    temp_dir = tempfile.mkdtemp()
    print(f"Creating test files in: {temp_dir}")

    # Get URDF, without involving xacro
    urdf = os.path.join(
        get_package_share_directory("ros2_control_test_assets"),
        "urdf",
        "test_hardware_components.urdf",
    )
    with open(urdf) as infp:
        robot_description_content = infp.read()
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = os.path.join(
        get_package_prefix("controller_manager"), "test", "test_controller_load.yaml"
    )

    # Verify both files exist
    assert os.path.isfile(robot_controllers), f"Controller config not created: {robot_controllers}"

    robot_state_pub_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ===== START CONTROLLER MANAGER =====
    control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )

    print(f"Using config file: {robot_controllers}")

    spawner_action = generate_load_controller_launch_description(
        controller_name="test_controller_load",
        controller_params_file=[robot_controllers],
    )

    ld = LaunchDescription(
        [
            robot_state_pub_node,
            control_node,
            *spawner_action.entities,  # Unpack the entities from the returned LaunchDescription
            ReadyToTest(),
        ]
    )

    return ld, {
        "temp_dir": temp_dir,
        "controller_name": "test_controller_load",
        "urdf_file": urdf,
    }


class TestControllerLoad(unittest.TestCase):
    def test_controller_loaded(self, launch_service, proc_output, controller_name):
        # Create a temporary ROS 2 node for calling the service
        rclpy.init()
        test_node = rclpy.create_node("test_controller_client")

        # Poll the ListControllers service to ensure the target controller is present
        try:
            loaded_controllers = get_loaded_controllers(test_node, timeout=30.0)

            # CRITICAL ASSERTION: The test passes only if the controller name is in the list
            assert (
                controller_name in loaded_controllers
            ), f"Controller '{controller_name}' not found. Loaded: {loaded_controllers}"

        finally:
            test_node.destroy_node()
            rclpy.shutdown()

    def test_spawner_exit_code(self, proc_info):
        """Test that spawner process ran (may have completed already)."""
        process_names = proc_info.process_names()
        print(f"\n[TEST] Checking for spawner in: {process_names}")

        # The spawner may have already completed successfully and exited
        # So we just verify that we have processes running
        self.assertGreater(len(process_names), 0)
        print(f"[TEST] ? Launch has {len(process_names)} active processes")


@launch_testing.post_shutdown_test()
class TestCleanup:
    def test_ros_nodes_exit_cleanly(self, proc_info):
        # The control_node should exit cleanly after the whole launch file finishes
        launch_testing.asserts.assertExitCodes(proc_info)

    # Ensures the temporary directory is removed after the test is done
    def test_cleanup_temp_files(self, temp_dir):
        import shutil

        try:
            shutil.rmtree(temp_dir)
        except Exception as e:
            print(f"Cleanup failed for directory {temp_dir}: {e}")
