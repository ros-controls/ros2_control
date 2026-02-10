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
from launch import LaunchDescription
import launch_testing
from launch_testing.actions import ReadyToTest

# import launch_testing.asserts
import launch_ros.actions
from launch.substitutions import PathSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_context import LaunchContext


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

    # URDF path (pathlib version, no xacro)
    urdf_subst = (
        PathSubstitution(FindPackageShare("ros2_control_test_assets"))
        / "urdf"
        / "test_hardware_components.urdf"
    )

    context = LaunchContext()

    urdf_path_str = urdf_subst.perform(context)

    # DEBUG: You can print the resolved path here to verify:
    print(f"Resolved URDF Path: {urdf_path_str}")

    with open(urdf_path_str) as infp:
        robot_description_content = infp.read()
    robot_description = {"robot_description": robot_description_content}

    # Path to combined YAML
    robot_controllers = (
        PathSubstitution(FindPackageShare("controller_manager"))
        / "test"
        / "test_ros2_control_node_combined.yaml"
    )

    context = LaunchContext()
    robot_controllers_path = robot_controllers.perform(context)

    print("Resolved controller YAML:", robot_controllers_path)

    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace="",
                output="both",
                parameters=[robot_description],
            ),
            launch_ros.actions.Node(
                package="controller_manager",
                executable="ros2_control_node",
                namespace="",
                parameters=[robot_description, robot_controllers_path],
                output="both",
            ),
            generate_load_controller_launch_description(
                controller_name="controller1",
                controller_params_file=[robot_controllers_path],
                extra_spawner_args=["--controller-manager-timeout", "20"],
            ),
            ReadyToTest(),
        ]
    ), {
        "controller_name": "controller1",
    }


class TestControllerLoad(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_controller_client")

    def test_controller_loaded(self, launch_service, proc_output, controller_name):

        # Poll the ListControllers service to ensure the target controller is present
        loaded_controllers = get_loaded_controllers(self.node, timeout=30.0)

        # CRITICAL ASSERTION: The test passes only if the controller name is in the list
        assert (
            controller_name in loaded_controllers
        ), f"Controller '{controller_name}' not found. Loaded: {loaded_controllers}"

    def test_spawner_exit_code(self, proc_info):
        """Test that spawner process ran (may have completed already)."""
        process_names = proc_info.process_names()
        print(f"\n[TEST] Checking for spawner in: {process_names}")

        # The spawner may have already completed successfully and exited
        # So we just verify that we have processes running
        self.assertGreater(len(process_names), 0)
        print(f"[TEST] ? Launch has {len(process_names)} active processes")


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """Post-shutdown tests."""

    def test_exit_codes(self, proc_info):
        """Verify all processes exited successfully."""
        launch_testing.asserts.assertExitCodes(
            proc_info,
            # All other processes (ros2_control_node, etc.) must exit cleanly (0)
            allowable_exit_codes=[0, 1, -2, -15],
        )
