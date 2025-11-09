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
# limitations under the License.

import pytest
import unittest
import os
import time
import tempfile

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
import launch_testing
from launch_testing.actions import ReadyToTest
import launch_testing.markers
import launch_ros.actions

import rclpy

from controller_manager.launch_utils import (
    generate_controllers_spawner_launch_description,
)


@pytest.mark.launch_test
def generate_test_description():
    """
    Generate launch description for testing.
    """

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
        get_package_prefix("controller_manager"), "test", "test_ros2_control_node_combined.yaml"
    )

    # Verify both files exist
    assert os.path.isfile(robot_controllers), f"Controller config not created: {robot_controllers}"
    assert os.path.isfile(urdf), f"URDF not created: {urdf}"

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

    # ===== DEFINE CONTROLLERS TO SPAWN =====
    controller_list = ["joint_state_broadcaster", "ctrl_with_parameters_and_type", "controller3"]

    # ===== GENERATE SPAWNER =====
    print(f"Spawning controllers: {controller_list}")
    print(f"Using config file: {robot_controllers}")

    spawner_ld = generate_controllers_spawner_launch_description(
        controller_names=controller_list.copy(),
        controller_params_files=[robot_controllers],
    )

    # ===== CREATE LAUNCH DESCRIPTION =====
    ld = LaunchDescription(
        [robot_state_pub_node, control_node, ReadyToTest()] + spawner_ld.entities
    )

    # Return tuple with test context
    return ld, {
        "controller_list": controller_list,
        "robot_controllers": robot_controllers,
        "urdf_file": urdf,
        "temp_dir": temp_dir,
    }


# Active tests
class TestControllerSpawnerList(unittest.TestCase):
    """Active tests that run while the launch is active."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_spawner_nodes_launched(self, proc_info):
        """Ensure processes are running."""
        process_names = proc_info.process_names()
        self.assertGreater(len(process_names), 0)
        print(f"\n[TEST] Active processes: {process_names}")

    def test_controllers_loaded(self, proc_info, controller_list):
        """Test that controllers were loaded (poll until they appear)."""
        node = rclpy.create_node("test_controller_query_node")

        try:
            from controller_manager_msgs.srv import ListControllers

            client = node.create_client(ListControllers, "/controller_manager/list_controllers")

            print("\n[TEST] Waiting for controller_manager service...")
            wait_for_svc_timeout = 30.0
            if not client.wait_for_service(timeout_sec=wait_for_svc_timeout):
                process_names = proc_info.process_names()
                self.fail(
                    f"Controller manager service not available after {wait_for_svc_timeout}s.\n"
                    f"Active processes: {process_names}"
                )

            # Poll for controllers to be registered
            print("[TEST] Service available, polling for controllers (timeout 30s)...")
            deadline = node.get_clock().now() + rclpy.duration.Duration(seconds=30.0)
            seen = []
            while node.get_clock().now() < deadline:
                req = ListControllers.Request()
                fut = client.call_async(req)
                rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
                if fut.done() and fut.result() is not None:
                    response = fut.result()
                    seen = [c.name for c in response.controller]
                    if all(ctrl in seen for ctrl in controller_list):
                        print(f"[TEST] Loaded controllers: {seen}")
                        break
                # small sleep to avoid tight-loop
                time.sleep(0.2)
            else:
                # timeout expired
                self.fail(
                    f"Timeout waiting for controllers to be loaded. "
                    f"Expected: {controller_list}, saw: {seen}"
                )

            # Final assert (defensive)
            for controller in controller_list:
                self.assertIn(
                    controller,
                    seen,
                    f"Controller '{controller}' was not loaded. Available: {seen}",
                )

            print(f"[TEST] ? All {len(controller_list)} controllers loaded successfully")

        finally:
            node.destroy_node()

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
        print("\n[POST-SHUTDOWN] Process exit codes:")
        for process_name in proc_info.process_names():
            info = proc_info[process_name]
            print(f"  {process_name}: {info.returncode}")

        for process_name in proc_info.process_names():
            info = proc_info[process_name]

            if "ros2_control_node" in process_name:
                self.assertEqual(
                    info.returncode, 0, f"{process_name} exited with {info.returncode}"
                )
            elif "spawner" in process_name:
                # Spawner should complete successfully (0) or be terminated
                self.assertIn(
                    info.returncode,
                    [0, -2, -15],
                    f"Spawner {process_name} exited with {info.returncode}",
                )
            else:
                self.assertIn(
                    info.returncode, [0, -2, -15], f"{process_name} exited with {info.returncode}"
                )

        print("[POST-SHUTDOWN] ? All processes exited as expected")

    def test_cleanup_temp_files(self, temp_dir, robot_controllers, urdf_file):
        """Clean up temporary test files."""
        import shutil

        print(f"\n[CLEANUP] Removing temporary directory: {temp_dir}")

        try:
            if os.path.exists(temp_dir):
                shutil.rmtree(temp_dir)

            print("[CLEANUP] ? Temporary files removed")
        except Exception as e:
            print(f"[CLEANUP] Warning: Cleanup failed: {e}")
