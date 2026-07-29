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
from launch import LaunchDescription
import launch_testing
from launch_testing.actions import ReadyToTest
import launch_ros.actions
from launch.substitutions import FileContent, PathSubstitution
from launch_ros.substitutions import FindPackageShare

import rclpy

from controller_manager.test_utils import check_controllers_running

from controller_manager.launch_utils import (
    generate_controllers_spawner_launch_description_from_dict,
)


@pytest.mark.launch_test
def generate_test_description():
    """
    Generate launch description for testing the dict-based spawner helper.

    Uses the combined controller YAML installed with the package.
    """

    urdf = FileContent(
        PathSubstitution(FindPackageShare("ros2_control_test_assets"))
        / "urdf"
        / "test_hardware_components.urdf"
    )
    robot_description = {"robot_description": urdf}

    # The dictionary keys are the controller names to be spawned/started.
    # Values can be empty lists since config is provided via the main YAML.
    ctrl_dict = {
        "test_broadcaster": [],
        "controller1": [],
        "controller2": [],
    }
    controller_list = list(ctrl_dict.keys())

    # ===== GENERATE SPAWNER LAUNCH DESCRIPTION =====
    print(f"Spawning controllers: {controller_list}")

    # ===== CREATE LAUNCH DESCRIPTION =====
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
                parameters=[
                    robot_description,
                    PathSubstitution(FindPackageShare("controller_manager"))
                    / "test"
                    / "test_launch_utils"
                    / "test_ros2_control_node_combined.yaml",
                ],
                output="both",
            ),
            generate_controllers_spawner_launch_description_from_dict(
                controller_info_dict=ctrl_dict,
                extra_spawner_args=["--inactive"],
            ),
            ReadyToTest(),
        ]
    ), {"controller_list": controller_list}


# Active tests
class TestControllerSpawnerList(unittest.TestCase):
    """Active tests that run while the launch is active."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_controller_spawner")

    def tearDown(self):
        self.node.destroy_node()

    def test_controllers_start(self, proc_info, controller_list):
        cnames = controller_list.copy()
        check_controllers_running(self.node, cnames, state="inactive")

        # Wait for controller_spawner to finish and verify successful exit.
        proc_info.assertWaitForShutdown(process="spawner", timeout=30)
        launch_testing.asserts.assertExitCodes(proc_info, process="spawner")

        # Re-check controllers after spawner has exited.
        check_controllers_running(self.node, cnames, state="inactive")


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Post-shutdown tests."""

    def test_exit_codes(self, proc_info):
        """Verify all processes exited successfully."""
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0])
