# Copyright 2026 Open Source Robotics Foundation, Inc.
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

import os
import subprocess
import unittest

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_testing
from launch_testing.actions import ReadyToTest
import launch_testing.markers
import launch_ros.actions
import pytest
import rclpy

from controller_manager.launch_utils import generate_controllers_spawner_launch_description
from controller_manager.test_utils import check_controllers_running
from controller_manager.test_utils import check_node_running


def run_cli(*args):
    return subprocess.run(
        ["ros2", "control", *args],
        check=False,
        capture_output=True,
        text=True,
    )


@pytest.mark.launch_test
def generate_test_description():
    robot_controllers = os.path.join(
        get_package_prefix("controller_manager"), "test", "test_ros2_controlcli.yaml"
    )

    control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )

    urdf = os.path.join(
        get_package_share_directory("ros2_control_test_assets"),
        "urdf",
        "test_hardware_components.urdf",
    )
    with open(urdf) as infp:
        robot_description_content = infp.read()
    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    ctrl_spawner = generate_controllers_spawner_launch_description(
        ["ctrl_1"], controller_params_files=[robot_controllers]
    )

    return LaunchDescription([robot_state_pub_node, control_node, ctrl_spawner, ReadyToTest()])


class TestRos2ControlCliVerbs(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_ros2_controlcli_verbs")
        self.robot_controllers = os.path.join(
            get_package_prefix("controller_manager"), "test", "test_ros2_controlcli.yaml"
        )

    def tearDown(self):
        self.node.destroy_node()

    def test_node_start(self):
        check_node_running(self.node, "controller_manager")
        check_controllers_running(self.node, ["ctrl_1"], state="active")

    def test_list_controller_types(self):
        result = run_cli("list_controller_types")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("controller_manager/test_controller", result.stdout)

    def test_list_hardware_interfaces(self):
        result = run_cli("list_hardware_interfaces")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("joint1/position", result.stdout)
        self.assertIn("joint2/position", result.stdout)

    def test_reload_controller_libraries_force_kill(self):
        result = run_cli("reload_controller_libraries", "--force-kill")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Reload successful", result.stdout)

        result = run_cli("load_controller", "--set-state", "active", "ctrl_1")
        self.assertEqual(result.returncode, 0, result.stderr)
        check_controllers_running(self.node, ["ctrl_1"], state="active")

    def test_switch_controllers_best_effort(self):
        check_controllers_running(self.node, ["ctrl_1"], state="active")

        result = run_cli(
            "load_controller",
            "--set-state",
            "inactive",
            "ctrl_2",
            self.robot_controllers,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        check_controllers_running(self.node, ["ctrl_2"], state="inactive")

        result = run_cli(
            "switch_controllers",
            "--best-effort",
            "--activate",
            "ctrl_2",
            "--deactivate",
            "ctrl_1",
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        check_controllers_running(self.node, ["ctrl_1"], state="inactive")
        check_controllers_running(self.node, ["ctrl_2"], state="active")


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
