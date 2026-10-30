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
import time
import subprocess
import threading
import unittest

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import HardwareDeviceStatus
from control_msgs.msg import HardwareStatus
from launch import LaunchDescription
import launch_testing
from launch_testing.actions import ReadyToTest
import launch_testing.markers
import launch_ros.actions
import pytest
import rclpy

from controller_manager.controller_manager_services import list_controllers
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

    def controller_states(self):
        controllers = list_controllers(self.node, "controller_manager", 5.0).controller
        return {controller.name: controller.state for controller in controllers}

    def ensure_ctrl1_active(self):
        states = self.controller_states()
        if "ctrl_1" not in states:
            result = run_cli(
                "load_controller", "--set-state", "active", "ctrl_1", self.robot_controllers
            )
            self.assertEqual(result.returncode, 0, result.stderr)
        elif states["ctrl_1"] == "unconfigured":
            result = run_cli("set_controller_state", "ctrl_1", "inactive")
            self.assertEqual(result.returncode, 0, result.stderr)
            result = run_cli("set_controller_state", "ctrl_1", "active")
            self.assertEqual(result.returncode, 0, result.stderr)
        elif states["ctrl_1"] == "inactive":
            result = run_cli("set_controller_state", "ctrl_1", "active")
            self.assertEqual(result.returncode, 0, result.stderr)

        check_controllers_running(self.node, ["ctrl_1"], state="active")

    def ensure_ctrl2_inactive(self):
        states = self.controller_states()
        if "ctrl_2" not in states:
            result = run_cli(
                "load_controller",
                "--set-state",
                "inactive",
                "ctrl_2",
                self.robot_controllers,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
        elif states["ctrl_2"] == "unconfigured":
            result = run_cli("set_controller_state", "ctrl_2", "inactive")
            self.assertEqual(result.returncode, 0, result.stderr)
        elif states["ctrl_2"] == "active":
            result = run_cli("set_controller_state", "ctrl_2", "inactive")
            self.assertEqual(result.returncode, 0, result.stderr)

        check_controllers_running(self.node, ["ctrl_2"], state="inactive")

    def unload_ctrl2_if_loaded(self):
        states = self.controller_states()
        if "ctrl_2" not in states:
            return

        if states["ctrl_2"] == "active":
            result = run_cli("set_controller_state", "ctrl_2", "inactive")
            self.assertEqual(result.returncode, 0, result.stderr)

        result = run_cli("unload_controller", "ctrl_2")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn("ctrl_2", self.controller_states())

    def test_node_start(self):
        check_node_running(self.node, "controller_manager")
        check_controllers_running(self.node, ["ctrl_1"], state="active")

    def test_list_controllers(self):
        self.ensure_ctrl1_active()
        result = run_cli("list_controllers")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("ctrl_1", result.stdout)
        self.assertIn("active", result.stdout)

    def test_list_controller_types(self):
        result = run_cli("list_controller_types")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("controller_manager/test_controller", result.stdout)

    def test_list_hardware_components(self):
        result = run_cli("list_hardware_components")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("TestSystemComponent", result.stdout)
        self.assertIn("TestSensorComponent", result.stdout)

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

    def test_reload_controller_libraries_without_force_kill_fails(self):
        self.ensure_ctrl1_active()

        result = run_cli("reload_controller_libraries")

        self.assertNotEqual(result.returncode, 0)
        check_controllers_running(self.node, ["ctrl_1"], state="active")

    def test_load_controller(self):
        self.unload_ctrl2_if_loaded()

        result = run_cli(
            "load_controller", "--set-state", "inactive", "ctrl_2", self.robot_controllers
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        check_controllers_running(self.node, ["ctrl_2"], state="inactive")

    def test_set_controller_state(self):
        self.ensure_ctrl2_inactive()

        result = run_cli("set_controller_state", "ctrl_2", "active")
        self.assertEqual(result.returncode, 0, result.stderr)
        check_controllers_running(self.node, ["ctrl_2"], state="active")

        result = run_cli("set_controller_state", "ctrl_2", "inactive")
        self.assertEqual(result.returncode, 0, result.stderr)
        check_controllers_running(self.node, ["ctrl_2"], state="inactive")

    def test_cleanup_controller(self):
        self.ensure_ctrl2_inactive()

        result = run_cli("cleanup_controller", "ctrl_2")
        self.assertEqual(result.returncode, 0, result.stderr)
        check_controllers_running(self.node, ["ctrl_2"], state="unconfigured")

    def test_unload_controller(self):
        self.ensure_ctrl2_inactive()

        result = run_cli("unload_controller", "ctrl_2")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn("ctrl_2", self.controller_states())

    def test_switch_controllers_best_effort(self):
        self.ensure_ctrl1_active()
        self.ensure_ctrl2_inactive()

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

    def test_view_hardware_status(self):
        publisher = self.node.create_publisher(HardwareStatus, "/test/hardware_status", 10)
        stop_publishing = threading.Event()

        def publish_status():
            while not stop_publishing.is_set():
                msg = HardwareStatus()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.hardware_id = "test_hardware_component"

                device_status = HardwareDeviceStatus()
                device_status.device_id = "test_device"
                msg.hardware_device_states.append(device_status)

                publisher.publish(msg)
                time.sleep(0.1)

        process = subprocess.Popen(
            ["ros2", "control", "view_hardware_status", "--hardware-id", "missing_hardware"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

        publisher_thread = threading.Thread(target=publish_status, daemon=True)
        publisher_thread.start()

        try:
            stdout, stderr = process.communicate(timeout=10.0)
        finally:
            stop_publishing.set()
            publisher_thread.join(timeout=1.0)
            if process.poll() is None:
                process.kill()
                process.communicate()

        self.assertEqual(process.returncode, 0, stderr)
        self.assertIn("Subscribing to the following topics", stdout)
        self.assertIn("/test/hardware_status", stdout)
        self.assertIn("Available Hardware IDs", stdout)
        self.assertIn("test_hardware_component", stdout)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
