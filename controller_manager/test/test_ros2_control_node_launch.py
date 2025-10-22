# Copyright (c) 2024 AIT - Austrian Institute of Technology GmbH
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Christoph Froehlich

import pytest
import unittest
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch_testing.actions import ReadyToTest
import launch_testing.markers
import launch_ros.actions

import rclpy
from controller_manager.test_utils import (
    check_controllers_running,
    check_node_running,
)
from controller_manager.launch_utils import generate_controllers_spawner_launch_description


# Executes the given launch file and checks if all nodes can be started
@pytest.mark.launch_test
def generate_test_description():

    robot_controllers = os.path.join(
        get_package_prefix("controller_manager"), "test", "test_ros2_control_node.yaml"
    )

    control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )
    # Get URDF, without involving xacro
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
        [
            "ctrl_with_parameters_and_type",
        ],
        controller_params_files=[robot_controllers],
    )
    return LaunchDescription([robot_state_pub_node, control_node, ctrl_spawner, ReadyToTest()])


# This is our test fixture. Each method is a test case.
# These run alongside the processes specified in generate_test_description()
class TestFixture(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_node")

    def tearDown(self):
        self.node.destroy_node()

    def test_node_start(self):
        check_node_running(self.node, "controller_manager")

    def test_controllers_start(self):
        cnames = ["ctrl_with_parameters_and_type"]
        check_controllers_running(self.node, cnames, state="active")


@launch_testing.post_shutdown_test()
# These tests are run after the processes in generate_test_description() have shutdown.
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)
