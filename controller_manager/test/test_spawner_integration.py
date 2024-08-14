# Copyright 2024 FZI Forschungszentrum Informatik
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
#
# Author: Felix Exner

import time
import pytest
import unittest

import launch
import launch_testing.actions

from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile

from launch_ros.actions import Node as LaunchNode

# Imports for tests
import rclpy
from rclpy.node import Node
import logging

from controller_manager_msgs.srv import ListControllers

CONTROLLER_SPAWNER_TIMEOUT = "10"

active_controllers = [
    "joint_state_broadcaster0",
    "joint_state_broadcaster1",
    "joint_state_broadcaster2",
    "joint_state_broadcaster3",
    "joint_state_broadcaster4",
    "joint_state_broadcaster5",
    "joint_state_broadcaster6",
    "joint_state_broadcaster7",
    "joint_state_broadcaster8",
    "joint_state_broadcaster9",
    "joint_state_broadcaster10",
    "joint_state_broadcaster11",
    "joint_state_broadcaster12",
    "joint_state_broadcaster13",
    "joint_state_broadcaster14",
    "joint_state_broadcaster15",
    "joint_state_broadcaster16",
    "joint_state_broadcaster17",
    "joint_state_broadcaster18",
    "joint_state_broadcaster19",
    "joint_state_broadcaster20",
    "joint_state_broadcaster21",
    "joint_state_broadcaster22",
    "joint_state_broadcaster23",
    "joint_state_broadcaster24",
    "joint_state_broadcaster25",
    "joint_state_broadcaster26",
    "joint_state_broadcaster27",
    "joint_state_broadcaster28",
    "joint_state_broadcaster29",
    "joint_state_broadcaster30",
    "joint_state_broadcaster31",
    "joint_state_broadcaster32",
    "joint_state_broadcaster33",
    "joint_state_broadcaster34",
    "joint_state_broadcaster35",
    "joint_state_broadcaster36",
    "joint_state_broadcaster37",
    "joint_state_broadcaster38",
    "joint_state_broadcaster39",
    "joint_state_broadcaster40",
    "joint_state_broadcaster41",
    "joint_state_broadcaster42",
    "joint_state_broadcaster43",
    "joint_state_broadcaster44",
    "joint_state_broadcaster45",
    "joint_state_broadcaster46",
    "joint_state_broadcaster47",
    "joint_state_broadcaster48",
    "joint_state_broadcaster49",
]


def controller_spawner(controllers, active=True):
    inactive_flags = ["--inactive"] if not active else []
    return LaunchNode(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            CONTROLLER_SPAWNER_TIMEOUT,
        ]
        + inactive_flags
        + controllers,
    )


@pytest.mark.launch_test
def generate_test_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ros2_control_test_assets"),
                    "urdf",
                    "test_description_mock.urdf",
                ]
            ),
        ]
    )
    robot_state_publisher_node = LaunchNode(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_content}],
    )

    control_node = LaunchNode(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ParameterFile(
                PathJoinSubstitution(
                    [FindPackageShare("controller_manager"), "test", "test_controllers.yaml"]
                )
            )
        ],
        output="screen",
    )

    spawners = [controller_spawner([x]) for x in active_controllers]

    return launch.LaunchDescription(
        [
            robot_state_publisher_node,
            control_node,
            launch_testing.actions.ReadyToTest(),
        ]
        + spawners
    )


class TestControllersRunning(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context
        rclpy.init()
        cls.node = Node("controller_spawner_test")

    def _wait_for_service(self, srv_name, srv_type, timeout=10):
        client = self.node.create_client(srv_type, srv_name)

        logging.info("Waiting for service '%s' with timeout %fs...", srv_name, timeout)
        if client.wait_for_service(timeout) is False:
            raise Exception(f"Could not reach service '{srv_name}' within timeout of {timeout}")
        logging.info("  Successfully connected to service '%s'", srv_name)

        return client

    def test_all_controllers_available(self):
        client = self._wait_for_service(
            srv_name="controller_manager/list_controllers", srv_type=ListControllers
        )
        # This is basically a dirty hack. It would be better to add events to all the spawners
        # exiting and emit ReadyToTest() only after all of them have quit. One problem there: They
        # do not necessarily quit, but can get into a deadlock waiting for a service response.
        time.sleep(30)
        request = ListControllers.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is None:
            raise Exception(
                f"Error while calling service '{client.srv_name}': {future.exception()}"
            )

        result = future.result()
        self.assertEqual(len(result.controller), len(active_controllers))
