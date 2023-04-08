# Copyright 2020 Open Source Robotics Foundation, Inc.
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

import unittest

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node

import launch_testing
import launch_testing.actions


def generate_test_description():
    joint_limits_path = get_package_share_directory("joint_limits")

    node_under_test = Node(
        package="joint_limits",
        executable="joint_limits_rosparam_test",
        output="screen",
        parameters=[os.path.join(joint_limits_path, "test", "joint_limits_rosparam.yaml")],
    )
    return (
        LaunchDescription(
            [
                node_under_test,
                launch_testing.util.KeepAliveProc(),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        locals(),
    )


class TestJointLimitInterface(unittest.TestCase):
    def test_termination(self, node_under_test, proc_info):
        proc_info.assertWaitForShutdown(process=node_under_test, timeout=(10))


@launch_testing.post_shutdown_test()
class TestJointLimitInterfaceTestAfterShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that all processes in the launch (in this case, there's just one) exit
        # with code 0
        launch_testing.asserts.assertExitCodes(proc_info)
