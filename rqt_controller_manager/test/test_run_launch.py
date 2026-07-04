# Copyright (c) 2026 Christian Rauch
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

import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.node import Node

node_name = "rqt_controller_manager_node"


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                executable="rqt_controller_manager",
                package="rqt_controller_manager",
                name=node_name,
                additional_env={"QT_QPA_PLATFORM": "offscreen"},
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestFixture(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = Node("test_node")

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_start(self, proc_output):
        time.sleep(2)
        assert node_name in self.node.get_node_names()
