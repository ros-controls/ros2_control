# Copyright (c) 2024 AIT - Austrian Institute of Technology GmbH
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


from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("controller_manager"),
            "test",
            "test_ros2_control_node.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )
    nodes = [
        control_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
