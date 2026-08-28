# Copyright 2022 PickNik, Inc.
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

import unittest
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.msg import HardwareInterface
from controller_manager_msgs.msg import HardwareComponentState
from controller_manager_msgs.msg import ChainConnection
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import ListHardwareComponents

from ros2controlcli.verb.view_controller_chains import parse_response


class TestViewControllerChains(unittest.TestCase):
    def test_expected(self):
        list_controllers_response = ListControllers.Response()
        list_hardware_response = ListHardwareComponents.Response()

        chained_to_controller = ControllerState()
        chained_from_controller = ControllerState()
        chain_connection = ChainConnection()

        state_interfaces = []
        command_interfaces = []

        chained_to_controller.name = "chained_controller"
        chained_to_controller.state = "active"
        chained_from_controller.name = "controller"
        chained_from_controller.state = "active"
        chain_connection.name = chained_to_controller.name

        for i in range(1, 7):
            pos_int = HardwareInterface(name=f"joint{i}/position")
            vel_int = HardwareInterface(name=f"joint{i}/velocity")
            state_interfaces.append(pos_int)
            state_interfaces.append(vel_int)
            command_interfaces.append(pos_int)
            command_interfaces.append(vel_int)

            chained_to_controller.claimed_interfaces.append(f"joint{i}/position")
            chained_to_controller.claimed_interfaces.append(f"joint{i}/velocity")
            chained_to_controller.required_state_interfaces.append(f"joint{i}/position")
            chained_to_controller.reference_interfaces.append(f"joint{i}/position")
            chained_to_controller.reference_interfaces.append(f"joint{i}/velocity")

            chained_from_controller.required_state_interfaces.append(f"joint{i}/position")
            chained_from_controller.required_state_interfaces.append(f"joint{i}/velocity")

            chain_connection.reference_interfaces.append(f"joint{i}/position")
            chain_connection.reference_interfaces.append(f"joint{i}/velocity")

        chained_to_controller.required_command_interfaces = (
            chained_to_controller.claimed_interfaces
        )

        chained_from_controller.required_command_interfaces = (
            chained_from_controller.claimed_interfaces
        )
        chained_from_controller.chain_connections.append(chain_connection)

        controller_list = [chained_from_controller, chained_to_controller]

        list_controllers_response.controller = controller_list

        list_controllers_response_component = HardwareComponentState()
        list_controllers_response_component.state_interfaces = state_interfaces
        list_controllers_response_component.command_interfaces = command_interfaces
        list_hardware_response.component.append(list_controllers_response_component)
        try:
            parse_response(list_controllers_response, list_hardware_response, visualize=False)
        except Exception:
            self.assertTrue(0, "parse_response failed!")

    def test_complete_example(self):
        list_controllers_response = ListControllers.Response()
        list_hardware_response = ListHardwareComponents.Response()

        tm_status_controller = ControllerState()
        joint_trajectory_controller = ControllerState()
        joint_state_broadcaster = ControllerState()

        tm_status_controller.name = "tm_status_controller"
        tm_status_controller.state = "active"
        tm_status_controller.type = "tm_controllers/TmStatusController"
        tm_status_controller.update_rate = 500
        tm_status_controller.claimed_interfaces = [
            "play/press",
            "stop/press",
            "pause/press",
            "set_io/Ctrl_DO0",
            "set_io/Ctrl_DO1",
            "set_io/Ctrl_DO2",
            "set_io/Ctrl_DO3",
            "set_io/Ctrl_DO4",
            "set_io/Ctrl_DO5",
            "set_io/Ctrl_DO6",
            "set_io/Ctrl_DO7",
            "set_io/Ctrl_DO8",
        ]
        tm_status_controller.required_command_interfaces = tm_status_controller.claimed_interfaces
        tm_status_controller.required_state_interfaces = [
            "robot_state/state",
            "robot_state/robot_error",
            "robot_state/estop",
            "robot_state/ext_safeguard",
            "robot_state/operation_mode",
            "robot_state/error_code",
            "robot_state/stick_enable",
            "driver_state/motion_active",
            "set_io/async_success",
        ]

        joint_trajectory_controller.name = "joint_trajectory_controller"
        joint_trajectory_controller.state = "active"
        joint_trajectory_controller.type = "joint_trajectory_controller/JointTrajectoryController"
        joint_trajectory_controller.update_rate = 500
        joint_trajectory_controller.claimed_interfaces = [
            f"joint_{i}/position" for i in range(1, 7)
        ]
        joint_trajectory_controller.required_command_interfaces = (
            joint_trajectory_controller.claimed_interfaces
        )
        joint_trajectory_controller.required_state_interfaces = [
            interface
            for i in range(1, 7)
            for interface in (f"joint_{i}/position", f"joint_{i}/velocity")
        ]

        joint_state_broadcaster.name = "joint_state_broadcaster"
        joint_state_broadcaster.state = "active"
        joint_state_broadcaster.type = "joint_state_broadcaster/JointStateBroadcaster"
        joint_state_broadcaster.update_rate = 500
        joint_state_broadcaster.required_state_interfaces = [
            "joint_6/effort",
            "joint_5/velocity",
            "joint_2/velocity",
            "joint_2/position",
            "joint_1/effort",
            "joint_4/velocity",
            "joint_3/position",
            "joint_1/velocity",
            "joint_6/position",
            "joint_6/velocity",
            "joint_1/position",
            "joint_3/velocity",
            "joint_3/effort",
            "joint_4/position",
            "joint_4/effort",
            "joint_5/position",
            "joint_2/effort",
            "joint_5/effort",
            "set_io/async_success",
            "driver_state/motion_active",
            "robot_state/state",
            "robot_state/stick_enable",
            "robot_state/robot_error",
            "robot_state/estop",
            "robot_state/error_code",
            "robot_state/ext_safeguard",
            "robot_state/operation_mode",
        ]

        list_controllers_response.controller = [
            tm_status_controller,
            joint_trajectory_controller,
            joint_state_broadcaster,
        ]

        hardware_component = HardwareComponentState()
        hardware_component.name = "tm_robot"
        hardware_component.type = "system"
        hardware_component.rw_rate = 500
        hardware_component.plugin_name = "tm_hardware_interface/TmPositionHardwareInterface"

        hardware_component.command_interfaces = [
            HardwareInterface(name=interface)
            for interface in [
                "joint_6/position",
                "joint_5/position",
                "joint_4/position",
                "joint_2/position",
                "joint_3/position",
                "joint_1/position",
                "set_io/Ctrl_DO7",
                "play/press",
                "stop/press",
                "set_io/Ctrl_DO0",
                "set_io/Ctrl_DO6",
                "set_io/Ctrl_DO2",
                "set_io/Ctrl_DO5",
                "set_io/Ctrl_DO3",
                "set_io/Ctrl_DO8",
                "pause/press",
                "set_io/Ctrl_DO4",
                "set_io/Ctrl_DO1",
            ]
        ]

        hardware_component.state_interfaces = [
            HardwareInterface(name=interface)
            for interface in [
                "joint_6/effort",
                "joint_5/velocity",
                "joint_2/velocity",
                "joint_2/position",
                "joint_1/effort",
                "joint_4/velocity",
                "joint_3/position",
                "joint_1/velocity",
                "joint_6/position",
                "joint_6/velocity",
                "joint_1/position",
                "joint_3/velocity",
                "joint_3/effort",
                "joint_4/position",
                "joint_4/effort",
                "joint_5/position",
                "joint_2/effort",
                "joint_5/effort",
                "set_io/async_success",
                "driver_state/motion_active",
                "robot_state/state",
                "robot_state/stick_enable",
                "robot_state/robot_error",
                "robot_state/estop",
                "robot_state/error_code",
                "robot_state/ext_safeguard",
                "robot_state/operation_mode",
            ]
        ]

        list_hardware_response.component = [hardware_component]

        try:
            parse_response(
                list_controllers_response,
                list_hardware_response,
                visualize=False,
            )
        except Exception:
            self.assertTrue(0, "parse_response failed!")
