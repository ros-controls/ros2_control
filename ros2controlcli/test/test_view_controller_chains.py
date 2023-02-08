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
from controller_manager_msgs.msg import ChainConnection
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import ListHardwareInterfaces

from ros2controlcli.verb.view_controller_chains import parse_response


class TestViewControllerChains(unittest.TestCase):
    def test_expected(self):
        list_controllers_response = ListControllers.Response()
        list_hardware_response = ListHardwareInterfaces.Response()

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
        list_hardware_response.state_interfaces = state_interfaces
        list_hardware_response.command_interfaces = command_interfaces
        try:
            parse_response(list_controllers_response, list_hardware_response, visualize=False)
        except Exception:
            self.assertTrue(0, "parse_response failed!")
