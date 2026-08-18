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

import shutil
import unittest
import warnings
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.msg import HardwareInterface
from controller_manager_msgs.msg import ChainConnection
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import ListHardwareInterfaces

from ros2controlcli.verb.view_controller_chains import make_controller_node
from ros2controlcli.verb.view_controller_chains import parse_response

import pygraphviz as pgz


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

    def make_graph(self):
        s = pgz.AGraph(name="g", strict=False, directed=True, rankdir="LR")
        s.node_attr["shape"] = "record"
        s.node_attr["style"] = "rounded"
        return s

    def assert_valid_record_label(self, s, node_name):
        """Assert that every field of a record label is separated from the next by a '|'."""
        label = s.get_node(node_name).attr["label"]
        # In a well-formed record label two consecutive ports always have a delimiter
        # between them, so a '>' is never followed by a '<' without a '|' in between.
        self.assertNotRegex(
            label,
            r">[^|]*<",
            f"record fields are not separated by '|' in the label of {node_name}",
        )
        if shutil.which("dot"):
            # graphviz reports a malformed label through a RuntimeWarning rather than an
            # exception, so it has to be promoted for this assertion to mean anything.
            with warnings.catch_warnings():
                warnings.simplefilter("error", RuntimeWarning)
                s.layout(prog="dot")

    def test_delimiter_does_not_depend_on_the_name_length(self):
        """The record delimiter must not depend on the length of an interface name.

        The delimiter used to be dropped whenever a field's index happened to equal the
        length of that interface's *name* minus one, instead of the number of interfaces
        minus one. graphviz then rejected the whole label with "bad label format" and drew
        nothing at all. Every name below is 12 characters long and zero padded, so the
        sorted order equals the generated order and interface number 11 is guaranteed to
        hit that collision.
        """
        state_interfaces = [f"joint_{joint:02d}/pos" for joint in range(20)]
        self.assertEqual(len(state_interfaces[11]), 12)

        s = self.make_graph()
        make_controller_node(s, "controller", state_interfaces, [], [], [], dict())
        self.assert_valid_record_label(s, "controller")

    def test_broadcaster_of_a_six_joint_arm(self):
        """A joint_state_broadcaster of a six joint arm reporting its driver state.

        This is the set of interfaces that first exposed the defect: 27 state interfaces
        in which "joint_5/effort" and "joint_5/velocity" land on a colliding index.
        """
        state_interfaces = [
            "driver_state/motion_active",
            "set_io/async_success",
            *(
                f"joint_{joint}/{interface}"
                for joint in range(1, 7)
                for interface in ("effort", "position", "velocity")
            ),
            *(
                f"robot_state/{state}"
                for state in (
                    "error_code",
                    "estop",
                    "ext_safeguard",
                    "operation_mode",
                    "robot_error",
                    "state",
                    "stick_enable",
                )
            ),
        ]

        s = self.make_graph()
        make_controller_node(s, "joint_state_broadcaster", state_interfaces, [], [], [], dict())
        self.assert_valid_record_label(s, "joint_state_broadcaster")

    def test_state_interfaces_and_reference_interfaces(self):
        """State interfaces and reference interfaces share one side of the record.

        Both are written into the same '{...}' group, so a delimiter is needed between the
        two groups as well and not only within each of them.
        """
        state_interfaces = [f"joint_{joint}/position" for joint in range(1, 7)]
        command_interfaces = [f"joint_{joint}/position" for joint in range(1, 7)]
        input_controllers = [f"chained_controller/joint_{joint}" for joint in range(1, 7)]
        output_controllers = [f"downstream_controller/joint_{joint}" for joint in range(1, 7)]

        s = self.make_graph()
        port_map = dict()
        make_controller_node(
            s,
            "controller",
            state_interfaces,
            command_interfaces,
            input_controllers,
            output_controllers,
            port_map,
        )
        self.assert_valid_record_label(s, "controller")
        # every reference interface has to stay resolvable to the controller owning it
        for input_controller in input_controllers:
            self.assertEqual(port_map["controller_end_" + input_controller], "controller")
