# Copyright 2020 PAL Robotics S.L.
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


from controller_manager import list_controllers, list_hardware_components

from ros2cli.node.direct import DirectNode

from ros2node.api import NodeNameCompleter

from ros2param.api import call_list_parameters

import argparse


class ControllerNameCompleter:
    """Callable returning a list of controllers parameter names."""

    def __call__(self, prefix, parsed_args, **kwargs):
        with DirectNode(parsed_args) as node:
            parameter_names = call_list_parameters(
                node=node, node_name=parsed_args.controller_manager
            )
            suffix = ".type"
            return [n[: -len(suffix)] for n in parameter_names if n.endswith(suffix)]


class LoadedControllerNameCompleter:
    """Callable returning a list of loaded controllers."""

    def __init__(self, valid_states=["active", "inactive", "configured", "unconfigured"]):
        self.valid_states = valid_states

    def __call__(self, prefix, parsed_args, **kwargs):
        with DirectNode(parsed_args) as node:
            controllers = list_controllers(node, parsed_args.controller_manager).controller
            return [c.name for c in controllers if c.state in self.valid_states]


class LoadedHardwareComponentNameCompleter:
    """Callable returning a list of loaded hardware components."""

    def __init__(self, valid_states=["active", "inactive", "configured", "unconfigured"]):
        self.valid_states = valid_states

    def __call__(self, prefix, parsed_args, **kwargs):
        with DirectNode(parsed_args) as node:
            hardware_components = list_hardware_components(
                node, parsed_args.controller_manager
            ).component
            return [c.name for c in hardware_components if c.state.label in self.valid_states]


class ParserROSArgs(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        values = [option_string] + values
        setattr(namespace, "argv", values)


def add_controller_mgr_parsers(parser):
    """Parser arguments to get controller manager node name, defaults to controller_manager."""
    arg = parser.add_argument(
        "-c",
        "--controller-manager",
        help="Name of the controller manager ROS node (default: controller_manager)",
        default="controller_manager",
        required=False,
    )
    arg.completer = NodeNameCompleter(include_hidden_nodes_key="include_hidden_nodes")
    parser.add_argument(
        "--include-hidden-nodes", action="store_true", help="Consider hidden nodes as well"
    )
    parser.add_argument(
        "--ros-args",
        nargs=argparse.REMAINDER,
        help="Pass arbitrary arguments to the executable",
        action=ParserROSArgs,
    )
