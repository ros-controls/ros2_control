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

import rclpy

from ros2cli.node.direct import DirectNode

from ros2node.api import NodeNameCompleter

from ros2param.api import call_list_parameters


def service_caller(service_name, service_type, request):
    try:
        rclpy.init()

        node = rclpy.create_node(f"ros2controlcli_{service_name.replace('/', '')}_requester")

        cli = node.create_client(service_type, service_name)

        if not cli.service_is_ready():
            node.get_logger().debug(f"waiting for service {service_name} to become available...")

            if not cli.wait_for_service(2.0):
                raise RuntimeError(f"Could not contact service {service_name}")

        node.get_logger().debug(f"requester: making request: {repr(request)}\n")
        future = cli.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            return future.result()
        else:
            future_exception = future.exception()
            raise RuntimeError(f"Exception while calling service: {repr(future_exception)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


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


def add_controller_mgr_parsers(parser):
    """Parser arguments to get controller manager node name, defaults to /controller_manager."""
    arg = parser.add_argument(
        "-c",
        "--controller-manager",
        help="Name of the controller manager ROS node",
        default="/controller_manager",
        required=False,
    )
    arg.completer = NodeNameCompleter(include_hidden_nodes_key="include_hidden_nodes")
    parser.add_argument(
        "--include-hidden-nodes", action="store_true", help="Consider hidden nodes as well"
    )
