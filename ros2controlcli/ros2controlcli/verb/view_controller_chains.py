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

from controller_manager import list_controllers, list_hardware_components

from ros2cli.node.direct import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2cli.verb import VerbExtension

from ros2controlcli.api import add_controller_mgr_parsers

import graphviz


def make_controller_node(
    s,
    controller_name,
    state_interfaces,
    command_interfaces,
    input_chain_connections,
    output_chain_connections,
    port_map,
):
    state_interfaces = sorted(list(state_interfaces))
    command_interfaces = sorted(list(command_interfaces))
    input_chain_connections = sorted(list(input_chain_connections))
    output_chain_connections = sorted(list(output_chain_connections))

    inputs_str = ""
    for ind, state_interface in enumerate(state_interfaces):
        deliminator = "|"
        if ind == len(state_interface) - 1:
            deliminator = ""
        inputs_str += "<{}> {} {} ".format(
            "state_end_" + state_interface, state_interface + " (state)", deliminator
        )

    for ind, input_controller in enumerate(input_chain_connections):
        deliminator = "|"
        if ind == len(input_controller) - 1:
            deliminator = ""
        inputs_str += "<{}> {} {} ".format(
            "controller_end_" + input_controller, input_controller + " (exp ref)", deliminator
        )
        port_map["controller_end_" + input_controller] = controller_name

    outputs_str = ""
    for ind, command_interface in enumerate(command_interfaces):
        deliminator = "|"
        if ind == len(command_interface) - 1:
            deliminator = ""
        outputs_str += "<{}> {} {} ".format(
            "command_start_" + command_interface, command_interface + " (cmd)", deliminator
        )

    for ind, output_controller in enumerate(output_chain_connections):
        deliminator = "|"
        if ind == len(output_controller) - 1:
            deliminator = ""
        outputs_str += "<{}> {} {} ".format(
            "controller_start_" + output_controller,
            output_controller + " (exp state)",
            deliminator,
        )

    s.node(controller_name, f"{controller_name}|{{{{{inputs_str}}}|{{{outputs_str}}}}}")


def make_command_node(s, command_interfaces):
    command_interfaces = sorted(list(command_interfaces))
    outputs_str = ""
    for ind, command_interface in enumerate(command_interfaces):
        deliminator = "|"
        if ind == len(command_interfaces) - 1:
            deliminator = ""
        outputs_str += "<{}> {} {} ".format(
            "command_end_" + command_interface, command_interface, deliminator
        )

    s.node("command_interfaces", "{}|{{{{{}}}}}".format("hw command_interfaces", outputs_str))


def make_state_node(s, state_interfaces):
    state_interfaces = sorted(list(state_interfaces))
    inputs_str = ""
    for ind, state_interface in enumerate(state_interfaces):
        deliminator = "|"
        if ind == len(state_interfaces) - 1:
            deliminator = ""
        inputs_str += "<{}> {} {} ".format(
            "state_start_" + state_interface, state_interface, deliminator
        )

    s.node("state_interfaces", "{}|{{{{{}}}}}".format("hw state_interfaces", inputs_str))


def show_graph(
    input_chain_connections,
    output_chain_connections,
    command_connections,
    state_connections,
    command_interfaces,
    state_interfaces,
    visualize,
):
    s = graphviz.Digraph(
        "g",
        filename="/tmp/controller_diagram.gv",
        node_attr={"shape": "record", "style": "rounded"},
    )
    port_map = dict()
    # get all controller names
    controller_names = set()
    controller_names = controller_names.union({name for name in input_chain_connections})
    controller_names = controller_names.union({name for name in output_chain_connections})
    controller_names = controller_names.union({name for name in command_connections})
    controller_names = controller_names.union({name for name in state_connections})
    # create node for each controller
    for controller_name in controller_names:
        make_controller_node(
            s,
            controller_name,
            state_connections[controller_name],
            command_connections[controller_name],
            input_chain_connections[controller_name],
            output_chain_connections[controller_name],
            port_map,
        )

    make_state_node(s, state_interfaces)
    make_command_node(s, command_interfaces)

    for controller_name in controller_names:
        for connection in output_chain_connections[controller_name]:
            s.edge(
                "{}:{}".format(controller_name, "controller_start_" + connection),
                "{}:{}".format(
                    port_map["controller_end_" + connection], "controller_end_" + connection
                ),
            )
        for state_connection in state_connections[controller_name]:
            s.edge(
                "{}:{}".format("state_interfaces", "state_start_" + state_connection),
                "{}:{}".format(controller_name, "state_end_" + state_connection),
            )
        for command_connection in command_connections[controller_name]:
            s.edge(
                "{}:{}".format(controller_name, "command_start_" + command_connection),
                "{}:{}".format("command_interfaces", "command_end_" + command_connection),
            )

    s.attr(ranksep="2")
    s.attr(rankdir="LR")
    if visualize:
        s.view()
    else:
        s.render(filename="controller_diagram", view=False, cleanup=True)


def parse_response(list_controllers_response, list_hardware_response, visualize=True):
    command_interfaces = {
        x.name for hw in list_hardware_response.component for x in hw.command_interfaces
    }
    state_interfaces = {
        x.name for hw in list_hardware_response.component for x in hw.state_interfaces
    }
    command_connections = dict()
    state_connections = dict()
    input_chain_connections = {x.name: set() for x in list_controllers_response.controller}
    output_chain_connections = {x.name: set() for x in list_controllers_response.controller}

    for controller in list_controllers_response.controller:
        for chain_connection in controller.chain_connections:
            for reference_interface in chain_connection.reference_interfaces:
                output_chain_connections[controller.name].add(reference_interface)
        for reference_interface in controller.reference_interfaces:
            input_chain_connections[controller.name].add(reference_interface)

        command_connections[controller.name] = set(controller.required_command_interfaces)
        state_connections[controller.name] = set(controller.required_state_interfaces)

    show_graph(
        input_chain_connections,
        output_chain_connections,
        command_connections,
        state_connections,
        command_interfaces,
        state_interfaces,
        visualize,
    )


class ViewControllerChainsVerb(VerbExtension):
    """Generates a diagram of the loaded chained controllers."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument(
            "--save",
            action="store_true",
            help="Save PDF to controller_diagram.pdf instead of viewing image",
        )
        add_controller_mgr_parsers(parser)

    def main(self, *, args):
        with NodeStrategy(args).direct_node as node:
            list_controllers_response = list_controllers(node, args.controller_manager)
            list_hardware_response = list_hardware_components(node, args.controller_manager)
            parse_response(
                list_controllers_response, list_hardware_response, visualize=not args.save
            )
            return 0
