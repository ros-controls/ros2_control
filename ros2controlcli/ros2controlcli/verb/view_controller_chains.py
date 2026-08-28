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
    chain_cmd_connections,
):
    state_interfaces = sorted(list(state_interfaces))
    command_interfaces = sorted(list(command_interfaces))
    input_chain_connections = sorted(list(input_chain_connections))
    output_chain_connections = sorted(
        list(output_chain_connections)
    )  # (target_ctrl, iface) tuples
    chain_cmd_connections = sorted(list(chain_cmd_connections))  # (target_ctrl, iface) tuples

    inputs_str = ""
    for ind, state_interface in enumerate(state_interfaces):
        deliminator = "|"
        if ind == len(state_interfaces) - 1:
            deliminator = ""
        inputs_str += "<{}> {} {} ".format(
            "state_end_" + state_interface, state_interface + " (state)", deliminator
        )

    for ind, input_controller in enumerate(input_chain_connections):
        deliminator = "|"
        if ind == len(input_chain_connections) - 1:
            deliminator = ""
        inputs_str += "<{}> {} {} ".format(
            "controller_end_" + input_controller, input_controller + " (exp ref)", deliminator
        )

    outputs_str = ""
    for ind, command_interface in enumerate(command_interfaces):
        deliminator = "|"
        if ind == len(command_interfaces) - 1:
            deliminator = ""
        outputs_str += "<{}> {} {} ".format(
            "command_start_" + command_interface, command_interface + " (cmd)", deliminator
        )

    for ind, output_iface in enumerate(output_chain_connections):
        deliminator = "|"
        if ind == len(output_chain_connections) - 1:
            deliminator = ""
        outputs_str += "<{}> {} {} ".format(
            "controller_start_exp_state_" + output_iface,
            output_iface + " (exp state)",
            deliminator,
        )

    for ind, (target_ctrl, chain_cmd) in enumerate(chain_cmd_connections):
        deliminator = "|"
        if ind == len(chain_cmd_connections) - 1:
            deliminator = ""
        port_key = target_ctrl + "/" + chain_cmd
        outputs_str += "<{}> {} {} ".format(
            "command_start_chain_" + port_key,
            chain_cmd + " (cmd)",
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
    chain_cmd_connections,
    exp_state_edges,
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
    # get all controller names
    controller_names = set()
    controller_names = controller_names.union({name for name in input_chain_connections})
    controller_names = controller_names.union({name for name in output_chain_connections})
    controller_names = controller_names.union({name for name in chain_cmd_connections})
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
            chain_cmd_connections[controller_name],
        )

    make_state_node(s, state_interfaces)
    make_command_node(s, command_interfaces)

    # Set of (consumer_ctrl, short_iface) pairs whose state comes from another controller
    exp_state_consumers = {(consumer_ctrl, iface) for _, consumer_ctrl, iface in exp_state_edges}

    for source_ctrl, consumer_ctrl, iface in exp_state_edges:
        s.edge(
            "{}:{}".format(source_ctrl, "controller_start_exp_state_" + iface),
            "{}:{}".format(consumer_ctrl, "state_end_" + iface),
        )

    for controller_name in controller_names:
        for target_ctrl, iface in chain_cmd_connections[controller_name]:
            # (cmd) output of preceding controller → (exp ref) input of following controller
            port_key = target_ctrl + "/" + iface
            s.edge(
                "{}:{}".format(controller_name, "command_start_chain_" + port_key),
                "{}:{}".format(target_ctrl, "controller_end_" + iface),
            )
        for state_connection in state_connections[controller_name]:
            # Only draw hw edge if this interface is not sourced from another controller's exp state
            if (controller_name, state_connection) not in exp_state_consumers:
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
    chain_cmd_connections = {x.name: set() for x in list_controllers_response.controller}

    # Build lookup: prefixed reference interface name -> owning controller name
    # e.g. "pid_controller_left_wheel_joint/left_wheel_joint/velocity" -> "pid_controller_left_wheel_joint"
    ref_interface_owners = {}
    for ctrl in list_controllers_response.controller:
        for ref_iface in ctrl.reference_interfaces:
            ref_interface_owners[f"{ctrl.name}/{ref_iface}"] = ctrl.name

    # Build lookup: prefixed exported state interface name -> (owning controller, short name)
    exported_state_owners = {}
    for ctrl in list_controllers_response.controller:
        for exp_state in ctrl.exported_state_interfaces:
            exported_state_owners[f"{ctrl.name}/{exp_state}"] = (ctrl.name, exp_state)

    for controller in list_controllers_response.controller:
        # Exported state output ports: what this controller publishes as state
        for exp_state in controller.exported_state_interfaces:
            output_chain_connections[controller.name].add(exp_state)

        for reference_interface in controller.reference_interfaces:
            input_chain_connections[controller.name].add(reference_interface)

        # Classify required_command_interfaces as hw or chained (cmd → exp ref)
        hw_cmds = set()
        for cmd_iface in controller.required_command_interfaces:
            if cmd_iface in ref_interface_owners:
                target_ctrl = ref_interface_owners[cmd_iface]
                short_iface = cmd_iface.removeprefix(target_ctrl + "/")
                chain_cmd_connections[controller.name].add((target_ctrl, short_iface))
            else:
                hw_cmds.add(cmd_iface)
        command_connections[controller.name] = hw_cmds

        # All required state interfaces need input ports; strip controller prefix for exp-state ones
        ctrl_states = set()
        for state_iface in controller.required_state_interfaces:
            if state_iface in exported_state_owners:
                _, short_iface = exported_state_owners[state_iface]
                ctrl_states.add(short_iface)
            else:
                ctrl_states.add(state_iface)
        state_connections[controller.name] = ctrl_states

    # Build edges for (exp state) → (state): controller exports state that another controller reads
    exp_state_edges = []
    for controller in list_controllers_response.controller:
        for state_iface in controller.required_state_interfaces:
            if state_iface in exported_state_owners:
                source_ctrl, short_iface = exported_state_owners[state_iface]
                exp_state_edges.append((source_ctrl, controller.name, short_iface))

    show_graph(
        input_chain_connections,
        output_chain_connections,
        chain_cmd_connections,
        exp_state_edges,
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
