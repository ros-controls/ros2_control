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

import os
import sys
import tempfile

from controller_manager import list_controllers, list_hardware_components

from ros2cli.node.direct import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2cli.verb import VerbExtension

from ros2controlcli.api import add_controller_mgr_parsers

import graphviz


# Port names of the record fields. Every port is keyed on the *full* interface name, i.e. the name
# including the owning controller's prefix for chained interfaces, because that is what makes a
# port unique: a controller can read both "joint1/position" from the hardware and
# "filter/joint1/position" exported by another controller.
def _state_input_port(interface):
    return "state_end_" + interface


def _command_output_port(interface):
    return "command_start_" + interface


def _reference_input_port(interface):
    return "controller_end_" + interface


def _exported_state_output_port(interface):
    return "controller_start_" + interface


def _hw_state_output_port(interface):
    return "state_start_" + interface


def _hw_command_input_port(interface):
    return "command_end_" + interface


def _record_label(title, inputs, outputs):
    """
    Assemble a graphviz record label.

    :param title: text of the first field, i.e. the name of the node.
    :param inputs: list of (port, text) tuples shown in the left column.
    :param outputs: list of (port, text) tuples shown in the right column.
    """

    def fields(entries):
        return " | ".join(f"<{port}> {text}" for port, text in entries)

    # an empty group would be rendered as an empty field, so skip it
    groups = "|".join("{" + fields(entries) + "}" for entries in (inputs, outputs) if entries)
    if not groups:
        return title
    return f"{title}|{{{groups}}}"


def make_controller_node(
    s,
    controller_name,
    state_interfaces,
    command_interfaces,
    reference_interfaces,
    exported_state_interfaces,
    reference_interface_owners,
    exported_state_owners,
):
    # Interfaces read from the hardware, and those read from another controller's exported state.
    hw_states = sorted(i for i in state_interfaces if i not in exported_state_owners)
    chained_states = sorted(i for i in state_interfaces if i in exported_state_owners)
    # Interfaces written to the hardware, and those written to another controller's reference.
    hw_commands = sorted(i for i in command_interfaces if i not in reference_interface_owners)
    chained_commands = sorted(i for i in command_interfaces if i in reference_interface_owners)

    inputs = [(_state_input_port(i), i + " (state)") for i in hw_states]
    inputs += [(_state_input_port(i), i + " (exp state)") for i in chained_states]
    inputs += [(_reference_input_port(i), i + " (exp ref)") for i in sorted(reference_interfaces)]

    outputs = [(_command_output_port(i), i + " (cmd)") for i in hw_commands]
    outputs += [
        (_exported_state_output_port(i), i + " (exp state)")
        for i in sorted(exported_state_interfaces)
    ]
    outputs += [(_command_output_port(i), i + " (cmd)") for i in chained_commands]

    s.node(controller_name, _record_label(controller_name, inputs, outputs))


def make_command_node(s, command_interfaces):
    outputs = [(_hw_command_input_port(i), i) for i in sorted(command_interfaces)]
    s.node("command_interfaces", _record_label("hw command_interfaces", outputs, []))


def make_state_node(s, state_interfaces):
    inputs = [(_hw_state_output_port(i), i) for i in sorted(state_interfaces)]
    s.node("state_interfaces", _record_label("hw state_interfaces", inputs, []))


def _render(s, view, directory):
    """Render the graph, either into a viewer or into a file, and report where it ended up."""
    if view:
        if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY") or os.name == "nt":
            try:
                s.render(directory=tempfile.gettempdir(), view=True, cleanup=True)
                return
            except Exception as ex:  # no viewer available for the format
                print(
                    f"Could not open a viewer ({ex}), saving the diagram instead.", file=sys.stderr
                )
        else:
            print(
                "No display detected, saving the diagram instead of opening a viewer.",
                file=sys.stderr,
            )
    # graphviz remembers the directory of a previous render() call, so always be explicit
    path = s.render(directory=directory or os.curdir, view=False, cleanup=True)
    print(f"Saved controller diagram to {os.path.abspath(path)}")


def show_graph(
    reference_interfaces,
    exported_state_interfaces,
    command_connections,
    state_connections,
    reference_interface_owners,
    exported_state_owners,
    command_interfaces,
    state_interfaces,
    view=True,
    directory=None,
):
    s = graphviz.Digraph(
        "g",
        filename="controller_diagram",
        graph_attr={"rankdir": "LR", "ranksep": "2"},
        node_attr={"shape": "record", "style": "rounded"},
    )
    # get all controller names
    controller_names = sorted(
        set(reference_interfaces)
        | set(exported_state_interfaces)
        | set(command_connections)
        | set(state_connections)
    )
    # create node for each controller
    for controller_name in controller_names:
        make_controller_node(
            s,
            controller_name,
            state_connections[controller_name],
            command_connections[controller_name],
            reference_interfaces[controller_name],
            exported_state_interfaces[controller_name],
            reference_interface_owners,
            exported_state_owners,
        )

    make_state_node(s, state_interfaces)
    make_command_node(s, command_interfaces)

    for controller_name in controller_names:
        for interface in sorted(state_connections[controller_name]):
            # state comes either from a hardware component or from another controller's exp state
            if interface in exported_state_owners:
                owner, exported_name = exported_state_owners[interface]
                source = f"{owner}:{_exported_state_output_port(exported_name)}"
            else:
                source = "{}:{}".format("state_interfaces", _hw_state_output_port(interface))
            s.edge(source, f"{controller_name}:{_state_input_port(interface)}")

        for interface in sorted(command_connections[controller_name]):
            # commands go either to a hardware component or to another controller's exp reference
            if interface in reference_interface_owners:
                owner, reference_name = reference_interface_owners[interface]
                target = f"{owner}:{_reference_input_port(reference_name)}"
            else:
                target = "{}:{}".format("command_interfaces", _hw_command_input_port(interface))
            s.edge(f"{controller_name}:{_command_output_port(interface)}", target)

    _render(s, view, directory)
    return s


def parse_response(list_controllers_response, list_hardware_response, view=True, directory=None):
    command_interfaces = {
        x.name for hw in list_hardware_response.component for x in hw.command_interfaces
    }
    state_interfaces = {
        x.name for hw in list_hardware_response.component for x in hw.state_interfaces
    }
    # interfaces a chainable controller exports, without the controller name prefix
    reference_interfaces = {
        x.name: set(x.reference_interfaces) for x in list_controllers_response.controller
    }
    exported_state_interfaces = {
        x.name: set(getattr(x, "exported_state_interfaces", []))
        for x in list_controllers_response.controller
    }

    # Lookups from the full interface name, as claimed by the *following* controller, to the
    # controller exporting it and the name it exports it under. The controller manager strips the
    # controller name prefix in the ListControllers response, so it has to be added back here, e.g.
    # ("left_wheel_joint/velocity", "pid_left") -> "pid_left/left_wheel_joint/velocity"
    reference_interface_owners = {}
    exported_state_owners = {}
    for controller in list_controllers_response.controller:
        for interface in controller.reference_interfaces:
            reference_interface_owners[f"{controller.name}/{interface}"] = (
                controller.name,
                interface,
            )
        for interface in getattr(controller, "exported_state_interfaces", []):
            exported_state_owners[f"{controller.name}/{interface}"] = (controller.name, interface)

    command_connections = {
        x.name: set(x.required_command_interfaces) for x in list_controllers_response.controller
    }
    state_connections = {
        x.name: set(x.required_state_interfaces) for x in list_controllers_response.controller
    }

    return show_graph(
        reference_interfaces,
        exported_state_interfaces,
        command_connections,
        state_connections,
        reference_interface_owners,
        exported_state_owners,
        command_interfaces,
        state_interfaces,
        view=view,
        directory=directory,
    )


class ViewControllerChainsVerb(VerbExtension):
    """Generates a diagram of the loaded chained controllers."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument(
            "--save",
            action="store_true",
            help="Save the diagram as controller_diagram.pdf in the current directory instead of "
            "opening it in a viewer",
        )
        add_controller_mgr_parsers(parser)

    def main(self, *, args):
        with NodeStrategy(args) as node:
            list_controllers_response = list_controllers(node, args.controller_manager)
            list_hardware_response = list_hardware_components(node, args.controller_manager)
            parse_response(list_controllers_response, list_hardware_response, view=not args.save)
            return 0
