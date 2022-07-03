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

from controller_manager import list_controllers

from ros2cli.node.direct import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2cli.verb import VerbExtension

from ros2controlcli.api import add_controller_mgr_parsers

import graphviz


def make_controller_node(s, controller_name, inputs, outputs, port_map):
    inputs = sorted(list(inputs))
    outputs = sorted(list(outputs))
    inputs_str = ''
    outputs_str = ''
    for ind, input in enumerate(inputs):
        deliminator = '|'
        if ind == len(inputs) - 1:
            deliminator = ''
        input = input.replace('/', '_')
        name = input
        inputs_str += '<%s> %s %s ' % ("input_" + input, name, deliminator)
        port_map["input_" + input] = controller_name

    for ind, output in enumerate(outputs):
        deliminator = '|'
        if ind == len(outputs) - 1:
            deliminator = ''
        output = output.replace('/', '_')
        outputs_str += '<%s> %s %s ' % ("output_" + output, '', deliminator)
        port_map["output_" + output] = controller_name

    s.node(controller_name, '%s|{{%s}|{%s}}' % (controller_name, inputs_str, outputs_str))


def make_command_node(s, output_interfaces, port_map):
    outputs = sorted(list(output_interfaces))
    outputs_str = ''
    for ind, output in enumerate(outputs):
        deliminator = '|'
        if ind == len(outputs) - 1:
            deliminator = ''
        output = output.replace('/', '_')
        name = output
        outputs_str += '<%s> %s %s ' % ("input_" + output, name, deliminator)
        port_map["input_" + output] = "command_interfaces"

    s.node("command_interfaces", '%s|{{%s}}' % ("command_interfaces", outputs_str))


def make_state_node(s, input_interfaces, port_map):
    inputs = sorted(list(input_interfaces))
    inputs_str = ''
    for ind, input in enumerate(inputs):
        deliminator = '|'
        if ind == len(inputs) - 1:
            deliminator = ''
        input = input.replace('/', '_')
        name = input
        inputs_str += '<%s> %s %s ' % ("output_" + input, name, deliminator)
        port_map["output_" + input] = "state_interfaces"

    s.node("state_interfaces", '%s|{{%s}}' % ("state_interfaces", inputs_str))


def show_graph(output_connections, input_connections, output_interfaces, input_interfaces):
    s = graphviz.Digraph('g', filename='/tmp/controller_diagram.gv', node_attr={'shape': 'record', 'style': 'rounded'})
    port_map = dict()
    for controller_name in input_connections:
        make_controller_node(s, controller_name, input_connections[controller_name],
                             output_connections[controller_name], port_map)

    make_state_node(s, input_interfaces, port_map)
    make_command_node(s, output_interfaces, port_map)

    for controller_name in output_connections:
        for connection in output_connections[controller_name]:
            connection = connection.replace('/', '_')
            s.edge('%s:%s' % (controller_name, "output_" + connection),
                   '%s:%s' % (port_map['input_' + connection], 'input_' + connection))
        for connection in input_connections[controller_name]:
            if connection in input_interfaces:
                connection = connection.replace('/', '_')
                s.edge('%s:%s' % ("state_interfaces", "output_" + connection),
                       '%s:%s' % (controller_name, 'input_' + connection))

    s.attr(splines="false")
    s.attr(ranksep='2')
    s.attr(rankdir="LR")
    s.view()


class ViewControllerChainsVerb(VerbExtension):
    """Generates a diagram of the loaded chained controllers."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        add_controller_mgr_parsers(parser)

    def main(self, *, args):
        with NodeStrategy(args) as node:
            response = list_controllers(node, args.controller_manager)
            output_interfaces = set()
            input_interfaces = set()
            output_connections = dict()
            input_connections = dict()
            for c_chained in response.controller_group:
                state_interfaces = set(c_chained.controller_state.required_state_interfaces)
                required_interfaces = set(c_chained.controller_state.required_command_interfaces)
                chained_interfaces = set()
                if len(c_chained.chained_connections) > 0:
                    chained_interfaces = set([y for x in c_chained.chained_connections for y in x.reference_interfaces])
                    output_connections[c_chained.controller_state.name] = chained_interfaces
                else:
                    output_connections[c_chained.controller_state.name] = required_interfaces

                input_interfaces = input_interfaces.union(state_interfaces)
                output_interfaces = output_interfaces.union(required_interfaces - chained_interfaces)
                input_connections[c_chained.controller_state.name] = state_interfaces

            for controller_name in output_connections:
                connections = output_connections[controller_name]
                for connection in connections:
                    connection_split = connection.split('/')
                    if len(connection_split) > 0:
                        prefix = connection_split[0]
                        if prefix in input_connections:
                            input_connections[prefix].add(connection)

            show_graph(output_connections, input_connections, output_interfaces, input_interfaces)
            return 0
