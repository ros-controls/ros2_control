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


def make_controller_node(s, controller_name, inputs, outputs):
    inputs = sorted(list(inputs))
    outputs = sorted(list(outputs))
    inputs_str = ''
    outputs_str = ''
    for ind, input in enumerate(inputs):
        deliminator = '|'
        if ind == len(inputs) - 1:
            deliminator = ''
        inputs_str += '<%s> %s %s ' % (input, '', deliminator)
    for ind, output in enumerate(outputs):
        deliminator = '|'
        if ind == len(outputs) - 1:
            deliminator = ''
        outputs_str += '<%s> %s %s ' % (output, '', deliminator)

    s.node(controller_name, r'%s|{{%s}|{%s}}' % (controller_name, inputs_str, outputs_str))
    pass


def show_graph(output_connections, input_connections, output_interfaces, input_interfaces):
    # s = graphviz.Digraph('structs', filename='/tmp/controller_diagram.gv', node_attr={'shape': 'record'})
    s = graphviz.Digraph('g', filename='/tmp/controller_diagram.gv', node_attr={'shape': 'record', 'height': '.1'})
    # s.node('struct1', '<f0> left|<f1> middle|<f2> right')
    # s.node('struct2', '<interface_0> one|<f1> two')
    # s.node('struct3', r'hello\nworld |{ b |{c|<here> d|e}| f}| g | h')
    for controller_name in input_connections:
        make_controller_node(s, controller_name, input_connections[controller_name],
                             output_connections[controller_name])

    controller_name = 'admittance_controller'
    controller_name_2 = 'joint_trajectory_controller'

    s.edge(r'%s:%s' % (controller_name_2, 'admittance_controller/elbow_joint/position'),
       r'%s:%s' % (controller_name, 'admittance_controller/elbow_joint/position') )

    s.edge(r'%s:%s' % (controller_name_2, 'admittance_controller/shoulder_lift/position'),
           r'%s:%s' % (controller_name, 'admittance_controller/shoulder_lift/position') )
    # s.edges([('struct1:f1', 'struct2:interface_0'), ('struct1:f2', 'struct3:here')])
    # s.attr(ratio="compress")
    # s.attr(splines="false")
    # s.attr(layout="neato")
    # s.attr(nodesep='3')
    s.attr(ranksep='2')
    s.attr(rankdir="LR")
    # s.attr(rank="same")
    # s.attr(size='1000,2000')
    s.view()
    input("press any key to exit")

    pass


class ViewControllerChainsVerb(VerbExtension):
    """Output the list of loaded controllers, their type and status."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument(
            '--verbose', '-v',
            action='store_true',
            help='List controller\'s claimed interfaces, required state interfaces and required command interfaces',
        )
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
