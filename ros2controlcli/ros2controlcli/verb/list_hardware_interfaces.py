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

from controller_manager import list_hardware_interfaces

from ros2cli.node.direct import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2cli.verb import VerbExtension
from ros2controlcli.api import add_controller_mgr_parsers


class ListHardwareInterfacesVerb(VerbExtension):
    """Output the list of loaded controllers, their type and status."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        add_controller_mgr_parsers(parser)

    def main(self, *, args):
        with NodeStrategy(args) as node:
            hardware_interfaces = list_hardware_interfaces(node, args.controller_manager)
            command_interfaces = sorted(
                hardware_interfaces.command_interfaces, key=lambda hwi: hwi.name
            )
            state_interfaces = sorted(
                hardware_interfaces.state_interfaces, key=lambda hwi: hwi.name
            )
            print('command interfaces')
            for command_interface in command_interfaces:
                print(
                    '\t%s [%s]'
                    % (
                        command_interface.name,
                        'claimed' if command_interface.is_claimed else 'unclaimed',
                    )
                )
            print('state interfaces')
            for state_interface in state_interfaces:
                print('\t', state_interface.name)

            return 0
