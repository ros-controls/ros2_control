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


class ListControllersVerb(VerbExtension):
    """Output the list of loaded controllers, their type and status."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument(
            "--claimed-interfaces",
            action="store_true",
            help="List controller's claimed interfaces",
        )
        parser.add_argument(
            "--required-state-interfaces",
            action="store_true",
            help="List controller's required state interfaces",
        )
        parser.add_argument(
            "--required-command-interfaces",
            action="store_true",
            help="List controller's required command interfaces",
        )
        add_controller_mgr_parsers(parser)

    def main(self, *, args):
        with NodeStrategy(args) as node:
            controllers = list_controllers(node, args.controller_manager).controller
            for c in controllers:
                print(f"{c.name:20s}[{c.type:20s}] {c.state:10s}")
                if args.claimed_interfaces:
                    print("claimed interfaces:")
                    for claimed_interface in c.claimed_interfaces:
                        print(f"\t{claimed_interface}")
                if args.required_command_interfaces:
                    print("required command interfaces:")
                    for required_command_interface in c.required_command_interfaces:
                        print(f"\t{required_command_interface}")
                if args.required_state_interfaces:
                    print("required state interfaces:")
                    for required_state_interface in c.required_state_interfaces:
                        print(f"\t{required_state_interface}")

            return 0
