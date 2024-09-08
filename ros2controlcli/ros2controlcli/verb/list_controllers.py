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

from controller_manager import list_controllers, bcolors

from ros2cli.node.direct import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2cli.verb import VerbExtension

from ros2controlcli.api import add_controller_mgr_parsers


def print_controller_state(c, args, col_width_name, col_width_state, col_width_type):
    state_color = ""
    if c.state == "active":
        state_color = bcolors.OKGREEN
    elif c.state == "inactive":
        state_color = bcolors.OKCYAN
    elif c.state == "unconfigured":
        state_color = bcolors.WARNING

    print(
        f"{state_color}{c.name:<{col_width_name}}{bcolors.ENDC} {c.type:<{col_width_type}}  {state_color}{c.state:<{col_width_state}}{bcolors.ENDC}"
    )
    if args.claimed_interfaces or args.verbose:
        print("\tclaimed interfaces:")
        for claimed_interface in c.claimed_interfaces:
            print(f"\t\t{claimed_interface}")
    if args.required_command_interfaces or args.verbose:
        print("\trequired command interfaces:")
        for required_command_interface in c.required_command_interfaces:
            print(f"\t\t{required_command_interface}")
    if args.required_state_interfaces or args.verbose:
        print("\trequired state interfaces:")
        for required_state_interface in c.required_state_interfaces:
            print(f"\t\t{required_state_interface}")
    if args.chained_interfaces or args.verbose:
        print("\tchained to interfaces:")
        for connection in c.chain_connections:
            for reference in connection.reference_interfaces:
                print(f"\t\t{reference:20s}")
    if args.reference_interfaces or args.verbose:
        print("\texported reference interfaces:")
        for reference_interfaces in c.reference_interfaces:
            print(f"\t\t{reference_interfaces}")


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
        parser.add_argument(
            "--chained-interfaces",
            action="store_true",
            help="List interfaces that the controllers are chained to",
        )
        parser.add_argument(
            "--reference-interfaces",
            action="store_true",
            help="List controller's exported references",
        )
        parser.add_argument(
            "--verbose",
            "-v",
            action="store_true",
            help="List controller's claimed interfaces, required state interfaces and required command interfaces",
        )
        add_controller_mgr_parsers(parser)

    def main(self, *, args):
        with NodeStrategy(args).direct_node as node:
            response = list_controllers(node, args.controller_manager)

            if not response.controller:
                print("No controllers are currently loaded!")
                return 0

            # Structure data as table for nicer output
            col_width_name = max(len(ctrl.name) for ctrl in response.controller)
            col_width_type = max(len(ctrl.type) for ctrl in response.controller)
            col_width_state = max(len(ctrl.state) for ctrl in response.controller)

            for c in response.controller:
                print_controller_state(c, args, col_width_name, col_width_state, col_width_type)

            return 0
