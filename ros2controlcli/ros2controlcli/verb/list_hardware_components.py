# Copyright 2023 ROS2-Control Development Team
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

from controller_manager import list_hardware_components, bcolors

from lifecycle_msgs.msg import State

from ros2cli.node.direct import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2cli.verb import VerbExtension
from ros2controlcli.api import add_controller_mgr_parsers


class ListHardwareComponentsVerb(VerbExtension):
    """Output the list of available hardware components."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument(
            "--verbose",
            "-v",
            action="store_true",
            help="List hardware components with command and state interfaces",
        )
        add_controller_mgr_parsers(parser)

    def main(self, *, args):
        with NodeStrategy(args) as node:
            hardware_components = list_hardware_components(node, args.controller_manager)

            for idx, component in enumerate(hardware_components.component):
                # Set activity color for nicer visualization
                activity_color = bcolors.FAIL
                if component.state.id == State.PRIMARY_STATE_UNCONFIGURED:
                    activity_color = bcolors.WARNING
                if component.state.id == State.PRIMARY_STATE_INACTIVE:
                    activity_color = bcolors.MAGENTA
                if component.state.id == State.PRIMARY_STATE_ACTIVE:
                    activity_color = bcolors.OKGREEN

                print(
                    f"Hardware Component {idx+1}\n\tname: {activity_color}{component.name}{bcolors.ENDC}\n\ttype: {component.type}"
                )
                if hasattr(component, "plugin_name"):
                    plugin_name = f"{component.plugin_name}"
                # Keep compatibility to the obsolete filed name in Humble
                elif hasattr(component, "class_type"):
                    plugin_name = f"{component.class_type}"
                else:
                    plugin_name = f"{bcolors.WARNING}plugin name missing!{bcolors.ENDC}"

                print(
                    f"\tplugin name: {plugin_name}\n"
                    f"\tstate: id={component.state.id} label={activity_color}{component.state.label}{bcolors.ENDC}\n"
                    f"\tcommand interfaces"
                )
                for cmd_interface in component.command_interfaces:

                    if cmd_interface.is_available:
                        available_str = f"{bcolors.OKBLUE}[available]{bcolors.ENDC}"
                    else:
                        available_str = f"{bcolors.WARNING}[unavailable]{bcolors.ENDC}"

                    if cmd_interface.is_claimed:
                        claimed_str = f"{bcolors.OKBLUE}[claimed]{bcolors.ENDC}"
                    else:
                        claimed_str = "[unclaimed]"

                    print(f"\t\t{cmd_interface.name} {available_str} {claimed_str}")

                if args.verbose:
                    print("\tstate interfaces")
                    for state_interface in component.state_interfaces:
                        if state_interface.is_available:
                            available_str = f"{bcolors.OKBLUE}[available]{bcolors.ENDC}"
                        else:
                            available_str = f"{bcolors.WARNING}[unavailable]{bcolors.ENDC}"

                        print(f"\t\t{state_interface.name} {available_str}")

        return 0
