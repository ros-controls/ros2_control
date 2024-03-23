# Copyright 2023 ROS2Control Developer Team
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

from controller_manager import list_hardware_components, set_hardware_component_state

from ros2cli.node.direct import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2cli.verb import VerbExtension
from lifecycle_msgs.msg import State

from ros2controlcli.api import add_controller_mgr_parsers, LoadedHardwareComponentNameCompleter


class SetHardwareComponentStateVerb(VerbExtension):
    """Adjust the state of the hardware component."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        arg = parser.add_argument(
            "hardware_component_name", help="Name of the hardware_component to be changed"
        )
        arg.completer = LoadedHardwareComponentNameCompleter()
        arg = parser.add_argument(
            "state",
            choices=["unconfigured", "inactive", "active"],
            help="State in which the hardware component should be changed to",
        )
        add_controller_mgr_parsers(parser)

    def main(self, *, args):
        with NodeStrategy(args) as node:
            hardware_components = list_hardware_components(node, args.controller_manager).component

            try:
                matched_hardware_components = [
                    c for c in hardware_components if c.name == args.hardware_component_name
                ][0]
            except IndexError:
                return f"component {args.hardware_component_name} does not seem to be loaded"

            if args.state == "unconfigured":
                if matched_hardware_components.state.label != "inactive":
                    return (
                        f"cannot cleanup {matched_hardware_components.name} "
                        f"from its current state {matched_hardware_components.state}"
                    )

                unconfigured_state = State()
                unconfigured_state.id = State.PRIMARY_STATE_UNCONFIGURED
                unconfigured_state.label = "unconfigured"

                response = set_hardware_component_state(
                    node, args.controller_manager, args.hardware_component_name, unconfigured_state
                )
                if not response.ok:
                    return "Error cleaning up hardware component, check controller_manager logs"

                print(f"Successfully set {args.hardware_component_name} to state {response.state}")
                return 0

            if args.state == "inactive":
                inactive_state = State()
                inactive_state.id = State.PRIMARY_STATE_INACTIVE
                inactive_state.label = "inactive"

                if matched_hardware_components.state.label == "unconfigured":
                    response = set_hardware_component_state(
                        node, args.controller_manager, args.hardware_component_name, inactive_state
                    )
                    if not response.ok:
                        return (
                            "Error configuring hardware component, check controller_manager logs"
                        )

                    print(
                        f"Successfully set {args.hardware_component_name} to state {response.state}"
                    )
                    return 0

                elif matched_hardware_components.state.label == "active":
                    response = set_hardware_component_state(
                        node, args.controller_manager, args.hardware_component_name, inactive_state
                    )
                    if not response.ok:
                        return "Error stopping hardware component, check controller_manager logs"

                    print(
                        f"Successfully set {args.hardware_component_name} to state {response.state}"
                    )
                    return 0

                else:
                    return (
                        f'cannot put {matched_hardware_components.name} in "inactive" state '
                        f"from its current state {matched_hardware_components.state}"
                    )

            if args.state == "active":
                if matched_hardware_components.state.label != "inactive":
                    return (
                        f"cannot activate {matched_hardware_components.name} "
                        f"from its current state {matched_hardware_components.state}"
                    )

                active_state = State()
                active_state.id = State.PRIMARY_STATE_ACTIVE
                active_state.label = "active"

                response = set_hardware_component_state(
                    node, args.controller_manager, args.hardware_component_name, active_state
                )
                if not response.ok:
                    return "Error activating hardware component, check controller_manager logs"

                print(f"Successfully set {args.hardware_component_name} to state {response.state}")
                return 0
