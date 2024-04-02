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

from controller_manager import configure_controller, list_controllers, switch_controllers

from ros2cli.node.direct import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2cli.verb import VerbExtension

from ros2controlcli.api import add_controller_mgr_parsers, LoadedControllerNameCompleter


class SetControllerStateVerb(VerbExtension):
    """Adjust the state of the controller."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        arg = parser.add_argument("controller_name", help="Name of the controller to be changed")
        arg.completer = LoadedControllerNameCompleter()
        arg = parser.add_argument(
            "state",
            # choices=['unconfigured', 'inactive', 'active'], TODO(destogl): when cleanup is impl
            choices=["inactive", "active"],
            help="State in which the controller should be changed to",
        )
        add_controller_mgr_parsers(parser)

    def main(self, *, args):
        with NodeStrategy(args) as node:
            controllers = list_controllers(node, args.controller_manager).controller

            try:
                matched_controller = [c for c in controllers if c.name == args.controller_name][0]
            except IndexError:
                return f"controller {args.controller_name} does not seem to be loaded"

            # TODO(destogl): This has to be implemented in CLI and controller manager
            # if args.state == 'unconfigured':
            #     if matched_controller.state != 'inactive':
            #         return f'cannot cleanup {matched_controller.name} ' \
            #                f'from its current state {matched_controller.state}'
            #     response = cleanup_controller(
            #         node, args.controller_manager, args.controller_name
            #     )
            #     if not response.ok:
            #         return 'Error cleaning up controller, check controller_manager logs'
            #          #      print(f'successfully cleaned up {args.controller_name}')
            #     return 0

            if args.state == "configure":
                args.state = "inactive"
                print('Setting state "configure" is deprecated, use "inactive" instead!')

            if args.state == "stop":
                args.state = "inactive"
                print('Setting state "stop" is deprecated, use "inactive" instead!')

            if args.state == "inactive":
                if matched_controller.state == "unconfigured":
                    response = configure_controller(
                        node, args.controller_manager, args.controller_name
                    )
                    if not response.ok:
                        return "Error configuring controller, check controller_manager logs"

                    print(f"Successfully configured {args.controller_name}")
                    return 0

                elif matched_controller.state == "active":
                    response = switch_controllers(
                        node, args.controller_manager, [args.controller_name], [], True, True, 5.0
                    )
                    if not response.ok:
                        return "Error stopping controller, check controller_manager logs"

                    print(f"Successfully deactivated {args.controller_name}")
                    return 0

                else:
                    return (
                        f"cannot put {matched_controller.name} in 'inactive' state "
                        f"from its current state {matched_controller.state}"
                    )

            if args.state == "start":
                args.state = "active"
                print('Setting state "start" is deprecated, use "active" instead!')

            if args.state == "active":
                if matched_controller.state != "inactive":
                    return (
                        f"cannot activate {matched_controller.name} "
                        f"from its current state {matched_controller.state}"
                    )
                response = switch_controllers(
                    node, args.controller_manager, [], [args.controller_name], True, True, 5.0
                )
                if not response.ok:
                    return "Error activating controller, check controller_manager logs"

                print(f"Successfully activated {args.controller_name}")
                return 0
