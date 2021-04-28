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

from controller_manager import configure_controller, load_controller, switch_controllers

from ros2cli.node.direct import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2cli.verb import VerbExtension

from ros2controlcli.api import add_controller_mgr_parsers, ControllerNameCompleter


class LoadVerb(VerbExtension):
    """Load a controller in a controller manager."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        arg = parser.add_argument('controller_name', help='Name of the controller')
        arg.completer = ControllerNameCompleter()
        arg = parser.add_argument(
            '--state',
            choices=['configure', 'start'],
            help='Set the state of the loaded controller',
        )
        add_controller_mgr_parsers(parser)

    def main(self, *, args):
        with NodeStrategy(args) as node:
            response = load_controller(node, args.controller_manager, args.controller_name)
            if not response.ok:
                return 'Error loading controller, check controller_manager logs'

            if not args.state:
                return f'Successfully loaded controller {args.controller_name}'

            # we in any case configure the controller
            response = configure_controller(node, args.controller_manager, args.controller_name)
            if not response.ok:
                return 'Error configuring controller'

            if args.state == 'start':
                response = switch_controllers(
                    node, args.controller_manager, [], [args.controller_name], True, True, 5.0
                )
                if not response.ok:
                    return 'Error starting controller, check controller_manager logs'

            return f"Sucessfully loaded controller {args.controller_name} into " \
                   f"state { 'inactive' if args.state == 'configure' else 'active' }"
