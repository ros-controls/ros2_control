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

from controller_manager import switch_controllers

from ros2cli.node.direct import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2cli.verb import VerbExtension

from ros2controlcli.api import add_controller_mgr_parsers, LoadedControllerNameCompleter


class SwitchControllersVerb(VerbExtension):
    """Switch controllers in a controller manager."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        arg = parser.add_argument(
            "--stop",
            nargs="*",
            default=[],
            help="Name of the controllers to be deactivated",
        )
        arg.completer = LoadedControllerNameCompleter(["active"])
        arg = parser.add_argument(
            "--deactivate",
            nargs="*",
            default=[],
            help="Name of the controllers to be deactivated",
        )
        arg.completer = LoadedControllerNameCompleter(["active"])
        arg = parser.add_argument(
            "--start",
            nargs="*",
            default=[],
            help="Name of the controllers to be activated",
        )
        arg.completer = LoadedControllerNameCompleter(["inactive"])
        arg = parser.add_argument(
            "--activate",
            nargs="*",
            default=[],
            help="Name of the controllers to be activated",
        )
        arg.completer = LoadedControllerNameCompleter(["inactive"])
        parser.add_argument("--strict", action="store_true", help="Strict switch")
        parser.add_argument("--start-asap", action="store_true", help="Start asap controllers")
        parser.add_argument("--activate-asap", action="store_true", help="Start asap controllers")
        parser.add_argument(
            "--switch-timeout",
            default=5.0,
            required=False,
            help="Timeout for switching controllers",
        )
        arg.completer = LoadedControllerNameCompleter(["inactive"])
        add_controller_mgr_parsers(parser)

    def main(self, *, args):
        if args.stop:
            print('"--stop" flag is deprecated, use "--deactivate" instead!')
            args.deactivate = args.stop
        if args.start:
            print('"--start" flag is deprecated, use "--activate" instead!')
            args.activate = args.start
        if args.start_asap:
            print('"--start-asap" flag is deprecated, use "--activate-asap" instead!')
            args.activate_asap = args.start_asap

        with NodeStrategy(args) as node:
            response = switch_controllers(
                node,
                args.controller_manager,
                args.deactivate,
                args.activate,
                args.strict,
                args.start_asap,
                args.switch_timeout,
            )
            if not response.ok:
                return "Error switching controllers, check controller_manager logs"

            print("Successfully switched controllers")
            return 0
