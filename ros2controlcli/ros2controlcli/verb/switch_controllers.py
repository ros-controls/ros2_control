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

from controller_manager import switch_controllers, bcolors

from ros2cli.node.direct import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2cli.verb import VerbExtension

from ros2controlcli.api import add_controller_mgr_parsers, LoadedControllerNameCompleter


class SwitchControllersVerb(VerbExtension):
    """Switch controllers in a controller manager."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        arg = parser.add_argument(
            "--deactivate",
            nargs="*",
            default=[],
            help="Name of the controllers to be deactivated",
        )
        arg.completer = LoadedControllerNameCompleter(["active"])
        arg = parser.add_argument(
            "--activate",
            nargs="*",
            default=[],
            help="Name of the controllers to be activated",
        )
        arg.completer = LoadedControllerNameCompleter(["inactive"])
        parser.add_argument("--strict", action="store_true", help="Strict switch")
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
        with NodeStrategy(args).direct_node as node:
            response = switch_controllers(
                node,
                args.controller_manager,
                args.deactivate,
                args.activate,
                args.strict,
                args.activate_asap,
                args.switch_timeout,
            )
            if not response.ok:
                print(
                    bcolors.FAIL
                    + "Error switching controllers, check controller_manager logs"
                    + bcolors.ENDC
                )
                return 1

            print(bcolors.OKBLUE + "Successfully switched controllers" + bcolors.ENDC)
            return 0
