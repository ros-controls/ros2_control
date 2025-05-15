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
from controller_manager_msgs.srv import SwitchController

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
        strictness_group = parser.add_mutually_exclusive_group(required=False)
        strictness_group.add_argument(
            "--strict",
            help="Set the switch_controllers service strictness to strict",
            action="store_true",
        )
        strictness_group.add_argument(
            "--best-effort",
            help="Set the switch_controllers service strictness to best effort",
            action="store_true",
        )
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
            strictness = 0
            if args.strict:
                strictness = SwitchController.Request.STRICT
            elif args.best_effort:
                strictness = SwitchController.Request.BEST_EFFORT
            response = switch_controllers(
                node,
                args.controller_manager,
                args.deactivate,
                args.activate,
                strictness,
                args.activate_asap,
                args.switch_timeout,
            )
            if not response.ok:
                print(bcolors.FAIL + response.message + bcolors.ENDC)
                return 1

            print(bcolors.OKBLUE + response.message + bcolors.ENDC)
            return 0
