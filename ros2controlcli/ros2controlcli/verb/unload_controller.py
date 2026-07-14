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

from controller_manager import list_controllers, unload_controller, bcolors

from ros2cli.node.direct import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2cli.verb import VerbExtension

from ros2controlcli.api import add_controller_mgr_parsers, LoadedControllerNameCompleter


def get_inactive_controller_names(controllers):
    """
    Return the names of controllers currently in the 'inactive' state.

    :return: list of controller name strings whose state == "inactive"
    """
    return [c.name for c in controllers if c.state == "inactive"]


class UnloadControllerVerb(VerbExtension):
    """Unload a controller in a controller manager."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        arg = parser.add_argument(
            "controller_name", nargs="?", default=None, help="Name of the controller"
        )
        arg.completer = LoadedControllerNameCompleter()
        parser.add_argument(
            "--all-inactive",
            action="store_true",
            help="Unload all controllers currently in the 'inactive' state",
        )
        add_controller_mgr_parsers(parser)

    def main(self, *, args):
        if args.all_inactive and args.controller_name is not None:
            print(
                f"{bcolors.FAIL}Cannot use --all-inactive together with a controller_name{bcolors.ENDC}"
            )
            return 1

        if not args.all_inactive and args.controller_name is None:
            print(
                f"{bcolors.FAIL}Either controller_name or --all-inactive is required{bcolors.ENDC}"
            )
            return 1

        with NodeStrategy(args).direct_node as node:
            if args.all_inactive:
                controllers = list_controllers(node, args.controller_manager).controller
                controller_names = get_inactive_controller_names(controllers)
                if not controller_names:
                    print(f"{bcolors.OKBLUE}No inactive controllers to unload{bcolors.ENDC}")
                    return 0

                any_failed = False
                for controller_name in controller_names:
                    response = unload_controller(node, args.controller_manager, controller_name)
                    if not response.ok:
                        any_failed = True
                        print(
                            f"{bcolors.FAIL}Error unloading controller {controller_name}, check controller_manager logs{bcolors.ENDC}"
                        )
                        continue

                    print(
                        f"{bcolors.OKBLUE}Successfully unloaded controller {controller_name}{bcolors.ENDC}"
                    )
                return 1 if any_failed else 0

            response = unload_controller(node, args.controller_manager, args.controller_name)
            if not response.ok:
                print(
                    f"{bcolors.FAIL}Error unloading controller {args.controller_name}, check controller_manager logs{bcolors.ENDC}"
                )
                return 1

            print(
                f"{bcolors.OKBLUE}Successfully unloaded controller {args.controller_name}{bcolors.ENDC}"
            )
            return 0
