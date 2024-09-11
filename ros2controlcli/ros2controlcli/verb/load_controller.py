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

from controller_manager import (
    configure_controller,
    load_controller,
    list_controllers,
    switch_controllers,
    set_controller_parameters_from_param_file,
    bcolors,
)

from ros2cli.node.direct import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2cli.verb import VerbExtension

from ros2controlcli.api import add_controller_mgr_parsers, ControllerNameCompleter
import os
from argparse import OPTIONAL


class LoadControllerVerb(VerbExtension):
    """Load a controller in a controller manager."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        arg = parser.add_argument("controller_name", help="Name of the controller")
        arg.completer = ControllerNameCompleter()
        arg = parser.add_argument(
            "param_file",
            help="The YAML file with the controller parameters",
            nargs=OPTIONAL,
            default=None,
        )
        arg = parser.add_argument(
            "--set-state",
            choices=["inactive", "active"],
            help="Set the state of the loaded controller",
            default=None,
        )
        add_controller_mgr_parsers(parser)

    def main(self, *, args):
        with NodeStrategy(args).direct_node as node:
            controllers = list_controllers(node, args.controller_manager, 20.0).controller
            if any(c.name == args.controller_name for c in controllers):
                print(
                    f"{bcolors.WARNING}Controller : {args.controller_name} already loaded, skipping load_controller!{bcolors.ENDC}"
                )
            else:
                if args.param_file:
                    if not os.path.exists(args.param_file):
                        print(
                            f"{bcolors.FAIL}Controller parameter file : {args.param_file} does not exist, Aborting!{bcolors.ENDC}"
                        )
                        return 1
                    if not os.path.isabs(args.param_file):
                        args.param_file = os.path.join(os.getcwd(), args.param_file)

                    if not set_controller_parameters_from_param_file(
                        node,
                        args.controller_manager,
                        args.controller_name,
                        args.param_file,
                        node.get_namespace(),
                    ):
                        return 1

                ret = load_controller(node, args.controller_manager, args.controller_name)
                if not ret.ok:
                    print(
                        f"{bcolors.FAIL}Failed loading controller {args.controller_name} check controller_manager logs{bcolors.ENDC}"
                    )
                    return 1
                print(
                    f"{bcolors.OKBLUE}Successfully loaded controller {args.controller_name}{bcolors.ENDC}"
                )

            if args.set_state:

                # we in any case configure the controller
                response = configure_controller(
                    node, args.controller_manager, args.controller_name
                )
                if not response.ok:
                    print(
                        f"{bcolors.FAIL}Error configuring controller : {args.controller_name}{bcolors.ENDC}"
                    )
                    return 1

                if args.set_state == "active":
                    response = switch_controllers(
                        node, args.controller_manager, [], [args.controller_name], True, True, 5.0
                    )
                    if not response.ok:
                        print(
                            f"{bcolors.FAIL}Error activating controller : {args.controller_name}, check controller_manager logs{bcolors.ENDC}"
                        )
                        return 1

                print(
                    f"{bcolors.OKBLUE}Successfully loaded controller {args.controller_name} into state {args.set_state}{bcolors.ENDC}"
                )
                return 0
