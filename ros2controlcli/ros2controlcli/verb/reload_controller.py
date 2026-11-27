# Copyright 2026 b»robotized group
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

import os
from argparse import OPTIONAL

from controller_manager import (
    bcolors,
    configure_controller,
    list_controllers,
    load_controller,
    set_controller_parameters_from_param_files,
    switch_controllers,
    unload_controller,
)

from ros2cli.node.direct import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2cli.verb import VerbExtension

from ros2controlcli.api import add_controller_mgr_parsers, LoadedControllerNameCompleter

RESTORABLE_STATES = ("unconfigured", "inactive", "active")


def check_reload_possible(controller):
    """
    Return an error message if the controller cannot be reloaded, else None.

    :param controller: ControllerState of the controller, or None if it is not loaded
    """
    if controller is None:
        return "is not loaded"
    if controller.state not in RESTORABLE_STATES:
        return f"is in state '{controller.state}', expected one of {list(RESTORABLE_STATES)}"
    if controller.is_chained:
        return "exports reference interfaces that other active controllers use"
    return None


class ReloadControllerVerb(VerbExtension):
    """Reload a controller and its parameters, and restore its previous state."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        arg = parser.add_argument("controller_name", help="Name of the controller")
        arg.completer = LoadedControllerNameCompleter()
        parser.add_argument(
            "param_file",
            help="The YAML file with the controller parameters. "
            "If not set, the controller manager uses the file set at the last load.",
            nargs=OPTIONAL,
            default=None,
        )
        add_controller_mgr_parsers(parser)

    def main(self, *, args):
        name = args.controller_name
        cm = args.controller_manager

        def fail(message):
            print(f"{bcolors.FAIL}{message}{bcolors.ENDC}")
            return 1

        if args.param_file:
            if not os.path.isfile(args.param_file):
                return fail(f"Controller parameter file '{args.param_file}' does not exist")
            args.param_file = os.path.abspath(args.param_file)

        with NodeStrategy(args).direct_node as node:
            controllers = list_controllers(node, cm).controller
            controller = next((c for c in controllers if c.name == name), None)
            error = check_reload_possible(controller)
            if error:
                return fail(f"Cannot reload controller '{name}': it {error}")
            previous_state = controller.state

            if previous_state == "active":
                if not switch_controllers(node, cm, [name], [], True, True, 5.0).ok:
                    return fail(f"Error deactivating controller '{name}'")

            if not unload_controller(node, cm, name).ok:
                return fail(f"Error unloading controller '{name}'")

            if args.param_file and not set_controller_parameters_from_param_files(
                node, cm, name, [args.param_file], node.get_namespace()
            ):
                return fail(f"Error setting parameters of controller '{name}', it is unloaded")

            if not load_controller(node, cm, name).ok:
                return fail(f"Error loading controller '{name}', it is unloaded")

            if previous_state in ("inactive", "active"):
                if not configure_controller(node, cm, name).ok:
                    return fail(f"Error configuring controller '{name}', it is unconfigured")

            if previous_state == "active":
                if not switch_controllers(node, cm, [], [name], True, True, 5.0).ok:
                    return fail(f"Error activating controller '{name}', it is inactive")

            print(
                f"{bcolors.OKBLUE}Successfully reloaded controller '{name}' "
                f"into state '{previous_state}'{bcolors.ENDC}"
            )
            return 0
