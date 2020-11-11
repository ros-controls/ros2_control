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

from ros2cli.node.direct import add_arguments
from ros2cli.verb import VerbExtension
from ros2controlcli.api import add_controller_mgr_parsers, reload_controller_libraries


class ReloadLibrariesVerb(VerbExtension):
    """Reload controller libraries."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument(
            '--force-kill', action='store_true',
            help='Force stop of loaded controllers')
        add_controller_mgr_parsers(parser)

    def main(self, *, args):
        response = reload_controller_libraries(args.controller_manager, force_kill=args.force_kill)
        if response.ok:
            print('Reload successful')
        else:
            print('Error in reload, check controller manager logs')
        return response.ok
