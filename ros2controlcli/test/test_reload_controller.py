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

import unittest

from controller_manager_msgs.msg import ControllerState

from ros2controlcli.verb.reload_controller import check_reload_possible


class TestReloadController(unittest.TestCase):
    def test_restorable_states_are_accepted(self):
        for state in ("unconfigured", "inactive", "active"):
            self.assertIsNone(check_reload_possible(ControllerState(name="ctrl", state=state)))

    def test_not_loaded_is_rejected(self):
        self.assertIsNotNone(check_reload_possible(None))

    def test_finalized_is_rejected(self):
        self.assertIsNotNone(
            check_reload_possible(ControllerState(name="ctrl", state="finalized"))
        )

    def test_chained_is_rejected(self):
        controller = ControllerState(name="ctrl", state="active", is_chained=True)
        self.assertIsNotNone(check_reload_possible(controller))
