# Copyright 2026 PAL Robotics S.L.
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

from ros2controlcli.verb.unload_controller import get_inactive_controller_names


class TestUnloadController(unittest.TestCase):
    def test_filters_only_inactive(self):
        controllers = [
            ControllerState(name="ctrl_active", state="active"),
            ControllerState(name="ctrl_inactive_1", state="inactive"),
            ControllerState(name="ctrl_unconfigured", state="unconfigured"),
            ControllerState(name="ctrl_inactive_2", state="inactive"),
        ]
        self.assertEqual(
            get_inactive_controller_names(controllers), ["ctrl_inactive_1", "ctrl_inactive_2"]
        )

    def test_no_inactive_returns_empty(self):
        controllers = [
            ControllerState(name="ctrl_active", state="active"),
            ControllerState(name="ctrl_unconfigured", state="unconfigured"),
        ]
        self.assertEqual(get_inactive_controller_names(controllers), [])

    def test_all_inactive(self):
        controllers = [
            ControllerState(name="ctrl_1", state="inactive"),
            ControllerState(name="ctrl_2", state="inactive"),
        ]
        self.assertEqual(get_inactive_controller_names(controllers), ["ctrl_1", "ctrl_2"])
