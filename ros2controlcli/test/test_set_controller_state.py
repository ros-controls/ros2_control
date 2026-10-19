# Copyright 2026 Levin
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
from types import SimpleNamespace
from unittest.mock import patch

from controller_manager_msgs.srv import SwitchController
from ros2controlcli.verb.set_controller_state import SetControllerStateVerb


class TestSetControllerState(unittest.TestCase):
    def patch_target(self, target, **kwargs):
        patcher = patch(target, **kwargs)
        result = patcher.start()
        self.addCleanup(patcher.stop)
        return result

    def setUp(self):
        self.node = object()
        self.strategy = self.patch_target("ros2controlcli.verb.set_controller_state.NodeStrategy")
        self.strategy.return_value.direct_node.__enter__.return_value = self.node
        self.list_controllers = self.patch_target(
            "ros2controlcli.verb.set_controller_state.list_controllers",
            return_value=SimpleNamespace(
                controller=[SimpleNamespace(name="arm", state="unconfigured")]
            ),
        )
        self.configure = self.patch_target(
            "ros2controlcli.verb.set_controller_state.configure_controller",
            return_value=SimpleNamespace(ok=True),
        )
        self.switch = self.patch_target(
            "ros2controlcli.verb.set_controller_state.switch_controllers",
            return_value=SimpleNamespace(ok=True),
        )
        self.cleanup = self.patch_target(
            "ros2controlcli.verb.set_controller_state.cleanup_controller",
            return_value=SimpleNamespace(ok=True),
        )

    @staticmethod
    def args(target_state):
        return SimpleNamespace(
            controller_manager="controller_manager",
            controller_name="arm",
            state=target_state,
        )

    def test_reaches_active_by_configuring_then_activating(self):
        result = SetControllerStateVerb().main(args=self.args("active"))

        self.assertEqual(result, 0)
        self.configure.assert_called_once_with(self.node, "controller_manager", "arm")
        self.switch.assert_called_once_with(
            self.node,
            "controller_manager",
            [],
            ["arm"],
            SwitchController.Request.STRICT,
            True,
            5.0,
        )
        self.cleanup.assert_not_called()

    def test_reaches_unconfigured_by_deactivating_then_cleaning_up(self):
        self.list_controllers.return_value.controller[0].state = "active"

        result = SetControllerStateVerb().main(args=self.args("unconfigured"))

        self.assertEqual(result, 0)
        self.switch.assert_called_once_with(
            self.node,
            "controller_manager",
            ["arm"],
            [],
            SwitchController.Request.STRICT,
            True,
            5.0,
        )
        self.cleanup.assert_called_once_with(self.node, "controller_manager", "arm")
        self.configure.assert_not_called()

    def test_reports_intermediate_state_when_a_later_transition_fails(self):
        self.switch.return_value.ok = False

        result = SetControllerStateVerb().main(args=self.args("active"))

        self.assertIn("Error activating arm", result)
        self.assertIn("last successfully reached state was inactive", result)
        self.configure.assert_called_once()
        self.switch.assert_called_once()
        self.cleanup.assert_not_called()

    def test_reports_initial_state_when_configure_fails(self):
        self.configure.return_value.ok = False

        result = SetControllerStateVerb().main(args=self.args("active"))

        self.assertIn("last successfully reached state was unconfigured", result)
        self.configure.assert_called_once()
        self.switch.assert_not_called()

    def test_reports_initial_state_when_deactivate_fails(self):
        self.list_controllers.return_value.controller[0].state = "active"
        self.switch.return_value.ok = False

        result = SetControllerStateVerb().main(args=self.args("unconfigured"))

        self.assertIn("last successfully reached state was active", result)
        self.switch.assert_called_once()
        self.cleanup.assert_not_called()

    def test_reports_inactive_state_when_cleanup_fails(self):
        self.list_controllers.return_value.controller[0].state = "active"
        self.cleanup.return_value.ok = False

        result = SetControllerStateVerb().main(args=self.args("unconfigured"))

        self.assertIn("last successfully reached state was inactive", result)
        self.switch.assert_called_once()
        self.cleanup.assert_called_once()

    def test_same_target_state_is_idempotent(self):
        self.list_controllers.return_value.controller[0].state = "active"

        result = SetControllerStateVerb().main(args=self.args("active"))

        self.assertEqual(result, 0)
        self.configure.assert_not_called()
        self.switch.assert_not_called()
        self.cleanup.assert_not_called()


if __name__ == "__main__":
    unittest.main()
