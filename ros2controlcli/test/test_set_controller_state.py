# Copyright 2026 Jooyoung Lim
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

import argparse
import unittest
from unittest.mock import MagicMock, patch

from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import (
    CleanupController,
    ConfigureController,
    SwitchController,
)

from ros2controlcli.verb.set_controller_state import SetControllerStateVerb, trigger_transition

MODULE = "ros2controlcli.verb.set_controller_state"


class TestSetControllerStateArguments(unittest.TestCase):
    def setUp(self):
        self.parser = argparse.ArgumentParser()
        SetControllerStateVerb().add_arguments(self.parser, "ros2 control")

    def test_accepts_primary_states_and_transitions(self):
        for state in [
            "unconfigured",
            "inactive",
            "active",
            "configure",
            "cleanup",
            "activate",
            "deactivate",
        ]:
            with self.subTest(state=state):
                self.assertEqual(self.parser.parse_args(["ctrl", state]).state, state)

    def test_rejects_shutdown(self):
        with self.assertRaises(SystemExit), patch("sys.stderr"):
            self.parser.parse_args(["ctrl", "shutdown"])


class TestTriggerTransition(unittest.TestCase):
    def test_configure_and_cleanup_call_their_services(self):
        for transition, service, response_type in [
            ("configure", "configure_controller", ConfigureController.Response),
            ("cleanup", "cleanup_controller", CleanupController.Response),
        ]:
            with self.subTest(transition=transition), patch(
                f"{MODULE}.{service}", return_value=response_type(ok=True)
            ) as mock_service, patch(f"{MODULE}.switch_controllers") as mock_switch:
                node = MagicMock()
                self.assertEqual(trigger_transition(node, "/cm", "ctrl", transition), 0)
                mock_service.assert_called_once_with(node, "/cm", "ctrl")
                mock_switch.assert_not_called()

    def test_activate_and_deactivate_use_a_strict_switch(self):
        for transition, deactivate, activate in [
            ("activate", [], ["ctrl"]),
            ("deactivate", ["ctrl"], []),
        ]:
            with self.subTest(transition=transition), patch(
                f"{MODULE}.switch_controllers", return_value=SwitchController.Response(ok=True)
            ) as mock_switch:
                node = MagicMock()
                self.assertEqual(trigger_transition(node, "/cm", "ctrl", transition), 0)
                mock_switch.assert_called_once_with(
                    node,
                    "/cm",
                    deactivate,
                    activate,
                    SwitchController.Request.STRICT,
                    True,
                    5.0,
                )

    def test_failed_switch_reports_the_reason(self):
        reason = "Controller with name 'ctrl' can not be deactivated since it is not active."
        with patch(
            f"{MODULE}.switch_controllers",
            return_value=SwitchController.Response(ok=False, message=reason),
        ):
            result = trigger_transition(MagicMock(), "/cm", "ctrl", "deactivate")
        self.assertIsInstance(result, str)
        self.assertIn("'deactivate'", result)
        self.assertIn(reason, result)

    def test_unknown_transition_raises(self):
        with self.assertRaises(ValueError):
            trigger_transition(MagicMock(), "/cm", "ctrl", "shutdown")


class TestMain(unittest.TestCase):
    def _run_main(self, state, controllers):
        args = argparse.Namespace(controller_name="ctrl", state=state, controller_manager="/cm")
        with patch(f"{MODULE}.NodeStrategy"), patch(
            f"{MODULE}.list_controllers", return_value=MagicMock(controller=controllers)
        ), patch(f"{MODULE}.trigger_transition", return_value=0) as mock_trigger, patch(
            f"{MODULE}.switch_controllers", return_value=SwitchController.Response(ok=True)
        ) as mock_switch:
            result = SetControllerStateVerb().main(args=args)
        return result, mock_trigger, mock_switch

    def test_transition_skips_the_current_state_check(self):
        # 'configure' on an active controller is passed on; the controller manager decides
        result, mock_trigger, _ = self._run_main(
            "configure", [ControllerState(name="ctrl", state="active")]
        )
        self.assertEqual(result, 0)
        mock_trigger.assert_called_once()
        self.assertEqual(mock_trigger.call_args.args[1:], ("/cm", "ctrl", "configure"))

    def test_transition_on_unloaded_controller_is_rejected(self):
        result, mock_trigger, _ = self._run_main("activate", [])
        self.assertIn("does not seem to be loaded", result)
        mock_trigger.assert_not_called()

    def test_primary_state_keeps_its_previous_behavior(self):
        # 'active' still checks the current state and uses the previous switch arguments
        result, mock_trigger, mock_switch = self._run_main(
            "active", [ControllerState(name="ctrl", state="inactive")]
        )
        self.assertEqual(result, 0)
        mock_trigger.assert_not_called()
        mock_switch.assert_called_once()
        self.assertEqual(mock_switch.call_args.args[1:], ("/cm", [], ["ctrl"], True, True, 5.0))
