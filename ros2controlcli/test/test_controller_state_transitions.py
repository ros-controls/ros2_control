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

from ros2controlcli.state_transitions import plan_state_transitions


class TestControllerStateTransitions(unittest.TestCase):
    def test_plans_all_transitions_between_primary_states(self):
        expected = {
            ("unconfigured", "unconfigured"): [],
            ("unconfigured", "inactive"): ["configure"],
            ("unconfigured", "active"): ["configure", "activate"],
            ("inactive", "unconfigured"): ["cleanup"],
            ("inactive", "inactive"): [],
            ("inactive", "active"): ["activate"],
            ("active", "unconfigured"): ["deactivate", "cleanup"],
            ("active", "inactive"): ["deactivate"],
            ("active", "active"): [],
        }

        for (current_state, target_state), transitions in expected.items():
            with self.subTest(current_state=current_state, target_state=target_state):
                self.assertEqual(plan_state_transitions(current_state, target_state), transitions)

    def test_rejects_states_outside_the_supported_primary_lifecycle(self):
        with self.assertRaises(ValueError):
            plan_state_transitions("finalized", "active")

        with self.assertRaises(ValueError):
            plan_state_transitions("active", "finalized")


if __name__ == "__main__":
    unittest.main()
