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

import argparse
import unittest
from unittest.mock import patch

from ros2controlcli.verb.set_controller_state import SetControllerStateVerb


class TestSetControllerStateChoices(unittest.TestCase):
    def setUp(self):
        with patch("ros2controlcli.verb.set_controller_state.add_arguments"):
            self.verb = SetControllerStateVerb()
            self.parser = argparse.ArgumentParser()
            self.verb.add_arguments(self.parser, "set_controller_state")

    def test_primary_states_accepted(self):
        for state in ["unconfigured", "inactive", "active"]:
            args = self.parser.parse_args(["my_controller", state])
            self.assertEqual(args.state, state)

    def test_transition_verbs_accepted(self):
        for state in ["configure", "activate", "deactivate", "cleanup", "shutdown"]:
            args = self.parser.parse_args(["my_controller", state])
            self.assertEqual(args.state, state)

    def test_invalid_state_rejected(self):
        with self.assertRaises(SystemExit):
            self.parser.parse_args(["my_controller", "running"])
