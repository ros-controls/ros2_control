#!/usr/bin/env python3
# Copyright 2025 Robert Kwan
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
"""Unit tests for launch_utils.py functions."""

import unittest
from unittest.mock import patch

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

from controller_manager.launch_utils import (
    generate_controllers_spawner_launch_description,
    generate_controllers_spawner_launch_description_from_dict,
    generate_load_controller_launch_description,
)


class TestLaunchUtils(unittest.TestCase):
    """
    Test suite for launch_utils.py functions.

    Tests three entry points:
    1. generate_controllers_spawner_launch_description (from list)
    2. generate_controllers_spawner_launch_description_from_dict (from dict)
    3. generate_load_controller_launch_description (load controllers)
    """

    # ------------------------------------------------------------------
    # Helper methods
    # ------------------------------------------------------------------
    def _extract_actions(self, result):
        """Return a list of launch actions, regardless of type."""
        if isinstance(result, list):
            return result
        elif hasattr(result, "entities"):  # LaunchDescription in newer ROS2
            return list(result.entities)
        else:
            return [result]

    def _assert_launch_result(self, result, min_len=0):
        """Verify that result is list- or LaunchDescription-like."""
        self.assertTrue(
            isinstance(result, (list, LaunchDescription)),
            f"Unexpected result type: {type(result)}",
        )
        actions = self._extract_actions(result)
        self.assertGreaterEqual(len(actions), min_len)
        for act in actions:
            self.assertTrue(
                hasattr(act, "execute") or hasattr(act, "visit"),
                f"Invalid action type: {type(act)}",
            )
        return actions

    # ------------------------------------------------------------------
    # Launch utility tests
    # ------------------------------------------------------------------
    def test_generate_controllers_spawner_from_list(self):
        controllers = ["test_controller_1", "test_controller_2"]
        result = generate_controllers_spawner_launch_description(controllers)
        actions = self._assert_launch_result(result, min_len=1)
        self.assertTrue(actions is not None and len(actions) > 0)

    def test_generate_controllers_spawner_from_dict(self):
        """Ensure dict-based API works with string param files (old-style)."""
        controllers = {
            "ctrl_A": ["/tmp/dummy.yaml"],
            "ctrl_B": ["/tmp/dummy.yaml"],
        }
        result = generate_controllers_spawner_launch_description_from_dict(controllers)
        actions = self._extract_actions(result)
        self.assertIsInstance(result, LaunchDescription)
        self.assertEqual(len(actions), 3)
        self.assertIsInstance(actions[-1], OpaqueFunction)

    def test_generate_load_controller_launch_description(self):
        """Test load controller description with valid single string params."""
        controllers = "test_controller_load"
        result = generate_load_controller_launch_description(controllers)
        actions = self._extract_actions(result)
        self.assertIsInstance(result, LaunchDescription)
        self.assertEqual(len(actions), 3)
        self.assertIsInstance(actions[-1], OpaqueFunction)

    def test_spawner_arguments_omit_disabled_flag_and_preserve_empty_values(self):
        for unload_on_kill in ("false", "true"):
            with self.subTest(unload_on_kill=unload_on_kill):
                context = LaunchContext()
                context.launch_configurations["unload_on_kill"] = unload_on_kill
                extra_args = [
                    "--controller-ros-args",
                    "",
                    "--controller-ros-args",
                    "-r foo:=bar",
                ]
                result = generate_controllers_spawner_launch_description(
                    ["ctrl_1"], extra_spawner_args=extra_args
                )
                actions = self._extract_actions(result)
                actions[0].execute(context)
                actions[1].execute(context)
                with patch("controller_manager.launch_utils.Node") as node:
                    self.assertEqual(actions[-1].execute(context), [node.return_value])
                    node.assert_called_once()
                    arguments = [
                        perform_substitutions(context, normalize_to_list_of_substitutions(arg))
                        for arg in node.call_args.kwargs["arguments"]
                    ]
                expected = ["ctrl_1", "--controller-manager", "controller_manager"]
                if unload_on_kill == "true":
                    expected.append("--unload-on-kill")
                self.assertEqual(arguments, expected + extra_args)


if __name__ == "__main__":
    unittest.main()
