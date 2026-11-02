#!/usr/bin/env python3
# Copyright 2026 the ros2_control maintainers
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
"""Parameter file selection.

A ROS 2 parameter file can name a node either as one key or as nested sections::

    /ns/controller:          /ns:
      ros__parameters:         controller:
        dummy: 0                 ros__parameters:
                                   dummy: 0

Both describe the same node. Only the first form used to be recognised, so a
file written the second way was silently dropped.
"""

import os
import tempfile
import unittest

from controller_manager.controller_manager_services import (
    get_params_files_with_controller_parameters,
)


class _Logger:
    def __init__(self):
        self.fatal_messages = []

    def fatal(self, message):
        self.fatal_messages.append(message)


class _Node:
    """Only the logger is used by the function under test."""

    def __init__(self):
        self._logger = _Logger()

    def get_logger(self):
        return self._logger


class TestGetParamsFilesWithControllerParameters(unittest.TestCase):

    def setUp(self):
        self._tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmpdir.cleanup)
        self.node = _Node()

    def _write(self, content, name="params.yaml"):
        path = os.path.join(self._tmpdir.name, name)
        with open(path, "w") as stream:
            stream.write(content)
        return path

    def _selected(self, content, controller_name, namespace, name="params.yaml"):
        path = self._write(content, name)
        return get_params_files_with_controller_parameters(
            self.node, controller_name, namespace, [path]) == [path]

    def test_flat_node_name_is_selected(self):
        self.assertTrue(self._selected(
            "controller:\n  ros__parameters:\n    dummy: 0\n", "controller", "/"))

    def test_flat_namespaced_node_name_is_selected(self):
        self.assertTrue(self._selected(
            "/ns/controller:\n  ros__parameters:\n    dummy: 0\n", "controller", "/ns"))

    def test_nested_namespaced_node_is_selected(self):
        """The form from the issue: the namespace is a section, not part of the key."""
        self.assertTrue(self._selected(
            "/ns:\n  controller:\n    ros__parameters:\n      dummy: 0\n",
            "controller", "/ns"))

    def test_nested_node_without_leading_slash_is_selected(self):
        self.assertTrue(self._selected(
            "ns:\n  controller:\n    ros__parameters:\n      dummy: 0\n",
            "controller", "ns"))

    def test_wildcard_node_is_selected(self):
        self.assertTrue(self._selected(
            '"/**/controller":\n  ros__parameters:\n    dummy: 0\n', "controller", "/"))

    def test_wildcard_parameters_block_is_selected(self):
        self.assertTrue(self._selected(
            '"/**":\n  ros__parameters:\n    dummy: 0\n', "controller", "/"))

    def test_unrelated_controller_is_rejected(self):
        self.assertFalse(self._selected(
            "/ns/other_controller:\n  ros__parameters:\n    dummy: 0\n",
            "controller", "/ns"))

    def test_nested_unrelated_controller_is_rejected(self):
        self.assertFalse(self._selected(
            "/ns:\n  other_controller:\n    ros__parameters:\n      dummy: 0\n",
            "controller", "/ns"))

    def test_nested_controller_under_wrong_namespace_is_rejected(self):
        self.assertFalse(self._selected(
            "/other_ns:\n  controller:\n    ros__parameters:\n      dummy: 0\n",
            "controller", "/ns"))

    def test_bare_name_under_a_namespace_warns(self):
        """A file that names the controller without its namespace is a configuration error."""
        self.assertFalse(self._selected(
            "controller:\n  ros__parameters:\n    dummy: 0\n", "controller", "/ns"))
        self.assertEqual(len(self.node.get_logger().fatal_messages), 1)

    def test_yaml_levels_that_are_not_nodes_do_not_select_a_file(self):
        """Filenames and similar keys must not be mistaken for node names."""
        self.assertFalse(self._selected(
            "some_controller:\n  ros__parameters:\n    dummy: 0\n", "controller", "/"))

    def test_each_file_is_selected_at_most_once(self):
        flat = self._write(
            "/ns/controller:\n  ros__parameters:\n    dummy: 0\n", "flat.yaml")
        nested = self._write(
            "/ns:\n  controller:\n    ros__parameters:\n      dummy: 0\n", "nested.yaml")
        self.assertEqual(
            get_params_files_with_controller_parameters(
                self.node, "controller", "/ns", [flat, nested]),
            [flat, nested])

    def test_empty_file_is_skipped(self):
        self.assertFalse(self._selected("", "controller", "/"))


if __name__ == "__main__":
    unittest.main()
