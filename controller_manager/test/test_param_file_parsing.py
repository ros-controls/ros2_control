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

from unittest.mock import MagicMock

import pytest

from controller_manager.controller_manager_services import (
    get_parameter_from_param_files,
    get_params_files_with_controller_parameters,
)

PARAMS = "ros__parameters:\n  type: dummy/Controller\n"


def _indent(text, level):
    return "".join("  " * level + line + "\n" for line in text.splitlines())


@pytest.mark.parametrize(
    "namespace, yaml_text",
    [
        ("/", "ctrl:\n" + _indent(PARAMS, 1)),
        ("/", "/ctrl:\n" + _indent(PARAMS, 1)),
        ("/", "/**:\n" + _indent(PARAMS, 1)),
        ("/", "/**:\n  ctrl:\n" + _indent(PARAMS, 2)),
        ("/ns", "/ns/ctrl:\n" + _indent(PARAMS, 1)),
        ("/ns", "/**/ctrl:\n" + _indent(PARAMS, 1)),
        ("/ns", "/**/ns/ctrl:\n" + _indent(PARAMS, 1)),
        # nested namespace forms reported in issue #3718
        ("/ns", "/ns:\n  ctrl:\n" + _indent(PARAMS, 2)),
        ("/ns", "ns:\n  ctrl:\n" + _indent(PARAMS, 2)),
        ("/ns", "ns/ctrl:\n" + _indent(PARAMS, 1)),
        ("/a/b", "/a:\n  b:\n    ctrl:\n" + _indent(PARAMS, 3)),
        ("/ns", "/**:\n  ns:\n    ctrl:\n" + _indent(PARAMS, 3)),
        # wildcards matched like rclcpp's is_node_name_matched
        ("/a", "/*/ctrl:\n" + _indent(PARAMS, 1)),
        ("/a/b", "/a/**/ctrl:\n" + _indent(PARAMS, 1)),
        ("/", "/*:\n" + _indent(PARAMS, 1)),
        # an entry without namespace must not hide a valid namespaced one
        ("/ns", "ctrl:\n  ros__parameters:\n    type: other\n/ns/ctrl:\n" + _indent(PARAMS, 1)),
    ],
)
def test_controller_found_in_param_file(tmp_path, namespace, yaml_text):
    param_file = tmp_path / "params.yaml"
    param_file.write_text(yaml_text)
    node = MagicMock()
    files = [str(param_file)]
    assert get_params_files_with_controller_parameters(node, "ctrl", namespace, files) == files
    assert (
        get_parameter_from_param_files(node, "ctrl", namespace, files, "type")
        == "dummy/Controller"
    )


@pytest.mark.parametrize(
    "namespace, yaml_text",
    [
        ("/ns", "ctrl:\n" + _indent(PARAMS, 1)),
        ("/ns", "/other:\n  ctrl:\n" + _indent(PARAMS, 2)),
        ("/", "/ns:\n  ctrl:\n" + _indent(PARAMS, 2)),
        # rcl rejects a nested key with a leading slash (repeated slash)
        ("/ns", "/ns:\n  /ctrl:\n" + _indent(PARAMS, 2)),
    ],
)
def test_controller_not_found_in_param_file(tmp_path, namespace, yaml_text):
    param_file = tmp_path / "params.yaml"
    param_file.write_text(yaml_text)
    node = MagicMock()
    files = [str(param_file)]
    assert get_params_files_with_controller_parameters(node, "ctrl", namespace, files) == []
    assert get_parameter_from_param_files(node, "ctrl", namespace, files, "type") is None


def _write(tmp_path, name, yaml_text):
    param_file = tmp_path / name
    param_file.write_text(yaml_text)
    return str(param_file)


def test_last_matching_entry_in_file_wins(tmp_path):
    files = [
        _write(
            tmp_path,
            "p.yaml",
            "/ns/ctrl:\n  ros__parameters:\n    type: A\n"
            "/**:\n  ros__parameters:\n    type: B\n",
        )
    ]
    assert get_parameter_from_param_files(MagicMock(), "ctrl", "/ns", files, "type") == "B"


def test_later_file_overrides_earlier(tmp_path):
    files = [
        _write(tmp_path, "a.yaml", "/ns/ctrl:\n  ros__parameters:\n    type: A\n"),
        _write(tmp_path, "b.yaml", "/**:\n  ros__parameters:\n    type: B\n"),
    ]
    assert get_parameter_from_param_files(MagicMock(), "ctrl", "/ns", files, "type") == "B"


def test_duplicate_node_entries_are_merged(tmp_path):
    # rcl merges entries naming the same node into the slot of its first appearance, the later
    # entry only overriding the keys it sets. So the /** entry below still wins over "ns: ctrl:"
    files = [
        _write(
            tmp_path,
            "p.yaml",
            "/ns/ctrl:\n  ros__parameters:\n    type: A\n    fallback_controllers: [f]\n"
            "/**:\n  ros__parameters:\n    type: C\n"
            "ns:\n  ctrl:\n    ros__parameters:\n      type: B\n",
        )
    ]
    node = MagicMock()
    assert get_parameter_from_param_files(node, "ctrl", "/ns", files, "type") == "C"
    assert get_parameter_from_param_files(node, "ctrl", "/ns", files, "fallback_controllers") == [
        "f"
    ]
