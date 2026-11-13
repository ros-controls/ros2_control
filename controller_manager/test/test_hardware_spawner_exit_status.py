# Copyright 2026 ros2_control Development Team
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

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from lifecycle_msgs.msg import State

from controller_manager import hardware_spawner


@pytest.fixture
def spawner(monkeypatch):
    node = MagicMock()
    node.get_namespace.return_value = "/"
    monkeypatch.setattr(hardware_spawner, "Node", lambda name: node)
    monkeypatch.setattr(hardware_spawner.rclpy, "init", MagicMock())
    monkeypatch.setattr(hardware_spawner.rclpy, "shutdown", MagicMock())
    return node


def run_spawner(monkeypatch, components, mode):
    monkeypatch.setattr(
        hardware_spawner.rclpy.utilities,
        "remove_ros_args",
        lambda **kwargs: ["hardware_spawner", *components, mode],
    )
    result = hardware_spawner.main()
    return 0 if result is None else result


@pytest.mark.parametrize("mode", ["--activate", "--configure"])
@pytest.mark.parametrize("outcome", ["success", "rejected", "wrong_state"])
def test_transition_exit_status(monkeypatch, spawner, mode, outcome):
    monkeypatch.setattr(hardware_spawner, "is_hardware_component_loaded", lambda *args: True)

    def set_state(node, manager, component, target):
        state = (
            State(id=State.PRIMARY_STATE_UNCONFIGURED, label="unconfigured")
            if outcome == "wrong_state"
            else target
        )
        return SimpleNamespace(ok=outcome != "rejected", state=state)

    monkeypatch.setattr(hardware_spawner, "set_hardware_component_state", set_state)
    assert run_spawner(monkeypatch, ["TestHardware"], mode) == (0 if outcome == "success" else 1)


@pytest.mark.parametrize("mode", ["--activate", "--configure"])
def test_missing_component_exit_status(monkeypatch, spawner, mode):
    monkeypatch.setattr(hardware_spawner, "is_hardware_component_loaded", lambda *args: False)
    set_state = MagicMock()
    monkeypatch.setattr(hardware_spawner, "set_hardware_component_state", set_state)
    assert run_spawner(monkeypatch, ["MissingHardware"], mode) == 1
    set_state.assert_not_called()


@pytest.mark.parametrize("mode", ["--activate", "--configure"])
@pytest.mark.parametrize("failure", ["missing", "rejected", "wrong_state"])
@pytest.mark.parametrize("failed_index", [0, 1])
def test_all_components_are_attempted(monkeypatch, spawner, mode, failure, failed_index):
    components = ["FirstHardware", "SecondHardware"]
    failed_component = components[failed_index]
    attempted = []
    monkeypatch.setattr(
        hardware_spawner,
        "is_hardware_component_loaded",
        lambda node, manager, component, timeout: not (
            failure == "missing" and component == failed_component
        ),
    )

    def set_state(node, manager, component, target):
        attempted.append(component)
        failed = component == failed_component
        state = (
            State(id=State.PRIMARY_STATE_UNCONFIGURED, label="unconfigured")
            if failed and failure == "wrong_state"
            else target
        )
        return SimpleNamespace(ok=not (failed and failure == "rejected"), state=state)

    monkeypatch.setattr(hardware_spawner, "set_hardware_component_state", set_state)
    assert run_spawner(monkeypatch, components, mode) == 1
    assert attempted == [
        component
        for component in components
        if not (failure == "missing" and component == failed_component)
    ]
