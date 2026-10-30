# Copyright (c) 2026 AIT - Austrian Institute of Technology GmbH
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

import os

os.environ["QT_QPA_PLATFORM"] = "offscreen"

import pytest  # noqa: E402
import rclpy  # noqa: E402
from controller_manager.controller_manager_services import ServiceNotFoundError  # noqa: E402
from controller_manager_msgs.msg import ControllerState, HardwareComponentState  # noqa: E402
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents  # noqa: E402
from python_qt_binding.QtCore import QObject  # noqa: E402
from python_qt_binding.QtWidgets import QApplication  # noqa: E402

from rqt_controller_manager import controller_manager  # noqa: E402


class FakeContext(QObject):
    """Minimal stand-in for the qt_gui plugin context."""

    def __init__(self, node):
        super().__init__()
        self.node = node

    def serial_number(self):
        return 1

    def add_widget(self, widget):
        self.widget = widget


@pytest.fixture
def plugin(monkeypatch):
    app = QApplication.instance() or QApplication([])  # noqa: F841
    rclpy.init()
    node = rclpy.create_node("test_deselect_controller_manager")
    # keep the context alive, it is the Qt parent of the plugin
    context = FakeContext(node)
    plugin = controller_manager.ControllerManager(context)
    # updates are triggered manually
    plugin._update_cm_list_timer.stop()
    plugin._update_ctrl_list_timer.stop()
    plugin._update_hw_components_list_timer.stop()
    monkeypatch.setattr(controller_manager, "_get_parameter_controller_names", lambda *_: [])
    plugin._widget.cm_combo.addItems(["/controller_manager"])
    yield plugin
    plugin.shutdown_plugin()
    node.destroy_node()
    rclpy.shutdown()


def reachable(monkeypatch):
    response = ListControllers.Response(controller=[ControllerState(name="ctrl", state="active")])
    monkeypatch.setattr(controller_manager, "list_controllers", lambda *_: response)
    hw_response = ListHardwareComponents.Response(component=[HardwareComponentState(name="hw")])
    monkeypatch.setattr(controller_manager, "list_hardware_components", lambda *_: hw_response)


def unreachable(monkeypatch, error):
    def fail(*_):
        raise error

    monkeypatch.setattr(controller_manager, "list_controllers", fail)
    monkeypatch.setattr(controller_manager, "list_hardware_components", fail)


def test_shows_selected_controller_manager(plugin, monkeypatch):
    reachable(monkeypatch)
    plugin._widget.cm_combo.setCurrentIndex(0)

    assert plugin._cm_name == "/controller_manager"
    assert [c.name for c in plugin._controllers] == ["ctrl"]
    assert [h.name for h in plugin._hw_components] == ["hw"]


@pytest.mark.parametrize(
    "error",
    [
        ServiceNotFoundError("Could not contact service"),
        RuntimeError("Could not successfully call service"),
    ],
)
def test_deselects_unreachable_controller_manager(plugin, monkeypatch, error):
    reachable(monkeypatch)
    plugin._widget.cm_combo.setCurrentIndex(0)
    unreachable(monkeypatch, error)

    plugin._update_controllers()

    assert plugin._widget.cm_combo.currentIndex() == -1
    assert plugin._cm_name == ""
    assert plugin._controllers == []
    assert plugin._hw_components == []
    assert plugin._ctrl_table_model.rowCount(None) == 0
    assert plugin._hw_table_model.rowCount(None) == 0

    # further updates do nothing until a controller manager is selected again
    plugin._update_controllers()
    plugin._update_hw_components()
    assert plugin._cm_name == ""


def test_deselects_when_selecting_unreachable_controller_manager(plugin, monkeypatch):
    unreachable(monkeypatch, ServiceNotFoundError("Could not contact service"))

    plugin._widget.cm_combo.setCurrentIndex(0)

    assert plugin._widget.cm_combo.currentIndex() == -1
    assert plugin._cm_name == ""
