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

import threading  # noqa: E402
import time  # noqa: E402

import pytest  # noqa: E402
import rclpy  # noqa: E402
from controller_manager_msgs.msg import ControllerState, HardwareComponentState  # noqa: E402
from controller_manager_msgs.srv import (  # noqa: E402
    ConfigureController,
    ListControllers,
    ListHardwareComponents,
    LoadController,
)
from python_qt_binding.QtCore import QObject  # noqa: E402
from python_qt_binding.QtWidgets import QApplication  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from rqt_controller_manager import controller_manager  # noqa: E402
from rqt_controller_manager.controller_manager import _name_step  # noqa: E402

TIMEOUT = 0.5  # seconds
UPDATE_INTERVAL = 100  # milliseconds


class FakeContext(QObject):
    """Minimal stand-in for the qt_gui plugin context."""

    def __init__(self, node):
        super().__init__()
        self.node = node

    def serial_number(self):
        return 1

    def add_widget(self, widget):
        self.widget = widget


class FakeControllerManager:
    """Controller manager node with a loaded controller and an unloaded controller config."""

    def __init__(self, name, load_ok=True):
        self.node = rclpy.create_node(
            name,
            parameter_overrides=[Parameter("unloaded_ctrl.type", value="pkg/Unloaded")],
            automatically_declare_parameters_from_overrides=True,
        )
        self.calls = []
        self.load_ok = load_ok
        self.list_requests = 0
        self.release = None  # if set, list services block until it is set

        self.node.create_service(ListControllers, "~/list_controllers", self._list_controllers)
        self.node.create_service(
            ListHardwareComponents, "~/list_hardware_components", self._list_hw_components
        )
        self.node.create_service(LoadController, "~/load_controller", self._load)
        self.node.create_service(ConfigureController, "~/configure_controller", self._configure)

    def _list_controllers(self, request, response):
        self.list_requests += 1
        if self.release is not None:
            self.release.wait()
        response.controller = [ControllerState(name="ctrl", state="active")]
        return response

    def _list_hw_components(self, request, response):
        self.list_requests += 1
        if self.release is not None:
            self.release.wait()
        response.component = [HardwareComponentState(name="hw")]
        return response

    def _load(self, request, response):
        self.calls.append(("load_controller", request.name))
        response.ok = self.load_ok
        return response

    def _configure(self, request, response):
        self.calls.append(("configure_controller", request.name))
        response.ok = True
        return response


@pytest.fixture
def ros():
    rclpy.init()
    # rqt spins the plugin's node in a background thread, so do the fake controller manager
    executor = MultiThreadedExecutor()
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield executor
    executor.shutdown()
    thread.join()
    rclpy.shutdown()


@pytest.fixture
def cm(ros):
    fake = FakeControllerManager("fake_controller_manager")
    ros.add_node(fake.node)
    yield fake
    if fake.release is not None:
        fake.release.set()
    fake.node.destroy_node()


@pytest.fixture
def plugin(ros):
    app = QApplication.instance() or QApplication([])  # noqa: F841
    node = rclpy.create_node("test_controller_manager")
    ros.add_node(node)
    # keep the context alive, it is the Qt parent of the plugin
    context = FakeContext(node)
    plugin = controller_manager.ControllerManager(context)
    plugin._service_timeout = TIMEOUT
    # the controller manager list is filled manually, faster updates speed up the tests
    plugin._update_cm_list_timer.stop()
    plugin._update_ctrl_list_timer.setInterval(UPDATE_INTERVAL)
    plugin._update_hw_components_list_timer.setInterval(UPDATE_INTERVAL)
    plugin._widget.cm_combo.addItems(["/fake_controller_manager", "/does_not_exist"])
    plugin._widget.cm_combo.setCurrentIndex(-1)
    yield plugin
    plugin.shutdown_plugin()
    node.destroy_node()


def wait_for(predicate, timeout=5.0):
    """Process Qt events until the predicate is true."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        QApplication.processEvents()
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def select(plugin, cm_name):
    combo = plugin._widget.cm_combo
    combo.setCurrentIndex(combo.findText(cm_name))


def deselected(plugin):
    return plugin._widget.cm_combo.currentIndex() == -1 and plugin._cm_name == ""


def select_and_wait(plugin, cm_name):
    """Select the controller manager and wait until its services are discovered."""
    select(plugin, cm_name)
    assert wait_for(lambda: len(plugin._controllers) == 2)


def test_shows_controllers_and_hw_components(plugin, cm):
    select(plugin, "/fake_controller_manager")

    assert wait_for(lambda: len(plugin._controllers) == 2)
    assert [(c.name, c.state, c.type) for c in plugin._controllers] == [
        ("ctrl", "active", ""),
        ("unloaded_ctrl", "", "pkg/Unloaded"),
    ]
    assert wait_for(lambda: [h.name for h in plugin._hw_components] == ["hw"])


def test_clients_are_reused(plugin, cm):
    select_and_wait(plugin, "/fake_controller_manager")

    for _ in range(3):
        plugin._update_controllers()
        plugin._update_hw_components()
        assert wait_for(lambda: not plugin._ctrl_update_pending)
        assert wait_for(lambda: not plugin._hw_update_pending)

    # list_controllers, list_parameters, get_parameters, list_hardware_components
    assert len(list(plugin._node.clients)) == 4


@pytest.mark.parametrize("unresponsive", [False, True])
def test_unreachable_controller_manager_does_not_block(plugin, cm, unresponsive):
    if unresponsive:
        cm.release = threading.Event()
        cm_name = "/fake_controller_manager"
    else:
        cm_name = "/does_not_exist"

    start = time.monotonic()
    select(plugin, cm_name)
    plugin._update_controllers()
    plugin._update_hw_components()
    assert time.monotonic() - start < TIMEOUT / 2

    assert not deselected(plugin)  # the service discovery gets some time
    assert wait_for(lambda: deselected(plugin), timeout=4 * TIMEOUT)
    assert plugin._controllers == []
    assert plugin._hw_components == []
    assert plugin._ctrl_table_model.rowCount(None) == 0
    assert plugin._hw_table_model.rowCount(None) == 0
    assert list(plugin._node.clients) == []


def test_ignores_responses_of_previous_controller_manager(plugin, cm):
    cm.release = threading.Event()
    select(plugin, "/fake_controller_manager")
    assert wait_for(lambda: cm.list_requests > 0)
    select(plugin, "/does_not_exist")
    cm.release.set()

    assert wait_for(lambda: deselected(plugin), timeout=4 * TIMEOUT)
    assert plugin._controllers == []
    assert plugin._hw_components == []


def test_steps_are_called_in_order(plugin, cm):
    select_and_wait(plugin, "/fake_controller_manager")
    plugin._call_steps(
        [
            _name_step(LoadController, "load_controller", "unloaded_ctrl"),
            _name_step(ConfigureController, "configure_controller", "unloaded_ctrl"),
        ]
    )

    assert wait_for(lambda: len(cm.calls) == 2)
    assert cm.calls == [
        ("load_controller", "unloaded_ctrl"),
        ("configure_controller", "unloaded_ctrl"),
    ]


def test_steps_fail_if_service_is_not_available(plugin, cm):
    select(plugin, "/does_not_exist")
    plugin._call_steps([_name_step(LoadController, "load_controller", "unloaded_ctrl")])

    assert not wait_for(lambda: len(cm.calls) > 0, timeout=2 * TIMEOUT)


def test_steps_stop_at_first_failure(plugin, cm):
    cm.load_ok = False
    select_and_wait(plugin, "/fake_controller_manager")
    plugin._call_steps(
        [
            _name_step(LoadController, "load_controller", "unloaded_ctrl"),
            _name_step(ConfigureController, "configure_controller", "unloaded_ctrl"),
        ]
    )

    assert wait_for(lambda: len(cm.calls) == 1)
    assert not wait_for(lambda: len(cm.calls) > 1, timeout=2 * TIMEOUT)
