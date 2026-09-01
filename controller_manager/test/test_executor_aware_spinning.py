#!/usr/bin/env python3
# Copyright (c) 2026 OpenAI
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

import importlib.util
import sys
import types
from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[2]


def load_controller_manager_services():
    module_name = "controller_manager.controller_manager_services"
    module_path = (
        REPO_ROOT / "controller_manager" / "controller_manager" / "controller_manager_services.py"
    )

    saved_modules = {}

    def install(name, module):
        saved_modules[name] = sys.modules.get(name)
        sys.modules[name] = module

    controller_manager_msgs = types.ModuleType("controller_manager_msgs")
    controller_manager_msgs_srv = types.ModuleType("controller_manager_msgs.srv")
    for name in (
        "CleanupController",
        "ConfigureController",
        "ListControllers",
        "ListControllerTypes",
        "ListHardwareComponents",
        "ListHardwareInterfaces",
        "LoadController",
        "ReloadControllerLibraries",
        "SetHardwareComponentState",
        "SwitchController",
        "UnloadController",
    ):
        setattr(controller_manager_msgs_srv, name, type(name, (), {"Request": type("Request", (), {})}))
    controller_manager_msgs.srv = controller_manager_msgs_srv
    install("controller_manager_msgs", controller_manager_msgs)
    install("controller_manager_msgs.srv", controller_manager_msgs_srv)

    rcl_interfaces = types.ModuleType("rcl_interfaces")
    rcl_interfaces_msg = types.ModuleType("rcl_interfaces.msg")
    rcl_interfaces_msg.Parameter = type("Parameter", (), {})
    rcl_interfaces_srv = types.ModuleType("rcl_interfaces.srv")
    rcl_interfaces_srv.SetParameters = type(
        "SetParameters", (), {"Request": type("Request", (), {})}
    )
    rcl_interfaces.msg = rcl_interfaces_msg
    rcl_interfaces.srv = rcl_interfaces_srv
    install("rcl_interfaces", rcl_interfaces)
    install("rcl_interfaces.msg", rcl_interfaces_msg)
    install("rcl_interfaces.srv", rcl_interfaces_srv)

    rclpy = types.ModuleType("rclpy")
    rclpy.calls = []

    def fake_spin_until_future_complete(node, future, timeout_sec=None):
        rclpy.calls.append((node, future, timeout_sec))

    rclpy.spin_until_future_complete = fake_spin_until_future_complete
    install("rclpy", rclpy)

    rclpy_parameter = types.ModuleType("rclpy.parameter")
    rclpy_parameter.get_parameter_value = lambda value: value
    install("rclpy.parameter", rclpy_parameter)

    install("yaml", types.ModuleType("yaml"))

    spec = importlib.util.spec_from_file_location(module_name, module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, rclpy, saved_modules


def load_rqt_controller_manager(services_module):
    module_name = "rqt_controller_manager.controller_manager"
    module_path = (
        REPO_ROOT / "rqt_controller_manager" / "rqt_controller_manager" / "controller_manager.py"
    )

    saved_modules = {}

    def install(name, module):
        saved_modules[name] = sys.modules.get(name)
        sys.modules[name] = module

    ament_index_python = types.ModuleType("ament_index_python")
    ament_index_python_packages = types.ModuleType("ament_index_python.packages")
    ament_index_python_packages.get_package_share_directory = lambda _: str(REPO_ROOT)
    ament_index_python.packages = ament_index_python_packages
    install("ament_index_python", ament_index_python)
    install("ament_index_python.packages", ament_index_python_packages)

    controller_manager_pkg = types.ModuleType("controller_manager")
    controller_manager_pkg.controller_manager_services = services_module
    install("controller_manager", controller_manager_pkg)
    install("controller_manager.controller_manager_services", services_module)

    controller_manager_msgs = sys.modules["controller_manager_msgs"]
    controller_manager_msgs_msg = types.ModuleType("controller_manager_msgs.msg")
    controller_manager_msgs_msg.ControllerState = type(
        "ControllerState", (), {"__init__": lambda self, **kwargs: self.__dict__.update(kwargs)}
    )
    controller_manager_msgs.srv.SwitchController = type(
        "SwitchController",
        (),
        {"Request": type("Request", (), {"STRICT": 1})},
    )
    controller_manager_msgs.msg = controller_manager_msgs_msg
    install("controller_manager_msgs.msg", controller_manager_msgs_msg)

    lifecycle_msgs = types.ModuleType("lifecycle_msgs")
    lifecycle_msgs_msg = types.ModuleType("lifecycle_msgs.msg")
    lifecycle_msgs_msg.State = type(
        "State",
        (),
        {
            "PRIMARY_STATE_ACTIVE": 1,
            "PRIMARY_STATE_INACTIVE": 2,
            "PRIMARY_STATE_UNCONFIGURED": 3,
        },
    )
    lifecycle_msgs.msg = lifecycle_msgs_msg
    install("lifecycle_msgs", lifecycle_msgs)
    install("lifecycle_msgs.msg", lifecycle_msgs_msg)

    qt_binding = types.ModuleType("python_qt_binding")
    qt_binding.loadUi = lambda *args, **kwargs: None
    install("python_qt_binding", qt_binding)

    qt_core = types.ModuleType("python_qt_binding.QtCore")
    qt_core.QAbstractTableModel = type("QAbstractTableModel", (), {})
    qt_core.QTimer = type("QTimer", (), {})
    qt_core.Qt = types.SimpleNamespace(
        ContextMenuPolicy=types.SimpleNamespace(CustomContextMenu=0),
        Orientation=types.SimpleNamespace(Horizontal=0),
        ItemDataRole=types.SimpleNamespace(
            DisplayRole=0, DecorationRole=1, FontRole=2, TextAlignmentRole=3
        ),
        AlignmentFlag=types.SimpleNamespace(AlignCenter=0),
    )
    install("python_qt_binding.QtCore", qt_core)

    qt_gui = types.ModuleType("python_qt_binding.QtGui")
    qt_gui.QCursor = type("QCursor", (), {})
    qt_gui.QFont = type("QFont", (), {"Weight": types.SimpleNamespace(Bold=1), "setBold": lambda *args: None})
    qt_gui.QIcon = type("QIcon", (), {})
    qt_gui.QStandardItem = type("QStandardItem", (), {})
    qt_gui.QStandardItemModel = type("QStandardItemModel", (), {})
    install("python_qt_binding.QtGui", qt_gui)

    qt_widgets = types.ModuleType("python_qt_binding.QtWidgets")
    for name in ("QHeaderView", "QMenu", "QStyledItemDelegate", "QWidget"):
        setattr(qt_widgets, name, type(name, (), {}))
    install("python_qt_binding.QtWidgets", qt_widgets)

    qt_gui_plugin = types.ModuleType("qt_gui.plugin")
    qt_gui_plugin.Plugin = type("Plugin", (), {})
    install("qt_gui.plugin", qt_gui_plugin)

    rcl_interfaces_srv = sys.modules["rcl_interfaces.srv"]
    rcl_interfaces_srv.GetParameters = type(
        "GetParameters", (), {"Request": type("Request", (), {"__init__": lambda self: setattr(self, "names", [])})}
    )
    rcl_interfaces_srv.ListParameters = type(
        "ListParameters", (), {"Request": type("Request", (), {})}
    )

    ros2service_api = types.ModuleType("ros2service.api")
    ros2service_api.get_service_names_and_types = lambda node=None: []
    install("ros2service.api", ros2service_api)

    install("rclpy", sys.modules["rclpy"])

    update_combo_module = types.ModuleType("rqt_controller_manager.update_combo")
    update_combo_module.update_combo = lambda *args, **kwargs: None
    install("rqt_controller_manager.update_combo", update_combo_module)

    package_module = types.ModuleType("rqt_controller_manager")
    package_module.__path__ = []
    install("rqt_controller_manager", package_module)

    spec = importlib.util.spec_from_file_location(module_name, module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, saved_modules


def restore_modules(*saved_maps):
    restored_names = set()
    for saved_modules in reversed(saved_maps):
        for name, original in saved_modules.items():
            if name in restored_names:
                continue
            restored_names.add(name)
            if original is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = original


class FakeLogger:
    def debug(self, *_args, **_kwargs):
        return None

    def info(self, *_args, **_kwargs):
        return None

    def warning(self, *_args, **_kwargs):
        return None


class FakeFuture:
    def __init__(self, result=None):
        self._result = result

    def result(self):
        return self._result


class FakeClient:
    def __init__(self, future):
        self.future = future
        self.service_ready_checks = 0

    def service_is_ready(self):
        return True

    def wait_for_service(self, timeout_sec):
        self.service_ready_checks += 1
        return True

    def call_async(self, _request):
        return self.future


class FakeExecutor:
    def __init__(self):
        self.calls = []

    def spin_until_future_complete(self, future, timeout_sec=None):
        self.calls.append((future, timeout_sec))


class FakeNode:
    def __init__(self, executor=None, client=None):
        self.executor = executor
        self._client = client
        self._logger = FakeLogger()

    def get_namespace(self):
        return "/"

    def get_name(self):
        return "test_node"

    def get_logger(self):
        return self._logger

    def create_client(self, *_args, **_kwargs):
        return self._client


class TestExecutorAwareSpinning(unittest.TestCase):
    def test_falls_back_to_rclpy_when_node_has_no_executor(self):
        services_module, rclpy_module, saved = load_controller_manager_services()
        try:
            node = FakeNode()
            future = FakeFuture()
            services_module.spin_until_future_complete(node, future, timeout_sec=1.5)
            self.assertEqual(rclpy_module.calls, [(node, future, 1.5)])
        finally:
            restore_modules(saved)

    def test_uses_node_executor_when_available(self):
        services_module, rclpy_module, saved = load_controller_manager_services()
        try:
            executor = FakeExecutor()
            node = FakeNode(executor=executor)
            future = FakeFuture()
            services_module.spin_until_future_complete(node, future, timeout_sec=2.0)
            self.assertEqual(executor.calls, [(future, 2.0)])
            self.assertEqual(rclpy_module.calls, [])
        finally:
            restore_modules(saved)

    def test_service_caller_reuses_existing_executor(self):
        services_module, _rclpy_module, saved = load_controller_manager_services()
        try:
            executor = FakeExecutor()
            future = FakeFuture(result="ok")
            client = FakeClient(future)
            node = FakeNode(executor=executor)
            original_singleton = services_module.SingletonServiceCaller
            services_module.SingletonServiceCaller = lambda *_args, **_kwargs: client
            try:
                result = services_module.service_caller(
                    node=node,
                    service_name="controller_manager/list_controllers",
                    service_type=object,
                    request=object(),
                    call_timeout=0.25,
                    max_attempts=1,
                )
            finally:
                services_module.SingletonServiceCaller = original_singleton
            self.assertEqual(result, "ok")
            self.assertEqual(executor.calls, [(future, 0.25)])
        finally:
            restore_modules(saved)

    def test_rqt_parameter_lookup_uses_shared_wait_helper(self):
        services_module, _rclpy_module, services_saved = load_controller_manager_services()
        rqt_module = None
        rqt_saved = None
        try:
            rqt_module, rqt_saved = load_rqt_controller_manager(services_module)
            calls = []

            def fake_wait(node, future, timeout_sec=None):
                calls.append((node, future, timeout_sec))

            original_wait = rqt_module.spin_until_future_complete
            rqt_module.spin_until_future_complete = fake_wait
            try:
                response = types.SimpleNamespace(
                    result=types.SimpleNamespace(names=["arm.type", "ignored.value"])
                )
                future = FakeFuture(result=response)
                client = FakeClient(future)
                node = FakeNode(client=client)
                names = rqt_module._get_parameter_controller_names(node, "/cm")
            finally:
                rqt_module.spin_until_future_complete = original_wait
            self.assertEqual(names, ["arm"])
            self.assertEqual(calls, [(node, future, None)])
        finally:
            if rqt_saved is not None:
                restore_modules(rqt_saved)
            restore_modules(services_saved)


if __name__ == "__main__":
    unittest.main()
