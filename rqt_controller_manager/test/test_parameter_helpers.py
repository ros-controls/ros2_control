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

import threading
import time

import pytest
import rclpy
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from rqt_controller_manager.controller_manager import (
    _get_controller_type,
    _get_parameter_controller_names,
)

TIMEOUT = 0.5  # seconds


@pytest.fixture
def node():
    rclpy.init()
    node = rclpy.create_node("test_parameter_helpers")
    yield node
    node.destroy_node()
    rclpy.shutdown()


def spin_in_background(server):
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return executor, thread


def wait_for_service(node, srv_type, name):
    client = node.create_client(srv_type, name)
    assert client.wait_for_service(timeout_sec=5.0)
    node.destroy_client(client)


@pytest.fixture
def controller_manager(node):
    """Node with a controller configuration in its parameters."""
    server = rclpy.create_node(
        "fake_controller_manager",
        parameter_overrides=[Parameter("my_controller.type", value="pkg/MyController")],
        automatically_declare_parameters_from_overrides=True,
    )
    executor, thread = spin_in_background(server)
    wait_for_service(node, GetParameters, "/fake_controller_manager/get_parameters")
    yield "/fake_controller_manager"
    executor.shutdown()
    thread.join()
    server.destroy_node()


@pytest.fixture
def unresponsive_controller_manager(node):
    """Node whose parameter services never respond."""
    server = rclpy.create_node("unresponsive_controller_manager", start_parameter_services=False)
    release = threading.Event()

    def block(request, response):
        release.wait()
        return response

    server.create_service(ListParameters, "~/list_parameters", block)
    server.create_service(GetParameters, "~/get_parameters", block)
    executor, thread = spin_in_background(server)
    wait_for_service(node, GetParameters, "/unresponsive_controller_manager/get_parameters")
    yield "/unresponsive_controller_manager"
    release.set()
    executor.shutdown()
    thread.join()
    server.destroy_node()


def test_get_controller_names_from_parameters(node, controller_manager):
    assert _get_parameter_controller_names(node, controller_manager, TIMEOUT) == ["my_controller"]


@pytest.mark.parametrize("cm_name", ["/does_not_exist", "unresponsive"])
def test_returns_empty_on_timeout(node, unresponsive_controller_manager, cm_name):
    if cm_name == "unresponsive":
        cm_name = unresponsive_controller_manager

    start = time.monotonic()
    assert _get_parameter_controller_names(node, cm_name, TIMEOUT) == []
    assert _get_controller_type(node, cm_name, "my_controller", TIMEOUT) == ""
    assert time.monotonic() - start < 4 * TIMEOUT


def test_clients_are_reused(node, controller_manager):
    for _ in range(3):
        _get_parameter_controller_names(node, controller_manager, TIMEOUT)
        _get_controller_type(node, controller_manager, "my_controller", TIMEOUT)

    assert len(list(node.clients)) == 2
