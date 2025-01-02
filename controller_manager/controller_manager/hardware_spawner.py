#!/usr/bin/env python3
# Copyright 2023 Stogl Robotics Consulting UG (haftungsbeschränkt)
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
import sys

from controller_manager import (
    list_hardware_components,
    set_hardware_component_state,
    bcolors,
)
from controller_manager.controller_manager_services import ServiceNotFoundError

from lifecycle_msgs.msg import State
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


def first_match(iterable, predicate):
    return next((n for n in iterable if predicate(n)), None)


def combine_name_and_namespace(name_and_namespace):
    node_name, namespace = name_and_namespace
    return namespace + ("" if namespace.endswith("/") else "/") + node_name


def find_node_and_namespace(node, full_node_name):
    node_names_and_namespaces = node.get_node_names_and_namespaces()
    return first_match(
        node_names_and_namespaces,
        lambda n: combine_name_and_namespace(n) == full_node_name,
    )


def has_service_names(node, node_name, node_namespace, service_names):
    client_names_and_types = node.get_service_names_and_types_by_node(node_name, node_namespace)
    if not client_names_and_types:
        return False
    client_names, _ = zip(*client_names_and_types)
    return all(service in client_names for service in service_names)


def is_hardware_component_loaded(
    node, controller_manager, hardware_component, service_timeout=0.0
):
    components = list_hardware_components(node, controller_manager, service_timeout).component
    return any(c.name == hardware_component for c in components)


def handle_set_component_state_service_call(
    node, controller_manager_name, component, target_state, action
):
    response = set_hardware_component_state(node, controller_manager_name, component, target_state)
    if response.ok and response.state == target_state:
        node.get_logger().info(
            f"{bcolors.OKGREEN}{action} component '{component}'. Hardware now in state: {response.state}.{bcolors.ENDC}"
        )
    elif response.ok and not response.state == target_state:
        node.get_logger().warn(
            f"{bcolors.WARNING}Could not {action} component '{component}'. Service call returned ok=True, but state: {response.state} is not equal to target state '{target_state}'.{bcolors.ENDC}"
        )
    else:
        node.get_logger().warn(
            f"{bcolors.WARNING}Could not {action} component '{component}'. Service call failed. Wrong component name?{bcolors.ENDC}"
        )


def activate_component(node, controller_manager_name, component_to_activate):
    active_state = State()
    active_state.id = State.PRIMARY_STATE_ACTIVE
    active_state.label = "active"
    handle_set_component_state_service_call(
        node, controller_manager_name, component_to_activate, active_state, "activated"
    )


def configure_component(node, controller_manager_name, component_to_configure):
    inactive_state = State()
    inactive_state.id = State.PRIMARY_STATE_INACTIVE
    inactive_state.label = "inactive"
    handle_set_component_state_service_call(
        node, controller_manager_name, component_to_configure, inactive_state, "configured"
    )


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    parser = argparse.ArgumentParser()
    activate_or_configure_grp = parser.add_mutually_exclusive_group(required=True)

    parser.add_argument(
        "hardware_component_names",
        help="The name of the hardware components which should be activated.",
        nargs="+",
    )
    parser.add_argument(
        "-c",
        "--controller-manager",
        help="Name of the controller manager ROS node",
        default="controller_manager",
        required=False,
    )
    parser.add_argument(
        "--controller-manager-timeout",
        help="Time to wait for the controller manager",
        required=False,
        default=0,
        type=float,
    )
    # add arguments which are mutually exclusive
    activate_or_configure_grp.add_argument(
        "--activate",
        help="Activates the given components. Note: Components are by default configured before activated. ",
        action="store_true",
        required=False,
    )
    activate_or_configure_grp.add_argument(
        "--configure",
        help="Configures the given components.",
        action="store_true",
        required=False,
    )

    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    args = parser.parse_args(command_line_args)
    hardware_components = args.hardware_component_names
    controller_manager_name = args.controller_manager
    controller_manager_timeout = args.controller_manager_timeout
    activate = args.activate
    configure = args.configure

    node = Node("hardware_spawner")
    if not controller_manager_name.startswith("/"):
        spawner_namespace = node.get_namespace()
        if spawner_namespace != "/":
            controller_manager_name = f"{spawner_namespace}/{controller_manager_name}"
        else:
            controller_manager_name = f"/{controller_manager_name}"

    try:
        for hardware_component in hardware_components:
            if not is_hardware_component_loaded(
                node, controller_manager_name, hardware_component, controller_manager_timeout
            ):
                node.get_logger().warn(
                    f"{bcolors.WARNING}Hardware Component is not loaded - state can not be changed.{bcolors.ENDC}"
                )
            elif activate:
                activate_component(node, controller_manager_name, hardware_component)
            elif configure:
                configure_component(node, controller_manager_name, hardware_component)
            else:
                node.get_logger().error(
                    f'{bcolors.FAIL}You need to either specify if the hardware component should be activated with the "--activate" flag or configured with the "--configure" flag{bcolors.ENDC}'
                )
                parser.print_help()
                return 0
    except KeyboardInterrupt:
        pass
    except ServiceNotFoundError as err:
        node.get_logger().fatal(f"{bcolors.FAIL}{str(err)}{bcolors.ENDC}")
        return 1
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    ret = main()
    sys.exit(ret)
