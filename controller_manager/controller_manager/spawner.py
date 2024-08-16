#!/usr/bin/env python3
# Copyright 2021 PAL Robotics S.L.
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
import errno
import os
import sys
import time
import warnings
import yaml

from controller_manager import (
    configure_controller,
    list_controllers,
    load_controller,
    switch_controllers,
    unload_controller,
)
from controller_manager.controller_manager_services import ServiceNotFoundError

import rclpy
from rcl_interfaces.msg import Parameter
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from ros2param.api import call_set_parameters
from ros2param.api import get_parameter_value

# from https://stackoverflow.com/a/287944


class bcolors:
    MAGENTA = "\033[95m"
    OKBLUE = "\033[94m"
    OKCYAN = "\033[96m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


def first_match(iterable, predicate):
    return next((n for n in iterable if predicate(n)), None)


def combine_name_and_namespace(name_and_namespace):
    node_name, namespace = name_and_namespace
    return namespace + ("" if namespace.endswith("/") else "/") + node_name


def find_node_and_namespace(node, full_node_name):
    node_names_and_namespaces = node.get_node_names_and_namespaces()
    return first_match(
        node_names_and_namespaces, lambda n: combine_name_and_namespace(n) == full_node_name
    )


def has_service_names(node, node_name, node_namespace, service_names):
    client_names_and_types = node.get_service_names_and_types_by_node(node_name, node_namespace)
    if not client_names_and_types:
        return False
    client_names, _ = zip(*client_names_and_types)
    return all(service in client_names for service in service_names)


def is_controller_loaded(node, controller_manager, controller_name, service_timeout=0.0):
    controllers = list_controllers(node, controller_manager, service_timeout).controller
    return any(c.name == controller_name for c in controllers)


def get_parameter_from_param_file(controller_name, namespace, parameter_file, parameter_name):
    with open(parameter_file) as f:
        namespaced_controller = (
            controller_name if namespace == "/" else f"{namespace}/{controller_name}"
        )
        parameters = yaml.safe_load(f)
        if namespaced_controller in parameters:
            value = parameters[namespaced_controller]
            if not isinstance(value, dict) or "ros__parameters" not in value:
                raise RuntimeError(
                    f"YAML file : {parameter_file} is not a valid ROS parameter file for controller : {namespaced_controller}"
                )
            if parameter_name in parameters[namespaced_controller]["ros__parameters"]:
                return parameters[namespaced_controller]["ros__parameters"][parameter_name]
            else:
                return None


def main(args=None):

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    parser = argparse.ArgumentParser()
    parser.add_argument("controller_names", help="List of controllers", nargs="+")
    parser.add_argument(
        "-c",
        "--controller-manager",
        help="Name of the controller manager ROS node",
        default="controller_manager",
        required=False,
    )
    parser.add_argument(
        "-p",
        "--param-file",
        help="Controller param file to be loaded into controller node before configure",
        required=False,
    )
    parser.add_argument(
        "-n", "--namespace", help="Namespace for the controller", default="", required=False
    )
    parser.add_argument(
        "--load-only",
        help="Only load the controller and leave unconfigured.",
        action="store_true",
        required=False,
    )
    parser.add_argument(
        "--stopped",
        help="Load and configure the controller, however do not activate them",
        action="store_true",
        required=False,
    )
    parser.add_argument(
        "--inactive",
        help="Load and configure the controller, however do not activate them",
        action="store_true",
        required=False,
    )
    parser.add_argument(
        "-t",
        "--controller-type",
        help="If not provided it should exist in the controller manager namespace",
        default=None,
        required=False,
    )
    parser.add_argument(
        "-u",
        "--unload-on-kill",
        help="Wait until this application is interrupted and unload controller",
        action="store_true",
    )
    parser.add_argument(
        "--controller-manager-timeout",
        help="Time to wait for the controller manager",
        required=False,
        default=0,
        type=float,
    )
    parser.add_argument(
        "--activate-as-group",
        help="Activates all the parsed controllers list together instead of one by one."
        " Useful for activating all chainable controllers altogether",
        action="store_true",
        required=False,
    )

    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    args = parser.parse_args(command_line_args)
    controller_names = args.controller_names
    controller_manager_name = args.controller_manager
    param_file = args.param_file
    controller_manager_timeout = args.controller_manager_timeout

    if param_file and not os.path.isfile(param_file):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), param_file)

    node = Node("spawner_" + controller_names[0])

    if node.get_namespace() != "/" and args.namespace:
        raise RuntimeError(
            f"Setting namespace through both '--namespace {args.namespace}' arg and the ROS 2 standard way "
            f"'--ros-args -r __ns:={node.get_namespace()}' is not allowed!"
        )

    spawner_namespace = args.namespace if args.namespace else node.get_namespace()

    if not spawner_namespace.startswith("/"):
        spawner_namespace = f"/{spawner_namespace}"

    if not controller_manager_name.startswith("/"):
        if spawner_namespace and spawner_namespace != "/":
            controller_manager_name = f"{spawner_namespace}/{controller_manager_name}"
        else:
            controller_manager_name = f"/{controller_manager_name}"

    try:
        for controller_name in controller_names:
            if is_controller_loaded(
                node, controller_manager_name, controller_name, controller_manager_timeout
            ):
                node.get_logger().warn(
                    bcolors.WARNING
                    + "Controller already loaded, skipping load_controller"
                    + bcolors.ENDC
                )
            else:
                controller_type = (
                    args.controller_type
                    if param_file is None
                    else get_parameter_from_param_file(
                        controller_name, spawner_namespace, param_file, "type"
                    )
                )
                if controller_type:
                    parameter = Parameter()
                    parameter.name = controller_name + ".type"
                    parameter.value = get_parameter_value(string_value=controller_type)

                    response = call_set_parameters(
                        node=node, node_name=controller_manager_name, parameters=[parameter]
                    )
                    assert len(response.results) == 1
                    result = response.results[0]
                    if result.successful:
                        node.get_logger().info(
                            bcolors.OKCYAN
                            + 'Set controller type to "'
                            + controller_type
                            + '" for '
                            + bcolors.BOLD
                            + controller_name
                            + bcolors.ENDC
                        )
                    else:
                        node.get_logger().fatal(
                            bcolors.FAIL
                            + 'Could not set controller type to "'
                            + controller_type
                            + '" for '
                            + bcolors.BOLD
                            + controller_name
                            + bcolors.ENDC
                        )
                        return 1

                if param_file:
                    parameter = Parameter()
                    parameter.name = controller_name + ".params_file"
                    parameter.value = get_parameter_value(string_value=param_file)

                    response = call_set_parameters(
                        node=node, node_name=controller_manager_name, parameters=[parameter]
                    )
                    assert len(response.results) == 1
                    result = response.results[0]
                    if result.successful:
                        node.get_logger().info(
                            bcolors.OKCYAN
                            + 'Set controller params file to "'
                            + param_file
                            + '" for '
                            + bcolors.BOLD
                            + controller_name
                            + bcolors.ENDC
                        )
                    else:
                        node.get_logger().fatal(
                            bcolors.FAIL
                            + 'Could not set controller params file to "'
                            + param_file
                            + '" for '
                            + bcolors.BOLD
                            + controller_name
                            + bcolors.ENDC
                        )
                        return 1

                ret = load_controller(node, controller_manager_name, controller_name)
                if not ret.ok:
                    node.get_logger().fatal(
                        bcolors.FAIL
                        + "Failed loading controller "
                        + bcolors.BOLD
                        + controller_name
                        + bcolors.ENDC
                    )
                    return 1
                node.get_logger().info(
                    bcolors.OKBLUE + "Loaded " + bcolors.BOLD + controller_name + bcolors.ENDC
                )

            if not args.load_only:
                ret = configure_controller(node, controller_manager_name, controller_name)
                if not ret.ok:
                    node.get_logger().error(
                        bcolors.FAIL + "Failed to configure controller" + bcolors.ENDC
                    )
                    return 1

                if not args.stopped and not args.inactive and not args.activate_as_group:
                    ret = switch_controllers(
                        node, controller_manager_name, [], [controller_name], True, True, 5.0
                    )
                    if not ret.ok:
                        node.get_logger().error(
                            bcolors.FAIL + "Failed to activate controller" + bcolors.ENDC
                        )
                        return 1

                    node.get_logger().info(
                        bcolors.OKGREEN
                        + "Configured and activated "
                        + bcolors.BOLD
                        + controller_name
                        + bcolors.ENDC
                    )

        if not args.stopped and not args.inactive and args.activate_as_group:
            ret = switch_controllers(
                node, controller_manager_name, [], controller_names, True, True, 5.0
            )
            if not ret.ok:
                node.get_logger().error(
                    bcolors.FAIL + "Failed to activate the parsed controllers list" + bcolors.ENDC
                )
                return 1

            node.get_logger().info(
                bcolors.OKGREEN
                + "Configured and activated all the parsed controllers list!"
                + bcolors.ENDC
            )
        if args.stopped:
            node.get_logger().warn('"--stopped" flag is deprecated use "--inactive" instead')

        if not args.unload_on_kill:
            return 0

        try:
            node.get_logger().info("Waiting until interrupt to unload controllers")
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            if not args.stopped and not args.inactive:
                node.get_logger().info("Interrupt captured, deactivating and unloading controller")
                # TODO(saikishor) we might have an issue in future, if any of these controllers is in chained mode
                ret = switch_controllers(
                    node, controller_manager_name, controller_names, [], True, True, 5.0
                )
                if not ret.ok:
                    node.get_logger().error("Failed to deactivate controller")
                    return 1

                node.get_logger().info("Deactivated controller")

            elif args.stopped:
                node.get_logger().warn('"--stopped" flag is deprecated use "--inactive" instead')

            ret = unload_controller(node, controller_manager_name, controller_name)
            if not ret.ok:
                node.get_logger().error("Failed to unload controller")
                return 1

            node.get_logger().info("Unloaded controller")
        return 0
    except KeyboardInterrupt:
        pass
    except ServiceNotFoundError as err:
        node.get_logger().fatal(str(err))
        return 1
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    warnings.warn(
        "'spawner.py' is deprecated, please use 'spawner' (without .py extension)",
        DeprecationWarning,
    )
    ret = main()
    sys.exit(ret)
