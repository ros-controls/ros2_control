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

from controller_manager import (
    configure_controller,
    list_controllers,
    load_controller,
    switch_controllers,
    unload_controller,
    set_controller_parameters,
    set_controller_parameters_from_param_files,
)
from controller_manager_msgs.srv import SwitchController
from controller_manager.controller_manager_services import ServiceNotFoundError, bcolors

from filelock import Timeout, FileLock
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


def is_controller_loaded(
    node, controller_manager, controller_name, service_timeout=0.0, call_timeout=10.0
):
    controllers = list_controllers(
        node, controller_manager, service_timeout, call_timeout
    ).controller
    return any(c.name == controller_name for c in controllers)


def parse_args_advanced(args):
    """Parse arguments split by --controller, extracting global args first."""
    # Global parser
    global_parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    global_parser.add_argument(
        "-c",
        "--controller-manager",
        default="controller_manager",
        help="Name of the controller manager",
    )
    global_parser.add_argument(
        "--controller-manager-timeout",
        type=float,
        default=0.0,
        help="Timeout for controller manager services",
    )
    global_parser.add_argument(
        "--switch-timeout", type=float, default=5.0, help="Timeout for switch controller service"
    )
    global_parser.add_argument(
        "--service-call-timeout", type=float, default=10.0, help="Timeout for service calls"
    )
    global_parser.add_argument(
        "--activate-as-group", action="store_true", help="Activate controllers as a group"
    )
    global_parser.add_argument(
        "--switch-asap",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Switch controllers as soon as possible",
    )
    global_parser.add_argument(
        "-u",
        "--unload-on-kill",
        action="store_true",
        help="Deactivate the active controllers and unload them on kill",
    )
    global_parser.add_argument("-h", "--help", action="store_true", help="Show help")

    # Per-controller parser
    controller_parser = argparse.ArgumentParser(add_help=False)
    controller_parser.add_argument("controller_name", help="Name of the controller")
    controller_parser.add_argument(
        "-p",
        "--param-file",
        action="append",
        default=[],
        help="Parameter files to load for the controller",
    )
    controller_parser.add_argument(
        "--load-only",
        action="store_true",
        help="Load the controller but do not configure/activate it",
    )
    controller_parser.add_argument(
        "--inactive", action="store_true", help="Configure the controller but do not switch it"
    )
    controller_parser.add_argument(
        "--controller-ros-args",
        action="append",
        default=None,
        help="ROS arguments to pass to the controller",
    )

    # Let's manually splitting it using `--controller` as delimiter

    raw_args = args

    # If help is requested, show help and exit
    if "-h" in raw_args or "--help" in raw_args:
        print(
            "Usage: spawner [global_options] --controller <name> [controller_options] --controller <name> ..."
        )
        print("\nGlobal Options:")
        global_parser.print_help()
        print("\nController Options:")
        controller_parser.print_help()
        sys.exit(0)

    # Extract global args first (ignoring unknown args which might be controller specific)
    global_namespace, unknown = global_parser.parse_known_args(raw_args)

    chunks = []

    # We need to find indices of '--controller'
    indices = [i for i, x in enumerate(raw_args) if x == "--controller"]

    if not indices:
        print("Error: No --controller arguments found. Usage: spawner --controller <name> ...")
        sys.exit(1)

    for i in range(len(indices)):
        start_index = indices[i]
        end_index = indices[i + 1] if i + 1 < len(indices) else len(raw_args)

        # chunk includes [--controller, name, ... args ...]
        chunk = raw_args[start_index:end_index]
        chunks.append(chunk)

    controllers = []
    for chunk in chunks:
        # chunk[0] is --controller
        if len(chunk) < 2:
            print(f"Error: --controller argument missing value in chunk: {chunk}")
            sys.exit(1)

        name = chunk[1]
        controller_args = chunk[2:]

        # Let's use a parser that DOES NOT expect the name, since we extracted it.
        c_parser = argparse.ArgumentParser(add_help=False)
        c_parser.add_argument("-p", "--param-file", action="append", default=[])
        c_parser.add_argument("--load-only", action="store_true")
        c_parser.add_argument("--inactive", action="store_true")
        c_parser.add_argument("--controller-ros-args", action="append", default=None)

        c_namespace, c_unknown = c_parser.parse_known_args(controller_args)

        controllers.append(
            {
                "name": name,
                "param_files": c_namespace.param_file,
                "load_only": c_namespace.load_only,
                "inactive": c_namespace.inactive,
                "controller_ros_args": c_namespace.controller_ros_args,
            }
        )

    return global_namespace, controllers


def parse_native_args(args):
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
        help="Controller param file to be loaded into controller node before configure. "
        "Pass multiple times to load different files for different controllers or to "
        "override the parameters of the same controller.",
        default=None,
        action="append",
        required=False,
    )
    parser.add_argument(
        "--load-only",
        help="Only load the controller and leave unconfigured.",
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
        "-u",
        "--unload-on-kill",
        help="Wait until this application is interrupted and unload controller",
        action="store_true",
    )
    parser.add_argument(
        "--controller-manager-timeout",
        help="Time to wait for the controller manager service to be available",
        required=False,
        default=0.0,
        type=float,
    )
    parser.add_argument(
        "--switch-timeout",
        help="Time to wait for a successful state switch of controllers."
        " Useful when switching cannot be performed immediately, e.g.,"
        " paused simulations at startup",
        required=False,
        default=5.0,
        type=float,
    )
    parser.add_argument(
        "--service-call-timeout",
        help="Time to wait for the service response from the controller manager",
        required=False,
        default=10.0,
        type=float,
    )
    parser.add_argument(
        "--activate-as-group",
        help="Activates all the parsed controllers list together instead of one by one."
        " Useful for activating all chainable controllers altogether",
        action="store_true",
        required=False,
    )
    parser.add_argument(
        "--switch-asap",
        help="Option to switch the controllers in the realtime loop at the earliest possible time or in the non-realtime loop.",
        required=False,
        default=False,
        action=argparse.BooleanOptionalAction,
    )
    parser.add_argument(
        "--controller-ros-args",
        help="The --ros-args to be passed to the controller node, e.g., for remapping topics. "
        "Pass multiple times for every argument.",
        default=None,
        action="append",
        required=False,
    )

    global_namespace_args = parser.parse_args(args)

    controllers_info = []
    for controller_name in global_namespace_args.controller_names:
        controllers_info.append(
            {
                "name": controller_name,
                "param_files": global_namespace_args.param_file,
                "load_only": global_namespace_args.load_only,
                "inactive": global_namespace_args.inactive,
                "controller_ros_args": global_namespace_args.controller_ros_args,
            }
        )

    return global_namespace_args, controllers_info


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    # Remove ROS args
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]

    # Check if we are in advanced mode
    if "--controller" in command_line_args:
        global_args, controllers = parse_args_advanced(command_line_args)
    else:
        global_args, controllers = parse_native_args(command_line_args)

    controller_manager_name = global_args.controller_manager
    controller_manager_timeout = global_args.controller_manager_timeout
    service_call_timeout = global_args.service_call_timeout
    switch_timeout = global_args.switch_timeout
    strictness = SwitchController.Request.STRICT
    switch_asap = global_args.switch_asap
    activate_as_group = global_args.activate_as_group
    unload_on_kill = global_args.unload_on_kill
    node = None

    # Check param files existence
    for c in controllers:
        if c["param_files"]:
            for param_file in c["param_files"]:
                if not os.path.isfile(param_file):
                    raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), param_file)

    # Use the first controller name for the logger/lock
    first_controller_name = controllers[0]["name"]
    logger = rclpy.logging.get_logger("ros2_control_controller_spawner_" + first_controller_name)

    try:
        spawner_node_name = "spawner_" + first_controller_name
        # Get the environment variable $ROS_HOME or default to ~/.ros
        ros_home = os.getenv("ROS_HOME", os.path.join(os.path.expanduser("~"), ".ros"))
        ros_control_lock_dir = os.path.join(ros_home, "locks")
        if not os.path.exists(ros_control_lock_dir):
            try:
                os.makedirs(ros_control_lock_dir)
            except FileExistsError:
                pass
        lock = FileLock(f"{ros_control_lock_dir}/ros2-control-controller-spawner.lock")
        max_retries = 5
        retry_delay = 3  # seconds
        for attempt in range(max_retries):
            try:
                logger.debug(
                    bcolors.OKGREEN + "Waiting for the spawner lock to be acquired!" + bcolors.ENDC
                )
                # timeout after 20 seconds and try again
                lock.acquire(timeout=20)
                logger.debug(bcolors.OKGREEN + "Spawner lock acquired!" + bcolors.ENDC)
                break
            except Timeout:
                logger.warning(
                    bcolors.WARNING
                    + f"Attempt {attempt+1} failed. Retrying in {retry_delay} seconds..."
                    + bcolors.ENDC
                )
                time.sleep(retry_delay)
        else:
            logger.error(
                bcolors.FAIL + "Failed to acquire lock after multiple attempts." + bcolors.ENDC
            )
            return 1

        node = Node(spawner_node_name)
        logger = node.get_logger()

        spawner_namespace = node.get_namespace()

        if not spawner_namespace.startswith("/"):
            spawner_namespace = f"/{spawner_namespace}"

        if not controller_manager_name.startswith("/"):
            if spawner_namespace and spawner_namespace != "/":
                controller_manager_name = f"{spawner_namespace}/{controller_manager_name}"
            else:
                controller_manager_name = f"/{controller_manager_name}"

        controllers_to_activate = []
        for controller in controllers:
            controller_name = controller["name"]

            if is_controller_loaded(
                node,
                controller_manager_name,
                controller_name,
                controller_manager_timeout,
                service_call_timeout,
            ):
                logger.warning(
                    bcolors.WARNING
                    + "Controller already loaded, skipping load_controller"
                    + bcolors.ENDC
                )
            else:
                if controller["controller_ros_args"]:
                    if not set_controller_parameters(
                        node,
                        controller_manager_name,
                        controller_name,
                        "node_options_args",
                        [
                            arg
                            for args in controller["controller_ros_args"]
                            for arg in args.split()
                        ],
                    ):
                        return 1
                if controller["param_files"]:
                    if not set_controller_parameters_from_param_files(
                        node,
                        controller_manager_name,
                        controller_name,
                        controller["param_files"],
                        spawner_namespace,
                    ):
                        return 1

                ret = load_controller(
                    node,
                    controller_manager_name,
                    controller_name,
                    controller_manager_timeout,
                    service_call_timeout,
                )
                if not ret.ok:
                    logger.fatal(
                        bcolors.FAIL
                        + "Failed loading controller "
                        + bcolors.BOLD
                        + controller_name
                        + bcolors.ENDC
                    )
                    return 1
                logger.info(
                    bcolors.OKBLUE + "Loaded " + bcolors.BOLD + controller_name + bcolors.ENDC
                )

            if not controller["load_only"]:
                ret = configure_controller(
                    node,
                    controller_manager_name,
                    controller_name,
                    controller_manager_timeout,
                    service_call_timeout,
                )
                if not ret.ok:
                    logger.error(bcolors.FAIL + "Failed to configure controller" + bcolors.ENDC)
                    return 1

                if not controller["inactive"]:
                    if activate_as_group:
                        controllers_to_activate.append(controller_name)
                    else:
                        ret = switch_controllers(
                            node,
                            controller_manager_name,
                            [],
                            [controller_name],
                            strictness,
                            switch_asap,
                            switch_timeout,
                            service_call_timeout,
                        )
                        if not ret.ok:
                            logger.error(
                                f"{bcolors.FAIL}Failed to activate controller : {controller_name}{bcolors.ENDC}"
                            )
                            return 1

                        logger.info(
                            bcolors.OKGREEN
                            + "Configured and activated "
                            + bcolors.BOLD
                            + controller_name
                            + bcolors.ENDC
                        )

        if activate_as_group and controllers_to_activate:
            ret = switch_controllers(
                node,
                controller_manager_name,
                [],
                controllers_to_activate,
                strictness,
                switch_asap,
                switch_timeout,
                service_call_timeout,
            )
            if not ret.ok:
                logger.error(
                    f"{bcolors.FAIL}Failed to activate the parsed controllers list : {controllers_to_activate}{bcolors.ENDC}"
                )
                return 1

            logger.info(
                bcolors.OKGREEN
                + f"Configured and activated all the parsed controllers list : {controllers_to_activate}!"
                + bcolors.ENDC
            )

        if not unload_on_kill:
            return 0

        # The lock has to be released to not block other spawner instances while waiting for the interrupt
        lock.release()
        logger.info("Waiting until interrupt to unload controllers")
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        if unload_on_kill:
            logger.info("KeyboardInterrupt successfully captured!")

            # Determine which controllers to deactivate
            controllers_to_deactivate = [c["name"] for c in controllers if not c["inactive"]]

            if controllers_to_deactivate:
                logger.info("Deactivating and unloading controllers...")
                ret = switch_controllers(
                    node,
                    controller_manager_name,
                    controllers_to_deactivate,
                    [],
                    strictness,
                    switch_asap,
                    switch_timeout,
                    service_call_timeout,
                )
                if not ret.ok:
                    logger.error(bcolors.FAIL + "Failed to deactivate controller" + bcolors.ENDC)
                    return 1

                logger.info(f"Successfully deactivated controllers : {controllers_to_deactivate}")

                unload_status = True
                for controller_name in controllers_to_deactivate:
                    ret = unload_controller(
                        node,
                        controller_manager_name,
                        controller_name,
                        controller_manager_timeout,
                        service_call_timeout,
                    )
                    if not ret.ok:
                        unload_status = False
                        logger.error(
                            bcolors.FAIL
                            + f"Failed to unload controller : {controller_name}"
                            + bcolors.ENDC
                        )

                if unload_status:
                    logger.info(f"Successfully unloaded controllers : {controllers_to_deactivate}")
                else:
                    return 1
            else:
                logger.info("No active controllers to unload.")
        else:
            logger.info("KeyboardInterrupt received! Exiting....")
            pass
    except ServiceNotFoundError as err:
        logger.fatal(str(err))
        return 1
    finally:
        if node:
            node.destroy_node()
        if lock.is_locked:
            lock.release()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    warnings.warn(
        "'spawner.py' is deprecated, please use 'spawner' (without .py extension)",
        DeprecationWarning,
    )
    ret = main()
    sys.exit(ret)
