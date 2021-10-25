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
import subprocess
import sys
import time
import warnings

from controller_manager import configure_controller, list_controllers, \
    load_controller, switch_controllers, unload_controller

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

# from https://stackoverflow.com/a/287944


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def wait_for_controller_manager(node, controller_manager, timeout_duration):
    def full_name(n):
        return n[1] + ('' if n[1].endswith('/') else '/') + n[0]

    # Wait for controller_manager
    timeout = node.get_clock().now() + Duration(seconds=timeout_duration)
    while node.get_clock().now() < timeout:
        node_names_and_namespaces = node.get_node_names_and_namespaces()
        if any(full_name(n) == controller_manager for n in node_names_and_namespaces):
            return True

        node.get_logger().info(
            f'Waiting for {controller_manager} services',
            throttle_duration_sec=2)
        time.sleep(0.2)

    return False


def is_controller_loaded(node, controller_manager, controller_name):
    controllers = list_controllers(node, controller_manager).controller
    return any(c.name == controller_name for c in controllers)


def make_absolute(name):
    return name if name.startswith('/') else ('/' + name)


def main(args=None):

    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'controller_name', help='Name of the controller')
    parser.add_argument(
        '-c', '--controller-manager', help='Name of the controller manager ROS node',
        default='/controller_manager', required=False)
    parser.add_argument(
        '-p', '--param-file',
        help='Controller param file to be loaded into controller node before configure',
        required=False)
    parser.add_argument(
        '--load-only', help='Only load the controller and leave unconfigured.',
        action='store_true', required=False)
    parser.add_argument(
        '--stopped', help='Load and configure the controller, however do not start them',
        action='store_true', required=False)
    parser.add_argument(
        '-t', '--controller-type',
        help='If not provided it should exist in the controller manager namespace',
        default=None, required=False)
    parser.add_argument(
        '-u', '--unload-on-kill',
        help='Wait until this application is interrupted and unload controller',
        action='store_true')
    parser.add_argument(
        '--controller-manager-timeout',
        help='Time to wait for the controller manager', required=False, default=10, type=int)

    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    args = parser.parse_args(command_line_args)
    controller_name = args.controller_name
    controller_manager_name = make_absolute(args.controller_manager)
    param_file = args.param_file
    controller_type = args.controller_type
    controller_manager_timeout = args.controller_manager_timeout

    if param_file and not os.path.isfile(param_file):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), param_file)

    node = Node('spawner_' + controller_name)
    try:
        if not wait_for_controller_manager(node, controller_manager_name,
                                           controller_manager_timeout):
            node.get_logger().error('Controller manager not available')
            return 1

        if is_controller_loaded(node, controller_manager_name, controller_name):
            node.get_logger().info('Controller already loaded, skipping load_controller')
        else:
            if controller_type:
                ret = subprocess.run(['ros2', 'param', 'set', controller_manager_name,
                                      controller_name + '.type', controller_type])
            ret = load_controller(
                node, controller_manager_name, controller_name)
            if not ret.ok:
                # Error message printed by ros2 control
                return 1
            node.get_logger().info(bcolors.OKBLUE + 'Loaded ' + controller_name + bcolors.ENDC)

        if param_file:
            ret = subprocess.run(['ros2', 'param', 'load', controller_name,
                                  param_file])
            if ret.returncode != 0:
                # Error message printed by ros2 param
                return ret.returncode
            node.get_logger().info('Loaded ' + param_file + ' into ' + controller_name)

        if not args.load_only:
            ret = configure_controller(
                node, controller_manager_name, controller_name)
            if not ret.ok:
                node.get_logger().info('Failed to configure controller')
                return 1

            if not args.stopped:
                ret = switch_controllers(
                    node,
                    controller_manager_name,
                    [],
                    [controller_name],
                    True,
                    True,
                    5.0)
                if not ret.ok:
                    node.get_logger().info('Failed to start controller')
                    return 1

                node.get_logger().info(bcolors.OKGREEN + 'Configured and started ' +
                                       bcolors.OKCYAN + controller_name + bcolors.ENDC)

        if not args.unload_on_kill:
            return 0

        try:
            node.get_logger().info('Waiting until interrupt to unload controllers')
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            if not args.stopped:
                node.get_logger().info('Interrupt captured, stopping and unloading controller')
                ret = switch_controllers(
                    node,
                    controller_manager_name,
                    [controller_name],
                    [],
                    True,
                    True,
                    5.0)
                if not ret.ok:
                    node.get_logger().info('Failed to stop controller')
                    return 1

                node.get_logger().info('Stopped controller')

            ret = unload_controller(
                node, controller_manager_name, controller_name)
            if not ret.ok:
                node.get_logger().info('Failed to unload controller')
                return 1

            node.get_logger().info('Unloaded controller')
        return 0
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    warnings.warn(
        "'spawner.py' is deprecated, please use 'spawner' (without .py extension)",
        DeprecationWarning)
    ret = main()
    sys.exit(ret)
