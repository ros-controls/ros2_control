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
import subprocess
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node


def is_controller_loaded(controller_manager_name, controller_name):
    ret = subprocess.run(['ros2', 'control', 'list_controllers',
                          '--controller-manager', controller_manager_name], capture_output=True,
                         encoding='utf8')
    output = str(ret.stdout)
    for line in output.splitlines():
        if controller_name in line.split('[')[0]:
            return True
    return False


def main(args=None):

    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'controller_name', help='Name of the controller')
    parser.add_argument(
        '-t', '--controller-type',
        help='If not provided it should exist in the controller manager namespace',
        default=None, required=False)
    parser.add_argument(
        '-c', '--controller-manager', help='Name of the controller manager ROS node',
        default='/controller_manager', required=False)
    parser.add_argument(
        '-p', '--param-file', help='Controller param file to be loaded into controller '
        'node before configure', required=False)
    parser.add_argument(
        '-u', '--unload-on-kill',
        help='Wait until this application is interrupted and unload controller',
        action='store_true')

    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    args = parser.parse_args(command_line_args)
    controller_name = args.controller_name
    controller_manager_name = args.controller_manager
    param_file = args.param_file
    controller_type = args.controller_type

    node = Node('spawner_' + controller_name)
    try:

        # Wait for controller_manager
        timeout = node.get_clock().now() + Duration(seconds=10)
        while node.get_clock().now() < timeout:
            ret = subprocess.run(
                ['ros2', 'service', 'type',
                 '/' + controller_manager_name + '/load_and_start_controller'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL)
            if ret.returncode == 0:
                break
            node.get_logger().info(
                'Waiting for {} services'.format(controller_manager_name),
                throttle_duration_sec=2)
            time.sleep(0.2)

        if controller_type:
            ret = subprocess.run(['ros2', 'param', 'set', controller_manager_name,
                                  controller_name + '.type', controller_type])

        if is_controller_loaded(controller_manager_name, controller_name):
            node.get_logger().info('Controller already loaded, skipping load_controller')
        else:
            ret = subprocess.run(['ros2', 'control', 'load_controller', controller_name,
                                  '--controller-manager', controller_manager_name])
            if ret.returncode != 0:
                # Error message printed by ros2 control
                return ret.returncode
            node.get_logger().info('Loaded ' + controller_name)

        if param_file:
            ret = subprocess.run(['ros2', 'param', 'load', controller_name,
                                  param_file])
            if ret.returncode != 0:
                # Error message printed by ros2 param
                return ret.returncode
            node.get_logger().info('Loaded ' + param_file + ' into ' + controller_name)

        ret = subprocess.run(['ros2', 'control', 'configure_start_controller', controller_name,
                              '--controller-manager', controller_manager_name])
        if ret.returncode != 0:
            # Error message printed by ros2 control
            return ret.returncode
        node.get_logger().info('Configured and started ' + controller_name)

        if not args.unload_on_kill:
            return 0
        try:
            node.get_logger().info('Waiting until interrupt to unload controllers')
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            node.get_logger().info('Interrupt captured, stopping and unloading controller')
            ret = subprocess.run(['ros2', 'control', 'switch_controllers', '--stop-controllers',
                                  controller_name,
                                  '--controller-manager', controller_manager_name])
            node.get_logger().info('Stopped controller')

            # Ignore returncode, because message is already printed and we'll try to unload anyway
            ret = subprocess.run(['ros2', 'control', 'unload_controller', controller_name,
                                  '--controller-manager', controller_manager_name])
            if ret.returncode != 0:
                return ret.returncode
            else:
                node.get_logger().info('Unloaded controller')
        return 0
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
