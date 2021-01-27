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
import time


def is_controller_loaded(controller_manager_name, controller_name):
    ret = subprocess.run(["ros2", "control", "list_controllers",
                          "--controller-manager", controller_manager_name], capture_output=True,
                         encoding='utf8')
    output = str(ret.stdout)
    for line in output.splitlines():
        if controller_name in line.split("[")[0]:
            return True
    return False


def main(args=None):

    parser = argparse.ArgumentParser()
    parser.add_argument(
        'controller_name', help='Name of the controller')
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

    args = parser.parse_args()
    controller_name = args.controller_name
    controller_manager_name = args.controller_manager
    param_file = args.param_file

    if is_controller_loaded(controller_manager_name, controller_name):
        print("Controller already loaded, skipping load_controller")
    else:
        ret = subprocess.run(["ros2", "control", "load_controller", controller_name,
                              "--controller-manager", controller_manager_name])
        if ret.returncode != 0:
            # Error message printed by ros2 control
            return ret.returncode
        print("Loaded " + controller_name)

    if param_file:
        ret = subprocess.run(["ros2", "param", "load", controller_name,
                              param_file])
        if ret.returncode != 0:
            # Error message printed by ros2 param
            return ret.returncode
        print("Loaded " + param_file + " into " + controller_name)

    ret = subprocess.run(["ros2", "control", "configure_start_controller", controller_name,
                          "--controller-manager", controller_manager_name])
    if ret.returncode != 0:
        # Error message printed by ros2 control
        return ret.returncode
    print("Configured and started " + controller_name)

    if not args.unload_on_kill:
        return 0
    try:
        print("Waiting until interrupt to unload controllers")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Interrupt captured, unloading stopping and unloading controller")
        ret = subprocess.run(["ros2", "control", "switch_controllers", "--stop-controllers",
                              controller_name, "--controller-manager", controller_manager_name])
        print("Stopped controller")

        # Ignore returncode, because message is already printed and we'll try to unload anyway
        ret = subprocess.run(["ros2", "control", "unload_controller", controller_name,
                              "--controller-manager", controller_manager_name])
        if ret.returncode != 0:
            return ret.returncode
        else:
            print("Unloaded controller")


if __name__ == '__main__':
    main()
