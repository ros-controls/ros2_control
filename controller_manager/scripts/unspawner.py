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


def main(args=None):

    parser = argparse.ArgumentParser()
    parser.add_argument(
        'controller_name', help='Name of the controller')
    parser.add_argument(
        '-c', '--controller-manager', help='Name of the controller manager ROS node',
        default='/controller_manager', required=False)

    args = parser.parse_args()
    controller_name = args.controller_name
    controller_manager_name = args.controller_manager

    # Ignore returncode, because message is already printed and we'll try to unload anyway
    ret = subprocess.run(['ros2', 'control', 'switch_controllers', '--stop-controllers',
                          controller_name, '--controller-manager', controller_manager_name])
    print('Stopped controller')

    ret = subprocess.run(['ros2', 'control', 'unload_controller', controller_name,
                          '--controller-manager', controller_manager_name])
    if ret.returncode != 0:
        return ret.returncode
    else:
        print('Unloaded controller')


if __name__ == '__main__':
    main()
