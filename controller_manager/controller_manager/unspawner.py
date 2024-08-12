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
import sys
import warnings

from controller_manager import switch_controllers, unload_controller
from controller_manager.controller_manager_services import ServiceNotFoundError

import rclpy
from rclpy.node import Node


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument("controller_names", help="Name of the controller", nargs="+")
    parser.add_argument(
        "-c",
        "--controller-manager",
        help="Name of the controller manager ROS node",
        default="/controller_manager",
        required=False,
    )

    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    args = parser.parse_args(command_line_args)
    controller_names = args.controller_names
    controller_manager_name = args.controller_manager

    node = Node("unspawner_" + controller_names[0])
    try:
        # Ignore returncode, because message is already printed and we'll try to unload anyway
        ret = switch_controllers(
            node, controller_manager_name, controller_names, [], True, True, 5.0
        )
        node.get_logger().info("Deactivated controller")

        for controller_name in controller_names:
            ret = unload_controller(node, controller_manager_name, controller_name)
            if not ret.ok:
                node.get_logger().info("Failed to unload controller")
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
        "'unspawner.py' is deprecated, please use 'unspawner' (without .py extension)",
        DeprecationWarning,
    )
    ret = main()
    sys.exit(ret)
