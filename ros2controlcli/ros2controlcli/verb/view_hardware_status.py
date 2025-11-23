# Copyright 2025 ROS-Control Development Team
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

from functools import partial
import datetime

import rclpy
from ros2cli.node.direct import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2cli.verb import VerbExtension
from ros2topic.api import get_topic_names_and_types

from control_msgs.msg import HardwareStatus
from controller_manager import bcolors

from rosidl_runtime_py import message_to_yaml

_DISCOVERY_THRESHOLD = 10


class ViewHardwareStatusVerb(VerbExtension):
    """Echo hardware status messages with filtering capabilities."""

    def __init__(self):
        super().__init__()
        self.found_hardware_ids = set()
        self.found_device_ids = set()
        self.message_count = 0
        self.discovery_complete = False

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument(
            "-i",
            "--hardware-id",
            dest="hardware_id",
            help="Filter by a specific hardware component ID.",
        )
        parser.add_argument(
            "-d",
            "--device-id",
            dest="device_id",
            help="Filter by a specific device ID within a hardware component.",
        )

    def _on_message(self, msg, args):
        self.message_count += 1
        self.found_hardware_ids.add(msg.hardware_id)
        for device_state in msg.hardware_device_states:
            self.found_device_ids.add(device_state.device_id)

        if not self.discovery_complete and self.message_count >= _DISCOVERY_THRESHOLD:
            self.discovery_complete = True

            if args.hardware_id and args.hardware_id not in self.found_hardware_ids:
                print(
                    f"\n{bcolors.FAIL}Error: Hardware ID '{args.hardware_id}' not found.{bcolors.ENDC}"
                )
                if self.found_hardware_ids:
                    print(f"{bcolors.OKBLUE}Available Hardware IDs:{bcolors.ENDC}")
                    for hw_id in sorted(self.found_hardware_ids):
                        print(f"\t{hw_id}")
                else:
                    print(f"{bcolors.WARNING}No hardware IDs discovered.{bcolors.ENDC}")
                rclpy.shutdown()
                return

            if args.device_id and args.device_id not in self.found_device_ids:
                print(
                    f"\n{bcolors.FAIL}Error: Device ID '{args.device_id}' not found.{bcolors.ENDC}"
                )
                if self.found_device_ids:
                    print(f"{bcolors.OKBLUE}Available Device IDs:{bcolors.ENDC}")
                    for dev_id in sorted(self.found_device_ids):
                        print(f"\t{dev_id}")
                else:
                    print(f"{bcolors.WARNING}No device IDs discovered.{bcolors.ENDC}")
                rclpy.shutdown()
                return

        if args.hardware_id and msg.hardware_id != args.hardware_id:
            return

        if args.device_id and not any(
            d.device_id == args.device_id for d in msg.hardware_device_states
        ):
            return

        try:
            dt_object = datetime.datetime.fromtimestamp(msg.header.stamp.sec)
            nano_str = f"{msg.header.stamp.nanosec:09d}"
            timestamp = f"{dt_object.strftime('%H:%M:%S')}.{nano_str[:3]}"
        except (ValueError, OSError):
            timestamp = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"

        print(
            f"{bcolors.OKGREEN}Hardware ID: {bcolors.ENDC}{msg.hardware_id} ({bcolors.WARNING}stamp: {timestamp}{bcolors.ENDC})"
        )

        for device_state in msg.hardware_device_states:
            if args.device_id and device_state.device_id != args.device_id:
                continue

            print(f"  {bcolors.OKCYAN}Device ID: {bcolors.ENDC}{device_state.device_id}")

            state_types = [
                ("Generic Hardware States", device_state.hardware_status),
                ("CANopen States", device_state.canopen_states),
                ("EtherCAT States", device_state.ethercat_states),
                ("VDA5050 States", device_state.vda5050_states),
            ]

            any_state_printed = False
            for title, states in state_types:
                if states:
                    any_state_printed = True
                    print(f"    {bcolors.OKBLUE}{title}:{bcolors.ENDC}")
                    for state in states:
                        print("      -")
                        yaml_str = message_to_yaml(state, flow_style=False)
                        indented_str = "\n".join(
                            [f"        {line}" for line in yaml_str.splitlines()]
                        )
                        print(indented_str)

            if not any_state_printed:
                print(f"    {bcolors.FAIL}Status: No specific states reported{bcolors.ENDC}")

        print("---")

    def main(self, *, args):
        with NodeStrategy(args).direct_node as node:
            topic_names_and_types = get_topic_names_and_types(
                node=node, include_hidden_topics=True
            )

            status_topics = sorted(
                [
                    name
                    for name, types in topic_names_and_types
                    if name.endswith("/hardware_status")
                    and "control_msgs/msg/HardwareStatus" in types
                ]
            )

            if not status_topics:
                print(
                    f"{bcolors.FAIL}No topics of type 'control_msgs/msg/HardwareStatus' found.{bcolors.ENDC}"
                )
                return 1

            print(f"{bcolors.OKBLUE}Subscribing to the following topics:{bcolors.ENDC}")
            for topic in status_topics:
                print(f"\t{topic}")
            print("---")

            _ = [
                node.create_subscription(
                    HardwareStatus, topic, partial(self._on_message, args=args), 10
                )
                for topic in status_topics
            ]

            rclpy.spin(node)

        return 0
