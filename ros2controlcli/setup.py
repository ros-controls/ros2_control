# Copyright 2023 ros2_control Development Team
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

from setuptools import find_packages
from setuptools import setup

package_name = "ros2controlcli"

setup(
    name=package_name,
    version="4.18.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ],
    install_requires=["ros2cli"],
    zip_safe=True,
    author="Victor Lopez",
    author_email="victor.lopez@pal-robotics.com",
    maintainer="Victor Lopez",
    maintainer_email="victor.lopez@pal-robotics.com",
    url="https://github.com/ros-controls/ros2_control",
    keywords=[],
    classifiers=[
        "Environment :: Console",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
    ],
    description="ROS2 Control command  interface.",
    long_description="""\
ROS2 Control command  interface.""",
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "ros2cli.command": [
            "control = ros2controlcli.command.control:ControlCommand",
        ],
        "ros2controlcli.verb": [
            "list_controllers = ros2controlcli.verb.list_controllers:ListControllersVerb",
            "view_controller_chains = ros2controlcli.verb.view_controller_chains:ViewControllerChainsVerb",
            "list_hardware_components = \
                ros2controlcli.verb.list_hardware_components:ListHardwareComponentsVerb",
            "list_hardware_interfaces = \
                ros2controlcli.verb.list_hardware_interfaces:ListHardwareInterfacesVerb",
            "list_controller_types = \
                ros2controlcli.verb.list_controller_types:ListControllerTypesVerb",
            "load_controller = ros2controlcli.verb.load_controller:LoadControllerVerb",
            "reload_controller_libraries = \
                ros2controlcli.verb.reload_controller_libraries:ReloadControllerLibrariesVerb",
            "set_controller_state = \
                ros2controlcli.verb.set_controller_state:SetControllerStateVerb",
            "set_hardware_component_state = \
                ros2controlcli.verb.set_hardware_component_state:SetHardwareComponentStateVerb",
            "switch_controllers = ros2controlcli.verb.switch_controllers:SwitchControllersVerb",
            "unload_controller = ros2controlcli.verb.unload_controller:UnloadControllerVerb",
        ],
    },
)
