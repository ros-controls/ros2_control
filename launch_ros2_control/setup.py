# Copyright 2025 Jasper van Brakel
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

from setuptools import find_packages, setup

package_name = "launch_ros2_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test", "examples"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jasper van Brakel",
    maintainer_email="J.C.vanBrakel@tudelft.nl",
    author="Jasper van Brakel",
    author_email="J.C.vanBrakel@tudelft.nl",
    description="A launch extension for ros2_control",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "launch.frontend.launch_extension": [
            "launch_ros2_control = launch_ros2_control",
        ],
    },
)
