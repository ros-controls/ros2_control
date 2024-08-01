# Copyright (c) 2021 PAL Robotics S.L.
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

from .controller_manager_services import (
    configure_controller,
    list_controller_types,
    list_controllers,
    list_hardware_components,
    list_hardware_interfaces,
    load_controller,
    reload_controller_libraries,
    set_hardware_component_state,
    switch_controllers,
    unload_controller,
)
import yaml


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
        else:
            return None


__all__ = [
    "configure_controller",
    "list_controller_types",
    "list_controllers",
    "list_hardware_components",
    "list_hardware_interfaces",
    "load_controller",
    "reload_controller_libraries",
    "set_hardware_component_state",
    "switch_controllers",
    "unload_controller",
    "get_parameter_from_param_file",
]
