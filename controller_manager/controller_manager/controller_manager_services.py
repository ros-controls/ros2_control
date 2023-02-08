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

from controller_manager_msgs.srv import ConfigureController, \
    ListControllers, ListControllerTypes, ListHardwareComponents, ListHardwareInterfaces, \
    LoadController, ReloadControllerLibraries, SwitchController, UnloadController

import rclpy


def service_caller(node, service_name, service_type, request, service_timeout=10.0):
    cli = node.create_client(service_type, service_name)

    if not cli.service_is_ready():
        node.get_logger().debug(
            f'waiting {service_timeout} seconds for service {service_name} to become available...')
        if not cli.wait_for_service(service_timeout):
            raise RuntimeError(f'Could not contact service {service_name}')

    node.get_logger().debug(f'requester: making request: {request}\n')
    future = cli.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        return future.result()
    else:
        raise RuntimeError(f'Exception while calling service: {future.exception()}')


def configure_controller(node, controller_manager_name, controller_name):
    request = ConfigureController.Request()
    request.name = controller_name
    return service_caller(node, f'{controller_manager_name}/configure_controller',
                          ConfigureController, request)


def list_controllers(node, controller_manager_name):
    request = ListControllers.Request()
    return service_caller(node, f'{controller_manager_name}/list_controllers',
                          ListControllers, request)


def list_controller_types(node, controller_manager_name):
    request = ListControllerTypes.Request()
    return service_caller(node,
                          f'{controller_manager_name}/list_controller_types',
                          ListControllerTypes, request)


def list_hardware_components(node, controller_manager_name):
    request = ListHardwareComponents.Request()
    return service_caller(node, f'{controller_manager_name}/list_hardware_components',
                          ListHardwareComponents, request)


def list_hardware_interfaces(node, controller_manager_name):
    request = ListHardwareInterfaces.Request()
    return service_caller(node, f'{controller_manager_name}/list_hardware_interfaces',
                          ListHardwareInterfaces, request)


def load_controller(node, controller_manager_name, controller_name):
    request = LoadController.Request()
    request.name = controller_name
    return service_caller(node, f'{controller_manager_name}/load_controller',
                          LoadController, request)


def reload_controller_libraries(node, controller_manager_name, force_kill):
    request = ReloadControllerLibraries.Request()
    request.force_kill = force_kill
    return service_caller(node,
                          f'{controller_manager_name}/reload_controller_libraries',
                          ReloadControllerLibraries, request)


def switch_controllers(node, controller_manager_name, deactivate_controllers,
                       activate_controllers, strict, activate_asap, timeout):
    request = SwitchController.Request()
    request.activate_controllers = activate_controllers
    request.deactivate_controllers = deactivate_controllers
    if strict:
        request.strictness = SwitchController.Request.STRICT
    else:
        request.strictness = SwitchController.Request.BEST_EFFORT
    request.activate_asap = activate_asap
    request.timeout = rclpy.duration.Duration(seconds=timeout).to_msg()
    return service_caller(node, f'{controller_manager_name}/switch_controller',
                          SwitchController, request)


def unload_controller(node, controller_manager_name, controller_name):
    request = UnloadController.Request()
    request.name = controller_name
    return service_caller(node, f'{controller_manager_name}/unload_controller',
                          UnloadController, request)
