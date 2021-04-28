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
    ListControllers, ListControllerTypes, ListHardwareInterfaces, \
    LoadController, ReloadControllerLibraries, SwitchController, UnloadController

import rclpy


def service_caller(node, service_name, service_type, request):
    cli = node.create_client(service_type, service_name)

    if not cli.service_is_ready():
        node.get_logger().debug('waiting for service {} to become available...'
                                .format(service_name))
        if not cli.wait_for_service(10.0):
            raise RuntimeError('Could not contact service {}'.format(service_name))

    node.get_logger().debug('requester: making request: %r\n' % request)
    future = cli.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        return future.result()
    else:
        raise RuntimeError('Exception while calling service: %r' % future.exception())


def configure_controller(node, controller_manager_name, controller_name):
    request = ConfigureController.Request()
    request.name = controller_name
    return service_caller(node, '{}/configure_controller'.format(controller_manager_name),
                          ConfigureController, request)


def list_controllers(node, controller_manager_name):
    request = ListControllers.Request()
    return service_caller(node, '{}/list_controllers'.format(controller_manager_name),
                          ListControllers, request)


def list_controller_types(node, controller_manager_name):
    request = ListControllerTypes.Request()
    return service_caller(node,
                          '{}/list_controller_types'.format(controller_manager_name),
                          ListControllerTypes, request)


def list_hardware_interfaces(node, controller_manager_name):
    request = ListHardwareInterfaces.Request()
    return service_caller(node, '{}/list_hardware_interfaces'.format(controller_manager_name),
                          ListHardwareInterfaces, request)


def load_controller(node, controller_manager_name, controller_name):
    request = LoadController.Request()
    request.name = controller_name
    return service_caller(node, '{}/load_controller'.format(controller_manager_name),
                          LoadController, request)


def reload_controller_libraries(node, controller_manager_name, force_kill):
    request = ReloadControllerLibraries.Request()
    request.force_kill = force_kill
    return service_caller(node,
                          '{}/reload_controller_libraries'.format(controller_manager_name),
                          ReloadControllerLibraries, request)


def switch_controllers(node, controller_manager_name, stop_controllers,
                       start_controllers, strict, start_asap, timeout):
    request = SwitchController.Request()
    request.start_controllers = start_controllers
    request.stop_controllers = stop_controllers
    if strict:
        request.strictness = SwitchController.Request.STRICT
    else:
        request.strictness = SwitchController.Request.BEST_EFFORT
    request.start_asap = start_asap
    request.timeout = rclpy.duration.Duration(seconds=timeout).to_msg()
    return service_caller(node, '{}/switch_controller'.format(controller_manager_name),
                          SwitchController, request)


def unload_controller(node, controller_manager_name, controller_name):
    request = UnloadController.Request()
    request.name = controller_name
    return service_caller(node, '{}/unload_controller'.format(controller_manager_name),
                          UnloadController, request)
