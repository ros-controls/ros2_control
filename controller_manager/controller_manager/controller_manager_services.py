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

from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    ListControllerTypes,
    ListHardwareComponents,
    ListHardwareInterfaces,
    LoadController,
    ReloadControllerLibraries,
    SetHardwareComponentState,
    SwitchController,
    UnloadController,
)

import rclpy
import yaml
from rcl_interfaces.msg import Parameter

# @note: The versions conditioning is added here to support the source-compatibility with Humble
# The `get_parameter_value` function is moved to `rclpy.parameter` module from `ros2param.api` module from version 3.6.0
try:
    from rclpy.parameter import get_parameter_value
except ImportError:
    from ros2param.api import get_parameter_value
from ros2param.api import call_set_parameters


# from https://stackoverflow.com/a/287944
class bcolors:
    MAGENTA = "\033[95m"
    OKBLUE = "\033[94m"
    OKCYAN = "\033[96m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


class ServiceNotFoundError(Exception):
    pass


class SingletonServiceCaller:
    """
    Singleton class to call services of controller manager.

    This class is used to create a service client for a given service name.
    If the service client already exists, it returns the existing client.
    It is used to avoid creating multiple service clients for the same service name.

    It needs Node object, service type and fully qualified service name to create a service client.

    """

    _clients = {}

    def __new__(cls, node, service_type, fully_qualified_service_name):
        if (node, fully_qualified_service_name) not in cls._clients:
            cls._clients[(node, fully_qualified_service_name)] = node.create_client(
                service_type, fully_qualified_service_name
            )
            node.get_logger().debug(
                f"{bcolors.MAGENTA}Creating a new service client : {fully_qualified_service_name} with node : {node.get_name()}{bcolors.ENDC}"
            )

        node.get_logger().debug(
            f"{bcolors.OKBLUE}Returning the existing service client : {fully_qualified_service_name} for node : {node.get_name()}{bcolors.ENDC}"
        )
        return cls._clients[(node, fully_qualified_service_name)]


def service_caller(
    node,
    service_name,
    service_type,
    request,
    service_timeout=0.0,
    call_timeout=10.0,
    max_attempts=3,
):
    """
    Abstraction of a service call.

    Has an optional timeout to find the service, receive the answer to a call
    and a mechanism to retry a call of no response is received.

    @param node Node object to be associated with
    @type rclpy.node.Node
    @param service_name Service URL
    @type str
    @param request The request to be sent
    @type service request type
    @param service_timeout Timeout (in seconds) to wait until the service is available. 0 means
    waiting forever, retrying every 10 seconds.
    @type float
    @param call_timeout Timeout (in seconds) for getting a response
    @type float
    @param max_attempts Number of attempts until a valid response is received. With some
    middlewares it can happen, that the service response doesn't reach the client leaving it in
    a waiting state forever.
    @type int
    @return The service response

    """
    namespace = "" if node.get_namespace() == "/" else node.get_namespace()
    fully_qualified_service_name = (
        f"{namespace}/{service_name}" if not service_name.startswith("/") else service_name
    )
    cli = SingletonServiceCaller(node, service_type, fully_qualified_service_name)

    while not cli.service_is_ready():
        node.get_logger().info(
            f"waiting for service {fully_qualified_service_name} to become available..."
        )
        if service_timeout:
            if not cli.wait_for_service(service_timeout):
                raise ServiceNotFoundError(
                    f"Could not contact service {fully_qualified_service_name}"
                )
        elif not cli.wait_for_service(10.0):
            node.get_logger().warn(f"Could not contact service {fully_qualified_service_name}")

    node.get_logger().debug(f"requester: making request: {request}\n")
    future = None
    for attempt in range(max_attempts):
        future = cli.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=call_timeout)
        if future.result() is None:
            node.get_logger().warning(
                f"Failed getting a result from calling {fully_qualified_service_name} in "
                f"{call_timeout}. (Attempt {attempt+1} of {max_attempts}.)"
            )
        else:
            return future.result()
    raise RuntimeError(
        f"Could not successfully call service {fully_qualified_service_name} after {max_attempts} attempts."
    )


def configure_controller(
    node, controller_manager_name, controller_name, service_timeout=0.0, call_timeout=10.0
):
    request = ConfigureController.Request()
    request.name = controller_name
    return service_caller(
        node,
        f"{controller_manager_name}/configure_controller",
        ConfigureController,
        request,
        service_timeout,
        call_timeout,
    )


def list_controllers(node, controller_manager_name, service_timeout=0.0, call_timeout=10.0):
    request = ListControllers.Request()
    return service_caller(
        node,
        f"{controller_manager_name}/list_controllers",
        ListControllers,
        request,
        service_timeout,
        call_timeout,
    )


def list_controller_types(node, controller_manager_name, service_timeout=0.0, call_timeout=10.0):
    request = ListControllerTypes.Request()
    return service_caller(
        node,
        f"{controller_manager_name}/list_controller_types",
        ListControllerTypes,
        request,
        service_timeout,
        call_timeout,
    )


def list_hardware_components(
    node, controller_manager_name, service_timeout=0.0, call_timeout=10.0
):
    request = ListHardwareComponents.Request()
    return service_caller(
        node,
        f"{controller_manager_name}/list_hardware_components",
        ListHardwareComponents,
        request,
        service_timeout,
        call_timeout,
    )


def list_hardware_interfaces(
    node, controller_manager_name, service_timeout=0.0, call_timeout=10.0
):
    request = ListHardwareInterfaces.Request()
    return service_caller(
        node,
        f"{controller_manager_name}/list_hardware_interfaces",
        ListHardwareInterfaces,
        request,
        service_timeout,
        call_timeout,
    )


def load_controller(
    node, controller_manager_name, controller_name, service_timeout=0.0, call_timeout=10.0
):
    request = LoadController.Request()
    request.name = controller_name
    return service_caller(
        node,
        f"{controller_manager_name}/load_controller",
        LoadController,
        request,
        service_timeout,
        call_timeout,
    )


def reload_controller_libraries(
    node, controller_manager_name, force_kill, service_timeout=0.0, call_timeout=10.0
):
    request = ReloadControllerLibraries.Request()
    request.force_kill = force_kill
    return service_caller(
        node,
        f"{controller_manager_name}/reload_controller_libraries",
        ReloadControllerLibraries,
        request,
        service_timeout,
        call_timeout,
    )


def set_hardware_component_state(
    node,
    controller_manager_name,
    component_name,
    lifecyle_state,
    service_timeout=0.0,
    call_timeout=10.0,
):
    request = SetHardwareComponentState.Request()
    request.name = component_name
    request.target_state = lifecyle_state
    return service_caller(
        node,
        f"{controller_manager_name}/set_hardware_component_state",
        SetHardwareComponentState,
        request,
        service_timeout,
        call_timeout,
    )


def switch_controllers(
    node,
    controller_manager_name,
    deactivate_controllers,
    activate_controllers,
    strict,
    activate_asap,
    timeout,
    call_timeout=10.0,
):
    request = SwitchController.Request()
    request.activate_controllers = activate_controllers
    request.deactivate_controllers = deactivate_controllers
    if strict:
        request.strictness = SwitchController.Request.STRICT
    else:
        request.strictness = SwitchController.Request.BEST_EFFORT
    request.activate_asap = activate_asap
    request.timeout = rclpy.duration.Duration(seconds=timeout).to_msg()
    return service_caller(
        node,
        f"{controller_manager_name}/switch_controller",
        SwitchController,
        request,
        call_timeout=call_timeout,
    )


def unload_controller(
    node, controller_manager_name, controller_name, service_timeout=0.0, call_timeout=10.0
):
    request = UnloadController.Request()
    request.name = controller_name
    return service_caller(
        node,
        f"{controller_manager_name}/unload_controller",
        UnloadController,
        request,
        service_timeout,
        call_timeout,
    )


def get_params_files_with_controller_parameters(
    node, controller_name: str, namespace: str, parameter_files: list
):
    controller_parameter_files = []
    for parameter_file in parameter_files:
        if parameter_file in controller_parameter_files:
            continue
        with open(parameter_file) as f:
            namespaced_controller = (
                f"/{controller_name}" if namespace == "/" else f"{namespace}/{controller_name}"
            )
            WILDCARD_KEY = "/**"
            ROS_PARAMS_KEY = "ros__parameters"
            parameters = yaml.safe_load(f)
            # check for the parameter in 'controller_name' or 'namespaced_controller' or '/**/namespaced_controller' or '/**/controller_name'
            for key in [
                controller_name,
                namespaced_controller,
                f"{WILDCARD_KEY}/{controller_name}",
                f"{WILDCARD_KEY}{namespaced_controller}",
            ]:
                if parameter_file in controller_parameter_files:
                    break
                if key in parameters:
                    if key == controller_name and namespace != "/":
                        node.get_logger().fatal(
                            f"{bcolors.FAIL}Missing namespace : {namespace} or wildcard in parameter file for controller : {controller_name}{bcolors.ENDC}"
                        )
                        break
                    controller_parameter_files.append(parameter_file)

                if WILDCARD_KEY in parameters and key in parameters[WILDCARD_KEY]:
                    controller_parameter_files.append(parameter_file)
                if WILDCARD_KEY in parameters and ROS_PARAMS_KEY in parameters[WILDCARD_KEY]:
                    controller_parameter_files.append(parameter_file)
    return controller_parameter_files


def get_parameter_from_param_files(
    node, controller_name: str, namespace: str, parameter_files: list, parameter_name: str
):
    for parameter_file in parameter_files:
        with open(parameter_file) as f:
            namespaced_controller = (
                f"/{controller_name}" if namespace == "/" else f"{namespace}/{controller_name}"
            )
            WILDCARD_KEY = "/**"
            ROS_PARAMS_KEY = "ros__parameters"
            parameters = yaml.safe_load(f)
            controller_param_dict = None
            # check for the parameter in 'controller_name' or 'namespaced_controller' or '/**/namespaced_controller' or '/**/controller_name'
            for key in [
                controller_name,
                namespaced_controller,
                f"{WILDCARD_KEY}/{controller_name}",
                f"{WILDCARD_KEY}{namespaced_controller}",
            ]:
                if key in parameters:
                    if key == controller_name and namespace != "/":
                        node.get_logger().fatal(
                            f"{bcolors.FAIL}Missing namespace : {namespace} or wildcard in parameter file for controller : {controller_name}{bcolors.ENDC}"
                        )
                        break
                    controller_param_dict = parameters[key]

                if WILDCARD_KEY in parameters and key in parameters[WILDCARD_KEY]:
                    controller_param_dict = parameters[WILDCARD_KEY][key]
                if WILDCARD_KEY in parameters and ROS_PARAMS_KEY in parameters[WILDCARD_KEY]:
                    controller_param_dict = parameters[WILDCARD_KEY]

                if controller_param_dict and (
                    not isinstance(controller_param_dict, dict)
                    or ROS_PARAMS_KEY not in controller_param_dict
                ):
                    raise RuntimeError(
                        f"YAML file : {parameter_file} is not a valid ROS parameter file for controller node : {namespaced_controller}"
                    )
                if (
                    controller_param_dict
                    and ROS_PARAMS_KEY in controller_param_dict
                    and parameter_name in controller_param_dict[ROS_PARAMS_KEY]
                ):
                    break

            if controller_param_dict and parameter_name in controller_param_dict[ROS_PARAMS_KEY]:
                return controller_param_dict[ROS_PARAMS_KEY][parameter_name]
    if controller_param_dict is None:
        node.get_logger().fatal(
            f"{bcolors.FAIL}Controller : {namespaced_controller} parameters not found in parameter files : {parameter_files}{bcolors.ENDC}"
        )
    return None


def set_controller_parameters(
    node, controller_manager_name, controller_name, parameter_name, parameter_value
):
    parameter = Parameter()
    parameter.name = controller_name + "." + parameter_name
    parameter_string = str(parameter_value)
    parameter.value = get_parameter_value(string_value=parameter_string)

    response = call_set_parameters(
        node=node, node_name=controller_manager_name, parameters=[parameter]
    )
    assert len(response.results) == 1
    result = response.results[0]
    if result.successful:
        node.get_logger().info(
            bcolors.OKCYAN
            + 'Setting controller param "'
            + parameter_name
            + '" to "'
            + parameter_string
            + '" for '
            + bcolors.BOLD
            + controller_name
            + bcolors.ENDC
        )
    else:
        node.get_logger().fatal(
            bcolors.FAIL
            + 'Could not set controller param "'
            + parameter_name
            + '" to "'
            + parameter_string
            + '" for '
            + bcolors.BOLD
            + controller_name
            + bcolors.ENDC
        )
        return False
    return True


def set_controller_parameters_from_param_files(
    node, controller_manager_name: str, controller_name: str, parameter_files: list, namespace=None
):
    spawner_namespace = namespace if namespace else node.get_namespace()
    controller_parameter_files = get_params_files_with_controller_parameters(
        node, controller_name, spawner_namespace, parameter_files
    )
    if controller_parameter_files:
        set_controller_parameters(
            node,
            controller_manager_name,
            controller_name,
            "params_file",
            controller_parameter_files,
        )

        controller_type = get_parameter_from_param_files(
            node, controller_name, spawner_namespace, controller_parameter_files, "type"
        )
        if controller_type and not set_controller_parameters(
            node, controller_manager_name, controller_name, "type", controller_type
        ):
            return False
    return True
