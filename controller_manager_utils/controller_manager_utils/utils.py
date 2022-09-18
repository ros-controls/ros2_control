#!/usr/bin/env python

# Copyright 2022 PAL Robotics S.L.
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

# NOTE: The Python API contained in this file is considered UNSTABLE and
# subject to change.
# No backwards compatibility guarantees are provided at this moment.


import rclpy
from controller_manager_msgs.srv import ListControllers

# Names of controller manager services, and their respective types
_LIST_CONTROLLERS_STR = "list_controllers"
_LIST_CONTROLLERS_TYPE = "controller_manager_msgs/srv/ListControllers"
_LIST_CONTROLLER_TYPES_STR = "list_controller_types"
_LIST_CONTROLLER_TYPES_TYPE = "controller_manager_msgs/srv/ListControllerTypes"
_LOAD_CONTROLLER_STR = "load_controller"
_LOAD_CONTROLLER_TYPE = "controller_manager_msgs/srv/LoadController"
_UNLOAD_CONTROLLER_STR = "unload_controller"
_UNLOAD_CONTROLLER_TYPE = "controller_manager_msgs/srv/UnloadController"
_SWITCH_CONTROLLER_STR = "switch_controller"
_SWITCH_CONTROLLER_TYPE = "controller_manager_msgs/srv/SwitchController"
_RELOAD_CONTROLLER_LIBS_STR = "reload_controller_libraries"
_RELOAD_CONTROLLER_LIBS_TYPE = "controller_manager_msgs/srv/" "ReloadControllerLibraries"

# Map from service names to their respective type
cm_services = {
    _LIST_CONTROLLERS_STR: _LIST_CONTROLLERS_TYPE,
    _LIST_CONTROLLER_TYPES_STR: _LIST_CONTROLLER_TYPES_TYPE,
    _LOAD_CONTROLLER_STR: _LOAD_CONTROLLER_TYPE,
    _UNLOAD_CONTROLLER_STR: _UNLOAD_CONTROLLER_TYPE,
    _SWITCH_CONTROLLER_STR: _SWITCH_CONTROLLER_TYPE,
    _RELOAD_CONTROLLER_LIBS_STR: _RELOAD_CONTROLLER_LIBS_TYPE,
}


def get_controller_managers(namespace="/", initial_guess=None):
    """
    Get list of active controller manager namespaces.

    @param namespace: Namespace where to look for controller managers.
    @type namespace: str
    @param initial_guess: Initial guess of the active controller managers.
    Typically c{initial_guess} is the output of a previous call to this method,
    and is useful when periodically checking for changes in the list of
    active controller managers.
    Elements in this list will go through a lazy validity check (as opposed to
    a full name+type API verification), so providing a good estimate can
    significantly reduce the number of ROS master queries incurred by this
    method.
    @type initial_guess: [str]
    @return: Sorted list of active controller manager namespaces.
    @rtype: [str]
    """
    ns_list = []
    if initial_guess is not None:
        ns_list = initial_guess[:]  # force copy

    # Get list of (potential) currently running controller managers
    node = rclpy.node.Node("get_controller_managers_node")
    ns_list_curr = _sloppy_get_controller_managers(node, namespace)

    # Update initial guess:
    # 1. Remove entries not found in current list
    # 2. Add new untracked controller managers
    ns_list[:] = [ns for ns in ns_list if ns in ns_list_curr]
    ns_list += [ns for ns in ns_list_curr if ns not in ns_list and is_controller_manager(node, ns)]

    return sorted(ns_list)


def is_controller_manager(node, namespace):
    """
    Check if the input namespace exposes the controller_manager ROS interface.

    This method has the overhead of several ROS master queries
    (one per ROS API member).

    @param namespace: Namespace to check
    @type namespace: str
    @return: True if namespace exposes the controller_manager ROS interface
    @rtype: bool
    """
    cm_ns = namespace
    if not cm_ns or cm_ns[-1] != "/":
        cm_ns += "/"
    for srv_name in cm_services.keys():
        if not _srv_exists(node, cm_ns + srv_name, cm_services[srv_name]):
            return False
    return True


def _sloppy_get_controller_managers(node, namespace):
    """
    Get list of I{potential} active controller manager namespaces.

    The method name is prepended with I{sloppy}, and the returned list contains
    I{potential} active controller managers because it does not perform a
    thorough check of the expected ROS API.
    It rather tries to minimize the number of ROS master queries.

    This method has the overhead of one ROS master query.

    @param namespace: Namespace where to look for controller managers.
    @type namespace: str
    @return: List of I{potential} active controller manager namespaces.
    @rtype: [str]
    """
    # refresh the list of controller managers we can find
    srv_list = node.get_service_names_and_types()

    ns_list = []
    for srv_info in srv_list:
        match_str = "/" + _LIST_CONTROLLERS_STR
        # First element of srv_name is the service name
        if match_str in srv_info[0]:
            ns = srv_info[0].split(match_str)[0]
            if ns == "":
                # controller manager services live in root namespace
                # unlikely, but still possible
                ns = "/"
            ns_list.append(ns)
    return ns_list


def _srv_exists(node, srv_name, srv_type):
    """
    Check if a ROS service of specific name and type exists.

    This method has the overhead of one ROS master query.

    @param srv_name: Fully qualified service name
    @type srv_name:  str
    @param srv_type: Service type
    @type srv_type: str
    """
    if not srv_name or not srv_type:
        return False

    srv_list = node.get_service_names_and_types()
    srv_info = [item for item in srv_list if item[0] == srv_name]
    srv_obtained_type = srv_info[0][1][0]
    return srv_type == srv_obtained_type


###############################################################################
#
# Convenience classes for querying controller managers and controllers
#
###############################################################################


class ControllerManagerLister:
    """
    Convenience functor for querying the list of active controller managers.

    Useful when frequently updating the list, as it internally performs
    some optimizations that reduce the number of interactions with the
    ROS master.

    Example usage:
        >>> list_cm = ControllerManagerLister()
        >>> print(list_cm())
    """

    def __init__(self, namespace="/"):
        self._ns = namespace
        self._cm_list = []

    def __call__(self):
        """Get list of running controller managers."""
        self._cm_list = get_controller_managers(self._ns, self._cm_list)
        return self._cm_list


class ControllerLister:
    """
    Convenience functor for querying loaded controller data.

    The output of calling this functor can be used as input to the different
    controller filtering functions available in this module.

    Example usage. Get I{running} controllers of type C{bar_base/bar}:
        >>> list_controllers = ControllerLister('foo_robot/controller_manager')
        >>> all_ctrl = list_controllers()
        >>> running_ctrl = filter_by_state(all_ctrl, 'running')
        >>> running_bar_ctrl = filter_by_type(running_ctrl, 'bar_base/bar')
    """

    def __init__(self, namespace="/controller_manager"):
        """
        @param namespace Namespace of controller manager to monitor.

        @type namespace str
        """
        self._node = rclpy.node.Node("controller_lister")
        self._srv_name = namespace + "/" + _LIST_CONTROLLERS_STR
        self._srv_client = self._create_client()

    """
    @return: Controller list.
    @rtype: [controller_manager_msgs/ControllerState]
    """

    def __call__(self):
        controller_list = self._srv_client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self._node, controller_list)
        return controller_list.result().controller

    def _create_client(self):
        return self._node.create_client(ListControllers, self._srv_name)


###############################################################################
#
# Convenience methods for filtering controller state information
#
###############################################################################


def filter_by_name(ctrl_list, ctrl_name, match_substring=False):
    """
    Filter controller state list by controller name.

    @param ctrl_list: Controller state list
    @type ctrl_list: [controller_manager_msgs/ControllerState]
    @param ctrl_name: Controller name
    @type ctrl_name: str
    @param match_substring: Set to True to allow substring matching
    @type match_substring: bool
    @return: Controllers matching the specified name
    @rtype: [controller_manager_msgs/ControllerState]
    """
    return _filter_by_attr(ctrl_list, "name", ctrl_name, match_substring)


def filter_by_type(ctrl_list, ctrl_type, match_substring=False):
    """
    Filter controller state list by controller type.

    @param ctrl_list: Controller state list
    @type ctrl_list: [controller_manager_msgs/ControllerState]
    @param ctrl_type: Controller type
    @type ctrl_type: str
    @param match_substring: Set to True to allow substring matching
    @type match_substring: bool
    @return: Controllers matching the specified type
    @rtype: [controller_manager_msgs/ControllerState]
    """
    return _filter_by_attr(ctrl_list, "type", ctrl_type, match_substring)


def filter_by_state(ctrl_list, ctrl_state, match_substring=False):
    """
    Filter controller state list by controller state.

    @param ctrl_list: Controller state list
    @type ctrl_list: [controller_manager_msgs/ControllerState]
    @param ctrl_state: Controller state
    @type ctrl_state: str
    @param match_substring: Set to True to allow substring matching
    @type match_substring: bool
    @return: Controllers matching the specified state
    @rtype: [controller_manager_msgs/ControllerState]
    """
    return _filter_by_attr(ctrl_list, "state", ctrl_state, match_substring)


def filter_by_hardware_interface(ctrl_list, hardware_interface, match_substring=False):
    """
    Filter controller state list by controller hardware interface.

    @param ctrl_list: Controller state list
    @type ctrl_list: [controller_manager_msgs/ControllerState]
    @param hardware_interface: Controller hardware interface
    @type hardware_interface: str
    @param match_substring: Set to True to allow substring matching
    @type match_substring: bool
    @return: Controllers matching the specified hardware interface
    @rtype: [controller_manager_msgs/ControllerState]
    """
    list_out = []
    for ctrl in ctrl_list:
        for resource_set in ctrl.claimed_resources:
            if match_substring:
                if hardware_interface in resource_set.hardware_interface:
                    list_out.append(ctrl)
                    break
            else:
                if resource_set.hardware_interface == hardware_interface:
                    list_out.append(ctrl)
                    break
    return list_out


def filter_by_resources(ctrl_list, resources, hardware_interface=None, match_any=False):
    """
    Filter controller state list by claimed resources.

    @param ctrl_list: Controller state list
    @type ctrl_list: [controller_manager_msgs/ControllerState]
    @param resources: Controller resources
    @type resources: [str]
    @param hardware_interface Controller hardware interface where to look for
    resources. If specified, the requested resources will only be searched for
    in this interface. If unspecified, all controller hardware interfaces will
    be searched for; i.e., if a controller claims resources from multiple
    interfaces, the method will succeed if _any_ interface contains the
    requested resources (any or all, depending on the value of C{match_any}).
    Specifying this parameter allows finer control over determining which
    interfaces claim specific resources.
    @param match_any: If set to False, all elements in C{resources} must
    be claimed by the interface specified in C{hardware_interface} (or _any_
    interface, if C{hardware_interface} is unspecified) for a positive match.
    Note that a controller's resources can contain additional entries than
    those in C{resources}).
    If set to True, at least one element in C{resources} must be claimed by
    the interface specified in C{hardware_interface} (or _any_ interface, if
    C{hardware_interface} is unspecified) for a positive match.
    @type match_any: bool
    @return: Controllers matching the specified hardware interface
    @rtype: [controller_manager_msgs/ControllerState]
    """
    list_out = []
    for ctrl in ctrl_list:
        for resource_set in ctrl.claimed_resources:
            if hardware_interface is None or hardware_interface == resource_set.hardware_interface:
                for res in resources:
                    add_ctrl = not match_any  # Initial flag value
                    if res in resource_set.resources:
                        if match_any:  # One hit: enough to accept controller
                            add_ctrl = True
                            break
                    else:
                        if not match_any:  # One miss: enough to discard controller
                            add_ctrl = False
                            break
                if add_ctrl:
                    list_out.append(ctrl)
                    break
    return list_out


def _filter_by_attr(list_in, attr_name, attr_val, match_substring=False):
    """Filter input list by the value of its elements' attributes."""
    list_out = []
    for val in list_in:
        if match_substring:
            if attr_val in getattr(val, attr_name):
                list_out.append(val)
        else:
            if getattr(val, attr_name) == attr_val:
                list_out.append(val)
    return list_out


###############################################################################
#
# Convenience methods for finding potential controller configurations
#
###############################################################################

# def get_rosparam_controller_names(namespace='/'):
#     """
#     Get list of ROS parameter names that potentially represent a controller
#     configuration.

#     Example usage:
#       - Assume the following parameters exist in the ROS parameter:
#         server:
#           - C{/foo/type}
#           - C{/xxx/type/xxx}
#           - C{/ns/bar/type}
#           - C{/ns/yyy/type/yyy}
#       - The potential controllers found by this method are:

#       >>> names    = get_rosparam_controller_names()      # returns ['foo']
#       >>> names_ns = get_rosparam_controller_names('/ns') # returns ['bar']

#     @param namespace: Namespace where to look for controllers.
#     @type namespace: str
#     @return: Sorted list of ROS parameter names.
#     @rtype: [str]
#     """
#     import rosparam
#     list_out = []
#     all_params = rosparam.list_params(namespace)
#     for param in all_params:
#         # Remove namespace from parameter string
#         if not namespace or namespace[-1] != '/':
#             namespace += '/'
#         param_no_ns = param.split(namespace, 1)[1]

#         # Check if parameter corresponds to a controller configuration
#         param_split = param_no_ns.split('/')
#         if (len(param_split) == 2 and param_split[1] == 'type'):
#             list_out.append(param_split[0]) # It does!
#     return sorted(list_out)
