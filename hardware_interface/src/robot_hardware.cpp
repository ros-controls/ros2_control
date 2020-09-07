// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hardware_interface/robot_hardware.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include "hardware_interface/macros.hpp"
#include "hardware_interface/operation_mode_handle.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr auto kJointStateLoggerName = "joint state handle";
constexpr auto kJointCommandLoggerName = "joint cmd handle";
constexpr auto kOperationModeLoggerName = "joint operation mode handle";
constexpr auto kActuatorLoggerName = "actuator handle";
constexpr auto kJointLoggerName = "joint handle";
}

namespace hardware_interface
{
/// Register the handle to the handle list if the handle's name is not found.
/**
 * \param[in] registered_handles The handle list.
 * \param[in] handle The handle to be registered.
 * \param[in] logger_name The name of the logger.
 * \return The return code, one of `OK` or `ERROR`.
 */
template<typename T>
return_type
register_handle(std::vector<T *> & registered_handles, T * handle, const std::string & logger_name)
{
  if (handle->get_name().empty()) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        logger_name), "cannot register handle! No name is specified");
    return return_type::ERROR;
  }

  if (!handle->valid_pointers()) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        logger_name), "cannot register handle! Points to nullptr!");
    return return_type::ERROR;
  }

  auto handle_pos = std::find_if(
    registered_handles.begin(), registered_handles.end(),
    [&](auto handle_ptr) -> bool {
      return handle_ptr->get_name() == handle->get_name();
    });

  // handle exists already
  if (handle_pos != registered_handles.end()) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(logger_name),
      "cannot register handle! Handle exists already");
    return return_type::ERROR;
  }
  registered_handles.push_back(handle);
  return return_type::OK;
}

return_type
RobotHardware::register_joint_state_handle(const JointStateHandle * joint_handle)
{
  return register_handle<const JointStateHandle>(
    registered_joint_state_handles_,
    joint_handle,
    kJointStateLoggerName);
}

return_type
RobotHardware::register_joint_command_handle(JointCommandHandle * joint_handle)
{
  return register_handle<JointCommandHandle>(
    registered_joint_command_handles_,
    joint_handle,
    kJointCommandLoggerName);
}

return_type
RobotHardware::register_operation_mode_handle(OperationModeHandle * operation_mode_handle)
{
  return register_handle<OperationModeHandle>(
    registered_operation_mode_handles_,
    operation_mode_handle,
    kOperationModeLoggerName);
}

/// Find the handle by name from the registered handle list.
/**
 * \param[in] registered_handles The registered handle list.
 * \param[in] name The handle's name.
 * \param[in] logger_name The name of the logger.
 * \param[out] handle the handle if found.
 * \return The return code, one of `OK` or `ERROR`.
 */
template<typename T>
return_type
get_handle(
  std::vector<T *> & registered_handles,
  const std::string & name,
  const std::string & logger_name,
  T ** handle)
{
  if (name.empty()) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(logger_name),
      "cannot get handle! No name given");
    return return_type::ERROR;
  }

  auto handle_pos = std::find_if(
    registered_handles.begin(), registered_handles.end(),
    [&](auto handle_ptr) -> bool {
      return handle_ptr->get_name() == name;
    });

  if (handle_pos == registered_handles.end()) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        logger_name), "cannot get handle. No joint " << name << " found.");
    return return_type::ERROR;
  }

  *handle = *handle_pos;
  return return_type::OK;
}

return_type
RobotHardware::get_joint_state_handle(
  const std::string & name, const JointStateHandle ** joint_state_handle)
{
  THROW_ON_NOT_NULLPTR(*joint_state_handle)
  return get_handle<const JointStateHandle>(
    registered_joint_state_handles_,
    name,
    kJointStateLoggerName,
    joint_state_handle);
}

return_type
RobotHardware::get_joint_command_handle(
  const std::string & name, JointCommandHandle ** joint_command_handle)
{
  THROW_ON_NOT_NULLPTR(*joint_command_handle)
  return get_handle<JointCommandHandle>(
    registered_joint_command_handles_,
    name,
    kJointCommandLoggerName,
    joint_command_handle);
}

return_type
RobotHardware::get_operation_mode_handle(
  const std::string & name, OperationModeHandle ** operation_mode_handle)
{
  THROW_ON_NOT_NULLPTR(*operation_mode_handle)
  return get_handle<OperationModeHandle>(
    registered_operation_mode_handles_,
    name,
    kOperationModeLoggerName,
    operation_mode_handle);
}

template<typename T>
std::vector<std::string>
get_registered_names(std::vector<T *> & registered_handles)
{
  std::vector<std::string> names;
  names.reserve(registered_handles.size());
  for (auto handle : registered_handles) {
    names.push_back(handle->get_name());
  }
  return names;
}

/*
std::vector<std::string>
RobotHardware::get_registered_joint_names()
{
  return get_registered_names<const JointStateHandle>(registered_joint_state_handles_);
}
*/

std::vector<std::string>
RobotHardware::get_registered_write_op_names()
{
  return get_registered_names<OperationModeHandle>(registered_operation_mode_handles_);
}

std::vector<const JointStateHandle *>
RobotHardware::get_registered_joint_state_handles()
{
  return registered_joint_state_handles_;
}

std::vector<JointCommandHandle *>
RobotHardware::get_registered_joint_command_handles()
{
  return registered_joint_command_handles_;
}

std::vector<OperationModeHandle *>
RobotHardware::get_registered_operation_mode_handles()
{
  return registered_operation_mode_handles_;
}

hardware_interface_ret_t register_handle(
  const std::string & handle_name,
  const std::string & interface_name,
  const double default_value,
  control_msgs::msg::DynamicJointState & registered,
  const std::string & logger_name)
{
  if (handle_name.empty() || interface_name.empty()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name), "handle name or interface is empty!");
    return return_type::ERROR;
  }

  const auto & names_list = registered.joint_names;
  const auto it = std::find(names_list.cbegin(), names_list.cend(), handle_name);
  if (it == names_list.cend()) {
    registered.joint_names.push_back(handle_name);
    control_msgs::msg::InterfaceValue iv;
    iv.interface_names = {interface_name};
    iv.values = {default_value};
    registered.interface_values.push_back(iv);
    return return_type::OK;
  } else {
    const auto index = std::distance(names_list.cbegin(), it);
    auto & ivs = registered.interface_values[static_cast<size_t>(index)];
    const auto interface_names = ivs.interface_names;
    const auto it = std::find(interface_names.cbegin(), interface_names.cend(), interface_name);
    if (it == interface_names.cend()) {
      ivs.interface_names.push_back(interface_name);
      ivs.values.push_back(default_value);
      return return_type::OK;
    } else {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger(logger_name), "handle with interface (" <<
          handle_name << ":" << interface_name <<
          ") is already registered!");
      return return_type::ERROR;
    }
  }
}

hardware_interface_ret_t RobotHardware::register_actuator(
  const std::string & actuator_name,
  const std::string & interface_name,
  const double default_value)
{
  return register_handle(
    actuator_name, interface_name, default_value, registered_actuators_,
    kActuatorLoggerName);
}

hardware_interface_ret_t RobotHardware::register_joint(
  const std::string & joint_name,
  const std::string & interface_name,
  double default_value)
{
  return register_handle(
    joint_name, interface_name, default_value, registered_joints_,
    kJointLoggerName);
}

template<class HandleType>
hardware_interface_ret_t get_handle(
  HandleType & handle,
  control_msgs::msg::DynamicJointState & registered,
  const std::string & logger_name)
{
  const auto & handle_name = handle.get_name();
  const auto & interface_name = handle.get_interface_name();

  if (handle_name.empty() || interface_name.empty()) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(logger_name), "name or interface is ill-defined!");
    return return_type::ERROR;
  }

  const auto & names_list = registered.joint_names;
  const auto it = std::find(names_list.cbegin(), names_list.cend(), handle_name);
  if (it == names_list.cend()) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        logger_name), "handle with name " << handle_name << " not found!");
    return return_type::ERROR;
  }

  const auto index = std::distance(names_list.cbegin(), it);
  auto & ivs = registered.interface_values[static_cast<size_t>(index)];
  const auto interface_names = ivs.interface_names;
  const auto if_it = std::find(interface_names.cbegin(), interface_names.cend(), interface_name);
  if (if_it != interface_names.cend()) {
    const auto value_index = std::distance(interface_names.cbegin(), if_it);
    handle = handle.with_value_ptr(&(ivs.values[static_cast<size_t>(value_index)]));
    return return_type::OK;
  } else {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        logger_name),
      "handle with interface (" << handle_name << ":" << interface_name << ") wasn't found!");
    return return_type::ERROR;
  }

  return return_type::ERROR;
}

hardware_interface_ret_t RobotHardware::get_actuator_handle(ActuatorHandle & actuator_handle)
{
  return get_handle<ActuatorHandle>(actuator_handle, registered_actuators_, kActuatorLoggerName);
}

hardware_interface_ret_t RobotHardware::get_joint_handle(JointHandle & joint_handle)
{
  return get_handle<JointHandle>(joint_handle, registered_joints_, kJointLoggerName);
}

template<class HandleType>
hardware_interface_ret_t get_handles(
  std::vector<HandleType> & handles,
  std::vector<HandleType> && registered,
  const std::string & interface_name)
{
  std::copy_if(
    registered.begin(), registered.end(), std::back_inserter(handles), [&](const auto & handle) {
      return handle.get_interface_name() == interface_name;
    });
  return return_type::OK;
}

hardware_interface_ret_t RobotHardware::get_actuator_handles(
  std::vector<ActuatorHandle> & actuator_handles, const std::string & interface_name)
{
  return get_handles<ActuatorHandle>(actuator_handles, get_registered_actuators(), interface_name);
}

hardware_interface_ret_t RobotHardware::get_joint_handles(
  std::vector<JointHandle> & joint_handles,
  const std::string & interface_name)
{
  return get_handles<JointHandle>(joint_handles, get_registered_joints(), interface_name);
}

const std::vector<std::string> & RobotHardware::get_registered_actuator_names()
{
  return registered_actuators_.joint_names;
}

const std::vector<std::string> & RobotHardware::get_registered_joint_names()
{
  return registered_joints_.joint_names;
}

template<class HandleType>
const std::vector<std::string> & get_registered_interface_names(
  const std::string & name,
  control_msgs::msg::DynamicJointState & registered)
{
  const auto & it = std::find(
    registered.joint_names.begin(), registered.joint_names.end(), name);

  if (it == registered.joint_names.end()) {
    throw std::runtime_error(name + " not found");
  }

  // joint found, can safely cast here
  const auto joint_index =
    static_cast<uint64_t>(std::distance(registered.joint_names.begin(), it));

  return registered.interface_values[joint_index].interface_names;
}

const std::vector<std::string> & RobotHardware::get_registered_actuator_interface_names(
  const std::string & actuator_name)
{
  return get_registered_interface_names<ActuatorHandle>(actuator_name, registered_actuators_);
}

const std::vector<std::string> & RobotHardware::get_registered_joint_interface_names(
  const std::string & joint_name)
{
  return get_registered_interface_names<JointHandle>(joint_name, registered_joints_);
}

template<class HandleType>
std::vector<HandleType> get_registered_handles(control_msgs::msg::DynamicJointState & registered)
{
  std::vector<HandleType> result;
  result.reserve(registered.joint_names.size());    // rough estimate

  auto & handle_names = registered.joint_names;
  auto & interface_values = registered.interface_values;

  assert(registered.joint_names.size() == registered.interface_values.size());
  for (auto i = 0u; i < handle_names.size(); ++i) {
    auto & joint_interfaces = interface_values[i];
    assert(joint_interfaces.interface_names.size() == joint_interfaces.values.size());

    for (auto j = 0u; j < joint_interfaces.interface_names.size(); ++j) {
      result.emplace_back(
        handle_names[i], joint_interfaces.interface_names[j],
        &joint_interfaces.values[j]);
    }
  }

  return result;
}

std::vector<ActuatorHandle> RobotHardware::get_registered_actuators()
{
  return get_registered_handles<ActuatorHandle>(registered_actuators_);
}

std::vector<JointHandle> RobotHardware::get_registered_joints()
{
  return get_registered_handles<JointHandle>(registered_joints_);
}

}  // namespace hardware_interface
