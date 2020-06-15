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
    RCLCPP_ERROR(rclcpp::get_logger(logger_name), "cannot register handle! No name is specified");
    return return_type::ERROR;
  }

  if (!handle->valid_pointers()) {
    RCLCPP_ERROR(rclcpp::get_logger(logger_name), "cannot register handle! Points to nullptr!");
    return return_type::ERROR;
  }

  auto handle_pos = std::find_if(
    registered_handles.begin(), registered_handles.end(),
    [&](auto handle_ptr) -> bool {
      return handle_ptr->get_name() == handle->get_name();
    });

  // handle exists already
  if (handle_pos != registered_handles.end()) {
    RCLCPP_ERROR(
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
    RCLCPP_ERROR(
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
    RCLCPP_ERROR(
      rclcpp::get_logger(logger_name),
      "cannot get handle. No joint %s found.\n", name.c_str());
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

std::vector<std::string>
RobotHardware::get_registered_joint_names()
{
  return get_registered_names<const JointStateHandle>(registered_joint_state_handles_);
}

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

hardware_interface_ret_t RobotHardware::register_actuator(
  const std::string & actuator_name,
  const std::string & interface_name, const double default_value)
{
  if (actuator_name.empty() || interface_name.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger(kActuatorLoggerName), "actuator name or interface is empty!");
    return HW_RET_ERROR;
  }

  const auto & names_list = registered_actuators_.joint_names;
  const auto it = std::find(names_list.cbegin(), names_list.cend(), actuator_name);
  if (it == names_list.cend()) {
    registered_actuators_.joint_names.push_back(actuator_name);
    control_msgs::msg::InterfaceValue iv;
    iv.interface_names = {interface_name};
    iv.values = {default_value};
    registered_actuators_.interface_values.push_back(iv);
    return HW_RET_OK;
  } else {
    const auto index = std::distance(names_list.cbegin(), it);
    auto & ivs = registered_actuators_.interface_values[index];
    const auto interface_names = ivs.interface_names;
    const auto it = std::find(interface_names.cbegin(), interface_names.cend(), interface_name);
    if (it == interface_names.cend()) {
      ivs.interface_names.push_back(interface_name);
      ivs.values.push_back(default_value);
      return HW_RET_OK;
    } else {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger(kActuatorLoggerName), "actuator with interface (" <<
          actuator_name << ":" << interface_name <<
          ") is already registered!");
      return HW_RET_ERROR;
    }
  }
}

hardware_interface_ret_t RobotHardware::get_actuator_handle(ActuatorHandle & actuator_handle)
{
  const auto & actuator_name = actuator_handle.get_name();
  const auto & interface_name = actuator_handle.get_interface_name();

  if (actuator_name.empty() || interface_name.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        kActuatorLoggerName), "actuator name or interface is ill-defined!");
    return HW_RET_ERROR;
  }

  const auto & names_list = registered_actuators_.joint_names;
  const auto it = std::find(names_list.cbegin(), names_list.cend(), actuator_name);
  if (it == names_list.cend()) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        kActuatorLoggerName), "actuator with name %s not found!", actuator_name);
    return HW_RET_ERROR;
  }

  const auto index = std::distance(names_list.cbegin(), it);
  auto & ivs = registered_actuators_.interface_values[index];
  const auto interface_names = ivs.interface_names;
  const auto if_it = std::find(interface_names.cbegin(), interface_names.cend(), interface_name);
  if (if_it != interface_names.cend()) {
    const auto value_index = std::distance(interface_names.cbegin(), if_it);
    actuator_handle = actuator_handle.with_value_ptr(&(ivs.values[value_index]));
    return HW_RET_OK;
  } else {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(kActuatorLoggerName),
      "actuator with interface (" << actuator_name << ":" << interface_name << ") wasn't found!");
    return HW_RET_ERROR;
  }

  return HW_RET_ERROR;
}

std::vector<ActuatorHandle> RobotHardware::get_registered_actuators()
{
  std::vector<ActuatorHandle> result;
  result.reserve(registered_actuators_.joint_names.size());    // rough estimate

  auto & actuator_names = registered_actuators_.joint_names;
  auto & interface_values = registered_actuators_.interface_values;

  assert(registered_actuators_.joint_names.size() == registered_actuators_.interface_values.size());
  for (size_t i = 0; i < actuator_names.size(); ++i) {
    auto & actuator_interfaces = interface_values[i];
    assert(actuator_interfaces.interface_names.size() == actuator_interfaces.values.size());

    for (size_t j = 0; j < actuator_interfaces.interface_names.size(); ++j) {
      result.emplace_back(
        actuator_names[i], actuator_interfaces.interface_names[j],
        &actuator_interfaces.values[j]);
    }
  }

  return result;
}

const std::vector<std::string> & RobotHardware::get_registered_actuator_names()
{
  return registered_actuators_.joint_names;
}

}  // namespace hardware_interface
