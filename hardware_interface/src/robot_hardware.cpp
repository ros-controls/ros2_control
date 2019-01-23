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
#include "rcutils/logging_macros.h"

namespace hardware_interface
{

hardware_interface_ret_t
RobotHardware::register_joint_state_handle(const JointStateHandle * joint_handle)
{
  auto handle_pos = std::find_if(
    registered_joint_state_handles_.begin(), registered_joint_state_handles_.end(),
    [&](auto joint_handle_ptr) -> bool {
      if (joint_handle_ptr->get_name() == joint_handle->get_name()) {
        return true;
      }
      return false;
    }
  );
  // handle exist already
  if (handle_pos != registered_joint_state_handles_.end()) {
    return HW_RET_ERROR;
  }
  registered_joint_state_handles_.push_back(joint_handle);
  return HW_RET_OK;
}

hardware_interface_ret_t
RobotHardware::register_joint_command_handle(JointCommandHandle * joint_handle)
{
  auto handle_pos = std::find_if(
    registered_joint_command_handles_.begin(), registered_joint_command_handles_.end(),
    [&](auto joint_handle_ptr) -> bool {
      return joint_handle_ptr->get_name() == joint_handle->get_name();
    });

  // handle exist already
  if (handle_pos != registered_joint_command_handles_.end()) {
    return HW_RET_ERROR;
  }
  registered_joint_command_handles_.push_back(joint_handle);
  return HW_RET_OK;
}

hardware_interface_ret_t
RobotHardware::register_operation_mode_handle(OperationModeHandle * operation_mode_handle)
{
  auto operation_pos = std::find_if(
    registered_operation_mode_handles_.begin(), registered_operation_mode_handles_.end(),
    [&](auto operation_handle_ptr) -> bool {
      return operation_handle_ptr->get_name() == operation_mode_handle->get_name();
    });

  // handle exist already
  if (operation_pos != registered_operation_mode_handles_.end()) {
    return HW_RET_ERROR;
  }
  registered_operation_mode_handles_.push_back(operation_mode_handle);
  return HW_RET_OK;
}

hardware_interface_ret_t
RobotHardware::get_joint_state_handle(
  const std::string & name, const JointStateHandle ** joint_state_handle)
{
  if (name.empty()) {
    RCUTILS_LOG_ERROR_NAMED(
      "joint state handle", "cannot get handle! No name given");
    return HW_RET_ERROR;
  }

  THROW_ON_NOT_NULLPTR(*joint_state_handle)

  auto handle_pos = std::find_if(
    registered_joint_state_handles_.begin(), registered_joint_state_handles_.end(),
    [&](auto joint_handle_ptr) -> bool {
      return joint_handle_ptr->get_name() == name;
    });

  if (handle_pos == registered_joint_state_handles_.end()) {
    RCUTILS_LOG_ERROR_NAMED(
      "joint state handle", "cannot get handle. No joint %s found.\n", name.c_str());
    return HW_RET_ERROR;
  }

  *joint_state_handle = *handle_pos;
  return HW_RET_OK;
}

hardware_interface_ret_t
RobotHardware::get_joint_command_handle(
  const std::string & name, JointCommandHandle ** joint_command_handle)
{
  if (name.empty()) {
    RCUTILS_LOG_ERROR_NAMED(
      "joint cmd handle", "cannot get handle! No name given");
    return HW_RET_ERROR;
  }

  THROW_ON_NOT_NULLPTR(*joint_command_handle)

  auto handle_pos = std::find_if(
    registered_joint_command_handles_.begin(), registered_joint_command_handles_.end(),
    [&](auto joint_handle_ptr) -> bool {
      return joint_handle_ptr->get_name() == name;
    });

  if (handle_pos == registered_joint_command_handles_.end()) {
    RCUTILS_LOG_ERROR_NAMED(
      "joint cmd handle", "cannot get handle. No joint %s found.\n", name.c_str());
    return HW_RET_ERROR;
  }

  *joint_command_handle = *handle_pos;
  return HW_RET_OK;
}

hardware_interface_ret_t
RobotHardware::get_operation_mode_handle(
  const std::string & name, OperationModeHandle ** operation_mode_handle)
{
  if (name.empty()) {
    RCUTILS_LOG_ERROR_NAMED(
      "joint operation mode handle", "cannot get handle! No name given");
    return HW_RET_ERROR;
  }

  THROW_ON_NOT_NULLPTR(*operation_mode_handle)

  auto handle_pos = std::find_if(
    registered_operation_mode_handles_.begin(), registered_operation_mode_handles_.end(),
    [&](auto operation_handle_ptr) -> bool {
      return operation_handle_ptr->get_name() == name;
    });

  if (handle_pos == registered_operation_mode_handles_.end()) {
    RCUTILS_LOG_ERROR_NAMED(
      "joint operation mode handle", "cannot get handle. No joint %s found.\n", name.c_str());
    return HW_RET_ERROR;
  }

  *operation_mode_handle = *handle_pos;
  return HW_RET_OK;
}

std::vector<std::string>
RobotHardware::get_registered_joint_names()
{
  std::vector<std::string> joint_names;
  joint_names.reserve(registered_joint_state_handles_.size());
  for (auto joint_state_handle : registered_joint_state_handles_) {
    joint_names.push_back(joint_state_handle->get_name());
  }
  return joint_names;
}

std::vector<std::string>
RobotHardware::get_registered_write_op_names()
{
  std::vector<std::string> op_names;
  op_names.reserve(registered_operation_mode_handles_.size());
  for (auto op_mode_handle : registered_operation_mode_handles_) {
    op_names.push_back(op_mode_handle->get_name());
  }
  return op_names;
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

}  // namespace hardware_interface
