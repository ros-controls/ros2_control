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

#ifndef HARDWARE_INTERFACE__ROBOT_HARDWARE_HPP_
#define HARDWARE_INTERFACE__ROBOT_HARDWARE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/joint_command_handle.hpp"
#include "hardware_interface/joint_state_handle.hpp"
#include "hardware_interface/operation_mode_handle.hpp"
#include "hardware_interface/robot_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{

class RobotHardware : public RobotHardwareInterface
{
public:
  HARDWARE_INTERFACE_PUBLIC
  RobotHardware() = default;

  HARDWARE_INTERFACE_PUBLIC
  virtual
  ~RobotHardware() = default;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface_ret_t
  register_joint_state_handle(const JointStateHandle * joint_handle);

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface_ret_t
  register_joint_command_handle(JointCommandHandle * joint_handle);

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface_ret_t
  register_operation_mode_handle(OperationModeHandle * operation_mode);

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface_ret_t
  get_joint_state_handle(const std::string & name, const JointStateHandle ** joint_state_handle);

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface_ret_t
  get_joint_command_handle(const std::string & name, JointCommandHandle ** joint_command_handle);

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface_ret_t
  get_operation_mode_handle(const std::string & name, OperationModeHandle ** operation_mode_handle);

  HARDWARE_INTERFACE_PUBLIC
  std::vector<std::string>
  get_registered_joint_names();

  HARDWARE_INTERFACE_PUBLIC
  std::vector<std::string>
  get_registered_write_op_names();

  HARDWARE_INTERFACE_PUBLIC
  std::vector<const JointStateHandle *>
  get_registered_joint_state_handles();

  HARDWARE_INTERFACE_PUBLIC
  std::vector<JointCommandHandle *>
  get_registered_joint_command_handles();

  HARDWARE_INTERFACE_PUBLIC
  std::vector<OperationModeHandle *>
  get_registered_operation_mode_handles();

private:
  std::vector<const JointStateHandle *> registered_joint_state_handles_;
  std::vector<JointCommandHandle *> registered_joint_command_handles_;
  std::vector<OperationModeHandle *> registered_operation_mode_handles_;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__ROBOT_HARDWARE_HPP_
