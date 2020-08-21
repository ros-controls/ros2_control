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

#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "hardware_interface/actuator_handle.hpp"
#include "hardware_interface/joint_command_handle.hpp"
#include "hardware_interface/joint_handle.hpp"
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
  return_type
  register_joint_state_handle(const JointStateHandle * joint_handle);

  HARDWARE_INTERFACE_PUBLIC
  return_type
  register_joint_command_handle(JointCommandHandle * joint_handle);

  HARDWARE_INTERFACE_PUBLIC
  return_type
  register_operation_mode_handle(OperationModeHandle * operation_mode);

  HARDWARE_INTERFACE_PUBLIC
  return_type
  get_joint_state_handle(const std::string & name, const JointStateHandle ** joint_state_handle);

  HARDWARE_INTERFACE_PUBLIC
  return_type
  get_joint_command_handle(const std::string & name, JointCommandHandle ** joint_command_handle);

  HARDWARE_INTERFACE_PUBLIC
  return_type
  get_operation_mode_handle(const std::string & name, OperationModeHandle ** operation_mode_handle);

  /*
  HARDWARE_INTERFACE_PUBLIC
  std::vector<std::string>
  get_registered_joint_names();
  */

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

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface_ret_t register_actuator(
    const std::string & actuator_name, const std::string & interface_name,
    double default_value = 0.0);

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface_ret_t register_joint(
    const std::string & joint_name, const std::string & interface_name, double default_value = 0.0);

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface_ret_t get_actuator_handle(ActuatorHandle & actuator_handle);

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface_ret_t get_joint_handle(JointHandle & joint_handle);

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface_ret_t get_actuator_handles(
    std::vector<ActuatorHandle> & actuator_handles,
    const std::string & interface_name);

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface_ret_t get_joint_handles(
    std::vector<JointHandle> & joint_handles,
    const std::string & interface_name);

  HARDWARE_INTERFACE_PUBLIC
  const std::vector<std::string> & get_registered_actuator_names();

  HARDWARE_INTERFACE_PUBLIC
  const std::vector<std::string> & get_registered_joint_names();

  HARDWARE_INTERFACE_PUBLIC
  const std::vector<std::string> & get_registered_actuator_interface_names(
    const std::string & actuator_name);

  HARDWARE_INTERFACE_PUBLIC
  const std::vector<std::string> & get_registered_joint_interface_names(
    const std::string & joint_name);

  HARDWARE_INTERFACE_PUBLIC
  std::vector<ActuatorHandle> get_registered_actuators();

  HARDWARE_INTERFACE_PUBLIC
  std::vector<JointHandle> get_registered_joints();

private:
  std::vector<const JointStateHandle *> registered_joint_state_handles_;
  std::vector<JointCommandHandle *> registered_joint_command_handles_;
  std::vector<OperationModeHandle *> registered_operation_mode_handles_;

  control_msgs::msg::DynamicJointState registered_actuators_;
  control_msgs::msg::DynamicJointState registered_joints_;
};

using RobotHardwareSharedPtr = std::shared_ptr<RobotHardware>;

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__ROBOT_HARDWARE_HPP_
