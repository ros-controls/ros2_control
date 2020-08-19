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

#include "test_robot_hardware/test_robot_hardware.hpp"

#include <memory>
#include <string>
#include <vector>

namespace test_robot_hardware
{

hardware_interface::return_type
TestRobotHardware::init()
{
  auto ret = hardware_interface::return_type::ERROR;

  js1 = hardware_interface::JointStateHandle(
    joint_name1, &pos1, &vel1, &eff1);
  ret = register_joint_state_handle(&js1);
  if (ret != hardware_interface::return_type::OK) {
    RCLCPP_WARN(logger, "can't register joint state handle %s", joint_name1.c_str());
    return ret;
  }

  js2 = hardware_interface::JointStateHandle(
    joint_name2, &pos2, &vel2, &eff2);
  ret = register_joint_state_handle(&js2);
  if (ret != hardware_interface::return_type::OK) {
    RCLCPP_WARN(logger, "can't register joint state handle %s", joint_name2.c_str());
    return ret;
  }

  js3 = hardware_interface::JointStateHandle(
    joint_name3, &pos3, &vel3, &eff3);
  ret = register_joint_state_handle(&js3);
  if (ret != hardware_interface::return_type::OK) {
    RCLCPP_WARN(logger, "can't register joint state handle %s", joint_name3.c_str());
    return ret;
  }

  jcmd1 = hardware_interface::JointCommandHandle(joint_name1, &cmd1);
  ret = register_joint_command_handle(&jcmd1);
  if (ret != hardware_interface::return_type::OK) {
    RCLCPP_WARN(logger, "can't register joint command handle %s", joint_name1.c_str());
    return ret;
  }

  jcmd2 = hardware_interface::JointCommandHandle(joint_name2, &cmd2);
  ret = register_joint_command_handle(&jcmd2);
  if (ret != hardware_interface::return_type::OK) {
    RCLCPP_WARN(logger, "can't register joint command handle %s", joint_name2.c_str());
    return ret;
  }

  jcmd3 = hardware_interface::JointCommandHandle(joint_name3, &cmd3);
  ret = register_joint_command_handle(&jcmd3);
  if (ret != hardware_interface::return_type::OK) {
    RCLCPP_WARN(logger, "can't register joint command handle %s", joint_name3.c_str());
    return ret;
  }

  read_op_handle1 = hardware_interface::OperationModeHandle(
    read_op_handle_name1,
    reinterpret_cast<hardware_interface::OperationMode *>(&read1));
  ret = register_operation_mode_handle(&read_op_handle1);
  if (ret != hardware_interface::return_type::OK) {
    RCLCPP_WARN(logger, "can't register operation mode handle %s", read_op_handle_name1.c_str());
    return ret;
  }

  read_op_handle2 = hardware_interface::OperationModeHandle(
    read_op_handle_name2,
    reinterpret_cast<hardware_interface::OperationMode *>(&read2));
  ret = register_operation_mode_handle(&read_op_handle2);
  if (ret != hardware_interface::return_type::OK) {
    RCLCPP_WARN(logger, "can't register operation mode handle %s", read_op_handle_name2.c_str());
    return ret;
  }

  write_op_handle1 = hardware_interface::OperationModeHandle(
    write_op_handle_name1,
    reinterpret_cast<hardware_interface::OperationMode *>(&write1));
  ret = register_operation_mode_handle(&write_op_handle1);
  if (ret != hardware_interface::return_type::OK) {
    RCLCPP_WARN(logger, "can't register operation mode handle %s", write_op_handle_name1.c_str());
    return ret;
  }

  write_op_handle2 = hardware_interface::OperationModeHandle(
    write_op_handle_name2,
    reinterpret_cast<hardware_interface::OperationMode *>(&write2));
  ret = register_operation_mode_handle(&write_op_handle2);
  if (ret != hardware_interface::return_type::OK) {
    RCLCPP_WARN(logger, "can't register operation mode handle %s", write_op_handle_name2.c_str());
    return ret;
  }

  //

  register_actuator("actuator1", "position", 1.1);
  register_actuator("actuator1", "velocity", 1.2);
  register_actuator("actuator1", "effort", 1.3);
  register_actuator("actuator1", "position_command", 1.1);
  register_actuator("actuator1", "velocity_command", 1.2);
  register_actuator("actuator1", "effort_command", 1.3);

  register_actuator("actuator2", "position", 2.1);
  register_actuator("actuator2", "velocity", 2.2);
  register_actuator("actuator2", "effort", 2.3);
  register_actuator("actuator2", "position_command", 2.1);
  register_actuator("actuator2", "velocity_command", 2.2);
  register_actuator("actuator2", "effort_command", 2.3);

  register_actuator("actuator3", "position", 3.1);
  register_actuator("actuator3", "velocity", 3.2);
  register_actuator("actuator3", "effort", 3.3);
  register_actuator("actuator3", "position_command", 3.1);
  register_actuator("actuator3", "velocity_command", 3.2);
  register_actuator("actuator3", "effort_command", 3.3);

  register_joint("joint1", "position", 1.1);
  register_joint("joint1", "velocity", 1.2);
  register_joint("joint1", "effort", 1.3);
  register_joint("joint1", "position_command", 1.1);
  register_joint("joint1", "velocity_command", 1.2);
  register_joint("joint1", "effort_command", 1.3);

  register_joint("joint2", "position", 2.1);
  register_joint("joint2", "velocity", 2.2);
  register_joint("joint2", "effort", 2.3);
  register_joint("joint2", "position_command", 2.1);
  register_joint("joint2", "velocity_command", 2.2);
  register_joint("joint2", "effort_command", 2.3);

  register_joint("joint3", "position", 3.1);
  register_joint("joint3", "velocity", 3.2);
  register_joint("joint3", "effort", 3.3);
  register_joint("joint3", "position_command", 3.1);
  register_joint("joint3", "velocity_command", 3.2);
  register_joint("joint3", "effort_command", 3.3);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
TestRobotHardware::read()
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
TestRobotHardware::write()
{
  pos1 = cmd1;
  pos2 = cmd2;
  pos3 = cmd3;
  return hardware_interface::return_type::OK;
}

}  // namespace test_robot_hardware
