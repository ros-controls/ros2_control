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

  // register actuators and joints
  for (auto index = 0u; index < 3u; ++index) {
    register_actuator(actuator_names[index], "position", pos_dflt_values[index]);
    register_actuator(actuator_names[index], "velocity", vel_dflt_values[index]);
    register_actuator(actuator_names[index], "effort", eff_dflt_values[index]);
    register_actuator(actuator_names[index], "position_command", pos_dflt_values[index]);
    register_actuator(actuator_names[index], "velocity_command", vel_dflt_values[index]);
    register_actuator(actuator_names[index], "effort_command", eff_dflt_values[index]);

    register_joint(joint_names[index], "position", pos_dflt_values[index]);
    register_joint(joint_names[index], "velocity", vel_dflt_values[index]);
    register_joint(joint_names[index], "effort", eff_dflt_values[index]);
    register_joint(joint_names[index], "position_command", pos_dflt_values[index]);
    register_joint(joint_names[index], "velocity_command", vel_dflt_values[index]);
    register_joint(joint_names[index], "effort_command", eff_dflt_values[index]);
  }

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

  auto update_handle = [&](const std::string & joint_name, const std::string & interface_name)
    {
      auto get_handle = [&](const std::string & joint_name, const std::string & interface_name)
        {
          auto joint_handle = std::make_shared<hardware_interface::JointHandle>(
            joint_name,
            interface_name);
          get_joint_handle(*joint_handle);
          return joint_handle;
        };

      get_handle(
        joint_name,
        interface_name)->set_value(
        get_handle(
          joint_name,
          interface_name + "_command")->get_value());
    };

  // update all the joint state handles with their respectives command values
  const std::vector<std::string> interface_names = {"position", "velocity", "effort"};
  for (const auto & joint_name : joint_names) {
    for (const auto & interface_name : interface_names) {
      update_handle(joint_name, interface_name);
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace test_robot_hardware
