// Copyright 2024 ros2_control Development Team
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

#include <memory>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using hardware_interface::ActuatorInterface;
using hardware_interface::CommandInterface;
using hardware_interface::return_type;
using hardware_interface::StateInterface;

static std::pair<std::string, std::string> extract_joint_and_interface(
  const std::string & full_name)
{
  // Signature is: interface/joint
  const auto joint_name = full_name.substr(0, full_name.find_last_of('/'));
  const auto interface_name = full_name.substr(full_name.find_last_of('/') + 1);

  return {joint_name, interface_name};
}
struct JointState
{
  double pos;
  double vel;
  double effort;
};

class TestActuatorExclusiveInterfaces : public ActuatorInterface
{
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    for (const auto & j : info.joints)
    {
      (void)j;  // Suppress unused warning
      current_states_.emplace_back(JointState{});
    }

    return CallbackReturn::SUCCESS;
  }

  std::vector<StateInterface> export_state_interfaces() override
  {
    std::vector<StateInterface> state_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
      const auto & joint = info_.joints[i];

      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          joint.name, hardware_interface::HW_IF_POSITION, &current_states_.at(i).pos));
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          joint.name, hardware_interface::HW_IF_VELOCITY, &current_states_.at(i).vel));
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          joint.name, hardware_interface::HW_IF_EFFORT, &current_states_.at(i).effort));
    }
    return state_interfaces;
  }

  std::vector<CommandInterface> export_command_interfaces() override
  {
    std::vector<CommandInterface> command_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
      const auto & joint = info_.joints[i];

      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          joint.name, hardware_interface::HW_IF_POSITION, &current_states_.at(i).pos));
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          joint.name, hardware_interface::HW_IF_VELOCITY, &current_states_.at(i).vel));
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          joint.name, hardware_interface::HW_IF_EFFORT, &current_states_.at(i).effort));
    }
    return command_interfaces;
  }

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override
  {
    std::vector<std::string> claimed_joint_copy = currently_claimed_joints_;

    for (const auto & interface : stop_interfaces)
    {
      const auto && [joint_name, interface_name] = extract_joint_and_interface(interface);
      if (
        interface_name == hardware_interface::HW_IF_POSITION ||
        interface_name == hardware_interface::HW_IF_VELOCITY ||
        interface_name == hardware_interface::HW_IF_EFFORT)
      {
        claimed_joint_copy.erase(
          std::remove(claimed_joint_copy.begin(), claimed_joint_copy.end(), joint_name),
          claimed_joint_copy.end());
      }
    }

    for (const auto & interface : start_interfaces)
    {
      const auto && [joint_name, interface_name] = extract_joint_and_interface(interface);
      if (
        interface_name == hardware_interface::HW_IF_POSITION ||
        interface_name == hardware_interface::HW_IF_VELOCITY ||
        interface_name == hardware_interface::HW_IF_EFFORT)
      {
        if (
          std::find(claimed_joint_copy.begin(), claimed_joint_copy.end(), joint_name) !=
          claimed_joint_copy.end())
        {
          RCLCPP_ERROR_STREAM(
            rclcpp::get_logger("TestActuatorExclusiveInterfaces"),
            "Joint : " << joint_name
                       << " is already claimed. This cannot happen as the hardware "
                          "interface is exclusive.");
          throw std::runtime_error(
            "Joint : " + joint_name +
            " is already claimed. This cannot happen as the hardware interface is exclusive.");
        }
      }
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override
  {
    for (const auto & interface : stop_interfaces)
    {
      const auto && [joint_name, interface_name] = extract_joint_and_interface(interface);
      currently_claimed_joints_.erase(
        std::remove(currently_claimed_joints_.begin(), currently_claimed_joints_.end(), joint_name),
        currently_claimed_joints_.end());
    }

    for (const auto & interface : start_interfaces)
    {
      const auto && [joint_name, interface_name] = extract_joint_and_interface(interface);

      currently_claimed_joints_.push_back(joint_name);
    }

    return hardware_interface::return_type::OK;
  }

  return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return return_type::OK;
  }

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return return_type::OK;
  }

private:
  std::vector<std::string> currently_claimed_joints_;
  std::vector<JointState> current_states_;
};

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(TestActuatorExclusiveInterfaces, hardware_interface::ActuatorInterface)
