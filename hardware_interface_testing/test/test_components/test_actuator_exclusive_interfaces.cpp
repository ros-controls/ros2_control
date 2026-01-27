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
#include "ros2_control_test_assets/test_hardware_interface_constants.hpp"

using hardware_interface::ActuatorInterface;
using hardware_interface::CommandInterface;
using hardware_interface::return_type;
using hardware_interface::StateInterface;

namespace
{
void verify_internal_lifecycle_id(uint8_t expected_id, uint8_t actual_id)
{
  if (expected_id != actual_id)
  {
    throw std::runtime_error(
      "Internal lifecycle ID does not match the expected lifecycle ID. Expected: " +
      std::to_string(expected_id) + ", Actual: " + std::to_string(actual_id));
  }
}
}  // namespace

static std::pair<std::string, std::string> extract_joint_and_interface(
  const std::string & full_name)
{
  // Signature is: interface/joint
  const auto joint_name = full_name.substr(0, full_name.find_last_of('/'));
  const auto interface_name = full_name.substr(full_name.find_last_of('/') + 1);

  return {joint_name, interface_name};
}

class TestActuatorExclusiveInterfaces : public ActuatorInterface
{
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (ActuatorInterface::on_init(params) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }
    verify_internal_lifecycle_id(get_lifecycle_id(), get_lifecycle_state().id());

    return CallbackReturn::SUCCESS;
  }

  std::vector<StateInterface::ConstSharedPtr> on_export_state_interfaces() override
  {
    verify_internal_lifecycle_id(get_lifecycle_id(), get_lifecycle_state().id());
    std::vector<StateInterface::ConstSharedPtr> state_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
      const auto & joint = info_.joints[i];

      position_states_.push_back(
        std::make_shared<StateInterface>(joint.name, hardware_interface::HW_IF_POSITION));
      (void)position_states_.back()->set_value(0.0, false);
      velocity_states_.push_back(
        std::make_shared<StateInterface>(joint.name, hardware_interface::HW_IF_VELOCITY));
      (void)velocity_states_.back()->set_value(0.0, false);
      effort_states_.push_back(
        std::make_shared<StateInterface>(joint.name, hardware_interface::HW_IF_EFFORT));
      (void)effort_states_.back()->set_value(0.0, false);
      state_interfaces.push_back(position_states_.back());
      state_interfaces.push_back(velocity_states_.back());
      state_interfaces.push_back(effort_states_.back());
    }
    return state_interfaces;
  }

  std::vector<CommandInterface::SharedPtr> on_export_command_interfaces() override
  {
    verify_internal_lifecycle_id(get_lifecycle_id(), get_lifecycle_state().id());
    std::vector<CommandInterface::SharedPtr> command_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
      const auto & joint = info_.joints[i];

      position_commands_.push_back(
        std::make_shared<CommandInterface>(joint.name, hardware_interface::HW_IF_POSITION));
      (void)position_commands_.back()->set_value(0.0, false);
      velocity_commands_.push_back(
        std::make_shared<CommandInterface>(joint.name, hardware_interface::HW_IF_VELOCITY));
      (void)velocity_commands_.back()->set_value(0.0, false);
      effort_commands_.push_back(
        std::make_shared<CommandInterface>(joint.name, hardware_interface::HW_IF_EFFORT));
      (void)effort_commands_.back()->set_value(0.0, false);
      command_interfaces.push_back(position_commands_.back());
      command_interfaces.push_back(velocity_commands_.back());
      command_interfaces.push_back(effort_commands_.back());
    }
    return command_interfaces;
  }

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override
  {
    verify_internal_lifecycle_id(get_lifecycle_id(), get_lifecycle_state().id());
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
    verify_internal_lifecycle_id(get_lifecycle_id(), get_lifecycle_state().id());
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
    verify_internal_lifecycle_id(get_lifecycle_id(), get_lifecycle_state().id());
    return return_type::OK;
  }

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    verify_internal_lifecycle_id(get_lifecycle_id(), get_lifecycle_state().id());
    return return_type::OK;
  }

private:
  std::vector<std::string> currently_claimed_joints_;
  std::vector<StateInterface::SharedPtr> position_states_;
  std::vector<StateInterface::SharedPtr> velocity_states_;
  std::vector<StateInterface::SharedPtr> effort_states_;
  std::vector<CommandInterface::SharedPtr> position_commands_;
  std::vector<CommandInterface::SharedPtr> velocity_commands_;
  std::vector<CommandInterface::SharedPtr> effort_commands_;
};

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(TestActuatorExclusiveInterfaces, hardware_interface::ActuatorInterface)
