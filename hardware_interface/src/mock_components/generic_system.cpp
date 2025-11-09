// Copyright (c) 2021 PickNik, Inc.
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
//
// Author: Jafar Abdi, Denis Stogl

#include "mock_components/generic_system.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

namespace mock_components
{

CallbackReturn GenericSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // check if to create mock command interface for sensor
  auto it = get_hardware_info().hardware_parameters.find("mock_sensor_commands");
  if (it != get_hardware_info().hardware_parameters.end())
  {
    use_mock_sensor_command_interfaces_ = hardware_interface::parse_bool(it->second);
  }
  else
  {
    use_mock_sensor_command_interfaces_ = false;
  }

  // check if to create mock command interface for gpio
  it = get_hardware_info().hardware_parameters.find("mock_gpio_commands");
  if (it != get_hardware_info().hardware_parameters.end())
  {
    use_mock_gpio_command_interfaces_ = hardware_interface::parse_bool(it->second);
  }
  else
  {
    use_mock_gpio_command_interfaces_ = false;
  }

  // check if there is parameter that disables commands
  // this way we simulate disconnected driver
  it = get_hardware_info().hardware_parameters.find("disable_commands");
  if (it != get_hardware_info().hardware_parameters.end())
  {
    command_propagation_disabled_ = hardware_interface::parse_bool(it->second);
  }
  else
  {
    command_propagation_disabled_ = false;
  }

  // check if there is parameter that enables dynamic calculation
  it = get_hardware_info().hardware_parameters.find("calculate_dynamics");
  if (it != get_hardware_info().hardware_parameters.end())
  {
    calculate_dynamics_ = hardware_interface::parse_bool(it->second);
  }
  else
  {
    calculate_dynamics_ = false;
  }
  // do loopback on all other interfaces - starts from 1 or 3 because 0, 1, 2 are position,
  // velocity, and acceleration interface
  // Create a subvector of standard_interfaces_ with the given indices
  for (size_t i = 0; i < (calculate_dynamics_ ? 3 : 1); ++i)
  {
    skip_interfaces_.push_back(standard_interfaces_[i]);
  }

  // process parameters about state following
  position_state_following_offset_ = 0.0;
  custom_interface_with_following_offset_ = "";

  it = get_hardware_info().hardware_parameters.find("position_state_following_offset");
  if (it != get_hardware_info().hardware_parameters.end())
  {
    position_state_following_offset_ = hardware_interface::stod(it->second);
    it = get_hardware_info().hardware_parameters.find("custom_interface_with_following_offset");
    if (it != get_hardware_info().hardware_parameters.end())
    {
      custom_interface_with_following_offset_ = it->second;
    }
  }

  // search for non-standard joint interfaces
  std::vector<std::string> other_interfaces;
  auto populate_non_standard_interfaces =
    [this](auto interface_list, auto & non_standard_interfaces)
  {
    for (const auto & interface : interface_list)
    {
      // add to list if non-standard interface
      if (
        std::find(standard_interfaces_.begin(), standard_interfaces_.end(), interface.name) ==
        standard_interfaces_.end())
      {
        // and does not exist yet
        if (
          std::find(
            non_standard_interfaces.begin(), non_standard_interfaces.end(), interface.name) ==
          non_standard_interfaces.end())
        {
          non_standard_interfaces.emplace_back(interface.name);
        }
      }
    }
  };
  for (const auto & joint : get_hardware_info().joints)
  {
    // populate non-standard command interfaces to other_interfaces
    populate_non_standard_interfaces(joint.command_interfaces, other_interfaces);

    // populate non-standard state interfaces to other_interfaces
    populate_non_standard_interfaces(joint.state_interfaces, other_interfaces);
  }

  // when following offset is used on custom interface then find its index
  if (!custom_interface_with_following_offset_.empty())
  {
    if (
      std::find(
        other_interfaces.begin(), other_interfaces.end(),
        custom_interface_with_following_offset_) != other_interfaces.end())
    {
      RCLCPP_INFO(
        get_logger(), "Custom interface with following offset '%s' found.",
        custom_interface_with_following_offset_.c_str());
    }
    else
    {
      RCLCPP_WARN(
        get_logger(),
        "Custom interface with following offset '%s' does not exist. Offset will not be applied",
        custom_interface_with_following_offset_.c_str());
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::InterfaceDescription>
GenericSystem::export_unlisted_command_interface_descriptions()
{
  std::vector<hardware_interface::InterfaceDescription> command_interface_descriptions;
  // Mock sensor command interfaces
  if (use_mock_sensor_command_interfaces_)
  {
    populate_interfaces(get_hardware_info().sensors, command_interface_descriptions);
  }

  // Mock gpio command interfaces (consider all state interfaces for command interfaces)
  if (use_mock_gpio_command_interfaces_)
  {
    populate_interfaces(get_hardware_info().gpios, command_interface_descriptions);
  }

  return command_interface_descriptions;
}

return_type GenericSystem::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  if (!calculate_dynamics_)
  {
    return ret_val;
  }

  const size_t FOUND_ONCE_FLAG = 1000000;

  const auto & info = get_hardware_info();
  std::vector<size_t> joint_found_in_x_requests_;
  joint_found_in_x_requests_.resize(info.joints.size(), 0);

  for (const auto & key : start_interfaces)
  {
    // check if interface is joint
    auto joint_it_found = std::find_if(
      info.joints.begin(), info.joints.end(),
      [key](const auto & joint) { return (key.find(joint.name) != std::string::npos); });

    if (joint_it_found != info.joints.end())
    {
      const size_t joint_index =
        static_cast<size_t>(std::distance(info.joints.begin(), joint_it_found));
      if (joint_found_in_x_requests_[joint_index] == 0)
      {
        joint_found_in_x_requests_[joint_index] = FOUND_ONCE_FLAG;
      }

      if (key == info.joints[joint_index].name + "/" + hardware_interface::HW_IF_POSITION)
      {
        joint_found_in_x_requests_[joint_index] += 1;
      }
      if (key == info.joints[joint_index].name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        joint_found_in_x_requests_[joint_index] += 1;
      }
      if (key == info.joints[joint_index].name + "/" + hardware_interface::HW_IF_ACCELERATION)
      {
        joint_found_in_x_requests_[joint_index] += 1;
      }
    }
    else
    {
      RCLCPP_DEBUG(
        get_logger(), "Got interface '%s' that is not joint - nothing to do!", key.c_str());
    }
  }

  for (size_t i = 0; i < info.joints.size(); ++i)
  {
    // There has to always be at least one control mode from the above three set
    if (joint_found_in_x_requests_[i] == FOUND_ONCE_FLAG)
    {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' has to have '%s', '%s', or '%s' interface!",
        info.joints[i].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION);
      ret_val = hardware_interface::return_type::ERROR;
    }

    // Currently we don't support multiple interface request
    if (joint_found_in_x_requests_[i] > (FOUND_ONCE_FLAG + 1))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Got multiple (%zu) starting interfaces for joint '%s' - this is not "
        "supported!",
        joint_found_in_x_requests_[i] - FOUND_ONCE_FLAG, info.joints[i].name.c_str());
      ret_val = hardware_interface::return_type::ERROR;
    }
  }

  return ret_val;
}

return_type GenericSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  if (!calculate_dynamics_)
  {
    return hardware_interface::return_type::OK;
  }

  for (const auto & key : start_interfaces)
  {
    // check if interface is joint
    const auto & info = get_hardware_info();
    auto joint_it_found = std::find_if(
      info.joints.begin(), info.joints.end(),
      [key](const auto & joint) { return (key.find(joint.name) != std::string::npos); });

    if (joint_it_found != info.joints.end())
    {
      const size_t joint_index =
        static_cast<size_t>(std::distance(info.joints.begin(), joint_it_found));
      if (key == info.joints[joint_index].name + "/" + hardware_interface::HW_IF_POSITION)
      {
        joint_control_mode_[joint_index] = POSITION_INTERFACE_INDEX;
      }
      if (key == info.joints[joint_index].name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        joint_control_mode_[joint_index] = VELOCITY_INTERFACE_INDEX;
      }
      if (key == info.joints[joint_index].name + "/" + hardware_interface::HW_IF_ACCELERATION)
      {
        joint_control_mode_[joint_index] = ACCELERATION_INTERFACE_INDEX;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn GenericSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (const auto & joint_state : joint_state_interfaces_)
  {
    const std::string & name = joint_state.second.get_name();
    if (joint_state.second.get_data_type() == hardware_interface::HandleDataType::DOUBLE)
    {
      // if initial values are set not set from the URDF
      if (std::isnan(get_state(name)))
      {
        set_state(name, 0.0);
      }
      // set offset anyways
      if (
        joint_state.second.get_interface_name() == hardware_interface::HW_IF_POSITION &&
        custom_interface_with_following_offset_.empty())
      {
        set_state(name, get_state(name) + position_state_following_offset_);
      }
    }
  }
  // Set position control mode per default
  // This will be populated by perform_command_mode_switch
  joint_control_mode_.resize(get_hardware_info().joints.size(), POSITION_INTERFACE_INDEX);
  return hardware_interface::CallbackReturn::SUCCESS;
}

return_type GenericSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (command_propagation_disabled_)
  {
    RCLCPP_WARN(get_logger(), "Command propagation is disabled - no values will be returned!");
    return return_type::OK;
  }

  auto mirror_command_to_state = [this](const auto & name, const auto & data_type) -> return_type
  {
    switch (data_type)
    {
      case hardware_interface::HandleDataType::DOUBLE:
      {
        auto cmd = get_command(name);
        if (std::isinf(cmd))
        {
          return return_type::ERROR;
        }
        else if (std::isfinite(cmd))
        {
          set_state(name, cmd);
        }
        else
        {
          // NaN - do nothing. Command might not be set yet
        }
        break;
      }
      case hardware_interface::HandleDataType::BOOL:
      {
        set_state(name, get_command<bool>(name));
        break;
      }
      default:
      {
      }
        // not handling other types
    }
    return return_type::OK;
  };

  for (size_t j = 0; j < get_hardware_info().joints.size(); ++j)
  {
    if (calculate_dynamics_)
    {
      std::array<double, 3> joint_state_values_ = {
        {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN()}};
      std::array<double, 3> joint_command_values_ = {
        {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN()}};
      const auto & joint_name = get_hardware_info().joints[j].name;
      for (size_t i = 0; i < 3; ++i)
      {
        const auto full_name = joint_name + "/" + standard_interfaces_[i];
        if (has_command(full_name))
        {
          joint_command_values_[i] = get_command(full_name);
        }
        if (has_state(full_name))
        {
          joint_state_values_[i] = get_state(full_name);
        }
      }

      switch (joint_control_mode_[j])
      {
        case ACCELERATION_INTERFACE_INDEX:
        {
          if (std::isnan(joint_state_values_[VELOCITY_INTERFACE_INDEX]))
          {
            joint_state_values_[VELOCITY_INTERFACE_INDEX] = 0.0;
          }

          if (std::isfinite(joint_command_values_[ACCELERATION_INTERFACE_INDEX]))
          {
            joint_state_values_[ACCELERATION_INTERFACE_INDEX] =
              joint_command_values_[ACCELERATION_INTERFACE_INDEX];
          }
          // currently we do backward Euler integration
          joint_state_values_[VELOCITY_INTERFACE_INDEX] +=
            std::isnan(joint_state_values_[ACCELERATION_INTERFACE_INDEX])
              ? 0.0
              : joint_state_values_[ACCELERATION_INTERFACE_INDEX] * period.seconds();
          joint_state_values_[POSITION_INTERFACE_INDEX] +=  // apply offset to positions only
            std::isfinite(joint_state_values_[VELOCITY_INTERFACE_INDEX])
              ? joint_state_values_[VELOCITY_INTERFACE_INDEX] * period.seconds()
              : 0.0 + (custom_interface_with_following_offset_.empty()
                         ? position_state_following_offset_
                         : 0.0);
          break;
        }
        case VELOCITY_INTERFACE_INDEX:
        {
          if (std::isfinite(joint_command_values_[VELOCITY_INTERFACE_INDEX]))
          {
            const double old_velocity = std::isfinite(joint_state_values_[VELOCITY_INTERFACE_INDEX])
                                          ? joint_state_values_[VELOCITY_INTERFACE_INDEX]
                                          : 0.0;
            joint_state_values_[VELOCITY_INTERFACE_INDEX] =
              joint_command_values_[VELOCITY_INTERFACE_INDEX];

            joint_state_values_[ACCELERATION_INTERFACE_INDEX] =
              (joint_state_values_[VELOCITY_INTERFACE_INDEX] - old_velocity) / period.seconds();
          }
          // currently we do backward Euler integration
          joint_state_values_[POSITION_INTERFACE_INDEX] +=  // apply offset to positions only
            std::isfinite(joint_state_values_[VELOCITY_INTERFACE_INDEX])
              ? joint_state_values_[VELOCITY_INTERFACE_INDEX] * period.seconds()
              : 0.0 + (custom_interface_with_following_offset_.empty()
                         ? position_state_following_offset_
                         : 0.0);
          break;
        }
        case POSITION_INTERFACE_INDEX:
        {
          if (std::isfinite(joint_command_values_[POSITION_INTERFACE_INDEX]))
          {
            const double old_position = joint_state_values_[POSITION_INTERFACE_INDEX];
            const double old_velocity = std::isfinite(joint_state_values_[VELOCITY_INTERFACE_INDEX])
                                          ? joint_state_values_[VELOCITY_INTERFACE_INDEX]
                                          : 0.0;

            joint_state_values_[POSITION_INTERFACE_INDEX] =  // apply offset to positions only
              joint_command_values_[POSITION_INTERFACE_INDEX] +
              (custom_interface_with_following_offset_.empty() ? position_state_following_offset_
                                                               : 0.0);

            joint_state_values_[VELOCITY_INTERFACE_INDEX] =
              (joint_state_values_[POSITION_INTERFACE_INDEX] - old_position) / period.seconds();

            joint_state_values_[ACCELERATION_INTERFACE_INDEX] =
              (joint_state_values_[VELOCITY_INTERFACE_INDEX] - old_velocity) / period.seconds();
          }
          break;
        }
      }
      // mirror them back
      for (size_t i = 0; i < standard_interfaces_.size(); ++i)
      {
        if (
          std::isfinite(joint_state_values_[i]) &&
          has_state(joint_name + "/" + standard_interfaces_[i]))
        {
          set_state(joint_name + "/" + standard_interfaces_[i], joint_state_values_[i]);
        }
      }
    }
    else
    {
      for (const auto & joint_command : joint_commands_)
      {
        if (joint_command.get()->get_interface_name() == hardware_interface::HW_IF_POSITION)
        {
          const std::string & name = joint_command.get()->get_name();
          if (has_state(name) && std::isfinite(get_command(name)))
          {
            set_state(
              name, get_command(name) + (custom_interface_with_following_offset_.empty()
                                           ? position_state_following_offset_
                                           : 0.0));
          }
        }
      }
    }
  }

  // do loopback on all other interfaces
  for (const auto & joint_state : joint_states_)
  {
    if (
      std::find(
        skip_interfaces_.begin(), skip_interfaces_.end(),
        joint_state.get()->get_interface_name()) != skip_interfaces_.end())
    {
      continue;
    }
    const std::string & full_interface_name = joint_state.get()->get_name();
    if (has_command(full_interface_name))
    {
      if (
        mirror_command_to_state(full_interface_name, joint_state.get()->get_data_type()) !=
        return_type::OK)
      {
        return return_type::ERROR;
      }
    }
    if (custom_interface_with_following_offset_ == joint_state.get()->get_interface_name())
    {
      const auto cmd =
        get_command(
          joint_state.get()->get_prefix_name() + "/" + hardware_interface::HW_IF_POSITION) +
        position_state_following_offset_;
      if (std::isfinite(cmd))
      {
        set_state(full_interface_name, cmd);
      }
    }
  }

  // Update mimic joints
  const auto & joints = get_hardware_info().joints;
  for (const auto & mimic_joint : get_hardware_info().mimic_joints)
  {
    const auto & mimic_joint_name = joints.at(mimic_joint.joint_index).name;
    const auto & mimicked_joint_name = joints.at(mimic_joint.mimicked_joint_index).name;
    if (has_state(mimic_joint_name + "/" + hardware_interface::HW_IF_POSITION))
    {
      set_state(
        mimic_joint_name + "/" + hardware_interface::HW_IF_POSITION,
        mimic_joint.offset +
          mimic_joint.multiplier *
            get_state(mimicked_joint_name + "/" + hardware_interface::HW_IF_POSITION));
    }
    if (has_state(mimic_joint_name + "/" + hardware_interface::HW_IF_VELOCITY))
    {
      set_state(
        mimic_joint_name + "/" + hardware_interface::HW_IF_VELOCITY,
        mimic_joint.multiplier *
          get_state(mimicked_joint_name + "/" + hardware_interface::HW_IF_VELOCITY));
    }
    if (has_state(mimic_joint_name + "/" + hardware_interface::HW_IF_ACCELERATION))
    {
      set_state(
        mimic_joint_name + "/" + hardware_interface::HW_IF_ACCELERATION,
        mimic_joint.multiplier *
          get_state(mimicked_joint_name + "/" + hardware_interface::HW_IF_ACCELERATION));
    }
  }

  if (use_mock_sensor_command_interfaces_)
  {
    // do loopback on all sensor interfaces as we have exported them all
    for (const auto & sensor_state : sensor_states_)
    {
      const std::string & name = sensor_state.get()->get_name();
      if (mirror_command_to_state(name, sensor_state.get()->get_data_type()) != return_type::OK)
      {
        return return_type::ERROR;
      }
    }
  }

  if (use_mock_gpio_command_interfaces_)
  {
    // do loopback on all gpio interfaces as we have exported them all
    // commands are created for all state interfaces, but in unlisted_commands_
    for (const auto & gpio_state : gpio_states_)
    {
      const std::string & name = gpio_state.get()->get_name();
      if (mirror_command_to_state(name, gpio_state.get()->get_data_type()) != return_type::OK)
      {
        return return_type::ERROR;
      }
    }
  }
  else
  {
    // do loopback on all gpio interfaces, where they exist
    for (const auto & gpio_command : gpio_commands_)
    {
      const std::string & name = gpio_command.get()->get_name();
      if (has_state(name))
      {
        if (mirror_command_to_state(name, gpio_command.get()->get_data_type()) != return_type::OK)
        {
          return return_type::ERROR;
        }
      }
    }
  }

  return return_type::OK;
}

// Private methods
bool GenericSystem::populate_interfaces(
  const std::vector<hardware_interface::ComponentInfo> & components,
  std::vector<hardware_interface::InterfaceDescription> & command_interface_descriptions) const
{
  for (const auto & component : components)
  {
    for (const auto & state_interface : component.state_interfaces)
    {
      // add to state interface to command interface list if not already there
      if (
        std::find_if(
          command_interface_descriptions.begin(), command_interface_descriptions.end(),
          [&component, &state_interface](const auto & desc)
          { return (desc.get_name() == (component.name + "/" + state_interface.name)); }) ==
          command_interface_descriptions.end() &&
        std::find_if(
          component.command_interfaces.begin(), component.command_interfaces.end(),
          [&component, &state_interface](const auto & cmd_if)
          {
            return (
              (component.name + "/" + cmd_if.name) ==
              (component.name + "/" + state_interface.name));
          }) == component.command_interfaces.end())
      {
        hardware_interface::InterfaceDescription description(component.name, state_interface);
        command_interface_descriptions.push_back(description);
      }
    }
  }

  return true;
}
}  // namespace mock_components

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mock_components::GenericSystem, hardware_interface::SystemInterface)
