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
  for (const auto & joint : get_hardware_info().joints)
  {
    // populate non-standard command interfaces to other_interfaces_
    populate_non_standard_interfaces(joint.command_interfaces, other_interfaces_);

    // populate non-standard state interfaces to other_interfaces_
    populate_non_standard_interfaces(joint.state_interfaces, other_interfaces_);
  }

  // when following offset is used on custom interface then find its index
  if (!custom_interface_with_following_offset_.empty())
  {
    auto if_it = std::find(
      other_interfaces_.begin(), other_interfaces_.end(), custom_interface_with_following_offset_);
    if (if_it != other_interfaces_.end())
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

  for (const auto & sensor : get_hardware_info().sensors)
  {
    for (const auto & interface : sensor.state_interfaces)
    {
      if (
        std::find(sensor_interfaces_.begin(), sensor_interfaces_.end(), interface.name) ==
        sensor_interfaces_.end())
      {
        sensor_interfaces_.emplace_back(interface.name);
      }
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
        if (!calculate_dynamics_)
        {
          RCLCPP_WARN(
            get_logger(),
            "Requested velocity mode for joint '%s' without dynamics calculation enabled - this "
            "might lead to wrong feedback and unexpected behavior.",
            info.joints[joint_index].name.c_str());
        }
        joint_found_in_x_requests_[joint_index] += 1;
      }
      if (key == info.joints[joint_index].name + "/" + hardware_interface::HW_IF_ACCELERATION)
      {
        if (!calculate_dynamics_)
        {
          RCLCPP_WARN(
            get_logger(),
            "Requested acceleration mode for joint '%s' without dynamics calculation enabled - "
            "this might lead to wrong feedback and unexpected behavior.",
            info.joints[joint_index].name.c_str());
        }
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

  // Set position control mode per default
  joint_control_mode_.resize(get_hardware_info().joints.size(), POSITION_INTERFACE_INDEX);

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

hardware_interface::CallbackReturn GenericSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (const auto & joint_state : joint_state_interfaces_)
  {
    const std::string & name = joint_state.second.get_name();
    // TODO(anyone): consider initial value for non-double interfaces as well
    double val = joint_state.second.interface_info.initial_value.empty()
                   ? 0.0
                   : hardware_interface::stod(joint_state.second.interface_info.initial_value);
    if (
      joint_state.second.get_interface_name() == standard_interfaces_[POSITION_INTERFACE_INDEX] &&
      custom_interface_with_following_offset_.empty())
    {
      val += position_state_following_offset_;
    }
    set_state(name, val);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

return_type GenericSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (command_propagation_disabled_)
  {
    RCLCPP_WARN(get_logger(), "Command propagation is disabled - no values will be returned!");
    return return_type::OK;
  }

  for (size_t j = 0; j < get_hardware_info().joints.size(); ++j)
  {
    if (calculate_dynamics_)
    {
      std::array<double, 3> joint_state_values_ = {
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
      std::array<double, 3> joint_command_values_ = {
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
      const auto joint_name = get_hardware_info().joints[j].name;
      {
        auto it_pos = std::find_if(
          joint_commands_.begin(), joint_commands_.end(),
          [this, &joint_name](const auto & command)
          {
            return command.get()->get_name() ==
                   joint_name + "/" + standard_interfaces_[POSITION_INTERFACE_INDEX];
          });
        if (it_pos != joint_commands_.end())
        {
          joint_command_values_[POSITION_INTERFACE_INDEX] = get_command(it_pos->get()->get_name());
        }
        auto it_vel = std::find_if(
          joint_commands_.begin(), joint_commands_.end(),
          [this, &joint_name](const auto & command)
          {
            return command.get()->get_name() ==
                   joint_name + "/" + standard_interfaces_[VELOCITY_INTERFACE_INDEX];
          });
        if (it_vel != joint_commands_.end())
        {
          joint_command_values_[VELOCITY_INTERFACE_INDEX] = get_command(it_vel->get()->get_name());
        }
        auto it_acc = std::find_if(
          joint_commands_.begin(), joint_commands_.end(),
          [this, &joint_name](const auto & command)
          {
            return command.get()->get_name() ==
                   joint_name + "/" + standard_interfaces_[ACCELERATION_INTERFACE_INDEX];
          });
        if (it_acc != joint_commands_.end())
        {
          joint_command_values_[ACCELERATION_INTERFACE_INDEX] =
            get_command(it_acc->get()->get_name());
        }
      }
      {
        auto it_pos = std::find_if(
          joint_states_.begin(), joint_states_.end(),
          [this, &joint_name](const auto & command)
          {
            return command.get()->get_name() ==
                   joint_name + "/" + standard_interfaces_[POSITION_INTERFACE_INDEX];
          });
        if (it_pos != joint_states_.end())
        {
          joint_state_values_[POSITION_INTERFACE_INDEX] = get_state(it_pos->get()->get_name());
        }
        auto it_vel = std::find_if(
          joint_states_.begin(), joint_states_.end(),
          [this, &joint_name](const auto & command)
          {
            return command.get()->get_name() ==
                   joint_name + "/" + standard_interfaces_[VELOCITY_INTERFACE_INDEX];
          });
        if (it_vel != joint_states_.end())
        {
          joint_state_values_[VELOCITY_INTERFACE_INDEX] = get_state(it_vel->get()->get_name());
        }
        auto it_acc = std::find_if(
          joint_states_.begin(), joint_states_.end(),
          [this, &joint_name](const auto & command)
          {
            return command.get()->get_name() ==
                   joint_name + "/" + standard_interfaces_[ACCELERATION_INTERFACE_INDEX];
          });
        if (it_acc != joint_states_.end())
        {
          joint_state_values_[ACCELERATION_INTERFACE_INDEX] = get_state(it_acc->get()->get_name());
        }
      }

      switch (joint_control_mode_[j])
      {
        case ACCELERATION_INTERFACE_INDEX:
        {
          // currently we do backward integration
          joint_state_values_[POSITION_INTERFACE_INDEX] +=  // apply offset to positions only
            joint_state_values_[VELOCITY_INTERFACE_INDEX] * period.seconds() +
            (custom_interface_with_following_offset_.empty() ? position_state_following_offset_
                                                             : 0.0);

          joint_state_values_[VELOCITY_INTERFACE_INDEX] +=
            joint_state_values_[ACCELERATION_INTERFACE_INDEX] * period.seconds();

          if (!std::isnan(joint_command_values_[ACCELERATION_INTERFACE_INDEX]))
          {
            joint_state_values_[ACCELERATION_INTERFACE_INDEX] =
              joint_command_values_[ACCELERATION_INTERFACE_INDEX];
          }
          break;
        }
        case VELOCITY_INTERFACE_INDEX:
        {
          // currently we do backward integration
          joint_state_values_[POSITION_INTERFACE_INDEX] +=  // apply offset to positions only
            joint_state_values_[VELOCITY_INTERFACE_INDEX] * period.seconds() +
            (custom_interface_with_following_offset_.empty() ? position_state_following_offset_
                                                             : 0.0);

          if (!std::isnan(joint_command_values_[VELOCITY_INTERFACE_INDEX]))
          {
            const double old_velocity = joint_state_values_[VELOCITY_INTERFACE_INDEX];

            joint_state_values_[VELOCITY_INTERFACE_INDEX] =
              joint_command_values_[VELOCITY_INTERFACE_INDEX];

            joint_state_values_[ACCELERATION_INTERFACE_INDEX] =
              (joint_state_values_[VELOCITY_INTERFACE_INDEX] - old_velocity) / period.seconds();
          }
          break;
        }
        case POSITION_INTERFACE_INDEX:
        {
          if (!std::isnan(joint_command_values_[POSITION_INTERFACE_INDEX]))
          {
            const double old_position = joint_state_values_[POSITION_INTERFACE_INDEX];
            const double old_velocity = joint_state_values_[VELOCITY_INTERFACE_INDEX];

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
    }
    else
    {
      for (const auto & joint_command : joint_command_interfaces_)
      {
        if (
          joint_command.second.get_interface_name() ==
          standard_interfaces_[POSITION_INTERFACE_INDEX])
        {
          const std::string & name = joint_command.second.get_name();
          auto it = joint_state_interfaces_.find(name);
          if (it != joint_state_interfaces_.end())
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

  // do loopback on all other interfaces - starts from 1 or 3 because 0, 1, 2 are position,
  // velocity, and acceleration interface
  // Create a subvector of standard_interfaces_ with the given indices
  std::vector<std::string> skip_interfaces;
  for (size_t i = 0; i < (calculate_dynamics_ ? 3 : 1); ++i)
  {
    skip_interfaces.push_back(standard_interfaces_[i]);
  }
  // TODO(anyone): optimize by using joint_command_interfaces_/joint_state_interfaces_ map
  for (const auto & joint_command : joint_commands_)
  {
    if (
      std::find(
        skip_interfaces.begin(), skip_interfaces.end(),
        joint_command.get()->get_interface_name()) != skip_interfaces.end())
    {
      continue;
    }
    const std::string & full_interface_name = joint_command.get()->get_name();
    auto it = std::find_if(
      joint_states_.begin(), joint_states_.end(), [&full_interface_name](const auto & state)
      { return state.get()->get_name() == full_interface_name; });
    if (it != joint_states_.end())
    {
      if (
        joint_command.get()->get_interface_name() ==
          standard_interfaces_[POSITION_INTERFACE_INDEX] &&
        custom_interface_with_following_offset_ == full_interface_name)
      {
        set_state(
          full_interface_name, get_command(full_interface_name) + position_state_following_offset_);
      }
      else
      {
        set_state(full_interface_name, get_command(full_interface_name));
      }
    }
  }

  for (const auto & mimic_joint : get_hardware_info().mimic_joints)
  {
    set_state(
      mimic_joint.joint_name,
      mimic_joint.offset + mimic_joint.multiplier * get_state(mimic_joint.mimicked_joint_name));
  }

  if (use_mock_sensor_command_interfaces_)
  {
    // do loopback on all sensor interfaces as we have exported them all
    for (const auto & sensor_state : sensor_states_)
    {
      const std::string & name = sensor_state.get()->get_name();
      set_state(name, get_command(name));
    }
  }

  if (use_mock_gpio_command_interfaces_)
  {
    // do loopback on all gpio interfaces as we have exported them all
    // TODO(anyone): how to pass data_type?
    // from gpio_command_interfaces_ maybe?
    for (const auto & gpio_command : gpio_commands_)
    {
      const std::string & name = gpio_command.get()->get_name();
      set_state(name, get_command(name));
    }
  }
  else
  {
    // do loopback on all gpio interfaces, where they exist
    for (const auto & gpio_command : gpio_commands_)
    {
      // TODO(anyone): how to pass data_type?
      // from gpio_command_interfaces_ maybe?
      const std::string & name = gpio_command.get()->get_name();
      auto it = std::find_if(
        gpio_states_.begin(), gpio_states_.end(),
        [&name](const auto & state) { return state.get()->get_name() == name; });
      if (it != gpio_states_.end())
      {
        set_state(name, get_command(name));
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
    for (const auto & interface : component.state_interfaces)
    {
      // add to list if not already there
      if (
        std::find_if(
          command_interface_descriptions.begin(), command_interface_descriptions.end(),
          [&component, &interface](const auto & desc)
          { return (desc.get_name() == (component.name + "/" + interface.name)); }) ==
        command_interface_descriptions.end())
      {
        hardware_interface::InterfaceDescription description(component.name, interface);
        command_interface_descriptions.push_back(description);
      }
    }
  }

  return true;
}
}  // namespace mock_components

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mock_components::GenericSystem, hardware_interface::SystemInterface)
