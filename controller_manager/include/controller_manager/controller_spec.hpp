// Copyright 2020 Open Source Robotics Foundation, Inc.
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

/*
 * Author: Wim Meeussen
 */

#ifndef CONTROLLER_MANAGER__CONTROLLER_SPEC_HPP_
#define CONTROLLER_MANAGER__CONTROLLER_SPEC_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>
#include "controller_interface/controller_interface_base.hpp"
#include "hardware_interface/controller_info.hpp"

namespace controller_manager
{
/// Controller Specification
/**
 * This struct contains both a pointer to a given controller, \ref c, as well
 * as information about the controller, \ref info.
 *
 */
struct ControllerSpec
{
  hardware_interface::ControllerInfo info;
  controller_interface::ControllerInterfaceBaseSharedPtr c;
  std::shared_ptr<rclcpp::Time> next_update_cycle_time;

  controller_interface::InterfaceConfiguration get_remapped_command_interface_configuration() const
  {
    return get_remapped_interface_configuration(
      c->command_interface_configuration(), c->get_command_interfaces_remap());
  }

  controller_interface::InterfaceConfiguration get_remapped_state_interface_configuration() const
  {
    return get_remapped_interface_configuration(
      c->state_interface_configuration(), c->get_state_interfaces_remap());
  }

private:
  controller_interface::InterfaceConfiguration get_remapped_interface_configuration(
    const controller_interface::InterfaceConfiguration & interface_cfg,
    const std::map<std::string, std::string> & remap) const
  {
    if (interface_cfg.type != controller_interface::interface_configuration_type::INDIVIDUAL)
    {
      return interface_cfg;
    }
    else
    {
      if (c->get_command_interfaces_remap().empty())
      {
        return interface_cfg;
      }
      else
      {
        controller_interface::InterfaceConfiguration remapped_cmd_itf_cfg = interface_cfg;
        for (auto & [key, value] : remap)
        {
          auto it =
            std::find(remapped_cmd_itf_cfg.names.begin(), remapped_cmd_itf_cfg.names.end(), key);
          if (it != remapped_cmd_itf_cfg.names.end())
          {
            *it = value;
          }
        }
        return remapped_cmd_itf_cfg;
      }
    }
  }
};

struct ControllerChainSpec
{
  std::vector<std::string> following_controllers;
  std::vector<std::string> preceding_controllers;
};
}  // namespace controller_manager
#endif  // CONTROLLER_MANAGER__CONTROLLER_SPEC_HPP_
