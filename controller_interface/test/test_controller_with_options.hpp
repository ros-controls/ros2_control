// Copyright 2021 ros2_control development team
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

#ifndef TEST_CONTROLLER_WITH_OPTIONS_HPP_
#define TEST_CONTROLLER_WITH_OPTIONS_HPP_

#include <map>
#include <string>

#include "controller_interface/controller_interface.hpp"

namespace controller_with_options
{
/**
 * Example of Controller using the ControllerInterface::init(const std::string &,
 * rclcpp::NodeOptions &) function. In this example, we set the node options so that parameters
 * are automatically declared by overrides. This is a rare use case, but it can be useful in some
 * situations.
 */
class ControllerWithOptions : public controller_interface::ControllerInterface
{
public:
  ControllerWithOptions() = default;
  LifecycleNodeInterface::CallbackReturn on_init() override
  {
    if (get_node()->get_parameters("parameter_list", params))
    {
      RCLCPP_INFO_STREAM(get_node()->get_logger(), "I found " << params.size() << " parameters.");
      return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  rclcpp::NodeOptions define_custom_node_options() const override
  {
    return rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true);
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::return_type update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return controller_interface::return_type::OK;
  }

  std::map<std::string, double> params;
};
}  // namespace controller_with_options

#endif  // TEST_CONTROLLER_WITH_OPTIONS_HPP_
