// Copyright 2020 Department of Engineering Cybernetics, NTNU
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

#ifndef TEST_CONTROLLER_WITH_INTERFACES__TEST_CONTROLLER_WITH_INTERFACES_HPP_
#define TEST_CONTROLLER_WITH_INTERFACES__TEST_CONTROLLER_WITH_INTERFACES_HPP_

#include <memory>
#include <string>

#include "controller_interface/visibility_control.h"
#include "controller_manager/controller_manager.hpp"

namespace test_controller_with_interfaces
{
// Corresponds to the name listed within the pluginglib xml
constexpr char TEST_CONTROLLER_WITH_INTERFACES_CLASS_NAME[] =
  "controller_manager/test_controller_with_interfaces";
// Corresponds to the command interface to claim
constexpr char TEST_CONTROLLER_COMMAND_INTERFACE[] = "joint2/velocity";
class TestControllerWithInterfaces : public controller_interface::ControllerInterface
{
public:
  CONTROLLER_MANAGER_PUBLIC
  TestControllerWithInterfaces();

  CONTROLLER_MANAGER_PUBLIC
  virtual ~TestControllerWithInterfaces() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::INDIVIDUAL,
      {TEST_CONTROLLER_COMMAND_INTERFACE}};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
  }

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CONTROLLER_MANAGER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;

  CONTROLLER_MANAGER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_MANAGER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
};

}  // namespace test_controller_with_interfaces

#endif  // TEST_CONTROLLER_WITH_INTERFACES__TEST_CONTROLLER_WITH_INTERFACES_HPP_
