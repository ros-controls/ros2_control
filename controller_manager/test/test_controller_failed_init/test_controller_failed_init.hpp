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

#ifndef TEST_CONTROLLER_FAILED_INIT__TEST_CONTROLLER_FAILED_INIT_HPP_
#define TEST_CONTROLLER_FAILED_INIT__TEST_CONTROLLER_FAILED_INIT_HPP_

#include "controller_interface/controller_interface.hpp"
#include "controller_manager/visibility_control.h"

namespace test_controller_failed_init
{
// indicating the node name under which the controller node
// is being loaded.
constexpr char TEST_CONTROLLER_NAME[] = "test_controller_failed_init_name";
// corresponds to the name listed within the pluginlib xml
constexpr char TEST_CONTROLLER_FAILED_INIT_CLASS_NAME[] =
  "controller_manager/test_controller_failed_init";
class TestControllerFailedInit : public controller_interface::ControllerInterface
{
public:
  CONTROLLER_MANAGER_PUBLIC
  TestControllerFailedInit();

  CONTROLLER_MANAGER_PUBLIC
  virtual ~TestControllerFailedInit() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
  }

  rclcpp::NodeOptions define_custom_node_options() const override
  {
    return rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true);
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
};

}  // namespace test_controller_failed_init

#endif  // TEST_CONTROLLER_FAILED_INIT__TEST_CONTROLLER_FAILED_INIT_HPP_
