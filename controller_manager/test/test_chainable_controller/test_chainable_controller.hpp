// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#ifndef TEST_CHAINABLE_CONTROLLER__TEST_CHAINABLE_CONTROLLER_HPP_
#define TEST_CHAINABLE_CONTROLLER__TEST_CHAINABLE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_manager/visibility_control.h"
#include "rclcpp/subscription.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace test_chainable_controller
{
using CmdType = std_msgs::msg::Float64MultiArray;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// indicating the node name under which the controller node
// is being loaded.
constexpr char TEST_CONTROLLER_NAME[] = "test_chainable_controller_name";
// corresponds to the name listed within the pluginlib xml
constexpr char TEST_CONTROLLER_CLASS_NAME[] = "controller_manager/test_chainable_controller";
class TestChainableController : public controller_interface::ChainableControllerInterface
{
public:
  CONTROLLER_MANAGER_PUBLIC
  TestChainableController();

  CONTROLLER_MANAGER_PUBLIC
  virtual ~TestChainableController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CONTROLLER_MANAGER_PUBLIC
  CallbackReturn on_init() override;

  CONTROLLER_MANAGER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_MANAGER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_MANAGER_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_MANAGER_PUBLIC
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  controller_interface::return_type update_reference_from_subscribers() override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Testing-relevant methods
  CONTROLLER_MANAGER_PUBLIC
  void set_command_interface_configuration(
    const controller_interface::InterfaceConfiguration & cfg);

  CONTROLLER_MANAGER_PUBLIC
  void set_state_interface_configuration(const controller_interface::InterfaceConfiguration & cfg);

  CONTROLLER_MANAGER_PUBLIC
  void set_reference_interface_names(const std::vector<std::string> & reference_interface_names);

  size_t internal_counter;
  controller_interface::InterfaceConfiguration cmd_iface_cfg_;
  controller_interface::InterfaceConfiguration state_iface_cfg_;
  std::vector<std::string> reference_interface_names_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;
};

}  // namespace test_chainable_controller

#endif  // TEST_CHAINABLE_CONTROLLER__TEST_CHAINABLE_CONTROLLER_HPP_
