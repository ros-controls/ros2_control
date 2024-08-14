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

#ifndef TEST_CONTROLLER__TEST_CONTROLLER_HPP_
#define TEST_CONTROLLER__TEST_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "controller_manager/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace test_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// indicating the node name under which the controller node
// is being loaded.
constexpr char TEST_CONTROLLER_NAME[] = "test_controller_name";
constexpr char TEST_CONTROLLER2_NAME[] = "test_controller2_name";
// corresponds to the name listed within the pluginlib xml
constexpr char TEST_CONTROLLER_CLASS_NAME[] = "controller_manager/test_controller";
class TestController : public controller_interface::ControllerInterface
{
public:
  CONTROLLER_MANAGER_PUBLIC
  TestController();

  CONTROLLER_MANAGER_PUBLIC
  virtual ~TestController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CONTROLLER_MANAGER_PUBLIC
  CallbackReturn on_init() override;

  CONTROLLER_MANAGER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_MANAGER_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_MANAGER_PUBLIC
  void set_command_interface_configuration(
    const controller_interface::InterfaceConfiguration & cfg);

  CONTROLLER_MANAGER_PUBLIC
  void set_state_interface_configuration(const controller_interface::InterfaceConfiguration & cfg);

  CONTROLLER_MANAGER_PUBLIC
  std::vector<double> get_state_interface_data() const;

  const std::string & getRobotDescription() const;

  unsigned int internal_counter = 0;
  bool simulate_cleanup_failure = false;
  // Variable where we store when cleanup was called, pointer because the controller
  // is usually destroyed after cleanup
  size_t * cleanup_calls = nullptr;
  controller_interface::InterfaceConfiguration cmd_iface_cfg_;
  controller_interface::InterfaceConfiguration state_iface_cfg_;

  std::vector<double> external_commands_for_testing_;
  // enables external setting of values to command interfaces - used for simulation of hardware
  // errors
  double set_first_command_interface_value_to;
  rclcpp::Duration update_period_ = rclcpp::Duration::from_seconds(0.);
};

}  // namespace test_controller

#endif  // TEST_CONTROLLER__TEST_CONTROLLER_HPP_
