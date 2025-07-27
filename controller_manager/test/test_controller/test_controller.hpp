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

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "example_interfaces/srv/set_bool.hpp"
#include "rclcpp/rclcpp.hpp"
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
  TestController();

  virtual ~TestController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CallbackReturn on_init() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  void set_command_interface_configuration(
    const controller_interface::InterfaceConfiguration & cfg);

  void set_state_interface_configuration(const controller_interface::InterfaceConfiguration & cfg);

  std::vector<double> get_state_interface_data() const;

  void set_external_commands_for_testing(const std::vector<double> & commands);

  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;
  unsigned int internal_counter = 0;
  bool simulate_cleanup_failure = false;
  // Variable where we store when shutdown was called, pointer because the controller
  // is usually destroyed after shutdown
  size_t * cleanup_calls = nullptr;
  size_t * shutdown_calls = nullptr;
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
