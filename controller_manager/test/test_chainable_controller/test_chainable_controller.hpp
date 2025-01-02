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
#include "rclcpp/subscription.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "semantic_components/imu_sensor.hpp"
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
constexpr double CONTROLLER_DT = 0.001;
class TestChainableController : public controller_interface::ChainableControllerInterface
{
public:
  TestChainableController();

  virtual ~TestChainableController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CallbackReturn on_init() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Testing-relevant methods
  void set_command_interface_configuration(
    const controller_interface::InterfaceConfiguration & cfg);

  void set_state_interface_configuration(const controller_interface::InterfaceConfiguration & cfg);

  void set_reference_interface_names(const std::vector<std::string> & reference_interface_names);

  void set_exported_state_interface_names(const std::vector<std::string> & state_interface_names);

  void set_imu_sensor_name(const std::string & name);

  std::vector<double> get_state_interface_data() const;

  size_t internal_counter;
  controller_interface::InterfaceConfiguration cmd_iface_cfg_;
  controller_interface::InterfaceConfiguration state_iface_cfg_;
  std::vector<std::string> reference_interface_names_;
  std::vector<std::string> exported_state_interface_names_;
  std::unique_ptr<semantic_components::IMUSensor> imu_sensor_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;
};

}  // namespace test_chainable_controller

#endif  // TEST_CHAINABLE_CONTROLLER__TEST_CHAINABLE_CONTROLLER_HPP_
