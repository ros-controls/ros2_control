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

#ifndef MOCK_COMPONENTS__GENERIC_SYSTEM_HPP_
#define MOCK_COMPONENTS__GENERIC_SYSTEM_HPP_

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/visibility_control.h"

using hardware_interface::return_type;

namespace mock_components
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

static constexpr size_t POSITION_INTERFACE_INDEX = 0;
static constexpr size_t VELOCITY_INTERFACE_INDEX = 1;
static constexpr size_t ACCELERATION_INTERFACE_INDEX = 2;

class HARDWARE_INTERFACE_PUBLIC GenericSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return return_type::OK;
  }

protected:
  /// Use standard interfaces for joints because they are relevant for dynamic behavior
  /**
   * By splitting the standard interfaces from other type, the users are able to inherit this
   * class and simply create small "simulation" with desired dynamic behavior.
   * The advantage over using Gazebo is that enables "quick & dirty" tests of robot's URDF and
   * controllers.
   */
  const std::vector<std::string> standard_interfaces_ = {
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_ACCELERATION, hardware_interface::HW_IF_EFFORT};

  /// The size of this vector is (standard_interfaces_.size() x nr_joints)
  std::vector<std::vector<double>> joint_commands_;
  std::vector<std::vector<double>> joint_states_;

  std::vector<std::string> other_interfaces_;
  /// The size of this vector is (other_interfaces_.size() x nr_joints)
  std::vector<std::vector<double>> other_commands_;
  std::vector<std::vector<double>> other_states_;

  std::vector<std::string> sensor_interfaces_;
  /// The size of this vector is (sensor_interfaces_.size() x nr_joints)
  std::vector<std::vector<double>> sensor_mock_commands_;
  std::vector<std::vector<double>> sensor_states_;

  std::vector<std::string> gpio_interfaces_;
  /// The size of this vector is (gpio_interfaces_.size() x nr_joints)
  std::vector<std::vector<double>> gpio_mock_commands_;
  std::vector<std::vector<double>> gpio_commands_;
  std::vector<std::vector<double>> gpio_states_;

private:
  template <typename HandleType>
  bool get_interface(
    const std::string & name, const std::vector<std::string> & interface_list,
    const std::string & interface_name, const size_t vector_index,
    std::vector<std::vector<double>> & values, std::vector<HandleType> & interfaces);

  void initialize_storage_vectors(
    std::vector<std::vector<double>> & commands, std::vector<std::vector<double>> & states,
    const std::vector<std::string> & interfaces,
    const std::vector<hardware_interface::ComponentInfo> & component_infos);

  template <typename InterfaceType>
  bool populate_interfaces(
    const std::vector<hardware_interface::ComponentInfo> & components,
    std::vector<std::string> & interfaces, std::vector<std::vector<double>> & storage,
    std::vector<InterfaceType> & target_interfaces, bool using_state_interfaces);

  bool use_mock_gpio_command_interfaces_;
  bool use_mock_sensor_command_interfaces_;

  double position_state_following_offset_;
  std::string custom_interface_with_following_offset_;
  size_t index_custom_interface_with_following_offset_;

  bool calculate_dynamics_;
  std::vector<size_t> joint_control_mode_;

  bool command_propagation_disabled_;
};

typedef GenericSystem GenericRobot;

}  // namespace mock_components

#endif  // MOCK_COMPONENTS__GENERIC_SYSTEM_HPP_
