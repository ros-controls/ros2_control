// Copyright 2020 - 2021 ros2_control Development Team
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

#ifndef HARDWARE_INTERFACE__ACTUATOR_HPP_
#define HARDWARE_INTERFACE__ACTUATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace hardware_interface
{
class ActuatorInterface;

class Actuator final
{
public:
  Actuator() = default;

  HARDWARE_INTERFACE_PUBLIC
  explicit Actuator(std::unique_ptr<ActuatorInterface> impl);

  Actuator(Actuator && other) = default;

  ~Actuator() = default;

  HARDWARE_INTERFACE_PUBLIC
  const rclcpp_lifecycle::State & initialize(const HardwareInfo & actuator_info);

  HARDWARE_INTERFACE_PUBLIC
  const rclcpp_lifecycle::State & configure();

  HARDWARE_INTERFACE_PUBLIC
  const rclcpp_lifecycle::State & cleanup();

  HARDWARE_INTERFACE_PUBLIC
  const rclcpp_lifecycle::State & shutdown();

  HARDWARE_INTERFACE_PUBLIC
  const rclcpp_lifecycle::State & activate();

  HARDWARE_INTERFACE_PUBLIC
  const rclcpp_lifecycle::State & deactivate();

  HARDWARE_INTERFACE_PUBLIC
  const rclcpp_lifecycle::State & error();

  HARDWARE_INTERFACE_PUBLIC
  std::vector<StateInterface> export_state_interfaces();

  HARDWARE_INTERFACE_PUBLIC
  std::vector<CommandInterface> export_command_interfaces();

  HARDWARE_INTERFACE_PUBLIC
  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces);

  HARDWARE_INTERFACE_PUBLIC
  return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces);

  HARDWARE_INTERFACE_PUBLIC
  std::string get_name() const;

  HARDWARE_INTERFACE_PUBLIC
  const rclcpp_lifecycle::State & get_state() const;

  HARDWARE_INTERFACE_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period);

  HARDWARE_INTERFACE_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period);

private:
  std::unique_ptr<ActuatorInterface> impl_;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__ACTUATOR_HPP_
