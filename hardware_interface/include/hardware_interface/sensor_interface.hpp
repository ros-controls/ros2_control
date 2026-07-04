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

#ifndef HARDWARE_INTERFACE__SENSOR_INTERFACE_HPP_
#define HARDWARE_INTERFACE__SENSOR_INTERFACE_HPP_

#include <vector>

#include "hardware_interface/hardware_component_interface.hpp"

namespace hardware_interface
{
/// Virtual Class to implement when integrating a stand-alone sensor into ros2_control.
/**
 * The typical examples are Force-Torque Sensor (FTS), Interial Measurement Unit (IMU).
 *
 * Methods return values have type
 * rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn with the following
 * meaning:
 *
 * \returns CallbackReturn::SUCCESS method execution was successful.
 * \returns CallbackReturn::FAILURE method execution has failed and and can be called again.
 * \returns CallbackReturn::ERROR critical error has happened that should be managed in
 * "on_error" method.
 *
 * The hardware ends after each method in a state with the following meaning:
 *
 * UNCONFIGURED (on_init, on_cleanup):
 *   Hardware is initialized but communication is not started and therefore no interface is
 * available.
 *
 * INACTIVE (on_configure, on_deactivate):
 *   Communication with the hardware is started and it is configured.
 *   States can be read.
 *
 * FINALIZED (on_shutdown):
 *   Hardware interface is ready for unloading/destruction.
 *   Allocated memory is cleaned up.
 *
 * ACTIVE (on_activate):
 *   States can be read.
 */
class SensorInterface : public HardwareComponentInterface
{
public:
  std::vector<CommandInterface::SharedPtr> on_export_command_interfaces() override { return {}; }

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) final
  {
    return return_type::OK;
  }
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__SENSOR_INTERFACE_HPP_
