// Copyright 2021 ros2_control Development Team
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

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace hardware_interface
{
/**
  * Virtual Class to implement when integrating a stand-alone sensor into ros2_control.
  * The typical examples are Force-Torque Sensor (FTS), Interial Measurement Unit (IMU).
  */
class SensorInterface
{
public:
  SensorInterface() = default;

  virtual ~SensorInterface() = default;

  /// Initialization of the sensor from data parsed from the robot's URDF.
  /**
   * \param[in] sensor_info structure with data from URDF.
   * \returns CallbackReturn::SUCCESS if required data are provided and can be parsed.
   * \returns CallbackReturn::ERROR if any error happens or data are missing.
   */
  virtual CallbackReturn on_init(const HardwareInfo & sensor_info) = 0;

  /// Configuration of the hardware and start communication with it.
  /**
   * Configuration of the hardware. It start communication with hardware and configures it.
   * After this method finishes states can be read, and non-movement hardware interfaces commanded.
   *
   * Hardware interfaces for movement (will NOT be available) are:
   * HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_ACCELERATION, and HW_IF_EFFORT.
   *
   * \returns LifecycleNodeInterface::CallbackReturn::SUCCESS if configuration is successful.
   * \returns LifecycleNodeInterface::CallbackReturn::FAILURE if configuration fails and can be
   * tried again.
   * \returns LifecycleNodeInterface::CallbackReturn::ERROR critical error has happened that
   * should be managed in "on_error" method.
   */
  virtual CallbackReturn on_configure() { return CallbackReturn::SUCCESS; }

  /// Cleanup hardware configuration and stop communication with it.
  /**
   * Cleanup of the hardware. Communication with hardware is stopped.
   * Interfaces are not available anymore.
   *
   * \returns LifecycleNodeInterface::CallbackReturn::SUCCESS if everything worked as expected.
   * \returns LifecycleNodeInterface::CallbackReturn::FAILURE if cleanup fails and can be
   * tried again.
   * \returns LifecycleNodeInterface::CallbackReturn::ERROR critical error has happened that
   * should be managed in "on_error" method.
   */
  virtual CallbackReturn on_cleanup() { return CallbackReturn::SUCCESS; }

  /// Shutdown hardware interface and prepare plugin for destruction.
  /**
   * Shutdown hardware interface.
   * Cleanup memory allocations and prepare plugin for clean unloading/destruction.
   *
   * \param[in] previous_state state from which the method is called.
   * \returns LifecycleNodeInterface::CallbackReturn::SUCCESS if everything worked as expected.
   * \returns LifecycleNodeInterface::CallbackReturn::ERROR critical error has happened that
   * should be managed in "on_error" method.
   */
  virtual CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return CallbackReturn::SUCCESS;
  }

  /// Activate hardware power and enable its movement.
  /**
   * Activate power circuits of the hardware and prepare everything for commanding movements, e.g.,
   * disable brakes.
   * Command interfaces for movement has to be available after this and movement commands have to
   * be accepted.
   *
   * Hardware interfaces for movement are:
   * HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_ACCELERATION, and HW_IF_EFFORT.
   *
   * \returns LifecycleNodeInterface::CallbackReturn::SUCCESS if everything worked as expected.
   * \returns LifecycleNodeInterface::CallbackReturn::FAILURE if activation fails and can be
   * tried again.
   * \returns LifecycleNodeInterface::CallbackReturn::ERROR critical error has happened that
   * should be managed in "on_error" method.
   */
  virtual CallbackReturn on_activate() = 0;

  /// Deactivate hardware power and disable its movement.
  /**
   * Deactivate power circuits of the hardware and disable possibility to move, e.g., activate
   * breaks.
   * Command interface for movement are disabled and movement commands are ignored
   *
   * Hardware interfaces for movement are:
   * HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_ACCELERATION, and HW_IF_EFFORT.
   *
   * \returns LifecycleNodeInterface::CallbackReturn::SUCCESS if everything worked as expected.
   * \returns LifecycleNodeInterface::CallbackReturn::FAILURE if deactivation fails and can be
   * tried again.
   * \returns LifecycleNodeInterface::CallbackReturn::ERROR critical error has happened that
   * should be managed in "on_error" method.
   */
  virtual CallbackReturn on_deactivate() = 0;

  /// Deal with critical errors that happen on state changes.
  /**
   * Deal with critical errors that happen on state changes.
   * The method is called from any state.
   *
   * \param[in] previous_state state from which the method is called.
   * \returns LifecycleNodeInterface::CallbackReturn::SUCCESS if everything worked as expected.
   * \returns LifecycleNodeInterface::CallbackReturn::FAILURE if any error happens.
   */
  virtual CallbackReturn on_error(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return CallbackReturn::SUCCESS;
  }

  /// Exports all state interfaces for this sensor.
  /**
   * The state interfaces have to be created and transferred according
   * to the sensor info passed in for the configuration.
   *
   * Note the ownership over the state interfaces is transferred to the caller.
   *
   * \return std::vector<StateInterface> vector of state interfaces
   */
  virtual std::vector<StateInterface> export_state_interfaces() = 0;

  /// Get name of the sensor hardware.
  /**
   * \return name.
   */
  virtual std::string get_name() const = 0;

  /// Read the current state values from the sensor.
  /**
   * The data readings from the physical hardware has to be updated
   * and reflected accordingly in the exported state interfaces.
   * That is, the data pointed by the interfaces shall be updated.
   *
   * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
   */
  virtual return_type read() = 0;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__SENSOR_INTERFACE_HPP_
