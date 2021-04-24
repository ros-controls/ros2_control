// Copyright 2020 ros2_control Development Team
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
#include "hardware_interface/visibility_control.h"

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

  /// Configuration of the sensor from data parsed from the robot's URDF.
  /**
   * \param[in] sensor_info structure with data from URDF.
   * \return return_type::OK if required data are provided and can be parsed,
   * return_type::ERROR otherwise.
   */
  virtual return_type configure(const HardwareInfo & sensor_info) = 0;

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

  /// Start exchange data with the hardware.
  /**
   * \return return_type:OK if everything worked as expected, return_type::ERROR otherwise.
   */
  virtual return_type start() = 0;

  /// Stop exchange data with the hardware.
  /**
   * \return return_type:OK if everything worked as expected, return_type::ERROR otherwise.
   */
  virtual return_type stop() = 0;

  /// Get name of the sensor hardware.
  /**
   * \return name.
   */
  virtual std::string get_name() const = 0;

  /// Get current state of the sensor hardware.
  /**
   * \return current status.
   */
  virtual status get_status() const = 0;

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
