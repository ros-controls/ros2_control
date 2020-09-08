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

#ifndef HARDWARE_INTERFACE__SENSOR_HARDWARE_INTERFACE_HPP_
#define HARDWARE_INTERFACE__SENSOR_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <vector>

#include "hardware_interface/components/sensor.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{

/**
  * \brief Virtual Class to implement when integrating a stand-alone sensor into ros2_control.
  * The typical examples are Force-Torque Sensor (FTS), Interial Measurement Unit (IMU).
  */
class SensorHardwareInterface
{
public:
  HARDWARE_INTERFACE_PUBLIC
  SensorHardwareInterface() = default;

  HARDWARE_INTERFACE_PUBLIC
  virtual
  ~SensorHardwareInterface() = default;

  /**
   * \brief Configuration of the sensor from data parsed from the robot's URDF.
   *
   * \param sensor_info structure with data from URDF.
   * \return return_type::OK if required data are provided and can be parsed,
   * return_type::ERROR otherwise.
   */
  HARDWARE_INTERFACE_PUBLIC
  virtual
  return_type configure(const HardwareInfo & sensor_info) = 0;

  /**
   * \brief Start exchange data with the hardware.
   *
   * \return return_type:OK if everything worked as expected, return_type::ERROR otherwise.
   */
  HARDWARE_INTERFACE_PUBLIC
  virtual
  return_type start() = 0;

  /**
   * \brief Stop exchange data with the hardware.
   *
   * \return return_type:OK if everything worked as expected, return_type::ERROR otherwise.
   */
  HARDWARE_INTERFACE_PUBLIC
  virtual
  return_type stop() = 0;

  /**
   * \brief Get current state of the system hardware.
   *
   * \return hardware_interface_status current status.
   */
  HARDWARE_INTERFACE_PUBLIC
  virtual
  hardware_interface_status get_status() const = 0;

  /**
   * \brief Read data from the hardware into sensors using "set_state" function in the Sensor class.
   * This function is always called by the resource manager.
   *
   * \param sensors list of sensors where data from the hardware are stored.
   * \return return_type:OK if everything worked as expected, return_type::ERROR otherwise.
   */
  HARDWARE_INTERFACE_PUBLIC
  virtual
  return_type read_sensors(const std::vector<std::shared_ptr<components::Sensor>> & sensors) const =
  0;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__SENSOR_HARDWARE_INTERFACE_HPP_
