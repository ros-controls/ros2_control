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

#ifndef HARDWARE_INTERFACE__SYSTEM_HARDWARE_INTERFACE_HPP_
#define HARDWARE_INTERFACE__SYSTEM_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <vector>

#include "hardware_interface/components/joint.hpp"
#include "hardware_interface/components/sensor.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{

/**
* \brief Virtual Class to implement when integrating a complex system into ros2_control.
* The common examples for these types of hardware are multi-joint systems with or without sensors
* such as industrial or humanoid robots.
*/
class SystemHardwareInterface
{
public:
  HARDWARE_INTERFACE_PUBLIC
  SystemHardwareInterface() = default;

  HARDWARE_INTERFACE_PUBLIC
  virtual
  ~SystemHardwareInterface() = default;

  /**
   * \brief Configuration of the system from data parsed from the robot's URDF.
   *
   * \param system_info structure with data from URDF.
   * \return return_type::OK if required data are provided and can be parsed,
   * return_type::ERROR otherwise.
   */
  HARDWARE_INTERFACE_PUBLIC
  virtual
  return_type configure(const HardwareInfo & system_info) = 0;

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
   * This is used only if the system hardware has attached sensors.
   * The function call from the resource manager is therefore optional.
   *
   * \param sensors list of sensors where data from the hardware are stored.
   * \return return_type:OK if everything worked as expected, return_type::ERROR otherwise.
   */
  HARDWARE_INTERFACE_PUBLIC
  virtual
  return_type read_sensors(std::vector<std::shared_ptr<components::Sensor>> & sensors) const = 0;

  /**
   * \brief Read data fromt the hardware into joints using "set_state" function of the Joint class.
   * This function is always called by the resource manager.
   *
   * \param joints list of joints where data from the hardware are stored.
   * \return return_type:OK if everything worked as expected, return_type::ERROR otherwise.
   */
  HARDWARE_INTERFACE_PUBLIC
  virtual
  return_type read_joints(std::vector<std::shared_ptr<components::Joint>> & joints) const = 0;

  /**
   * \brief Write data from the joints to the hardware using "get_command" function of the Joint class.
   * This function is always called by the resource manager.
   *
   * \param joints list of joints from which data are written to the hardware.
   * \return return_type:OK if everything worked as expected, return_type::ERROR otherwise.
   */
  HARDWARE_INTERFACE_PUBLIC
  return_type
  virtual
  write_joints(const std::vector<std::shared_ptr<components::Joint>> & joints) = 0;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__SYSTEM_HARDWARE_INTERFACE_HPP_
