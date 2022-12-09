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

#ifndef HARDWARE_INTERFACE__HARDWARE_INTERFACE_HPP_
#define HARDWARE_INTERFACE__HARDWARE_INTERFACE_HPP_

#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/loaned_hw_command_interface.hpp"
#include "hardware_interface/loaned_hw_state_interface.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{

class HardwareInterface
{
public:
  virtual ~HardwareInterface() {}
  // Could be possible to provide default implementation and parse the hardware_info here. Only if
  // user wants to export subset or special cases he needs to override.

  /**
   * @brief Only export information describing the interfaces. Handle construction
   * and management internally. No need for the user to initialize and manage shared memory.
   *
   * @return std::vector<InterfaceConfiguration> A vector containing all the information
   *  needed to create the interfaces exported by the hardware.
   */
  HARDWARE_INTERFACE_PUBLIC
  virtual std::vector<InterfaceDescription> export_state_interfaces_descriptions() = 0;

  /**
   * @brief Only export information describing the interfaces. Handle construction
   * and management internally. No need for the user to initialize and manage shared memory.
   *
   * @return std::vector<InterfaceConfiguration> A vector containing all the information
   *  needed to create the interfaces exported by the hardware.
   */
  HARDWARE_INTERFACE_PUBLIC
  virtual std::vector<InterfaceDescription> export_command_interfaces_descriptions() = 0;

  // Could be possible to provide default implementation and store the loans in the interface
  // itself. User could then set/get states via function calls. Ordering could be made explicit.

  /**
   * @brief Import the LoanedHwStateInterface to the before exported StateInterface InterfaceDescription.
   *
   */
  HARDWARE_INTERFACE_PUBLIC
  virtual void import_loaned_hw_state_interfaces(
    std::vector<LoanedHwStateInterface> loaned_hw_state_interfaces) = 0;

  /**
   * @brief Import the LoanedHwCommandInterface to the before exported CommandInterface InterfaceDescription.
   *
   */
  HARDWARE_INTERFACE_PUBLIC
  virtual void import_loaned_hw_command_interfaces(
    std::vector<LoanedHwCommandInterface> loaned_hw_command_interfaces) = 0;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__HARDWARE_INTERFACE_HPP_
