// Copyright 2025 ros2_control Development Team
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

#ifndef HARDWARE_INTERFACE__TYPES__HARDWARE_COMPONENT_INTERFACE_PARAMS_HPP_
#define HARDWARE_INTERFACE__TYPES__HARDWARE_COMPONENT_INTERFACE_PARAMS_HPP_

#include <memory>
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_component_params.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hardware_interface
{

/**
 * @brief Parameters required for the initialization of a specific hardware component plugin.
 * Typically used for on_init methods of hardware interfaces, and is parsed by the user.
 * This struct is typically populated with data from HardwareComponentParams from each component.
 */
struct HardwareComponentInterfaceParams
{
  HardwareComponentInterfaceParams() = default;
  /**
   * @brief Reference to the HardwareInfo struct for this specific component,
   * parsed from the URDF. The HardwareInfo object's lifetime must be guaranteed
   * by the caller (e.g., ResourceManager) for the duration this struct is used.
   */
  hardware_interface::HardwareInfo hardware_info;

  /**
   * @brief Weak pointer to the rclcpp::Executor instance. Hardware components
   * can use this (after locking) to add their own internal ROS 2 nodes
   * to the ControllerManager's executor.
   */
  rclcpp::Executor::WeakPtr executor;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__TYPES__HARDWARE_COMPONENT_INTERFACE_PARAMS_HPP_
