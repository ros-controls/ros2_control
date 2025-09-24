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

#ifndef HARDWARE_INTERFACE__TYPES__RESOURCE_MANAGER_PARAMS_HPP_
#define HARDWARE_INTERFACE__TYPES__RESOURCE_MANAGER_PARAMS_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"

namespace hardware_interface
{

/**
 * @brief Parameters required for the construction and initial setup of a ResourceManager.
 * This struct is typically populated by the ControllerManager.
 */
struct ResourceManagerParams
{
  ResourceManagerParams() = default;
  /**
   * @brief The URDF string describing the robot's hardware components.
   * Can be empty if ResourceManager is constructed without an initial URDF
   * and components are loaded later or via other means.
   */
  std::string robot_description = "";

  /**
   * @brief Shared pointer to the Clock used by the ResourceManager.
   * This is typically obtained from the node via get_clock().
   */
  rclcpp::Clock::SharedPtr clock = nullptr;

  /**
   * @brief Logger instance used by the ResourceManager.
   * This is typically obtained from the node via get_logger().
   */
  rclcpp::Logger logger = rclcpp::get_logger("resource_manager");

  /**
   * @brief Shared pointer to the rclcpp::Executor instance that the
   * ResourceManager and its components (including plugins that opt-in) will use.
   * This is typically the ControllerManager's main executor.
   */
  rclcpp::Executor::SharedPtr executor = nullptr;

  /**
   * @brief Flag indicating if all hardware components found in the URDF
   * should be automatically activated after successful loading and initialization.
   */
  bool activate_all = false;

  /**
   * @brief If true, controllers are allowed to claim resources from inactive hardware components.
   * If false, controllers can only claim resources from active hardware components.
   * Moreover, when the hardware component returns DEACTIVATE on read/write cycle: If set to true,
   * the controllers using those interfaces will continue to run. If set to false, the controllers
   * using those interfaces will be deactivated.
   * @warning Allowing control with inactive hardware is not recommended for safety reasons.
   * Use with caution only if you really know what you are doing.
   * @note This parameter might be deprecated or removed in the future releases. Please use with
   * caution.
   */
  bool allow_controller_activation_with_inactive_hardware = false;

  /**
   * @brief If true, when a hardware component returns DEACTIVATE on the write cycle,
   * its name will be included in the returned HardwareReadWriteStatus.failed_hardware_names list.
   * If false, the names of such hardware components will not be included in that list.
   * This can be useful when controllers are allowed to operate with inactive hardware components.
   * @note This parameter might be deprecated or removed in future releases. Please use with
   * caution.
   */
  bool return_failed_hardware_names_on_return_deactivate_write_cycle_ = true;

  /**
   * @brief The update rate (in Hz) of the ControllerManager.
   * This can be used by ResourceManager to configure asynchronous hardware components
   * or for other timing considerations.
   */
  unsigned int update_rate = 100;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__TYPES__RESOURCE_MANAGER_PARAMS_HPP_
