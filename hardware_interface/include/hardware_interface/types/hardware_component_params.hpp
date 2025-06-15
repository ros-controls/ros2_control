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

#ifndef HARDWARE_INTERFACE__TYPES__HARDWARE_COMPONENT_PARAMS_HPP_
#define HARDWARE_INTERFACE__TYPES__HARDWARE_COMPONENT_PARAMS_HPP_

#include <memory>
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hardware_interface
{

/**
 * @brief Parameters required for the initialization of a specific hardware component plugin
 * (e.g., for its on_init method or the init method of its Impl class).
 * This struct is typically populated by the ResourceManager for each component.
 */
struct HardwareComponentParams
{
  /**
   * @brief Constant reference to the HardwareInfo struct for this specific component,
   * parsed from the URDF. The HardwareInfo object's lifetime must be guaranteed
   * by the caller (e.g., ResourceManager) for the duration this struct is used.
   */
  const hardware_interface::HardwareInfo & hardware_info;

  /**
   * @brief A logger instance specifically for this hardware component
   * (e.g., a child logger of the ResourceManager's logger).
   */
  rclcpp::Logger logger;

  /**
   * @brief Shared pointer to the rclcpp::Clock to be used by this hardware component.
   * Typically, this is the same clock used by the ResourceManager/ControllerManager.
   */
  rclcpp::Clock::SharedPtr clock;

  /**
   * @brief Weak pointer to the rclcpp::Executor instance. Hardware components
   * can use this (after locking) to add their own internal ROS 2 nodes
   * to the ControllerManager's executor.
   */
  rclcpp::Executor::WeakPtr executor;

  HardwareComponentParams(
    const hardware_interface::HardwareInfo & hardware_info_param, rclcpp::Logger logger_param,
    rclcpp::Clock::SharedPtr clock_param, rclcpp::Executor::WeakPtr executor_param)
  : hardware_info(hardware_info_param),
    logger(logger_param),
    clock(clock_param),
    executor(executor_param)
  {
  }
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__TYPES__HARDWARE_COMPONENT_PARAMS_HPP_
