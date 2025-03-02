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

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/introspection.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/async_function_handler.hpp"

namespace hardware_interface
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// Virtual class for integrating a stand-alone sensor into ros2_control.
/**
 * @brief Base class for standalone sensor integration in ros2_control.
 *
 * Typical examples include Force-Torque Sensors (FTS) and Inertial Measurement Units (IMU).
 *
 * Methods return values of type 
 * `rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn` with the following meanings:
 *
 * @returns `CallbackReturn::SUCCESS` - Method executed successfully.
 * @returns `CallbackReturn::FAILURE` - Execution failed but can be retried.
 * @returns `CallbackReturn::ERROR` - Critical error requiring handling in `on_error`.
 *
 * Hardware states after each method execution:
 *
 * - **UNCONFIGURED** (on_init, on_cleanup):  
 *   Hardware is initialized but not communicating; no interfaces are available.
 * - **INACTIVE** (on_configure, on_deactivate):  
 *   Communication is established, hardware is configured, and states can be read.
 * - **FINALIZED** (on_shutdown):  
 *   Hardware interface is ready for unloading; allocated memory is cleaned up.
 * - **ACTIVE** (on_activate):  
 *   States can be read.
 */

class SensorInterface : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
{
public:
  SensorInterface()
  : lifecycle_state_(rclcpp_lifecycle::State(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, lifecycle_state_names::UNKNOWN)),
    sensor_logger_(rclcpp::get_logger("sensor_interface"))
  {
  }

 /// Deleted copy constructor for SensorInterface.
/**
 * @brief Prevents copying of SensorInterface to ensure unique ownership.
 *
 * Hardware interfaces require unique ownership to prevent failed or simultaneous 
 * access to hardware resources.
 */

  SensorInterface(const SensorInterface & other) = delete;

  SensorInterface(SensorInterface && other) = delete;

  virtual ~SensorInterface() = default;

  /// Initializes the hardware interface using data from the robot's URDF, clock, and logger.
/**
 * @brief Sets up the hardware interface with parsed URDF data and system interfaces.
 *
 * @param[in] hardware_info Structure containing data from the URDF.
 * @param[in] clock_interface Pointer to the clock interface.
 * @param[in] logger_interface Pointer to the logger interface.
 * @return `CallbackReturn::SUCCESS` if initialization succeeds.
 * @return `CallbackReturn::ERROR` if data is missing or an error occurs.
 */

  CallbackReturn init(
    const HardwareInfo & hardware_info, rclcpp::Logger logger,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface)
  {
    clock_interface_ = clock_interface;
    sensor_logger_ = logger.get_child("hardware_component.sensor." + hardware_info.name);
    info_ = hardware_info;
    if (info_.is_async)
    {
      RCLCPP_INFO_STREAM(
        get_logger(), "Starting async handler with scheduler priority: " << info_.thread_priority);
      read_async_handler_ = std::make_unique<realtime_tools::AsyncFunctionHandler<return_type>>();
      read_async_handler_->init(
        std::bind(&SensorInterface::read, this, std::placeholders::_1, std::placeholders::_2),
        info_.thread_priority);
      read_async_handler_->start_thread();
    }
    return on_init(hardware_info);
  };

  /// Initializes the hardware interface from parsed robot URDF data.
/**
 * @brief Sets up the hardware interface using the provided URDF data.
 *
 * @param[in] hardware_info Structure containing data from the URDF.
 * @return `CallbackReturn::SUCCESS` if data is successfully parsed and available.
 * @return `CallbackReturn::ERROR` if data is missing or an error occurs.
 */

  virtual CallbackReturn on_init(const HardwareInfo & hardware_info)
  {
    info_ = hardware_info;
    parse_state_interface_descriptions(info_.joints, joint_state_interfaces_);
    parse_state_interface_descriptions(info_.sensors, sensor_state_interfaces_);
    return CallbackReturn::SUCCESS;
  };
/// Exports all state interfaces for this hardware interface.
/**
 * @brief Exports state interfaces, either via return value or through `on_export_state_interfaces()`.
 *
 * If an empty vector is returned, the `on_export_state_interfaces()` method is invoked.
 * If a vector with state interfaces is returned, the state interfaces are exported directly,
 * and ownership is transferred to the resource manager. Afterward, functions like `set_command(...)`, 
 * `get_command(...)`, etc., cannot be used.
 *
 * @note Ownership of the state interfaces is transferred to the caller.
 *
 * @return A vector of state interfaces.
 */

  [[deprecated(
    "Replaced by vector<StateInterface::ConstSharedPtr> on_export_state_interfaces() method. "
    "Exporting is handled "
    "by the Framework.")]] virtual std::vector<StateInterface>
  export_state_interfaces()
  {
    // return empty vector by default. For backward compatibility we try calling
    // export_state_interfaces() and only when empty vector is returned call
    // on_export_state_interfaces()
    return {};
  }

/// Override to export custom StateInterfaces not defined in the URDF.
/**
 * @brief Adds custom StateInterfaces to the unlisted_state_interfaces_ map.
 *
 * Override this method to export additional StateInterfaces that are not defined in the URDF file.
 * These interfaces will be added to the `unlisted_state_interfaces_` map.
 *
 * @return A vector of descriptions for the unlisted StateInterfaces.
 */

  virtual std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_state_interface_descriptions()
  {
    // return empty vector by default.
    return {};
  }

/// Default implementation for exporting StateInterfaces.
/**
 * @brief Creates and stores StateInterfaces based on the InterfaceDescription.
 *
 * This default implementation generates the StateInterfaces and assigns the memory used by the controllers
 * and hardware to reside in the `sensor_interface`.
 *
 * @return A vector of shared pointers to the created and stored StateInterfaces.
 */

  virtual std::vector<StateInterface::ConstSharedPtr> on_export_state_interfaces()
  {
    // import the unlisted interfaces
    std::vector<hardware_interface::InterfaceDescription> unlisted_interface_descriptions =
      export_unlisted_state_interface_descriptions();

    std::vector<StateInterface::ConstSharedPtr> state_interfaces;
    state_interfaces.reserve(
      unlisted_interface_descriptions.size() + sensor_state_interfaces_.size() +
      joint_state_interfaces_.size());

    // add InterfaceDescriptions and create StateInterfaces from the descriptions and add to maps.
    for (const auto & description : unlisted_interface_descriptions)
    {
      auto name = description.get_name();
      unlisted_state_interfaces_.insert(std::make_pair(name, description));
      auto state_interface = std::make_shared<StateInterface>(description);
      sensor_states_map_.insert(std::make_pair(name, state_interface));
      unlisted_states_.push_back(state_interface);
      state_interfaces.push_back(std::const_pointer_cast<const StateInterface>(state_interface));
    }

    for (const auto & [name, descr] : sensor_state_interfaces_)
    {
      // TODO(Manuel) check for duplicates otherwise only the first appearance of "name" is inserted
      auto state_interface = std::make_shared<StateInterface>(descr);
      sensor_states_map_.insert(std::make_pair(name, state_interface));
      sensor_states_.push_back(state_interface);
      state_interfaces.push_back(std::const_pointer_cast<const StateInterface>(state_interface));
    }

    for (const auto & [name, descr] : joint_state_interfaces_)
    {
      auto state_interface = std::make_shared<StateInterface>(descr);
      sensor_states_map_.insert(std::make_pair(name, state_interface));
      joint_states_.push_back(state_interface);
      state_interfaces.push_back(std::const_pointer_cast<const StateInterface>(state_interface));
    }

    return state_interfaces;
  }

/// Triggers the read method synchronously or asynchronously based on HardwareInfo.
/**
 * @brief Updates the state interfaces with the latest data from the physical hardware.
 *
 * The data readings from the hardware must be updated and reflected in the exported state interfaces.
 * Specifically, the data pointed to by the interfaces will be updated.
 *
 * @param[in] time The time at the start of the control loop iteration.
 * @param[in] period The duration of the last control loop iteration.
 * @return `return_type::OK` if the read was successful, `return_type::ERROR` otherwise.
 */

  HardwareComponentCycleStatus trigger_read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    HardwareComponentCycleStatus status;
    status.result = return_type::ERROR;
    if (info_.is_async)
    {
      const auto result = read_async_handler_->trigger_async_callback(time, period);
      status.successful = result.first;
      status.result = result.second;
      const auto execution_time = read_async_handler_->get_last_execution_time();
      if (execution_time.count() > 0)
      {
        status.execution_time = execution_time;
      }
      if (!status.successful)
      {
        RCLCPP_WARN(
          get_logger(),
          "Trigger read called while read async handler is still in progress for hardware "
          "interface : '%s'. Failed to trigger read cycle!",
          info_.name.c_str());
        status.result = return_type::OK;
        return status;
      }
    }
    else
    {
      const auto start_time = std::chrono::steady_clock::now();
      status.successful = true;
      status.result = read(time, period);
      status.execution_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - start_time);
    }
    return status;
  }

 /// Reads the current state values from the actuator.
/**
 * @brief Updates the state interfaces with the latest data from the actuator.
 *
 * The data readings from the physical hardware must be updated and reflected in the exported state interfaces.
 * Specifically, the data pointed to by the interfaces will be updated.
 *
 * @param[in] time The time at the start of the control loop iteration.
 * @param[in] period The duration of the last control loop iteration.
 * @return `return_type::OK` if the read was successful, `return_type::ERROR` otherwise.
 */

  virtual return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

 /// Gets the name of the actuator hardware.
/**
 * @brief Retrieves the name of the actuator hardware.
 *
 * @return The name of the actuator hardware.
 */

  const std::string & get_name() const { return info_.name; }

  /// Gets the name of the actuator hardware group.
/**
 * @brief Retrieves the name of the hardware group to which the actuator belongs.
 *
 * @return The name of the actuator hardware group.
 */

  const std::string & get_group_name() const { return info_.group; }

 /// Gets the lifecycle state of the actuator hardware.
/**
 * @brief Retrieves the lifecycle state of the actuator hardware.
 *
 * @return The lifecycle state of the actuator hardware.
 */

  const rclcpp_lifecycle::State & get_lifecycle_state() const { return lifecycle_state_; }

  /// Set life-cycle state of the actuator hardware.
  /**
   * \return state.
   */
  void set_lifecycle_state(const rclcpp_lifecycle::State & new_state)
  {
    lifecycle_state_ = new_state;
  }

  void set_state(const std::string & interface_name, const double & value)
  {
    sensor_states_map_.at(interface_name)->set_value(value);
  }

  double get_state(const std::string & interface_name) const
  {
    return sensor_states_map_.at(interface_name)->get_value();
  }

 /// Gets the logger of the SensorInterface.
/**
 * @brief Retrieves the logger instance associated with the SensorInterface.
 *
 * @return The logger of the SensorInterface.
 */

  rclcpp::Logger get_logger() const { return sensor_logger_; }

  /// Gets the clock of the SensorInterface.
/**
 * @brief Retrieves the clock instance associated with the SensorInterface.
 *
 * @return The clock of the SensorInterface.
 */

  rclcpp::Clock::SharedPtr get_clock() const { return clock_interface_->get_clock(); }

/// Gets the hardware info of the SensorInterface.
/**
 * @brief Retrieves the hardware information associated with the SensorInterface.
 *
 * @return The hardware info of the SensorInterface.
 */

  const HardwareInfo & get_hardware_info() const { return info_; }

  /// Enables or disables introspection of the sensor hardware.
/**
 * @brief Controls the introspection of the sensor hardware.
 *
 * @param[in] enable `true` to enable introspection, `false` to disable it.
 */

  void enable_introspection(bool enable)
  {
    if (enable)
    {
      stats_registrations_.enableAll();
    }
    else
    {
      stats_registrations_.disableAll();
    }
  }

protected:
  HardwareInfo info_;
  // interface names to InterfaceDescription
  std::unordered_map<std::string, InterfaceDescription> joint_state_interfaces_;
  std::unordered_map<std::string, InterfaceDescription> sensor_state_interfaces_;
  std::unordered_map<std::string, InterfaceDescription> unlisted_state_interfaces_;

  // Exported Command- and StateInterfaces in order they are listed in the hardware description.
  std::vector<StateInterface::SharedPtr> joint_states_;
  std::vector<StateInterface::SharedPtr> sensor_states_;
  std::vector<StateInterface::SharedPtr> unlisted_states_;

  rclcpp_lifecycle::State lifecycle_state_;
  std::unique_ptr<realtime_tools::AsyncFunctionHandler<return_type>> read_async_handler_;

private:
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface_;
  rclcpp::Logger sensor_logger_;
  // interface names to Handle accessed through getters/setters
  std::unordered_map<std::string, StateInterface::SharedPtr> sensor_states_map_;

protected:
  pal_statistics::RegistrationsRAII stats_registrations_;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__SENSOR_INTERFACE_HPP_
