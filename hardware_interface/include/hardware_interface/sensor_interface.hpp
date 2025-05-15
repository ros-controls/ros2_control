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
class SensorInterface : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
{
public:
  SensorInterface()
  : lifecycle_state_(
      rclcpp_lifecycle::State(
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, lifecycle_state_names::UNKNOWN)),
    sensor_logger_(rclcpp::get_logger("sensor_interface"))
  {
  }

  /// SensorInterface copy constructor is actively deleted.
  /**
   * Hardware interfaces are having a unique ownership and thus can't be copied in order to avoid
   * failed or simultaneous access to hardware.
   */
  SensorInterface(const SensorInterface & other) = delete;

  SensorInterface(SensorInterface && other) = delete;

  virtual ~SensorInterface() = default;

  /// Initialization of the hardware interface from data parsed from the robot's URDF and also the
  /// clock and logger interfaces.
  /**
   * \param[in] hardware_info structure with data from URDF.
   * \param[in] logger Logger for the hardware component.
   * \param[in] clock_interface pointer to the clock interface.
   * \returns CallbackReturn::SUCCESS if required data are provided and can be parsed.
   * \returns CallbackReturn::ERROR if any error happens or data are missing.
   */
  [[deprecated("Use init(HardwareInfo, rclcpp::Logger, rclcpp::Clock::SharedPtr) instead.")]]
  CallbackReturn init(
    const HardwareInfo & hardware_info, rclcpp::Logger logger,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface)
  {
    return this->init(hardware_info, logger, clock_interface->get_clock());
  }

  /// Initialization of the hardware interface from data parsed from the robot's URDF and also the
  /// clock and logger interfaces.
  /**
   * \param[in] hardware_info structure with data from URDF.
   * \param[in] clock pointer to the resource manager clock.
   * \param[in] logger Logger for the hardware component.
   * \returns CallbackReturn::SUCCESS if required data are provided and can be parsed.
   * \returns CallbackReturn::ERROR if any error happens or data are missing.
   */
  CallbackReturn init(
    const HardwareInfo & hardware_info, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
  {
    sensor_clock_ = clock;
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

  /// Initialization of the hardware interface from data parsed from the robot's URDF.
  /**
   * \param[in] hardware_info structure with data from URDF.
   * \returns CallbackReturn::SUCCESS if required data are provided and can be parsed.
   * \returns CallbackReturn::ERROR if any error happens or data are missing.
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
   * Old way of exporting the StateInterfaces. If a empty vector is returned then
   * the on_export_state_interfaces() method is called. If a vector with StateInterfaces is returned
   * then the exporting of the StateInterfaces is only done with this function and the ownership is
   * transferred to the resource manager. The set_command(...), get_command(...), ..., can then not
   * be used.
   *
   * Note the ownership over the state interfaces is transferred to the caller.
   *
   * \return vector of state interfaces
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

  /**
   * Override this method to export custom StateInterfaces which are not defined in the URDF file.
   * Those interfaces will be added to the unlisted_state_interfaces_ map.
   *
   * \return vector of descriptions to the unlisted StateInterfaces
   */
  virtual std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_state_interface_descriptions()
  {
    // return empty vector by default.
    return {};
  }

  /**
   * Default implementation for exporting the StateInterfaces. The StateInterfaces are created
   * according to the InterfaceDescription. The memory accessed by the controllers and hardware is
   * assigned here and resides in the sensor_interface.
   *
   * \return vector of shared pointers to the created and stored StateInterfaces
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

  /// Triggers the read method synchronously or asynchronously depending on the HardwareInfo
  /**
   * The data readings from the physical hardware has to be updated
   * and reflected accordingly in the exported state interfaces.
   * That is, the data pointed by the interfaces shall be updated.
   *
   * \param[in] time The time at the start of this control loop iteration
   * \param[in] period The measured time taken by the last control loop iteration
   * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
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

  /// Read the current state values from the actuator.
  /**
   * The data readings from the physical hardware has to be updated
   * and reflected accordingly in the exported state interfaces.
   * That is, the data pointed by the interfaces shall be updated.
   *
   * \param[in] time The time at the start of this control loop iteration
   * \param[in] period The measured time taken by the last control loop iteration
   * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
   */
  virtual return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

  /// Get name of the actuator hardware.
  /**
   * \return name.
   */
  const std::string & get_name() const { return info_.name; }

  /// Get name of the actuator hardware group to which it belongs to.
  /**
   * \return group name.
   */
  const std::string & get_group_name() const { return info_.group; }

  /// Get life-cycle state of the actuator hardware.
  /**
   * \return state.
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

  template <typename T>
  void set_state(const std::string & interface_name, const T & value)
  {
    auto it = sensor_states_map_.find(interface_name);
    if (it == sensor_states_map_.end())
    {
      throw std::runtime_error(
        "State interface not found: " + interface_name +
        " in sensor hardware component: " + info_.name + ". This should not happen.");
    }
    auto & handle = it->second;
    std::unique_lock<std::shared_mutex> lock(handle->get_mutex());
    std::ignore = handle->set_value(lock, value);
  }

  template <typename T = double>
  T get_state(const std::string & interface_name) const
  {
    auto it = sensor_states_map_.find(interface_name);
    if (it == sensor_states_map_.end())
    {
      throw std::runtime_error(
        "State interface not found: " + interface_name +
        " in sensor hardware component: " + info_.name + ". This should not happen.");
    }
    auto & handle = it->second;
    std::shared_lock<std::shared_mutex> lock(handle->get_mutex());
    const auto opt_value = handle->get_optional<T>(lock);
    if (!opt_value)
    {
      throw std::runtime_error(
        "Failed to get state value from interface: " + interface_name +
        ". This should not happen.");
    }
    return opt_value.value();
  }

  /// Get the logger of the SensorInterface.
  /**
   * \return logger of the SensorInterface.
   */
  rclcpp::Logger get_logger() const { return sensor_logger_; }

  /// Get the clock of the SensorInterface.
  /**
   * \return clock of the SensorInterface.
   */
  rclcpp::Clock::SharedPtr get_clock() const { return sensor_clock_; }

  /// Get the hardware info of the SensorInterface.
  /**
   * \return hardware info of the SensorInterface.
   */
  const HardwareInfo & get_hardware_info() const { return info_; }

  /// Enable or disable introspection of the sensor hardware.
  /**
   * \param[in] enable Enable introspection if true, disable otherwise.
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
  rclcpp::Clock::SharedPtr sensor_clock_;
  rclcpp::Logger sensor_logger_;
  // interface names to Handle accessed through getters/setters
  std::unordered_map<std::string, StateInterface::SharedPtr> sensor_states_map_;

protected:
  pal_statistics::RegistrationsRAII stats_registrations_;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__SENSOR_INTERFACE_HPP_
