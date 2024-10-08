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
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace hardware_interface
{
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
 *   States can be read and non-movement hardware interfaces commanded.
 *   Hardware interfaces for movement will NOT be available.
 *   Those interfaces are: HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_ACCELERATION, and HW_IF_EFFORT.
 *
 * FINALIZED (on_shutdown):
 *   Hardware interface is ready for unloading/destruction.
 *   Allocated memory is cleaned up.
 *
 * ACTIVE (on_activate):
 *   Power circuits of hardware are active and hardware can be moved, e.g., brakes are disabled.
 *   Command interfaces for movement are available and have to be accepted.
 *   Those interfaces are: HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_ACCELERATION, and HW_IF_EFFORT.
 */

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SensorInterface : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
{
public:
  SensorInterface()
  : lifecycle_state_(rclcpp_lifecycle::State(
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

  SensorInterface(SensorInterface && other) = default;

  virtual ~SensorInterface() = default;

  /// Initialization of the hardware interface from data parsed from the robot's URDF and also the
  /// clock and logger interfaces.
  /**
   * \param[in] hardware_info structure with data from URDF.
   * \param[in] clock_interface pointer to the clock interface.
   * \param[in] logger_interface pointer to the logger interface.
   * \returns CallbackReturn::SUCCESS if required data are provided and can be parsed.
   * \returns CallbackReturn::ERROR if any error happens or data are missing.
   */
  CallbackReturn init(
    const HardwareInfo & hardware_info, rclcpp::Logger logger,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface)
  {
    clock_interface_ = clock_interface;
    sensor_logger_ = logger.get_child("hardware_component.sensor." + hardware_info.name);
    info_ = hardware_info;
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
    import_state_interface_descriptions(info_);
    return CallbackReturn::SUCCESS;
  };

  /**
   * Import the InterfaceDescription for the StateInterfaces from the HardwareInfo.
   * Separate them into the possible types: Sensor and store them.
   */
  virtual void import_state_interface_descriptions(const HardwareInfo & hardware_info)
  {
    auto sensor_state_interface_descriptions =
      parse_state_interface_descriptions(hardware_info.sensors);
    for (const auto & description : sensor_state_interface_descriptions)
    {
      sensor_state_interfaces_.insert(std::make_pair(description.get_name(), description));
    }
  }

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
      unlisted_interface_descriptions.size() + sensor_state_interfaces_.size());

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

    return state_interfaces;
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
  virtual std::string get_name() const { return info_.name; }

  /// Get name of the actuator hardware group to which it belongs to.
  /**
   * \return group name.
   */
  virtual std::string get_group_name() const { return info_.group; }

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

  void set_state(const std::string & interface_name, const double & value)
  {
    sensor_states_map_.at(interface_name)->set_value(value);
  }

  double get_state(const std::string & interface_name) const
  {
    return sensor_states_map_.at(interface_name)->get_value();
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
  rclcpp::Clock::SharedPtr get_clock() const { return clock_interface_->get_clock(); }

  /// Get the hardware info of the SensorInterface.
  /**
   * \return hardware info of the SensorInterface.
   */
  const HardwareInfo & get_hardware_info() const { return info_; }

protected:
  HardwareInfo info_;
  // interface names to InterfaceDescription
  std::unordered_map<std::string, InterfaceDescription> sensor_state_interfaces_;
  std::unordered_map<std::string, InterfaceDescription> unlisted_state_interfaces_;

  // Exported Command- and StateInterfaces in order they are listed in the hardware description.
  std::vector<StateInterface::SharedPtr> sensor_states_;
  std::vector<StateInterface::SharedPtr> unlisted_states_;

  rclcpp_lifecycle::State lifecycle_state_;

private:
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface_;
  rclcpp::Logger sensor_logger_;
  // interface names to Handle accessed through getters/setters
  std::unordered_map<std::string, StateInterface::SharedPtr> sensor_states_map_;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__SENSOR_INTERFACE_HPP_
