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

#ifndef HARDWARE_INTERFACE__SYSTEM_INTERFACE_HPP_
#define HARDWARE_INTERFACE__SYSTEM_INTERFACE_HPP_

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
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
/// Virtual Class to implement when integrating a complex system into ros2_control.
/**
 * The common examples for these types of hardware are multi-joint systems with or without sensors
 * such as industrial or humanoid robots.
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

class SystemInterface : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
{
public:
  SystemInterface()
  : lifecycle_state_(rclcpp_lifecycle::State(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, lifecycle_state_names::UNKNOWN)),
    system_logger_(rclcpp::get_logger("system_interface"))
  {
  }

  /// SystemInterface copy constructor is actively deleted.
  /**
   * Hardware interfaces are having a unique ownership and thus can't be copied in order to avoid
   * failed or simultaneous access to hardware.
   */
  SystemInterface(const SystemInterface & other) = delete;

  SystemInterface(SystemInterface && other) = default;

  virtual ~SystemInterface() = default;

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
    system_logger_ = logger.get_child("hardware_component.system." + hardware_info.name);
    info_ = hardware_info;
    return on_init(hardware_info);
  };

  /// Initialization of the hardware interface from data parsed from the robot's URDF.
  /**
   * \param[in] hardware_info structure with data from URDF.
   * \returns CallbackReturn::SUCCESS if required data are provided and can be parsed.
   * \returns CallbackReturn::ERROR if any error happens or data are missing.
   */
  virtual CallbackReturn on_init(const HardwareInfo & /*hardware_info*/)
  {
    info_ = hardware_info;
    import_state_interface_descriptions(info_);
    import_command_interface_descriptions(info_);
    return CallbackReturn::SUCCESS;
  };

  /**
   * Import the InterfaceDescription for the StateInterfaces from the HardwareInfo.
   * Separate them into the possible types: Joint, GPIO, Sensor and store them.
   */
  void import_state_interface_descriptions(const HardwareInfo & hardware_info)
  {
    auto joint_state_interface_descriptions =
      parse_joint_state_interface_descriptions_from_hardware_info(hardware_info);
    for (const auto & description : joint_state_interface_descriptions)
    {
      joint_state_interfaces_.insert(std::make_pair(description.get_name(), description));
    }
    auto sensor_state_interface_descriptions =
      parse_sensor_state_interface_descriptions_from_hardware_info(hardware_info);
    for (const auto & description : sensor_state_interface_descriptions)
    {
      sensor_state_interfaces_.insert(std::make_pair(description.get_name(), description));
    }
    auto gpio_state_interface_descriptions =
      parse_gpio_state_interface_descriptions_from_hardware_info(hardware_info);
    for (const auto & description : gpio_state_interface_descriptions)
    {
      gpio_state_interfaces_.insert(std::make_pair(description.get_name(), description));
    }
  }

  /**
   * Import the InterfaceDescription for the CommandInterfaces from the HardwareInfo.
   * Separate them into the possible types: Joint and GPIO and store them.
   */
  void import_command_interface_descriptions(const HardwareInfo & hardware_info)
  {
    auto joint_command_interface_descriptions =
      parse_joint_command_interface_descriptions_from_hardware_info(hardware_info);
    for (const auto & description : joint_command_interface_descriptions)
    {
      joint_command_interfaces_.insert(std::make_pair(description.get_name(), description));
    }
    auto gpio_command_interface_descriptions =
      parse_gpio_command_interface_descriptions_from_hardware_info(hardware_info);
    for (const auto & description : gpio_command_interface_descriptions)
    {
      gpio_command_interfaces_.insert(std::make_pair(description.get_name(), description));
    }
  }

  /// Exports all state interfaces for this hardware interface.
  /**
   * Default implementation for exporting the StateInterfaces. The StateInterfaces are created
   * according to the InterfaceDescription. The memory accessed by the controllers and hardware is
   * assigned here and resides in the system_interface.
   *
   * If overwritten:
   * The state interfaces have to be created and transferred according
   * to the hardware info passed in for the configuration.
   *
   * Note the ownership over the state interfaces is transferred to the caller.
   *
   * \return vector of state interfaces
   */
  [[deprecated(
    "Replaced by vector<std::shared_ptr<StateInterface>> on_export_state_interfaces() method. "
    "Exporting is handled "
    "by the Framework.")]] virtual std::vector<StateInterface>
  export_state_interfaces()
  {
    // return empty vector by default. For backward compatibility we check if all vectors is empty
    // and if so call on_export_state_interfaces()
    std::vector<StateInterface> state_interfaces;
    return state_interfaces;
  }

  std::vector<std::shared_ptr<StateInterface>> on_export_state_interfaces()
  {
    std::vector<std::shared_ptr<StateInterface>> state_interfaces;
    state_interfaces.reserve(
      joint_state_interfaces_.size() + sensor_state_interfaces_.size() +
      gpio_state_interfaces_.size());

    for (const auto & [name, descr] : joint_state_interfaces_)
    {
      system_states_.insert(std::make_pair(name, std::make_shared<StateInterface>(descr)));
      state_interfaces.push_back(system_states_.at(name));
    }
    for (const auto & [name, descr] : sensor_state_interfaces_)
    {
      system_states_.insert(std::make_pair(name, std::make_shared<StateInterface>(descr)));
      state_interfaces.push_back(system_states_.at(name));
    }
    for (const auto & [name, descr] : gpio_state_interfaces_)
    {
      system_states_.insert(std::make_pair(name, std::make_shared<StateInterface>(descr)));
      state_interfaces.push_back(system_states_.at(name));
    }
    return state_interfaces;
  }

  /// Exports all command interfaces for this hardware interface.
  /**
   * Default implementation for exporting the CommandInterfaces. The CommandInterfaces are created
   * according to the InterfaceDescription. The memory accessed by the controllers and hardware is
   * assigned here and resides in the system_interface.
   *
   * The command interfaces have to be created and transferred according
   * to the hardware info passed in for the configuration.
   *
   * Note the ownership over the state interfaces is transferred to the caller.
   *
   * \return vector of command interfaces
   */
  [[deprecated(
    "Replaced by vector<std::shared_ptr<CommandInterface>> on_export_command_interfaces() method. "
    "Exporting is "
    "handled "
    "by the Framework.")]] virtual std::vector<CommandInterface>
  export_command_interfaces()
  {
    // return empty vector by default. For backward compatibility we check if all vectors is empty
    // and if so call on_export_command_interfaces()
    std::vector<CommandInterface> command_interfaces;
    return command_interfaces;
  }

  std::vector<std::shared_ptr<CommandInterface>> on_export_command_interfaces()
  {
    std::vector<std::shared_ptr<CommandInterface>> command_interfaces;
    command_interfaces.reserve(joint_command_interfaces_.size() + gpio_command_interfaces_.size());

    for (const auto & [name, descr] : joint_command_interfaces_)
    {
      system_commands_.insert(std::make_pair(name, std::make_shared<CommandInterface>(descr)));
      command_interfaces.push_back(system_commands_.at(name));
    }

    for (const auto & [name, descr] : gpio_command_interfaces_)
    {
      system_commands_.insert(std::make_pair(name, std::make_shared<CommandInterface>(descr)));
      command_interfaces.push_back(system_commands_.at(name));
    }
    return command_interfaces;
  }

  /// Prepare for a new command interface switch.
  /**
   * Prepare for any mode-switching required by the new command interface combination.
   *
   * \note This is a non-realtime evaluation of whether a set of command interface claims are
   * possible, and call to start preparing data structures for the upcoming switch that will occur.
   * \note All starting and stopping interface keys are passed to all components, so the function
   * should return return_type::OK by default when given interface keys not relevant for this
   * component. \param[in] start_interfaces vector of string identifiers for the command interfaces
   * starting. \param[in] stop_interfaces vector of string identifiers for the command interfaces
   * stopping. \return return_type::OK if the new command interface combination can be prepared, or
   * if the interface key is not relevant to this system. Returns return_type::ERROR otherwise.
   */
  virtual return_type prepare_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/)
  {
    return return_type::OK;
  }

  // Perform switching to the new command interface.
  /**
   * Perform the mode-switching for the new command interface combination.
   *
   * \note This is part of the realtime update loop, and should be fast.
   * \note All starting and stopping interface keys are passed to all components, so the function
   * should return return_type::OK by default when given interface keys not relevant for this
   * component. \param[in] start_interfaces vector of string identifiers for the command interfaces
   * starting. \param[in] stop_interfaces vector of string identifiers for the command interfaces
   * stopping. \return return_type::OK if the new command interface combination can be switched to,
   * or if the interface key is not relevant to this system. Returns return_type::ERROR otherwise.
   */
  virtual return_type perform_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/)
  {
    return return_type::OK;
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

  /// Write the current command values to the actuator.
  /**
   * The physical hardware shall be updated with the latest value from
   * the exported command interfaces.
   *
   * \param[in] time The time at the start of this control loop iteration
   * \param[in] period The measured time taken by the last control loop iteration
   * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
   */
  virtual return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

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
  const rclcpp_lifecycle::State & get_state() const { return lifecycle_state_; }

  /// Set life-cycle state of the actuator hardware.
  /**
   * \return state.
   */
  void set_state(const rclcpp_lifecycle::State & new_state) { lifecycle_state_ = new_state; }

  void set_state(const std::string & interface_name, const double & value)
  {
    system_states_.at(interface_name)->set_value(value);
  }

  double get_state(const std::string & interface_name) const
  {
    return system_states_.at(interface_name)->get_value();
  }

  void set_command(const std::string & interface_name, const double & value)
  {
    system_commands_.at(interface_name)->set_value(value);
  }

  double get_command(const std::string & interface_name) const
  {
    return system_commands_.at(interface_name)->get_value();
  }

protected:
  /// Get the logger of the SystemInterface.
  /**
   * \return logger of the SystemInterface.
   */
  rclcpp::Logger get_logger() const { return system_logger_; }

  /// Get the clock of the SystemInterface.
  /**
   * \return clock of the SystemInterface.
   */
  rclcpp::Clock::SharedPtr get_clock() const { return clock_interface_->get_clock(); }

  /// Get the hardware info of the SystemInterface.
  /**
   * \return hardware info of the SystemInterface.
   */
  const HardwareInfo & get_hardware_info() const { return info_; }

  HardwareInfo info_;
  std::map<std::string, InterfaceDescription> joint_state_interfaces_;
  std::map<std::string, InterfaceDescription> joint_command_interfaces_;

  std::map<std::string, InterfaceDescription> sensor_state_interfaces_;

  std::map<std::string, InterfaceDescription> gpio_state_interfaces_;
  std::map<std::string, InterfaceDescription> gpio_command_interfaces_;

private:
  std::map<std::string, std::shared_ptr<StateInterface>> system_states_;
  std::map<std::string, std::shared_ptr<CommandInterface>> system_commands_;

  rclcpp_lifecycle::State lifecycle_state_;

private:
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface_;
  rclcpp::Logger system_logger_;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__SYSTEM_INTERFACE_HPP_
