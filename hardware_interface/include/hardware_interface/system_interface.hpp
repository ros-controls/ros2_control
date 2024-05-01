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
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_emergency_stop_signal.hpp"
#include "hardware_interface/types/hardware_interface_error_signals.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_warning_signals.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/duration.hpp"
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
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, lifecycle_state_names::UNKNOWN))
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
    import_command_interface_descriptions(info_);
    create_report_interfaces();
    return CallbackReturn::SUCCESS;
  };

  /**
   * Import the InterfaceDescription for the StateInterfaces from the HardwareInfo.
   * Separate them into the possible types: Joint, GPIO, Sensor and store them.
   */
  void import_state_interface_descriptions(const HardwareInfo & hardware_info)
  {
    auto joint_state_interface_descriptions =
      parse_state_interface_descriptions_from_hardware_info(hardware_info.joints);
    for (const auto & description : joint_state_interface_descriptions)
    {
      joint_state_interfaces_.insert(std::make_pair(description.get_name(), description));
    }
    auto sensor_state_interface_descriptions =
      parse_state_interface_descriptions_from_hardware_info(hardware_info.sensors);
    for (const auto & description : sensor_state_interface_descriptions)
    {
      sensor_state_interfaces_.insert(std::make_pair(description.get_name(), description));
    }
    auto gpio_state_interface_descriptions =
      parse_state_interface_descriptions_from_hardware_info(hardware_info.gpios);
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
      parse_command_interface_descriptions_from_hardware_info(hardware_info.joints);
    for (const auto & description : joint_command_interface_descriptions)
    {
      joint_command_interfaces_.insert(std::make_pair(description.get_name(), description));
    }
    auto gpio_command_interface_descriptions =
      parse_command_interface_descriptions_from_hardware_info(hardware_info.gpios);
    for (const auto & description : gpio_command_interface_descriptions)
    {
      gpio_command_interfaces_.insert(std::make_pair(description.get_name(), description));
    }
  }

  /**
   * Creates all interfaces used for reporting emergency stop, warning and error messages.
   * The available report interfaces are: EMERGENCY_STOP_SIGNAL, ERROR_SIGNAL, ERROR_SIGNAL_MESSAGE,
   * WARNING_SIGNAL and WARNING_SIGNAL_MESSAGE. Where the <report_type>_MESSAGE hold the message for
   * the corresponding report signal.
   * The interfaces are named like <hardware_name>/<report_interface_type>. E.g. if hardware is
   * called rrbot -> interface for WARNING_SIGNAL is called: rrbot/WARNING_SIGNAL
   */
  void create_report_interfaces()
  {
    // EMERGENCY STOP
    InterfaceInfo emergency_interface_info;
    emergency_interface_info.name = hardware_interface::EMERGENCY_STOP_SIGNAL;
    emergency_interface_info.data_type = "bool";
    InterfaceDescription emergency_interface_descr(info_.name, emergency_interface_info);
    emergency_stop_ = std::make_shared<StateInterface>(emergency_interface_descr);

    // ERROR
    // create error signal interface
    InterfaceInfo error_interface_info;
    error_interface_info.name = hardware_interface::ERROR_SIGNAL_INTERFACE_NAME;
    error_interface_info.data_type = "array<uint8_t>[32]";
    InterfaceDescription error_interface_descr(info_.name, error_interface_info);
    error_signal_ = std::make_shared<StateInterface>(error_interface_descr);
    // create error signal report message interface
    InterfaceInfo error_msg_interface_info;
    error_msg_interface_info.name = hardware_interface::ERROR_SIGNAL_MESSAGE_INTERFACE_NAME;
    error_msg_interface_info.data_type = "array<string>[32]";
    InterfaceDescription error_msg_interface_descr(info_.name, error_msg_interface_info);
    error_signal_message_ = std::make_shared<StateInterface>(error_msg_interface_descr);

    // WARNING
    //  create warning signal interface
    InterfaceInfo warning_interface_info;
    warning_interface_info.name = hardware_interface::WARNING_SIGNAL_INTERFACE_NAME;
    warning_interface_info.data_type = "array<int8_t>[32]";
    InterfaceDescription warning_interface_descr(info_.name, warning_interface_info);
    warning_signal_ = std::make_shared<StateInterface>(warning_interface_descr);
    // create warning signal report message interface
    InterfaceInfo warning_msg_interface_info;
    warning_msg_interface_info.name = hardware_interface::WARNING_SIGNAL_MESSAGE_INTERFACE_NAME;
    warning_msg_interface_info.data_type = "array<string>[32]";
    InterfaceDescription warning_msg_interface_descr(info_.name, warning_msg_interface_info);
    warning_signal_message_ = std::make_shared<StateInterface>(warning_msg_interface_descr);
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
    "Replaced by vector<std::shared_ptr<StateInterface>> on_export_state_interfaces() method. "
    "Exporting is handled "
    "by the Framework.")]] virtual std::vector<StateInterface>
  export_state_interfaces()
  {
    // return empty vector by default. For backward compatibility we check if all vectors is empty
    // and if so call on_export_state_interfaces()
    return {};
  }

  /**
   * Override this method to export custom StateInterfaces which are not defined in the URDF file.
   * Those interfaces will be added to the unlisted_state_interfaces_ map.
   *
   *  Note method name is going to be changed to export_state_interfaces() as soon as the deprecated
   * version is removed.
   *
   * \return vector of descriptions to the unlisted StateInterfaces
   */
  virtual std::vector<hardware_interface::InterfaceDescription> export_state_interfaces_2()
  {
    // return empty vector by default.
    return {};
  }

  /**
   * Default implementation for exporting the StateInterfaces. The StateInterfaces are created
   * according to the InterfaceDescription. The memory accessed by the controllers and hardware is
   * assigned here and resides in the system_interface.
   *
   * \return vector of shared pointers to the created and stored StateInterfaces
   */
  std::vector<std::shared_ptr<StateInterface>> on_export_state_interfaces()
  {
    // import the unlisted interfaces
    std::vector<hardware_interface::InterfaceDescription> unlisted_interface_descriptions =
      export_state_interfaces_2();

    std::vector<std::shared_ptr<StateInterface>> state_interfaces;
    state_interfaces.reserve(
      unlisted_interface_descriptions.size() + joint_state_interfaces_.size() +
      sensor_state_interfaces_.size() + gpio_state_interfaces_.size());

    // add InterfaceDescriptions and create the StateInterfaces from the descriptions and add to
    // maps.
    for (const auto & description : unlisted_interface_descriptions)
    {
      auto name = description.get_name();
      unlisted_state_interfaces_.insert(std::make_pair(name, description));
      auto state_interface = std::make_shared<StateInterface>(description);
      system_states_.insert(std::make_pair(name, state_interface));
      state_interfaces.push_back(state_interface);
    }

    for (const auto & [name, descr] : joint_state_interfaces_)
    {
      auto state_interface = std::make_shared<StateInterface>(descr);
      system_states_.insert(std::make_pair(name, state_interface));
      state_interfaces.push_back(state_interface);
    }
    for (const auto & [name, descr] : sensor_state_interfaces_)
    {
      auto state_interface = std::make_shared<StateInterface>(descr);
      system_states_.insert(std::make_pair(name, state_interface));
      state_interfaces.push_back(state_interface);
    }
    for (const auto & [name, descr] : gpio_state_interfaces_)
    {
      auto state_interface = std::make_shared<StateInterface>(descr);
      system_states_.insert(std::make_pair(name, state_interface));
      state_interfaces.push_back(state_interface);
    }

    // export warning signal interfaces
    state_interfaces.push_back(emergency_stop_);
    state_interfaces.push_back(error_signal_);
    state_interfaces.push_back(error_signal_message_);
    state_interfaces.push_back(warning_signal_);
    state_interfaces.push_back(warning_signal_message_);

    return state_interfaces;
  }

  /// Exports all command interfaces for this hardware interface.
  /**
   * Old way of exporting the CommandInterfaces. If a empty vector is returned then
   * the on_export_command_interfaces() method is called. If a vector with CommandInterfaces is
   * returned then the exporting of the CommandInterfaces is only done with this function and the
   * ownership is transferred to the resource manager. The set_command(...), get_command(...), ...,
   * can then not be used.
   *
   * Note the ownership over the state interfaces is transferred to the caller.
   *
   * \return vector of state interfaces
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
    return {};
  }

  /**
   * Override this method to export custom CommandInterfaces which are not defined in the URDF file.
   * Those interfaces will be added to the unlisted_command_interfaces_ map.
   *
   *  Note method name is going to be changed to export_command_interfaces() as soon as the
   * deprecated version is removed.
   *
   * \return vector of descriptions to the unlisted CommandInterfaces
   */
  virtual std::vector<hardware_interface::InterfaceDescription> export_command_interfaces_2()
  {
    // return empty vector by default.
    return {};
  }

  /**
   * Default implementation for exporting the CommandInterfaces. The CommandInterfaces are created
   * according to the InterfaceDescription. The memory accessed by the controllers and hardware is
   * assigned here and resides in the system_interface.
   *
   * \return vector of shared pointers to the created and stored CommandInterfaces
   */
  std::vector<std::shared_ptr<CommandInterface>> on_export_command_interfaces()
  {
    // import the unlisted interfaces
    std::vector<hardware_interface::InterfaceDescription> unlisted_interface_descriptions =
      export_command_interfaces_2();

    std::vector<std::shared_ptr<CommandInterface>> command_interfaces;
    command_interfaces.reserve(
      unlisted_interface_descriptions.size() + joint_command_interfaces_.size() +
      gpio_command_interfaces_.size());

    // add InterfaceDescriptions and create the CommandInterfaces from the descriptions and add to
    // maps.
    for (const auto & description : unlisted_interface_descriptions)
    {
      auto name = description.get_name();
      unlisted_command_interfaces_.insert(std::make_pair(name, description));
      auto command_interface = std::make_shared<CommandInterface>(description);
      system_commands_.insert(std::make_pair(name, command_interface));
      command_interfaces.push_back(command_interface);
    }

    for (const auto & [name, descr] : joint_command_interfaces_)
    {
      auto command_interface = std::make_shared<CommandInterface>(descr);
      system_commands_.insert(std::make_pair(name, command_interface));
      command_interfaces.push_back(command_interface);
    }

    for (const auto & [name, descr] : gpio_command_interfaces_)
    {
      auto command_interface = std::make_shared<CommandInterface>(descr);
      system_commands_.insert(std::make_pair(name, command_interface));
      command_interfaces.push_back(command_interface);
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
   * starting. \param[in] stop_interfaces vector of string identifiers for the command interfacs
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
   * starting. \param[in] stop_interfaces vector of string identifiers for the command interfacs
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

  void set_emergency_stop(const double & emergency_stop)
  {
    emergency_stop_->set_value(emergency_stop);
  }

  double get_emergency_stop() const { return emergency_stop_->get_value(); }

  void set_error_code(const double & error_code) { error_signal_->set_value(error_code); }

  double get_error_code() const { return error_signal_->get_value(); }

  void set_error_message(const double & error_message)
  {
    error_signal_message_->set_value(error_message);
  }

  double get_error_message() const { return error_signal_message_->get_value(); }

  void set_warning_code(const double & warning_codes) { warning_signal_->set_value(warning_codes); }

  double get_warning_code() const { return warning_signal_->get_value(); }

  void set_warning_message(const double & error_message)
  {
    warning_signal_message_->set_value(error_message);
  }

  double get_warning_message() const { return warning_signal_message_->get_value(); }

protected:
  HardwareInfo info_;
  std::unordered_map<std::string, InterfaceDescription> joint_state_interfaces_;
  std::unordered_map<std::string, InterfaceDescription> joint_command_interfaces_;

  std::unordered_map<std::string, InterfaceDescription> sensor_state_interfaces_;

  std::unordered_map<std::string, InterfaceDescription> gpio_state_interfaces_;
  std::unordered_map<std::string, InterfaceDescription> gpio_command_interfaces_;

  std::unordered_map<std::string, InterfaceDescription> unlisted_state_interfaces_;
  std::unordered_map<std::string, InterfaceDescription> unlisted_command_interfaces_;

private:
  std::unordered_map<std::string, std::shared_ptr<StateInterface>> system_states_;
  std::unordered_map<std::string, std::shared_ptr<CommandInterface>> system_commands_;

  std::shared_ptr<StateInterface> emergency_stop_;
  std::shared_ptr<StateInterface> error_signal_;
  std::shared_ptr<StateInterface> error_signal_message_;
  std::shared_ptr<StateInterface> warning_signal_;
  std::shared_ptr<StateInterface> warning_signal_message_;

  rclcpp_lifecycle::State lifecycle_state_;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__SYSTEM_INTERFACE_HPP_
