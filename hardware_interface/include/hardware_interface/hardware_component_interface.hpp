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

#ifndef HARDWARE_INTERFACE__HARDWARE_COMPONENT_INTERFACE_HPP_
#define HARDWARE_INTERFACE__HARDWARE_COMPONENT_INTERFACE_HPP_

#include <fmt/compile.h>

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "control_msgs/msg/hardware_status.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/introspection.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_component_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "hardware_interface/types/trigger_type.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/version.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/async_function_handler.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"

namespace hardware_interface
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief Virtual base class for all hardware components (Actuators, Sensors, and Systems).
 *
 * This class provides the common structure and functionality for all hardware components,
 * including lifecycle management, interface handling, and asynchronous support. Hardware
 * plugins should inherit from one of its derivatives: ActuatorInterface, SensorInterface,
 * or SystemInterface.
 */
class HardwareComponentInterface : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
{
public:
  HardwareComponentInterface();

  /// HardwareComponentInterface copy constructor is actively deleted.
  /**
   * Hardware interfaces have unique ownership and thus can't be copied in order to avoid
   * failed or simultaneous access to hardware.
   */
  HardwareComponentInterface(const HardwareComponentInterface & other) = delete;

  HardwareComponentInterface(HardwareComponentInterface && other) = delete;

  virtual ~HardwareComponentInterface();

  /// Initialization of the hardware interface from data parsed from the robot's URDF and also the
  /// clock and logger interfaces.
  /**
   * \param[in] hardware_info structure with data from URDF.
   * \param[in] clock pointer to the resource manager clock.
   * \param[in] logger Logger for the hardware component.
   * \returns CallbackReturn::SUCCESS if required data are provided and can be parsed.
   * \returns CallbackReturn::ERROR if any error happens or data are missing.
   */
  [[deprecated(
    "Replaced by CallbackReturn init(const hardware_interface::HardwareComponentParams & "
    "params). Initialization is handled by the Framework.")]] CallbackReturn
  init(const HardwareInfo & hardware_info, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
  {
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = hardware_info;
    params.clock = clock;
    params.logger = logger;
    return init(params);
  };

  /// Initialization of the hardware interface from data parsed from the robot's URDF and also the
  /// clock and logger interfaces.
  /**
   * \param[in] params  A struct of type HardwareComponentParams containing all necessary
   * parameters for initializing this specific hardware component,
   * including its HardwareInfo, a dedicated logger, a clock, and a
   * weak_ptr to the executor.
   * \warning The parsed executor should not be used to call `cancel()` or use blocking callbacks
   * such as `spin()`.
   * \returns CallbackReturn::SUCCESS if required data are provided and can be parsed.
   * \returns CallbackReturn::ERROR if any error happens or data are missing.
   */
  CallbackReturn init(const hardware_interface::HardwareComponentParams & params);

  /// User-overridable method to configure the structure of the HardwareStatus message.
  /**
   * To enable status publishing, override this method to pre-allocate the message structure
   * and fill in static information like device IDs and interface names. This method is called
   * once during the non-realtime `init()` phase. If the `hardware_device_states` vector is
   * left empty, publishing will be disabled.
   *
   * \param[out] msg_template A reference to a HardwareStatus message to be configured.
   * \returns CallbackReturn::SUCCESS if configured successfully, CallbackReturn::ERROR on failure.
   */
  virtual CallbackReturn init_hardware_status_message(
    control_msgs::msg::HardwareStatus & msg_template);

  /// User-overridable method to fill the hardware status message with real-time data.
  /**
   * This real-time safe method is called by the framework within the `trigger_read()` loop.
   * Override this method to populate the `value` fields of the pre-allocated message with the
   * latest hardware states that were updated in your `read()` method.
   *
   * \param[in,out] msg The pre-allocated message to be filled with the latest values.
   * \returns return_type::OK on success, return_type::ERROR on failure.
   */
  virtual return_type update_hardware_status_message(control_msgs::msg::HardwareStatus & msg);

  /// Initialization of the hardware interface from data parsed from the robot's URDF.
  /**
   * \param[in] params  A struct of type hardware_interface::HardwareComponentInterfaceParams
   * containing all necessary parameters for initializing this specific hardware component,
   * specifically its HardwareInfo, and a weak_ptr to the executor.
   * \warning The parsed executor should not be used to call `cancel()` or use blocking callbacks
   * such as `spin()`.
   * \returns CallbackReturn::SUCCESS if required data are provided and can be parsed.
   * \returns CallbackReturn::ERROR if any error happens or data are missing.
   */
  virtual CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params);

  /// Define custom node options for the hardware component interface.
  /**
   * Method used by the hardware component to instantiate the Lifecycle node
   * of the hardware component upon loading the hardware component.
   */
  virtual rclcpp::NodeOptions define_custom_node_options() const;

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
    "Exporting is handled by the Framework.")]] virtual std::vector<StateInterface>
  export_state_interfaces();

  /**
   * Override this method to export custom StateInterfaces which are not defined in the URDF file.
   * Those interfaces will be added to the unlisted_state_interfaces_ map.
   *
   * \return vector of descriptions to the unlisted StateInterfaces
   */
  virtual std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_state_interface_descriptions();

  /**
   * Default implementation for exporting the StateInterfaces. The StateInterfaces are created
   * according to the InterfaceDescription. The memory accessed by the controllers and hardware is
   * assigned here and resides in the interface.
   *
   * \return vector of shared pointers to the created and stored StateInterfaces
   */
  virtual std::vector<StateInterface::ConstSharedPtr> on_export_state_interfaces();

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
    "Replaced by vector<CommandInterface::SharedPtr> on_export_command_interfaces() method. "
    "Exporting is handled by the Framework.")]] virtual std::vector<CommandInterface>
  export_command_interfaces();

  /**
   * Override this method to export custom CommandInterfaces which are not defined in the URDF file.
   * Those interfaces will be added to the unlisted_command_interfaces_ map.
   *
   * \return vector of descriptions to the unlisted CommandInterfaces
   */
  virtual std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_command_interface_descriptions();

  /**
   * Default implementation for exporting the CommandInterfaces. The CommandInterfaces are created
   * according to the InterfaceDescription. The memory accessed by the controllers and hardware is
   * assigned here and resides in the system_interface.
   *
   * Actuator and System components should override this method. Sensor components can use the
   * default.
   *
   * \return vector of shared pointers to the created and stored CommandInterfaces
   */
  virtual std::vector<CommandInterface::SharedPtr> on_export_command_interfaces();

  /// Prepare for a new command interface switch.
  /**
   * Prepare for any mode-switching required by the new command interface combination.
   *
   * \note This is a non-realtime evaluation of whether a set of command interface claims are
   * possible, and call to start preparing data structures for the upcoming switch that will occur.
   * \param[in] start_interfaces vector of string identifiers for the command interfaces starting.
   * \param[in] stop_interfaces vector of string identifiers for the command interfaces stopping.
   * \return return_type::OK if the new command interface combination can be prepared (or) if the
   * interface key is not relevant to this system. Returns return_type::ERROR otherwise.
   */
  virtual return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces);

  // Perform switching to the new command interface.
  /**
   * Perform the mode-switching for the new command interface combination.
   *
   * \note This is part of the realtime update loop, and should be fast.
   * \param[in] start_interfaces vector of string identifiers for the command interfaces starting.
   * \param[in] stop_interfaces vector of string identifiers for the command interfaces stopping.
   * \return return_type::OK if the new command interface combination can be switched to (or) if the
   * interface key is not relevant to this system. Returns return_type::ERROR otherwise.
   */
  virtual return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces);

  /// Triggers the read method synchronously or asynchronously depending on the HardwareInfo
  /**
   * The data readings from the physical hardware has to be updated
   * and reflected accordingly in the exported state interfaces.
   * That is, the data pointed by the interfaces shall be updated.
   * The method is called in the resource_manager's read loop
   *
   * \param[in] time The time at the start of this control loop iteration
   * \param[in] period The measured time taken by the last control loop iteration
   * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
   */
  HardwareComponentCycleStatus trigger_read(
    const rclcpp::Time & time, const rclcpp::Duration & period);

  /// Read the current state values from the hardware.
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

  /// Triggers the write method synchronously or asynchronously depending on the HardwareInfo
  /**
   * The physical hardware shall be updated with the latest value from
   * the exported command interfaces.
   * The method is called in the resource_manager's write loop
   *
   * \param[in] time The time at the start of this control loop iteration
   * \param[in] period The measured time taken by the last control loop iteration
   * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
   */
  HardwareComponentCycleStatus trigger_write(
    const rclcpp::Time & time, const rclcpp::Duration & period);

  /// Write the current command values to the hardware.
  /**
   * The physical hardware shall be updated with the latest value from
   * the exported command interfaces.
   *
   * \param[in] time The time at the start of this control loop iteration
   * \param[in] period The measured time taken by the last control loop iteration
   * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
   */
  virtual return_type write(const rclcpp::Time & time, const rclcpp::Duration & period);

  /// Get name of the hardware.
  /**
   * \return name.
   */
  const std::string & get_name() const;

  /// Get name of the hardware group to which it belongs to.
  /**
   * \return group name.
   */
  const std::string & get_group_name() const;

  /// Get life-cycle state of the hardware.
  /**
   * \note Accessing members of the returned rclcpp_lifecycle::State is not real-time safe and
   * should not be called in the control loop.
   * \note This method is thread safe.
   * \return state.
   */
  const rclcpp_lifecycle::State & get_lifecycle_state() const;

  /// Set life-cycle state of the hardware.
  /**
   * Get the lifecycle id of the hardware component interface that is cached internally to avoid
   * calls to get_lifecycle_state() in the real-time control loop.
   * \note This method is real-time safe and thread safe and can be called in the control loop.
   * \return state.
   */
  void set_lifecycle_state(const rclcpp_lifecycle::State & new_state);

  /// Get the lifecycle id of the hardware component interface.
  uint8_t get_lifecycle_id() const;

  /// Does the state interface exist?
  /**
   * \param[in] interface_name The name of the state interface.
   * \return true if the state interface exists, false otherwise.
   */
  virtual bool has_state(const std::string & interface_name) const;

  /// Get the state interface handle
  /**
   * \param[in] interface_name The name of the state interface to access.
   * \return Shared pointer to the state interface handle.
   * \throws std::runtime_error This method throws a runtime error if it cannot find the state
   * interface with the given name.
   */
  virtual const StateInterface::SharedPtr & get_state_interface_handle(
    const std::string & interface_name) const;

  /// Set the value of a state interface.
  /**
   * \tparam T The type of the value to be stored.
   * \param interface_handle The shared pointer to the state interface to access.
   * \param value The value to store.
   * \param wait_until_set If true, the method ensures that the value is set successfully
   * \note This method is not real-time safe, when used with wait_until_set = true.
   * \throws std::runtime_error This method throws a runtime error if it cannot
   * access the state interface.
   * \return True if the value was set successfully, false otherwise.
   */
  template <typename T>
  bool set_state(
    const StateInterface::SharedPtr & interface_handle, const T & value, bool wait_until_set)
  {
    if (!interface_handle)
    {
      throw std::runtime_error(
        fmt::format(
          "State interface handle is null in hardware component: {}, while calling set_state "
          "method. This should not happen.",
          info_.name));
    }
    return interface_handle->set_value(value, wait_until_set);
  }

  /// Set the value of a state interface.
  /**
   * \tparam T The type of the value to be stored.
   * \param[in] interface_name The name of the state interface to access.
   * \param[in] value The value to store.
   * \note This method is not real-time safe.
   * \throws std::runtime_error This method throws a runtime error if it cannot
   * access the state interface.
   */
  template <typename T>
  void set_state(const std::string & interface_name, const T & value)
  {
    std::ignore = set_state(get_state_interface_handle(interface_name), value, true);
  }

  /**
   * \tparam T The type of the value to be retrieved.
   * \param[in] interface_handle The shared pointer to the state interface to access.
   * \param[out] state The variable to store the retrieved value.
   * \param[in] wait_until_get If true, the method ensures that the value is retrieved successfully
   * \note This method is not real-time safe, when used with wait_until_get = true.
   * \throws std::runtime_error This method throws a runtime error if it cannot
   * access the state interface or its stored value, when wait_until_get is true.
   * \return True if the value was retrieved successfully, false otherwise.
   */
  template <typename T>
  bool get_state(
    const StateInterface::SharedPtr & interface_handle, T & state, bool wait_until_get) const
  {
    if (!interface_handle)
    {
      throw std::runtime_error(
        fmt::format(
          "State interface handle is null in hardware component: {}, while calling get_state "
          "method. This should not happen.",
          info_.name));
    }
    const bool success = interface_handle->get_value(state, wait_until_get);
    if (!success && wait_until_get)
    {
      throw std::runtime_error(
        fmt::format(
          "Failed to get state value from interface: {} in hardware component: {}. This should "
          "not happen.",
          interface_handle->get_name(), info_.name));
    }
    return success;
  }

  /// Get the value from a state interface.
  /**
   * \tparam T The type of the value to be retrieved.
   * \param[in] interface_name The name of the state interface to access.
   * \return The value obtained from the interface.
   * \note This method is not real-time safe.
   * \throws std::runtime_error This method throws a runtime error if it cannot
   * access the state interface or its stored value.
   */
  template <typename T = double>
  T get_state(const std::string & interface_name) const
  {
    T state;
    get_state<T>(get_state_interface_handle(interface_name), state, true);
    return state;
  }

  /// Does the command interface exist?
  /**
   * \param[in] interface_name The name of the command interface.
   * \return true if the command interface exists, false otherwise.
   */
  virtual bool has_command(const std::string & interface_name) const;

  /// Get the command interface handle
  /**
   * \param[in] interface_name The name of the command interface to access.
   * \return Shared pointer to the command interface handle.
   * \throws std::runtime_error This method throws a runtime error if it cannot find the command
   * interface with the given name.
   */
  virtual const CommandInterface::SharedPtr & get_command_interface_handle(
    const std::string & interface_name) const;

  /// Set the value of a command interface.
  /**
   * \tparam T The type of the value to be stored.
   * \param interface_handle The shared pointer to the command interface to access.
   * \param value The value to store.
   * \param wait_until_set If true, the method ensures that the value is set successfully
   * \note This method is not real-time safe, when used with wait_until_set = true
   * \throws This method throws a runtime error if it cannot access the command interface.
   * \return True if the value was set successfully, false otherwise.
   */
  template <typename T>
  bool set_command(
    const CommandInterface::SharedPtr & interface_handle, const T & value, bool wait_until_set)
  {
    if (!interface_handle)
    {
      throw std::runtime_error(
        fmt::format(
          "Command interface handle is null in hardware component: {}, while calling set_command "
          "method. This should not happen.",
          info_.name));
    }
    return interface_handle->set_value(value, wait_until_set);
  }

  /// Set the value of a command interface.
  /**
   * \tparam T The type of the value to be stored.
   * \param interface_name The name of the command
   * interface to access.
   * \param value The value to store.
   * \note This method is not real-time safe.
   * \throws This method throws a runtime error if it cannot access the command interface.
   */
  template <typename T>
  void set_command(const std::string & interface_name, const T & value)
  {
    std::ignore = set_command(get_command_interface_handle(interface_name), value, true);
  }

  /**
   * \tparam T The type of the value to be retrieved.
   * \param[in] interface_handle The shared pointer to the command interface to access.
   * \param[out] command The variable to store the retrieved value.
   * \param[in] wait_until_get If true, the method ensures that the value is retrieved successfully
   * \note This method is not real-time safe, when used with wait_until_get = true.
   * \return True if the value was retrieved successfully, false otherwise.
   * \throws std::runtime_error This method throws a runtime error if it cannot
   * access the command interface or its stored value, when wait_until_get is true.
   */
  template <typename T>
  bool get_command(
    const CommandInterface::SharedPtr & interface_handle, T & command, bool wait_until_get) const
  {
    if (!interface_handle)
    {
      throw std::runtime_error(
        fmt::format(
          "Command interface handle is null in hardware component: {}, while calling get_command "
          "method. This should not happen.",
          info_.name));
    }
    const bool success = interface_handle->get_value(command, wait_until_get);
    if (!success && wait_until_get)
    {
      throw std::runtime_error(
        fmt::format(
          "Failed to get command value from interface: {} in hardware component: {}. This should "
          "not happen.",
          interface_handle->get_name(), info_.name));
    }
    return success;
  }

  ///  Get the value from a command interface.
  /**
   * \tparam T The type of the value to be retrieved.
   * \param[in] interface_name The name of the command interface to access.
   * \return The value obtained from the interface.
   * \note This method is not real-time safe.
   * \throws std::runtime_error This method throws a runtime error if it cannot
   * access the command interface or its stored value.
   */
  template <typename T = double>
  T get_command(const std::string & interface_name) const
  {
    T command;
    get_command<T>(get_command_interface_handle(interface_name), command, true);
    return command;
  }

  /// Get the logger of the HardwareComponentInterface.
  /**
   * \return logger of the HardwareComponentInterface.
   */
  virtual rclcpp::Logger get_logger() const;

  /// Get the clock
  /**
   * \return clock that is shared with the controller manager
   */
  virtual rclcpp::Clock::SharedPtr get_clock() const;

  /// Get the default node of the HardwareComponentInterface.
  /**
   * \return node of the HardwareComponentInterface.
   */
  virtual rclcpp::Node::SharedPtr get_node() const;

  /// Get the hardware info of the HardwareComponentInterface.
  /**
   * \return hardware info of the HardwareComponentInterface.
   */
  const HardwareInfo & get_hardware_info() const;

  /// Pause any asynchronous operations.
  /**
   * This method is called to pause any ongoing asynchronous operations, such as read/write cycles.
   * It is typically used during lifecycle transitions or when the hardware needs to be paused.
   */
  void pause_async_operations();

  /// Prepare for the activation of the hardware.
  /**
   * This method is called before the hardware is activated by the resource manager.
   */
  void prepare_for_activation();

  /// Enable or disable introspection of the hardware.
  /**
   * \param[in] enable Enable introspection if true, disable otherwise.
   */
  void enable_introspection(bool enable);

protected:
  HardwareInfo info_;
  // interface names to InterfaceDescription
  std::unordered_map<std::string, InterfaceDescription> joint_state_interfaces_;
  std::unordered_map<std::string, InterfaceDescription> joint_command_interfaces_;

  std::unordered_map<std::string, InterfaceDescription> sensor_state_interfaces_;

  std::unordered_map<std::string, InterfaceDescription> gpio_state_interfaces_;
  std::unordered_map<std::string, InterfaceDescription> gpio_command_interfaces_;

  std::unordered_map<std::string, InterfaceDescription> unlisted_state_interfaces_;
  std::unordered_map<std::string, InterfaceDescription> unlisted_command_interfaces_;

  rclcpp_lifecycle::State lifecycle_state_;
  std::unique_ptr<realtime_tools::AsyncFunctionHandler<return_type>> async_handler_;

  // Exported Command- and StateInterfaces in order they are listed in the hardware description.
  std::vector<StateInterface::SharedPtr> joint_states_;
  std::vector<CommandInterface::SharedPtr> joint_commands_;

  std::vector<StateInterface::SharedPtr> sensor_states_;

  std::vector<StateInterface::SharedPtr> gpio_states_;
  std::vector<CommandInterface::SharedPtr> gpio_commands_;

  std::vector<StateInterface::SharedPtr> unlisted_states_;
  std::vector<CommandInterface::SharedPtr> unlisted_commands_;

private:
  class HardwareComponentInterfaceImpl;
  std::unique_ptr<HardwareComponentInterfaceImpl> impl_;

protected:
  pal_statistics::RegistrationsRAII stats_registrations_;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__HARDWARE_COMPONENT_INTERFACE_HPP_
