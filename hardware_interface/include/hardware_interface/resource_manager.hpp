// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef HARDWARE_INTERFACE__RESOURCE_MANAGER_HPP_
#define HARDWARE_INTERFACE__RESOURCE_MANAGER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/actuator.hpp"
#include "hardware_interface/hardware_component_info.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/sensor.hpp"
#include "hardware_interface/system.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/time.hpp"

namespace hardware_interface
{
class ResourceStorage;
class ControllerManager;

struct HardwareReadWriteStatus
{
  bool ok;
  std::vector<std::string> failed_hardware_names;
};

class ResourceManager
{
public:
  /// Default constructor for the Resource Manager.
  explicit ResourceManager(
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_interface);

  /// Constructor for the Resource Manager.
/**
 * @brief Initializes hardware components and their interfaces from a URDF.
 *
 * This constructor loads the provided URDF, initializes listed hardware components, 
 * and populates their respective state and command interfaces.
 *
 * @param[in] urdf String containing the URDF.
 * @param[in] activate_all If true, activates all resources immediately (used in tests).
 * @param[in] update_rate Update rate of the controller manager, determining async component frequency.
 * @param[in] clock_interface Reference to the CM node's clock interface for time-based async triggers.
 */

  explicit ResourceManager(
    const std::string & urdf,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_interface,
    bool activate_all = false, const unsigned int update_rate = 100);

  ResourceManager(const ResourceManager &) = delete;

  virtual ~ResourceManager();

 /// Shutdown all hardware components, regardless of their state.
/**
 * @brief Safely shuts down all hardware components.
 *
 * This method is invoked when the controller manager is shutting down.
 *
 * @return True if all hardware components are successfully shut down, false otherwise.
 */

  bool shutdown_components();

  /// Load resources from a given URDF.
/**
 * @brief Initializes the resource manager with a URDF.
 *
 * This method allows post-initialization of the resource manager when the URDF
 * is not available during the first initialization.
 *
 * @param[in] urdf String containing the URDF.
 * @param[in] update_rate Update rate of the main control loop (controller manager).
 * @return False if URDF validation fails.
 */

  virtual bool load_and_initialize_components(
    const std::string & urdf, const unsigned int update_rate = 100);

  /**
 * @brief Checks if the resource manager has loaded and initialized components.
 *
 * Ensures that `load_and_initialize_components(...)` is only called once.
 * While loading the URDF later is permitted, multiple calls to this function
 * (e.g., for reloading or loading a different URDF) are not allowed.
 *
 * @return True if components are successfully loaded and initialized.
 * @return False if no components are loaded or initialized.
 */

  bool are_components_initialized() const;

 /**
 * @brief Claims a state interface using its key.
 *
 * The resource remains claimed while in scope. Once it goes out of scope, 
 * it is automatically released by the destructor.
 *
 * @param[in] key String identifier of the state interface to claim.
 * @return The claimed state interface.
 */

  LoanedStateInterface claim_state_interface(const std::string & key);

 /**
 * @brief Retrieves all registered state interface keys.
 *
 * The keys are gathered from all loaded hardware components.
 *
 * @return A vector of strings containing all registered state interface keys.
 */

  std::vector<std::string> state_interface_keys() const;

  /**
 * @brief Retrieves all available state interface keys.
 *
 * The keys are gathered from the available list.
 *
 * @return A vector of strings containing all available 

  std::vector<std::string> available_state_interfaces() const;

  /**
 * @brief Checks if a state interface is available for the given key.
 *
 * @param[in] key String identifier of the state interface.
 * @return true if the interface is available, false otherwise.
 */

  bool state_interface_is_available(const std::string & name) const;

 /**
 * @brief Adds controllers' exported state interfaces to the resource manager.
 *
 * This method transfers management of exported state interfaces to the resource manager.
 * When chaining controllers, state interfaces are utilized by preceding controllers and
 * should be managed similarly to hardware state interfaces.
 *
 * @param[in] controller_name Name of the controller whose state interfaces are imported.
 * @param[in] interfaces List of the controller's state interfaces as StateInterfaces.
 */

  void import_controller_exported_state_interfaces(
    const std::string & controller_name, std::vector<StateInterface::ConstSharedPtr> & interfaces);

  /// Get list of exported state interfaces of a controller.
/**
 * Retrieves the list of stored exported state interface names for a given controller.
 *
 * @param[in] controller_name Name of the controller whose state interfaces are requested.
 * @return List of reference interface names.
 */

  std::vector<std::string> get_controller_exported_state_interface_names(
    const std::string & controller_name);

 /// Add controller's exported state interfaces to the available list.
/**
 * Adds the state interfaces of a controller to the available list. 
 * This method should be called when a controller is activated with chained mode enabled, 
 * allowing its exported state interfaces to be used by other controllers in a chained architecture.
 *
 * @param[in] controller_name Name of the controller whose interfaces should become available.
 */

  void make_controller_exported_state_interfaces_available(const std::string & controller_name);

  /// Remove controller's exported state interfaces from the available list.
/**
 * Removes the state interfaces of a controller from the available list. 
 * This method should be called when a controller is deactivated, 
 * ensuring its reference interfaces are no longer accessible to other controllers.
 *
 * @param[in] controller_name Name of the controller whose interfaces should be removed.
 */

  void make_controller_exported_state_interfaces_unavailable(const std::string & controller_name);

 /// Remove controller's exported state interfaces from the resource manager.
/**
 * Removes the exported state interfaces of a controller from the resource manager, 
 * ensuring they are deleted from all internal maps and lists.
 *
 * @param[in] controller_name Name of the controller whose interfaces will be removed.
 */

  void remove_controller_exported_state_interfaces(const std::string & controller_name);

 /// Add controllers' reference interfaces to the resource manager.
/**
 * Transfers management of reference interfaces to the resource manager. 
 * In chained controllers, reference interfaces serve as command interfaces 
 * for preceding controllers and must be managed similarly to hardware command interfaces.
 *
 * @param[in] controller_name Name of the controller whose reference interfaces are imported.
 * @param[in] interfaces List of the controller's reference interfaces as CommandInterfaces.
 */

  void import_controller_reference_interfaces(
    const std::string & controller_name,
    const std::vector<hardware_interface::CommandInterface::SharedPtr> & interfaces);

  /// Get list of reference interfaces for a controller.
/**
 * Retrieves the stored reference interface names associated with a given controller.
 *
 * @param[in] controller_name Name of the controller whose reference interfaces are requested.
 * @return List of reference interface names.
 */

  std::vector<std::string> get_controller_reference_interface_names(
    const std::string & controller_name);

  /// Add a controller's reference interfaces to the available list.
/**
 * Marks the reference interfaces of a specified controller as available.
 * This method should be called when a controller is activated in chained mode,
 * allowing its reference interfaces to be used by other controllers in a chained setup.
 *
 * @param[in] controller_name Name of the controller whose reference interfaces should be available.
 */

  void make_controller_reference_interfaces_available(const std::string & controller_name);

  /// Remove a controller's reference interfaces from the available list.
/**
 * This method unregisters the reference interfaces of a specified controller,
 * ensuring they are no longer available for use by other controllers.
 * It should be called when the controller is deactivated.
 *
 * @param[in] controller_name Name of the controller whose reference interfaces should be removed.
 */

  void make_controller_reference_interfaces_unavailable(const std::string & controller_name);

  /// Remove controllers' reference interfaces from the resource manager.
/**
 * Deletes the reference interfaces associated with a given controller from the resource manager.
 * This ensures that the interfaces are no longer stored or accessible within the system.
 *
 * @param[in] controller_name Name of the controller whose reference interfaces should be removed.
 */

  void remove_controller_reference_interfaces(const std::string & controller_name);

/// Cache the mapping between hardware and controllers using it.
/**
 * Identifies and stores the relationship between a controller and the hardware it interacts with.
 * This mapping is based on the interfaces used by the controller, allowing for efficient 
 * access in future operations.
 *
 * @param[in] controller_name Name of the controller whose interface mappings are being cached.
 * @param[in] interfaces List of interfaces used by the specified controller.
 */

  void cache_controller_to_hardware(
    const std::string & controller_name, const std::vector<std::string> & interfaces);

  /// Return cached controllers for a specific hardware.
/**
 * Retrieves a list of controller names that have been cached as using the specified hardware.
 * This allows for efficient lookup of controllers associated with a given hardware component.
 *
 * @param[in] hardware_name Name of the hardware for which cached controllers should be retrieved.
 * @return List of cached controller names that depend on the specified hardware.
 */

  std::vector<std::string> get_cached_controllers_to_hardware(const std::string & hardware_name);

  /// Checks whether a command interface is already claimed.
/**
 * Determines if a given command interface is currently claimed.
 * A command interface can only be claimed by a single instance at a time.
 * 
 * @note There is no equivalent function for state interfaces since they are read-only 
 *       and can be accessed by multiple instances simultaneously.
 *
 * @param[in] key String identifier of the interface to check.
 * @return true if the interface is already claimed, false if it is available.
 */

  bool command_interface_is_claimed(const std::string & key) const;

  /// Claim a command interface given its key.
/**
 * Claims ownership of a command interface identified by the given key.
 * The resource remains claimed as long as it is in scope.
 * Once it goes out of scope, the destructor releases it, allowing others to claim it.
 *
 * @param[in] key String identifier of the command interface to claim.
 * @return The claimed command interface.
 */

  LoanedCommandInterface claim_command_interface(const std::string & key);

 /// Returns all registered command interface keys.
/**
 * Retrieves a list of all command interface keys registered across the loaded hardware components.
 *
 * @return A vector of strings containing all registered command interface keys.
 */

  std::vector<std::string> command_interface_keys() const;

  /// Returns all available command interface keys.
/**
 * Retrieves a list of command interface keys that are currently available for use.
 *
 * @return A vector of strings containing all available command interface names.
 */

  std::vector<std::string> available_command_interfaces() const;

/// Checks whether a command interface is available under the given name.
/**
 * Determines if a command interface with the specified name is currently available.
 *
 * @param[in] name String identifier of the command interface to check.
 * @return true if the interface is available, false otherwise.
 */

  bool command_interface_is_available(const std::string & interface) const;

  /// Return the number of loaded actuator components.
/**
 * Retrieves the total count of actuator components that have been loaded.
 *
 * @return The number of loaded actuator components as a size_t value.
 */

  size_t actuator_components_size() const;

  /// Return the number of loaded sensor components.
/**
 * Retrieves the total count of sensor components that have been loaded.
 *
 * @return The number of loaded sensor components as a size_t value.
 */

  size_t sensor_components_size() const;

  /// Return the number of loaded system components.
/**
 * Retrieves the total count of system components that have been loaded.
 *
 * @return The number of loaded system components as a size_t value.
 */

  size_t system_components_size() const;

  /// Import a hardware component which is not listed in the URDF.
/**
 * Allows adding hardware components that are initialized outside of the URDF after 
 * the initial setup. The component must have `HardwareInfo` available, which can be 
 * either parsed from a URDF string or manually filled.
 *
 * @note This operation might invalidate existing state and command interfaces and 
 *       should not be called while a controller is running.
 * @note If no `hardware_info` is available, the component must be externally configured 
 *       before calling this function.
 *
 * @param[in] actuator Pointer to the actuator interface.
 * @param[in] hardware_info Information about the hardware component.
 */

  void import_component(
    std::unique_ptr<ActuatorInterface> actuator, const HardwareInfo & hardware_info);

 /// Import a hardware component which is not listed in the URDF.
/**
 * Allows adding hardware sensor components that are initialized outside of the URDF 
 * after the initial setup. The component must have `HardwareInfo` available, which 
 * can be either parsed from a URDF string or manually filled.
 *
 * @note This operation might invalidate existing state and command interfaces and 
 *       should not be called while a controller is running.
 * @note If no `hardware_info` is available, the component must be externally configured 
 *       before calling this function.
 *
 * @param[in] sensor Pointer to the sensor interface.
 * @param[in] hardware_info Information about the hardware component.
 */

  void import_component(
    std::unique_ptr<SensorInterface> sensor, const HardwareInfo & hardware_info);

  /// Import a hardware component which is not listed in the URDF.
/**
 * Allows adding system hardware components that are initialized outside of the URDF 
 * after the initial setup. The component must have `HardwareInfo` available, which 
 * can be either parsed from a URDF string or manually filled.
 *
 * @note This operation might invalidate existing state and command interfaces and 
 *       should not be called while a controller is running.
 * @note If no `hardware_info` is available, the component must be externally configured 
 *       before calling this function.
 *
 * @param[in] system Pointer to the system interface.
 * @param[in] hardware_info Information about the hardware component.
 */

  void import_component(
    std::unique_ptr<SystemInterface> system, const HardwareInfo & hardware_info);

/// Return status for all components.
/**
 * Retrieves the current status of all registered hardware components.
 *
 * This function provides an overview of all hardware components along with their 
 * respective statuses. It helps monitor and manage hardware availability and state.
 *
 * @return A map where the keys are hardware component names and the values represent 
 *         their current status.
 */


  const std::unordered_map<std::string, HardwareComponentInfo> & get_components_status();

  /// Prepare the hardware components for a new command interface mode.
/**
 * Prepares hardware components for a new command interface claim.
 *
 * This function facilitates the transition of hardware components to a new command interface 
 * mode. It ensures that the required resources are available and prepares the system for 
 * mode switching.
 *
 * @note This is intended for mode-switching when a hardware interface needs 
 *       to change control mode depending on which command interface is claimed.
 * @note This is for non-realtime preparation and acceptance of new command 
 *       resource combinations.
 * @note `accept_command_resource_claim` is called on all actuators and system 
 *       components. Hardware interfaces should return `hardware_interface::return_type::OK` 
 *       by default.
 *
 * @param[in] start_interfaces A vector of string identifiers for the command interfaces starting.
 * @param[in] stop_interfaces A vector of string identifiers for the command interfaces stopping.
 * @return `true` if the switch can be prepared; `false` if a component rejects 
 *         the switch request or if at least one of the input interfaces does not exist 
 *         or is not available (i.e., the component is not in `ACTIVE` or `INACTIVE` state).
 */


  bool prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces);

  /// Notifies hardware components to perform realtime command interface mode switching.
/**
 * @brief Triggers realtime switching of command interface modes in hardware components.
 *
 * @note Intended for mode-switching when a hardware interface changes control mode 
 *       based on the claimed command interface.
 * @note Assumes `prepare_command_mode_switch` was called just before this method 
 *       with the same arguments.
 *
 * @param[in] start_interfaces Vector of command interface identifiers to start.
 * @param[in] stop_interfaces Vector of command interface identifiers to stop.
 * @return `true` if the switch is performed; `false` if rejected by a component.
 */


  bool perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces);

  /// Sets the state of a hardware component.
/**
 * @brief Transitions a hardware component to the specified state.
 *
 * Handles all necessary state transitions to reach the target state, following 
 * the state machine defined in: https://design.ros2.org/articles/node_lifecycle.html.
 *
 * @note This method is not part of the real-time critical update loop.
 *
 * @param[in] component_name Name of the hardware component to update.
 * @param[in] target_state Desired state for the hardware component.
 * @return `hardware_interface::return_type::OK` if the state transition succeeds,  
 *         `hardware_interface::return_type::ERROR` if any transition fails.
 */


  return_type set_component_state(
    const std::string & component_name, rclcpp_lifecycle::State & target_state);

  /// Reads data from all loaded hardware components.
/**
 * @brief Collects data from all active hardware components.
 *
 * This function is part of the real-time critical update loop and is 
 * real-time safe if the hardware interfaces are correctly implemented.
 */


  HardwareReadWriteStatus read(const rclcpp::Time & time, const rclcpp::Duration & period);

/// Writes data to all loaded hardware components.
/**
 * @brief Sends data to all active hardware components.
 *
 * This function is part of the real-time critical update loop and is 
 * real-time safe if the hardware interfaces are correctly implemented.
 */


  HardwareReadWriteStatus write(const rclcpp::Time & time, const rclcpp::Duration & period);

 /// Checks if a command interface is registered under the given key.
/**
 * @brief Verifies the existence of a command interface.
 *
 * @param[in] key String identifier of the command interface to check.
 * @return `true` if the interface exists, `false` otherwise.
 */

  bool command_interface_exists(const std::string & key) const;

  /// Checks if a state interface is registered under the given key.
/**
 * @brief Verifies the existence of a state interface.
 *
 * @return `true` if the interface exists, `false` otherwise.
 */

  bool state_interface_exists(const std::string & key) const;

  /// A method to register a callback to be called when the component state changes.
  /**
   * \param[in] callback function to be called when the component state changes.
   */
  void set_on_component_state_switch_callback(std::function<void()> callback);

protected:
  /// Retrieves the logger for the resource manager.
/**
 * @brief Gets the logger instance used by the resource manager.
 *
 * @return The logger of the resource manager.
 */

  rclcpp::Logger get_logger() const;

  /// Retrieves the clock for the resource manager.
/**
 * @brief Gets the clock instance used by the resource manager.
 *
 * @return The clock of the resource manager.
 */

  rclcpp::Clock::SharedPtr get_clock() const;

  bool components_are_loaded_and_initialized_ = false;

  mutable std::recursive_mutex resource_interfaces_lock_;
  mutable std::recursive_mutex claimed_command_interfaces_lock_;
  mutable std::recursive_mutex resources_lock_;

private:
  bool validate_storage(const std::vector<hardware_interface::HardwareInfo> & hardware_info) const;

  void release_command_interface(const std::string & key);

  std::unordered_map<std::string, bool> claimed_command_interface_map_;

  std::unique_ptr<ResourceStorage> resource_storage_;

  // Structure to store read and write status so it is not initialized in the real-time loop
  HardwareReadWriteStatus read_write_status;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__RESOURCE_MANAGER_HPP_
