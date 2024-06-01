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
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/hardware_component_info.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

namespace hardware_interface
{
class ActuatorInterface;
class SensorInterface;
class SystemInterface;
class ResourceStorage;

struct HardwareReadWriteStatus
{
  bool ok;
  std::vector<std::string> failed_hardware_names;
};

class HARDWARE_INTERFACE_PUBLIC ResourceManager
{
public:
  /// Default constructor for the Resource Manager.
  ResourceManager();

  /// Constructor for the Resource Manager.
  /**
   * The implementation loads the specified urdf and initializes the
   * hardware components listed within as well as populate their respective
   * state and command interfaces.
   *
   * If the interfaces ought to be validated, the constructor throws an exception
   * in case the URDF lists interfaces which are not available.
   *
   * \param[in] urdf string containing the URDF.
   * \param[in] validate_interfaces boolean argument indicating whether the exported
   * interfaces ought to be validated. Defaults to true.
   * \param[in] activate_all boolean argument indicating if all resources should be immediately
   * activated. Currently used only in tests.
   */
  explicit ResourceManager(
    const std::string & urdf, bool validate_interfaces = true, bool activate_all = false);

  ResourceManager(const ResourceManager &) = delete;

  ~ResourceManager();

  /// Load resources from on a given URDF.
  /**
   * The resource manager can be post initialized with a given URDF.
   * This is mainly used in conjunction with the default constructor
   * in which the URDF might not be present at first initialization.
   *
   * \param[in] urdf string containing the URDF.
   * \param[in] validate_interfaces boolean argument indicating whether the exported
   * interfaces ought to be validated. Defaults to true.
   * \param[in] load_and_initialize_components boolean argument indicating whether to load and
   * initialize the components present in the parsed URDF. Defaults to true.
   */
  void load_urdf(
    const std::string & urdf, bool validate_interfaces = true,
    bool load_and_initialize_components = true);

  /**
   * @brief if the resource manager load_urdf(...) function has been called this returns true.
   * We want to permit to load the urdf later on but we currently don't want to permit multiple
   * calls to load_urdf (reloading/loading different urdf).
   *
   * @return true if resource manager's load_urdf() has been already called.
   * @return false if resource manager's load_urdf() has not been yet called.
   */
  bool is_urdf_already_loaded() const;

  /// Claim a state interface given its key.
  /**
   * The resource is claimed as long as being in scope.
   * Once the resource is going out of scope, the destructor
   * returns.
   *
   * \param[in] key String identifier which state interface to claim
   * \return state interface
   */
  LoanedStateInterface claim_state_interface(const std::string & key);

  /// Returns all registered state interfaces keys.
  /**
   * The keys are collected from each loaded hardware component.
   * \return Vector of strings, containing all registered keys.
   */
  std::vector<std::string> state_interface_keys() const;

  /// Returns all available state interfaces keys.
  /**
   * The keys are collected from the available list.
   * \return Vector of strings, containing all available state interface names.
   */
  std::vector<std::string> available_state_interfaces() const;

  /// Checks whether a state interface is available under the given key.
  /**
   * \return true if interface is available, false otherwise.
   */
  bool state_interface_is_available(const std::string & name) const;

  /// Add controllers' reference interfaces to resource manager.
  /**
   * Interface for transferring management of reference interfaces to resource manager.
   * When chaining controllers, reference interfaces are used as command interface of preceding
   * controllers.
   * Therefore, they should be managed in the same way as command interface of hardware.
   *
   * \param[in] controller_name name of the controller which reference interfaces are imported.
   * \param[in] interfaces list of controller's reference interfaces as CommandInterfaces.
   */
  void import_controller_reference_interfaces(
    const std::string & controller_name, std::vector<CommandInterface> & interfaces);

  /// Get list of reference interface of a controller.
  /**
   * Returns lists of stored reference interfaces names for a controller.
   *
   * \param[in] controller_name for which list of reference interface names is returned.
   * \returns list of reference interface names.
   */
  std::vector<std::string> get_controller_reference_interface_names(
    const std::string & controller_name);

  /// Add controller's reference interface to available list.
  /**
   * Adds interfaces of a controller with given name to the available list. This method should be
   * called when a controller gets activated with chained mode turned on. That means, the
   * controller's reference interfaces can be used by another controller in chained architectures.
   *
   * \param[in] controller_name name of the controller which interfaces should become available.
   */
  void make_controller_reference_interfaces_available(const std::string & controller_name);

  /// Remove controller's reference interface to available list.
  /**
   * Removes interfaces of a controller with given name from the available list. This method should
   * be called when a controller gets deactivated and its reference interfaces cannot be used by
   * another controller anymore.
   *
   * \param[in] controller_name name of the controller which interfaces should become unavailable.
   */
  void make_controller_reference_interfaces_unavailable(const std::string & controller_name);

  /// Remove controllers reference interfaces from resource manager.
  /**
   * Remove reference interfaces from resource manager, i.e., resource storage.
   * The interfaces will be deleted from all internal maps and lists.
   *
   * \param[in] controller_name list of interface names that will be deleted from resource manager.
   */
  void remove_controller_reference_interfaces(const std::string & controller_name);

  /// Cache mapping between hardware and controllers using it
  /**
   * Find mapping between controller and hardware based on interfaces controller with
   * \p controller_name is using and cache those for later usage.
   *
   * \param[in] controller_name name of the controller which interfaces are provided.
   * \param[in] interfaces list of interfaces controller with \p controller_name is using.
   */
  void cache_controller_to_hardware(
    const std::string & controller_name, const std::vector<std::string> & interfaces);

  /// Return cached controllers for a specific hardware.
  /**
   * Return list of cached controller names that use the hardware with name \p hardware_name.
   *
   * \param[in] hardware_name the name of the hardware for which cached controllers should be
   * returned. \returns list of cached controller names that depend on hardware with name \p
   * hardware_name.
   */
  std::vector<std::string> get_cached_controllers_to_hardware(const std::string & hardware_name);

  /// Checks whether a command interface is already claimed.
  /**
   * Any command interface can only be claimed by a single instance.
   * \note the equivalent function does not exist for state interfaces.
   * These are solely read-only and can thus be used by multiple instances.
   *
   * \param[in] key string identifying the interface to check.
   * \return true if interface is already claimed, false if available.
   */
  bool command_interface_is_claimed(const std::string & key) const;

  /// Claim a command interface given its key.
  /**
   * The resource is claimed as long as being in scope.
   * Once the resource is going out of scope, the destructor
   * returns and thus frees the resource to claimed by others.
   *
   * \param[in] key String identifier which command interface to claim
   * \return command interface
   */
  LoanedCommandInterface claim_command_interface(const std::string & key);

  /// Returns all registered command interfaces keys.
  /**
   * The keys are collected from each loaded hardware component.
   * \return vector of strings, containing all registered keys.
   */
  std::vector<std::string> command_interface_keys() const;

  /// Returns all available command interfaces keys.
  /**
   * The keys are collected from the available list.
   * \return vector of strings, containing all available command interface names.
   */
  std::vector<std::string> available_command_interfaces() const;

  /// Checks whether a command interface is available under the given name.
  /**
   * \param[in] name string identifying the interface to check.
   * \return true if interface is available, false otherwise.
   */
  bool command_interface_is_available(const std::string & interface) const;

  /// Return the number size_t of loaded actuator components.
  /**
   * \return number of actuator components.
   */
  size_t actuator_components_size() const;

  /// Return the number of loaded sensor components.
  /**
   * \return number of sensor components.
   */
  size_t sensor_components_size() const;

  /// Return the number of loaded system components.
  /**
   * \return size_t number of system components.
   */
  size_t system_components_size() const;

  /// Import a hardware component which is not listed in the URDF
  /**
   * Components which are initialized outside a URDF can be added post initialization.
   * Nevertheless, there should still be `HardwareInfo` available for this component,
   * either parsed from a URDF string (easiest) or filled manually.
   *
   * \note this might invalidate existing state and command interfaces and should thus
   * not be called when a controller is running.
   * \note given that no hardware_info is available, the component has to be configured
   * externally and prior to the call to import.
   * \param[in] actuator pointer to the actuator interface.
   * \param[in] hardware_info hardware info
   */
  void import_component(
    std::unique_ptr<ActuatorInterface> actuator, const HardwareInfo & hardware_info);

  /// Import a hardware component which is not listed in the URDF
  /**
   * Components which are initialized outside a URDF can be added post initialization.
   * Nevertheless, there should still be `HardwareInfo` available for this component,
   * either parsed from a URDF string (easiest) or filled manually.
   *
   * \note this might invalidate existing state and command interfaces and should thus
   * not be called when a controller is running.
   * \note given that no hardware_info is available, the component has to be configured
   * externally and prior to the call to import.
   * \param[in] sensor pointer to the sensor interface.
   * \param[in] hardware_info hardware info
   */
  void import_component(
    std::unique_ptr<SensorInterface> sensor, const HardwareInfo & hardware_info);

  /// Import a hardware component which is not listed in the URDF
  /**
   * Components which are initialized outside a URDF can be added post initialization.
   * Nevertheless, there should still be `HardwareInfo` available for this component,
   * either parsed from a URDF string (easiest) or filled manually.
   *
   * \note this might invalidate existing state and command interfaces and should thus
   * not be called when a controller is running.
   * \note given that no hardware_info is available, the component has to be configured
   * externally and prior to the call to import.
   * \param[in] system pointer to the system interface.
   * \param[in] hardware_info hardware info
   */
  void import_component(
    std::unique_ptr<SystemInterface> system, const HardwareInfo & hardware_info);

  /// Return status for all components.
  /**
   * \return map of hardware names and their status.
   */
  std::unordered_map<std::string, HardwareComponentInfo> get_components_status();

  /// Prepare the hardware components for a new command interface mode
  /**
   * Hardware components are asked to prepare a new command interface claim.
   *
   * \note this is intended for mode-switching when a hardware interface needs to change
   * control mode depending on which command interface is claimed.
   * \note this is for non-realtime preparing for and accepting new command resource
   * combinations.
   * \note accept_command_resource_claim is called on all actuators and system components
   * and hardware interfaces should return hardware_interface::return_type::OK
   * by default
   * \param[in] start_interfaces vector of string identifiers for the command interfaces starting.
   * \param[in] stop_interfaces vector of string identifiers for the command interfaces stopping.
   * \return true if switch can be prepared; false if a component rejects switch request, and if
   * at least one of the input interfaces are not existing or not available (i.e., component is not
   * in ACTIVE or INACTIVE state).
   */
  bool prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces);

  /// Notify the hardware components that realtime hardware mode switching should occur.
  /**
   * Hardware components are asked to perform the command interface mode switching.
   *
   * \note this is intended for mode-switching when a hardware interface needs to change
   * control mode depending on which command interface is claimed.
   * \note this is for realtime switching of the command interface.
   * \note it is assumed that `prepare_command_mode_switch` is called just before this method
   * with the same input arguments.
   * \param[in] start_interfaces vector of string identifiers for the command interfaces starting.
   * \param[in] stop_interfaces vector of string identifiers for the command interfaces stopping.
   * \return true if switch is performed, false if a component rejects switching.
   */
  bool perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces);

  /// Sets state of hardware component.
  /**
   * Set set of hardware component if possible.
   * Takes care of all transitions needed to reach the target state.
   * It implements the state machine from: https://design.ros2.org/articles/node_lifecycle.html
   *
   * The method is not part of the real-time critical update loop.
   *
   * \param[in] component_name component name to change state.
   * \param[in] target_state target state to set for a hardware component.
   * \return hardware_interface::retun_type::OK if component successfully switched its state and
   *         hardware_interface::return_type::ERROR any of state transitions has failed.
   */
  return_type set_component_state(
    const std::string & component_name, rclcpp_lifecycle::State & target_state);

  /// Reads all loaded hardware components.
  /**
   * Reads from all active hardware components.
   *
   * Part of the real-time critical update loop.
   * It is realtime-safe if used hardware interfaces are implemented adequately.
   */
  HardwareReadWriteStatus read(const rclcpp::Time & time, const rclcpp::Duration & period);

  /// Write all loaded hardware components.
  /**
   * Writes to all active hardware components.
   *
   * Part of the real-time critical update loop.
   * It is realtime-safe if used hardware interfaces are implemented adequately.
   */
  HardwareReadWriteStatus write(const rclcpp::Time & time, const rclcpp::Duration & period);

  /// Activates all available hardware components in the system.
  /**
   * All available hardware components int the ros2_control framework are activated.
   * This is used to preserve default behavior from previous versions where all hardware components
   * are activated per default.
   */
  [[deprecated(
    "The method 'activate_all_components' is deprecated. "
    "Use the new 'hardware_components_initial_state' parameter structure to setup the "
    "components")]] void
  activate_all_components();

  /// Checks whether a command interface is registered under the given key.
  /**
   * \param[in] key string identifying the interface to check.
   * \return true if interface exist, false otherwise.
   */
  bool command_interface_exists(const std::string & key) const;

  /// Checks whether a state interface is registered under the given key.
  /**
   * \return true if interface exist, false otherwise.
   */
  bool state_interface_exists(const std::string & key) const;

private:
  void validate_storage(const std::vector<hardware_interface::HardwareInfo> & hardware_info) const;

  void release_command_interface(const std::string & key);

  std::unordered_map<std::string, bool> claimed_command_interface_map_;

  mutable std::recursive_mutex resource_interfaces_lock_;
  mutable std::recursive_mutex claimed_command_interfaces_lock_;
  mutable std::recursive_mutex resources_lock_;

  std::unique_ptr<ResourceStorage> resource_storage_;

  // Structure to store read and write status so it is not initialized in the real-time loop
  HardwareReadWriteStatus read_write_status;

  bool is_urdf_loaded__ = false;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__RESOURCE_MANAGER_HPP_
