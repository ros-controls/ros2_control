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

#ifndef CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_
#define CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/controller_interface_base.hpp"

#include "controller_manager/controller_spec.hpp"
#include "controller_manager_msgs/msg/controller_manager_activity.hpp"
#include "controller_manager_msgs/srv/configure_controller.hpp"
#include "controller_manager_msgs/srv/list_controller_types.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/list_hardware_components.hpp"
#include "controller_manager_msgs/srv/list_hardware_interfaces.hpp"
#include "controller_manager_msgs/srv/load_controller.hpp"
#include "controller_manager_msgs/srv/reload_controller_libraries.hpp"
#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/srv/unload_controller.hpp"

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "hardware_interface/resource_manager.hpp"

#include "pluginlib/class_loader.hpp"

#include "rclcpp/executor.hpp"
#include "rclcpp/node.hpp"
#include "std_msgs/msg/string.hpp"

namespace controller_manager
{
class ParamListener;
class Params;
using ControllersListIterator = std::vector<controller_manager::ControllerSpec>::const_iterator;

rclcpp::NodeOptions get_cm_node_options();

class ControllerManager : public rclcpp::Node
{
public:
  static constexpr bool kWaitForAllResources = false;
  static constexpr auto kInfiniteTimeout = 0;

  ControllerManager(
    std::unique_ptr<hardware_interface::ResourceManager> resource_manager,
    std::shared_ptr<rclcpp::Executor> executor,
    const std::string & manager_node_name = "controller_manager",
    const std::string & node_namespace = "",
    const rclcpp::NodeOptions & options = get_cm_node_options());

  ControllerManager(
    std::shared_ptr<rclcpp::Executor> executor,
    const std::string & manager_node_name = "controller_manager",
    const std::string & node_namespace = "",
    const rclcpp::NodeOptions & options = get_cm_node_options());

  ControllerManager(
    std::shared_ptr<rclcpp::Executor> executor, const std::string & urdf,
    bool activate_all_hw_components, const std::string & manager_node_name = "controller_manager",
    const std::string & node_namespace = "",
    const rclcpp::NodeOptions & options = get_cm_node_options());

  virtual ~ControllerManager();

  /**
 * @brief Shutdown all controllers in the controller manager.
 *
 * @return `true` if all controllers are successfully shut down, `false` otherwise.
 */

  bool shutdown_controllers();

  void robot_description_callback(const std_msgs::msg::String & msg);

  void init_resource_manager(const std::string & robot_description);

  controller_interface::ControllerInterfaceBaseSharedPtr load_controller(
    const std::string & controller_name, const std::string & controller_type);

 /**
 * @brief Load a controller by name.
 *
 * The controller type must be defined in the parameter server.
 *
 * @param[in] controller_name Name of the controller as a string.
 * @return Loaded controller.
 * @see Documentation in controller_manager_msgs/LoadController.srv
 */

  controller_interface::ControllerInterfaceBaseSharedPtr load_controller(
    const std::string & controller_name);

  controller_interface::return_type unload_controller(const std::string & controller_name);

  std::vector<ControllerSpec> get_loaded_controllers() const;

  template <
    typename T, typename std::enable_if<
                  std::is_convertible<T *, controller_interface::ControllerInterfaceBase *>::value,
                  T>::type * = nullptr>
  controller_interface::ControllerInterfaceBaseSharedPtr add_controller(
    std::shared_ptr<T> controller, const std::string & controller_name,
    const std::string & controller_type)
  {
    ControllerSpec controller_spec;
    controller_spec.c = controller;
    controller_spec.info.name = controller_name;
    controller_spec.info.type = controller_type;
    controller_spec.last_update_cycle_time = std::make_shared<rclcpp::Time>(0);
    return add_controller_impl(controller_spec);
  }

  controller_interface::ControllerInterfaceBaseSharedPtr add_controller(
    const ControllerSpec & controller_spec)
  {
    return add_controller_impl(controller_spec);
  }

  /**
 * @brief Configure a controller by name.
 *
 * Calls the "configure" method of the specified controller.
 *
 * @param[in] controller_name Name of the controller as a string.
 * @return Response of the configure controller operation.
 * @see Documentation in controller_manager_msgs/ConfigureController.srv
 */

  controller_interface::return_type configure_controller(const std::string & controller_name);

 /**
 * @brief Switch controllers by deactivating some and activating others.
 *
 * This method deactivates a list of specified controllers and activates another list of controllers.
 *
 * @param[in] activate_controllers List of controllers to activate.
 * @param[in] deactivate_controllers List of controllers to deactivate.
 * @param[in] strictness Level of strictness (BEST_EFFORT or STRICT).
 * @see Documentation in controller_manager_msgs/SwitchController.srv
 */

  controller_interface::return_type switch_controller(
    const std::vector<std::string> & activate_controllers,
    const std::vector<std::string> & deactivate_controllers, int strictness,
    bool activate_asap = kWaitForAllResources,
    const rclcpp::Duration & timeout = rclcpp::Duration::from_nanoseconds(kInfiniteTimeout));

/**
 * @brief Read current values from hardware to state interfaces.
 *
 * This method is called during the real-time control loop to update state interfaces with 
 * the latest values from the hardware.
 *
 * @param[in] time The time at the start of the current control loop iteration.
 * @param[in] period The measured duration of the last control loop iteration.
 */

  void read(const rclcpp::Time & time, const rclcpp::Duration & period);

 /**
 * @brief Execute update on all controllers.
 *
 * Calls the update method for all controllers during the real-time control loop.
 *
 * @param[in] time The time at the start of the current control loop iteration.
 * @param[in] period The measured duration of the last control loop iteration.
 */

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period);

  /**
 * @brief Write values from command interfaces to hardware.
 *
 * Transfers values from the command interfaces to the hardware during the real-time control loop.
 *
 * @param[in] time The time at the start of the current control loop iteration.
 * @param[in] period The measured duration of the last control loop iteration.
 */

  void write(const rclcpp::Time & time, const rclcpp::Duration & period);

 /**
 * @brief Deterministic (real-time safe) callback group for the update function.
 *
 * This callback group ensures real-time safe execution for the update function. The default 
 * behavior follows a structured sequence:
 * 1. Read values from the hardware.
 * 2. Update the controller.
 * 3. Write new values back to the hardware.
 */

  // TODO(anyone): Due to issues with the MutliThreadedExecutor, this control loop does not rely on
  // the executor (see issue #260).
  // rclcpp::CallbackGroup::SharedPtr deterministic_callback_group_;

 /**
 * @brief Checks if the Resource Manager is initialized.
 *
 * This method allows external components to verify whether all components 
 * in the Resource Manager have been successfully loaded and initialized.
 *
 * @return true if the Resource Manager is initialized, false otherwise.
 */

  bool is_resource_manager_initialized() const
  {
    return resource_manager_ && resource_manager_->are_components_initialized();
  }

 /**
 * @brief Gets the update rate of the main control loop in the controller manager.
 *
 * This method retrieves the update rate of the main control loop, which is 
 * used to support per-controller update rates.
 *
 * @return The update rate of the controller manager.
 */

  unsigned int get_update_rate() const;

protected:
  void init_services();

  controller_interface::ControllerInterfaceBaseSharedPtr add_controller_impl(
    const ControllerSpec & controller);

  void manage_switch();

  /**
 * @brief Deactivates selected controllers from the real-time controller list.
 *
 * This method deactivates controllers specified in \p controllers_to_deactivate 
 * from the given \p rt_controller_list. The controller list will be iterated as 
 * many times as there are controller names.
 *
 * @param[in] rt_controller_list List of controllers in the real-time system.
 * @param[in] controllers_to_deactivate Names of controllers to be deactivated.
 */

  void deactivate_controllers(
    const std::vector<ControllerSpec> & rt_controller_list,
    const std::vector<std::string> controllers_to_deactivate);

 /**
 * @brief Switches chained mode for all controllers based on activation state.
 *
 * This method updates the chained mode of controllers according to the following conditions:
 * - A preceding controller is being activated → switch the controller *to* chained mode.
 * - All preceding controllers are deactivated → switch the controller *from* chained mode.
 *
 * @param[in] chained_mode_switch_list List of controllers to switch chained mode.
 * @param[in] to_chained_mode Flag indicating whether to switch *to* (`true`) or *from* (`false`) chained mode.
 */

  void switch_chained_mode(
    const std::vector<std::string> & chained_mode_switch_list, bool to_chained_mode);

  /**
 * @brief Activates selected controllers from the real-time controller list.
 *
 * This method activates controllers specified in \p controllers_to_activate from the 
 * given \p rt_controller_list. The controller list will be iterated as many times as 
 * there are controller names to ensure proper activation.
 *
 * @param[in] rt_controller_list List of controllers in the real-time system.
 * @param[in] controllers_to_activate Names of the controllers that should be activated.
 */

  void activate_controllers(
    const std::vector<ControllerSpec> & rt_controller_list,
    const std::vector<std::string> controllers_to_activate);

 /**
 * @brief Activates the specified controllers from the real-time controller list.
 *
 * This method activates controllers listed in \p controllers_to_activate from the provided 
 * \p rt_controller_list. The controller list is iterated as many times as there are 
 * controller names to ensure activation.
 *
 * @note Currently, this method behaves identically to `activate_controllers`.
 *       See <https://github.com/ros-controls/ros2_control/issues/263> for details.
 *
 * @param[in] rt_controller_list List of controllers in the real-time system.
 * @param[in] controllers_to_activate Names of the controllers to activate.
 */

  void activate_controllers_asap(
    const std::vector<ControllerSpec> & rt_controller_list,
    const std::vector<std::string> controllers_to_activate);

  void list_controllers_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListControllers::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response> response);

  void list_hardware_interfaces_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListHardwareInterfaces::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListHardwareInterfaces::Response> response);

  void load_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::LoadController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::LoadController::Response> response);

  void configure_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ConfigureController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ConfigureController::Response> response);

  void reload_controller_libraries_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ReloadControllerLibraries::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ReloadControllerLibraries::Response> response);

  void switch_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::SwitchController::Response> response);

  void unload_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::UnloadController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::UnloadController::Response> response);

  void list_controller_types_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListControllerTypes::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListControllerTypes::Response> response);

  void list_hardware_components_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListHardwareComponents::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListHardwareComponents::Response> response);

  void set_hardware_component_state_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::SetHardwareComponentState::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::SetHardwareComponentState::Response> response);

  // Per controller update rate support
  unsigned int update_loop_counter_ = 0;
  unsigned int update_rate_;
  std::vector<std::vector<std::string>> chained_controllers_configuration_;

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager_;

private:
  std::vector<std::string> get_controller_names();
  std::pair<std::string, std::string> split_command_interface(
    const std::string & command_interface);
  void init_controller_manager();

  void initialize_parameters();

  /**
   * Call shutdown to change the given controller lifecycle node to the finalized state.
   *
   * \param[in] controller controller to be shutdown.
   */
  void shutdown_controller(controller_manager::ControllerSpec & controller) const;

  /**
   * Clear request lists used when switching controllers. The lists are shared between "callback"
   * and "control loop" threads.
   */
  void clear_requests();

  /**
   * If a controller is deactivated all following controllers (if any exist) should be switched
   * 'from' the chained mode.
   *
   * \param[in] controllers list with controllers.
   */
  void propagate_deactivation_of_chained_mode(const std::vector<ControllerSpec> & controllers);

  /**
 * @brief Checks the activation state and chained mode of subsequent controllers.
 *
 * Validates that all controllers following \p controller_it:
 * - Are already active,
 * - Will not be deactivated,
 * - Or will be activated.
 *
 * If necessary, subsequent controllers are added to or removed from the request 
 * to switch in/out of chained mode. The entire chain of controllers is verified.
 *
 * @note Automatic addition of subsequent controllers to the activation list 
 *       is not yet implemented.
 *
 * @param[in] controllers List of controllers.
 * @param[in] strictness If `"MANIPULATE_CONTROLLERS_CHAIN"`, subsequent controllers 
 *                       are automatically added to the activation request unless 
 *                       already in the deactivation request.
 * @param[in] controller_it Iterator to the controller whose subsequent controllers 
 *                          are being checked.
 *
 * @return `return_type::OK` if all checks pass, otherwise `return_type::ERROR`.
 */


  controller_interface::return_type check_following_controllers_for_activate(
    const std::vector<ControllerSpec> & controllers, int strictness,
    const ControllersListIterator controller_it);

  /**
 * @brief Validate that all preceding controllers will be inactive after a controllers' switch.
 *
 * This method checks whether all controllers preceding \p controller_it:
 * - Are currently inactive,
 * - Will be deactivated,
 * - And will not be activated.
 *
 * If any of these conditions are not met, the validation fails.
 *
 * @note Automatic addition of preceding controllers to the deactivation request list 
 *       is not yet implemented.
 *
 * @param[in] controllers List of controllers.
 * @param[in] strictness If set to `"MANIPULATE_CONTROLLERS_CHAIN"`, all preceding controllers 
 *                       will be automatically added to the deactivation request list.
 * @param[in] controller_it Iterator pointing to the controller whose preceding controllers 
 *                          are being checked.
 *
 * @return `return_type::OK` if all preceding controllers pass the checks, otherwise 
 *         `return_type::ERROR`.
 */

  controller_interface::return_type check_preceeding_controllers_for_deactivate(
    const std::vector<ControllerSpec> & controllers, int strictness,
    const ControllersListIterator controller_it);

  /// Checks if the fallback controllers of the given controllers are in the right
  /**
 * @brief Ensure that fallback controllers are in the correct state for immediate activation.
 *
 * This function checks whether all fallback controllers are in a state that allows 
 * them to be activated immediately.
 *
 * @param[in] controllers List of controllers to activate.
 * @param[in] controller_it Iterator pointing to the specific controller to be activated.
 *
 * @return `return_type::OK` if all fallback controllers are in the correct state, 
 *         otherwise `return_type::ERROR`.
 */

  controller_interface::return_type check_fallback_controllers_state_pre_activation(
    const std::vector<ControllerSpec> & controllers, const ControllersListIterator controller_it);

  /**
   * @brief Inserts a controller into an ordered list based on dependencies to compute the
   * controller chain.
   *
   * This method computes the controller chain by inserting the provided controller name into an
   * ordered list of controllers based on dependencies. It ensures that controllers are inserted in
   * the correct order so that dependencies are satisfied.
   *
   * @param ctrl_name The name of the controller to be inserted into the chain.
   * @param controller_iterator An iterator pointing to the position in the ordered list where the
   * controller should be inserted.
   * @param append_to_controller Flag indicating whether the controller should be appended or
   * prepended to the parsed iterator.
   * @note The specification of controller dependencies is in the ControllerChainSpec,
   * containing information about following and preceding controllers. This struct should include
   * the neighboring controllers with their relationships to the provided controller.
   * `following_controllers` specify controllers that come after the provided controller.
   * `preceding_controllers` specify controllers that come before the provided controller.
   */
  void update_list_with_controller_chain(
    const std::string & ctrl_name, std::vector<std::string>::iterator controller_iterator,
    bool append_to_controller);

  /**
   * @brief Method to publish the state of the controller manager.
   * The state includes the list of controllers and the list of hardware interfaces along with
   * their states.
   */
  void publish_activity();

  void controller_activity_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper & stat);

  void hardware_components_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper & stat);

  void controller_manager_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief determine_controller_node_options - A method that retrieves the controller defined node
   * options and adapts them, based on if there is a params file to be loaded or the use_sim_time
   * needs to be set
   * @param controller - controller info
   * @return The node options that will be set to the controller LifeCycleNode
   */
  rclcpp::NodeOptions determine_controller_node_options(const ControllerSpec & controller) const;

  /**
   * @brief cleanup_controller_exported_interfaces - A method that cleans up the exported interfaces
   * of a chainable controller
   * @param controller - controller info
   */
  void cleanup_controller_exported_interfaces(const ControllerSpec & controller);

  std::shared_ptr<controller_manager::ParamListener> cm_param_listener_;
  std::shared_ptr<controller_manager::Params> params_;
  diagnostic_updater::Updater diagnostics_updater_;

  std::shared_ptr<rclcpp::Executor> executor_;

  std::shared_ptr<pluginlib::ClassLoader<controller_interface::ControllerInterface>> loader_;
  std::shared_ptr<pluginlib::ClassLoader<controller_interface::ChainableControllerInterface>>
    chainable_loader_;

  /**
 * @brief Best-effort (non real-time safe) callback group.
 *
 * This callback group is intended for operations that do not meet real-time 
 * constraints. It is suitable for tasks that may introduce delays or 
 * unpredictability, such as service callbacks.
 */

  rclcpp::CallbackGroup::SharedPtr best_effort_callback_group_;

  /**
   * The RTControllerListWrapper class wraps a double-buffered list of controllers
   * to avoid needing to lock the real-time thread when switching controllers in
   * the non-real-time thread.
   *
   * There's always an "updated" list and an "outdated" one
   * There's always an "used by rt" list and an "unused by rt" list
   *
   * The updated state changes on the switch_updated_list()
   * The rt usage state changes on the update_and_get_used_by_rt_list()
   */
  class RTControllerListWrapper
  {
    // *INDENT-OFF*
  public:
    // *INDENT-ON*
    /**
 * @brief Updates and retrieves the real-time usage list.
 *
 * This function replaces the "used by RT" list with the "updated" list.
 *
 * @warning This function should only be called by the real-time (RT) thread.
 * Modifying the updated list while it is being used may lead to unpredictable behavior.
 *
 * @return Reference to the updated list.
 */

    std::vector<ControllerSpec> & update_and_get_used_by_rt_list();

    /**
     * get_unused_list Waits until the "outdated" and "unused by rt"
     * lists match and returns a reference to it
     * This referenced list can be modified safely until switch_updated_controller_list()
     * is called, at this point the RT thread may start using it at any time
     * \param[in] guard Guard needed to make sure the caller is the only one accessing the unused by
     * rt list
     */
    std::vector<ControllerSpec> & get_unused_list(
      const std::lock_guard<std::recursive_mutex> & guard);

    /**
 * @brief Retrieves a constant reference to the most recently updated list.
 *
 * This function returns a read-only reference to the latest updated list.
 *
 * @warning The list may or may not be currently used by the real-time (RT) thread.
 * Ensure that it is only accesse

    const std::vector<ControllerSpec> & get_updated_list(
      const std::lock_guard<std::recursive_mutex> & guard) const;

    /**
     * switch_updated_list Switches the "updated" and "outdated" lists, and waits
     *  until the RT thread is using the new "updated" list.
     * \param[in] guard Guard needed to make sure the caller is the only one accessing the unused by
     * rt list
     */
    void switch_updated_list(const std::lock_guard<std::recursive_mutex> & guard);

    /// A method to register a callback to be called when the list is switched
    /**
     * \param[in] callback Callback to be called when the list is switched
     */
    void set_on_switch_callback(std::function<void()> callback);

    // Mutex protecting the controllers list
    // must be acquired before using any list other than the "used by rt"
    mutable std::recursive_mutex controllers_lock_;

    // *INDENT-OFF*
  private:
    // *INDENT-ON*
    /// get_other_list get the list not pointed by index
    /**
     * \param[in] index int
     */
    int get_other_list(int index) const;

    void wait_until_rt_not_using(
      int index, std::chrono::microseconds sleep_delay = std::chrono::microseconds(200)) const;

    std::vector<ControllerSpec> controllers_lists_[2];
    /// The index of the controller list with the most updated information
    int updated_controllers_index_ = 0;
    /// The index of the controllers list being used in the real-time thread.
    int used_by_realtime_controllers_index_ = -1;
    /// The callback to be called when the list is switched
    std::function<void()> on_switch_callback_ = nullptr;
  };

  std::unique_ptr<rclcpp::PreShutdownCallbackHandle> preshutdown_cb_handle_{nullptr};
  RTControllerListWrapper rt_controllers_wrapper_;
  std::unordered_map<std::string, ControllerChainSpec> controller_chain_spec_;
  std::vector<std::string> ordered_controllers_names_;
  /// mutex copied from ROS1 Control, protects service callbacks
  /// not needed if we're guaranteed that the callbacks don't come from multiple threads
  std::mutex services_lock_;
  rclcpp::Publisher<controller_manager_msgs::msg::ControllerManagerActivity>::SharedPtr
    controller_manager_activity_publisher_;
  rclcpp::Service<controller_manager_msgs::srv::ListControllers>::SharedPtr
    list_controllers_service_;
  rclcpp::Service<controller_manager_msgs::srv::ListControllerTypes>::SharedPtr
    list_controller_types_service_;
  rclcpp::Service<controller_manager_msgs::srv::LoadController>::SharedPtr load_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::ConfigureController>::SharedPtr
    configure_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::ReloadControllerLibraries>::SharedPtr
    reload_controller_libraries_service_;
  rclcpp::Service<controller_manager_msgs::srv::SwitchController>::SharedPtr
    switch_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::UnloadController>::SharedPtr
    unload_controller_service_;

  rclcpp::Service<controller_manager_msgs::srv::ListHardwareComponents>::SharedPtr
    list_hardware_components_service_;
  rclcpp::Service<controller_manager_msgs::srv::ListHardwareInterfaces>::SharedPtr
    list_hardware_interfaces_service_;
  rclcpp::Service<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr
    set_hardware_component_state_service_;

  std::vector<std::string> activate_request_, deactivate_request_;
  std::vector<std::string> to_chained_mode_request_, from_chained_mode_request_;
  std::vector<std::string> activate_command_interface_request_,
    deactivate_command_interface_request_;

  std::map<std::string, std::vector<std::string>> controller_chained_reference_interfaces_cache_;
  std::map<std::string, std::vector<std::string>> controller_chained_state_interfaces_cache_;

  rclcpp::NodeOptions cm_node_options_;
  std::string robot_description_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;
  rclcpp::TimerBase::SharedPtr robot_description_notification_timer_;

  controller_manager::MovingAverageStatistics periodicity_stats_;

  struct SwitchParams
  {
    void reset()
    {
      do_switch = false;
      started = false;
      strictness = 0;
      activate_asap = false;
    }

    bool do_switch;
    bool started;

    // Switch options
    int strictness;
    bool activate_asap;
    std::chrono::nanoseconds timeout;

    // conditional variable and mutex to wait for the switch to complete
    std::condition_variable cv;
    std::mutex mutex;
  };

  SwitchParams switch_params_;

  struct RTBufferVariables
  {
    RTBufferVariables()
    {
      deactivate_controllers_list.reserve(1000);
      activate_controllers_using_interfaces_list.reserve(1000);
      fallback_controllers_list.reserve(1000);
      interfaces_to_start.reserve(1000);
      interfaces_to_stop.reserve(1000);
      concatenated_string.reserve(5000);
    }

    const std::string & get_concatenated_string(
      const std::vector<std::string> & strings, bool clear_string = true)
    {
      if (clear_string)
      {
        concatenated_string.clear();
      }
      for (const auto & str : strings)
      {
        concatenated_string.append(str);
        concatenated_string.append(" ");
      }
      return concatenated_string;
    }

    std::vector<std::string> deactivate_controllers_list;
    std::vector<std::string> activate_controllers_using_interfaces_list;
    std::vector<std::string> fallback_controllers_list;
    std::vector<std::string> interfaces_to_start;
    std::vector<std::string> interfaces_to_stop;
    std::string concatenated_string;
  };
  RTBufferVariables rt_buffer_;
};

}  // namespace controller_manager

#endif  // CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_
