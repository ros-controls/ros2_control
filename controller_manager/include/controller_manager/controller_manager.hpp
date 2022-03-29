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
#include <tuple>
#include <unordered_map>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/controller_interface_base.hpp"

#include "controller_manager/controller_spec.hpp"
#include "controller_manager/visibility_control.h"
#include "controller_manager_msgs/srv/configure_controller.hpp"
#include "controller_manager_msgs/srv/configure_start_controller.hpp"
#include "controller_manager_msgs/srv/list_controller_types.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/list_hardware_components.hpp"
#include "controller_manager_msgs/srv/list_hardware_interfaces.hpp"
#include "controller_manager_msgs/srv/load_configure_controller.hpp"
#include "controller_manager_msgs/srv/load_controller.hpp"
#include "controller_manager_msgs/srv/load_start_controller.hpp"
#include "controller_manager_msgs/srv/reload_controller_libraries.hpp"
#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/srv/unload_controller.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/resource_manager.hpp"

#include "pluginlib/class_loader.hpp"

#include "rclcpp/executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/parameter.hpp"

namespace
{  // utility
/**
 * \brief Read chained controllers configuration parameters and figure out groups
 */
inline bool load_chained_controller_configuration(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_params,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & node_logging,
  std::vector<std::vector<std::string>> & chained_controllers_configuration)
{
  std::string param_prefix = "chained_controllers.";

  if (node_params->has_parameter(param_prefix + "parallel_group_0"))
  {
    RCLCPP_ERROR(
      node_logging->get_logger(),
      "'parallel_group' indexes in chained_controllers parameter start "
      "enumeration from 1 and not 0. Chained controller functionality will not be used.");
    chained_controllers_configuration.clear();
    return false;
  }

  for (size_t group_num = 1; group_num > chained_controllers_configuration.size(); ++group_num)
  {
    // Parameters in chain are prefixed with 'parallel_group_1'
    const std::string group_n = param_prefix + "parallel_group_" + std::to_string(group_num);

    // There is no need to declare parameters but just to check if they are there
    rclcpp::Parameter param_group_list;
    const bool got_group_list = node_params->get_parameter(group_n, param_group_list);

    if (!got_group_list)
    {
      RCLCPP_DEBUG(
        node_logging->get_logger(), "Parameter for parallel group '%zu' not found.", group_num);
      break;
    }

    RCLCPP_DEBUG(node_logging->get_logger(), "Found parameter for parallel group '%zu'", group_num);

    // Make sure that the parameter has correct type
    if (param_group_list.get_type() != rclcpp::PARAMETER_STRING_ARRAY)
    {
      RCLCPP_ERROR(
        node_logging->get_logger(),
        "%s must be a string array. "
        "Chained controller functionality will not be used.",
        group_n.c_str());
      chained_controllers_configuration.clear();
      return false;
    }

    std::vector<std::string> found_parallel_group =
      param_group_list.get_parameter_value().get<std::vector<std::string>>();

    // check if controller name is duplicated in any groups. It is not allowed to be.
    for (const auto & parallel_group : chained_controllers_configuration)
    {
      for (const auto & controller_in_group : parallel_group)
      {
        for (const auto & found_controller : found_parallel_group)
        {
          if (controller_in_group == found_controller)
          {
            RCLCPP_ERROR(
              node_logging->get_logger(),
              "%s controller is duplicated in the chained controller configuration. "
              "Chained controller functionality will not be used.",
              found_controller.c_str());
            chained_controllers_configuration.clear();
            return false;
          }
        }
      }
    }

    // Data are OK, we should add them to the configuration
    chained_controllers_configuration.push_back(found_parallel_group);
  }

  return true;
}

}  // namespace

namespace controller_manager
{
using ControllersListIterator = std::vector<controller_manager::ControllerSpec>::const_iterator;

class ControllerManager : public rclcpp::Node
{
public:
  static constexpr bool kWaitForAllResources = false;
  static constexpr auto kInfiniteTimeout = 0;

  CONTROLLER_MANAGER_PUBLIC
  ControllerManager(
    std::unique_ptr<hardware_interface::ResourceManager> resource_manager,
    std::shared_ptr<rclcpp::Executor> executor,
    const std::string & manager_node_name = "controller_manager",
    const std::string & namespace_ = "");

  CONTROLLER_MANAGER_PUBLIC
  ControllerManager(
    std::shared_ptr<rclcpp::Executor> executor,
    const std::string & manager_node_name = "controller_manager",
    const std::string & namespace_ = "");

  CONTROLLER_MANAGER_PUBLIC
  virtual ~ControllerManager() = default;

  CONTROLLER_MANAGER_PUBLIC
  void init_resource_manager(const std::string & robot_description);

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::ControllerInterfaceBaseSharedPtr load_controller(
    const std::string & controller_name, const std::string & controller_type);

  /// load_controller loads a controller by name, the type must be defined in the parameter server.
  /**
   * \param[in] controller_name as a string.
   * \return controller
   * \see Documentation in controller_manager_msgs/LoadController.srv
   */
  CONTROLLER_MANAGER_PUBLIC
  controller_interface::ControllerInterfaceBaseSharedPtr load_controller(
    const std::string & controller_name);

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type unload_controller(const std::string & controller_name);

  CONTROLLER_MANAGER_PUBLIC
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
    return add_controller_impl(controller_spec);
  }

  /// configure_controller Configure controller by name calling their "configure" method.
  /**
   * \param[in] controller_name as a string.
   * \return configure controller response
   * \see Documentation in controller_manager_msgs/ConfigureController.srv
   */
  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type configure_controller(const std::string & controller_name);

  /// switch_controller Stops some controllers and start others.
  /**
   * \param[in] start_controllers is a list of controllers to start
   * \param[in] stop_controllers is a list of controllers to stop
   * \param[in] set level of strictness (BEST_EFFORT or STRICT)
   * \see Documentation in controller_manager_msgs/SwitchController.srv
   */
  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type switch_controller(
    const std::vector<std::string> & start_controllers,
    const std::vector<std::string> & stop_controllers, int strictness,
    bool start_asap = kWaitForAllResources,
    const rclcpp::Duration & timeout = rclcpp::Duration::from_nanoseconds(kInfiniteTimeout));

  CONTROLLER_MANAGER_PUBLIC
  void read(const rclcpp::Time & time, const rclcpp::Duration & period);

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period);

  CONTROLLER_MANAGER_PUBLIC
  void write(const rclcpp::Time & time, const rclcpp::Duration & period);

  /// Deterministic (real-time safe) callback group, e.g., update function.
  /**
   * Deterministic (real-time safe) callback group for the update function. Default behavior
   * is read hardware, update controller and finally write new values to the hardware.
   */
  // TODO(anyone): Due to issues with the MutliThreadedExecutor, this control loop does not rely on
  // the executor (see issue #260).
  // rclcpp::CallbackGroup::SharedPtr deterministic_callback_group_;

  // Per controller update rate support
  CONTROLLER_MANAGER_PUBLIC
  unsigned int get_update_rate() const;

protected:
  CONTROLLER_MANAGER_PUBLIC
  void init_services();

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::ControllerInterfaceBaseSharedPtr add_controller_impl(
    const ControllerSpec & controller);

  CONTROLLER_MANAGER_PUBLIC
  void manage_switch();

  CONTROLLER_MANAGER_PUBLIC
  void stop_controllers();

  CONTROLLER_MANAGER_PUBLIC
  void switch_chained_mode();

  CONTROLLER_MANAGER_PUBLIC
  void start_controllers();

  CONTROLLER_MANAGER_PUBLIC
  void start_controllers_asap();

  CONTROLLER_MANAGER_PUBLIC
  void list_controllers_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListControllers::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void list_hardware_interfaces_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListHardwareInterfaces::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListHardwareInterfaces::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void load_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::LoadController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::LoadController::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void configure_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ConfigureController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ConfigureController::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void load_and_configure_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::LoadConfigureController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::LoadConfigureController::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void load_and_start_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::LoadStartController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::LoadStartController::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void configure_and_start_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ConfigureStartController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ConfigureStartController::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void reload_controller_libraries_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ReloadControllerLibraries::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ReloadControllerLibraries::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void switch_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::SwitchController::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void unload_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::UnloadController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::UnloadController::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void list_controller_types_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListControllerTypes::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListControllerTypes::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void list_hardware_components_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListHardwareComponents::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListHardwareComponents::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void set_hardware_component_state_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::SetHardwareComponentState::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::SetHardwareComponentState::Response> response);

  // Per controller update rate support
  unsigned int update_loop_counter_ = 0;
  unsigned int update_rate_ = 100;
  std::vector<std::vector<std::string>> chained_controllers_configuration_;

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager_;

private:
  std::vector<std::string> get_controller_names();

  std::shared_ptr<rclcpp::Executor> executor_;

  std::shared_ptr<pluginlib::ClassLoader<controller_interface::ControllerInterface>> loader_;
  std::shared_ptr<pluginlib::ClassLoader<controller_interface::ChainableControllerInterface>>
    chainable_loader_;

  /// Best effort (non real-time safe) callback group, e.g., service callbacks.
  /**
   * Best effort (non real-time safe) callback group for callbacks that can possibly break
   * real-time requirements, for example, service callbacks.
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
    /// update_and_get_used_by_rt_list Makes the "updated" list the "used by rt" list
    /**
     * \warning Should only be called by the RT thread, no one should modify the
     * updated list while it's being used
     * \return reference to the updated list
     */
    std::vector<ControllerSpec> & update_and_get_used_by_rt_list();

    /**
     * get_unused_list Waits until the "outdated" and "unused by rt"
     * lists match and returns a reference to it
     * This referenced list can be modified safely until switch_updated_controller_list()
     * is called, at this point the RT thread may start using it at any time
     * \param[in] guard Guard needed to make sure the caller is the only one accessing the unused by rt list
     */
    std::vector<ControllerSpec> & get_unused_list(
      const std::lock_guard<std::recursive_mutex> & guard);

    /// get_updated_list Returns a const reference to the most updated list.
    /**
     * \warning May or may not being used by the realtime thread, read-only reference for safety
     * \param[in] guard Guard needed to make sure the caller is the only one accessing the unused by rt list
     */
    const std::vector<ControllerSpec> & get_updated_list(
      const std::lock_guard<std::recursive_mutex> & guard) const;

    /**
     * switch_updated_list Switches the "updated" and "outdated" lists, and waits
     *  until the RT thread is using the new "updated" list.
     * \param[in] guard Guard needed to make sure the caller is the only one accessing the unused by rt list
     */
    void switch_updated_list(const std::lock_guard<std::recursive_mutex> & guard);

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
  };

  RTControllerListWrapper rt_controllers_wrapper_;
  /// mutex copied from ROS1 Control, protects service callbacks
  /// not needed if we're guaranteed that the callbacks don't come from multiple threads
  std::mutex services_lock_;
  rclcpp::Service<controller_manager_msgs::srv::ListControllers>::SharedPtr
    list_controllers_service_;
  rclcpp::Service<controller_manager_msgs::srv::ListControllerTypes>::SharedPtr
    list_controller_types_service_;
  rclcpp::Service<controller_manager_msgs::srv::LoadController>::SharedPtr load_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::ConfigureController>::SharedPtr
    configure_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::LoadConfigureController>::SharedPtr
    load_and_configure_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::LoadStartController>::SharedPtr
    load_and_start_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::ConfigureStartController>::SharedPtr
    configure_and_start_controller_service_;
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

  std::vector<std::string> start_request_, stop_request_;
  std::vector<std::string> to_chained_mode_request_, from_chained_mode_request_;
  std::vector<std::string> start_command_interface_request_, stop_command_interface_request_;

  struct SwitchParams
  {
    bool do_switch = {false};
    bool started = {false};
    rclcpp::Time init_time = {rclcpp::Time::max()};

    // Switch options
    int strictness = {0};
    bool start_asap = {false};
    rclcpp::Duration timeout = rclcpp::Duration{0, 0};
  };

  SwitchParams switch_params_;
};

}  // namespace controller_manager

#endif  // CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_
