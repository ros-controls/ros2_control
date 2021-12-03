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

#include "controller_manager/controller_manager.hpp"

#include <list>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace controller_manager
{
static constexpr const char * kControllerInterfaceName = "controller_interface";
static constexpr const char * kControllerInterface = "controller_interface::ControllerInterface";

inline bool is_controller_inactive(const controller_interface::ControllerInterface & controller)
{
  return controller.get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
}

inline bool is_controller_active(controller_interface::ControllerInterface & controller)
{
  return controller.get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

bool controller_name_compare(const ControllerSpec & a, const std::string & name)
{
  return a.info.name == name;
}

rclcpp::NodeOptions get_cm_node_options()
{
  rclcpp::NodeOptions node_options;
  // Required for getting types of controllers to be loaded via service call
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  return node_options;
}

ControllerManager::ControllerManager(
  std::shared_ptr<rclcpp::Executor> executor, const std::string & manager_node_name,
  const std::string & namespace_)
: rclcpp::Node(manager_node_name, namespace_, get_cm_node_options()),
  resource_manager_(std::make_unique<hardware_interface::ResourceManager>()),
  executor_(executor),
  loader_(std::make_shared<pluginlib::ClassLoader<controller_interface::ControllerInterface>>(
    kControllerInterfaceName, kControllerInterface))
{
  if (!get_parameter("update_rate", update_rate_))
  {
    RCLCPP_WARN(get_logger(), "'update_rate' parameter not set, using default value.");
  }

  std::string robot_description = "";
  get_parameter("robot_description", robot_description);
  if (robot_description.empty())
  {
    throw std::runtime_error("Unable to initialize resource manager, no robot description found.");
  }

  resource_manager_->load_urdf(robot_description);

  // TODO(all): Here we should start only "auto-start" resources
  resource_manager_->start_components();

  init_services();
}

ControllerManager::ControllerManager(
  std::unique_ptr<hardware_interface::ResourceManager> resource_manager,
  std::shared_ptr<rclcpp::Executor> executor, const std::string & manager_node_name,
  const std::string & namespace_)
: rclcpp::Node(manager_node_name, namespace_, get_cm_node_options()),
  resource_manager_(std::move(resource_manager)),
  executor_(executor),
  loader_(std::make_shared<pluginlib::ClassLoader<controller_interface::ControllerInterface>>(
    kControllerInterfaceName, kControllerInterface))
{
  resource_manager_->start_components();

  init_services();
}

void ControllerManager::init_services()
{
  // TODO(anyone): Due to issues with the MutliThreadedExecutor, this control loop does not rely on
  // the executor (see issue #260).
  // deterministic_callback_group_ = create_callback_group(
  //   rclcpp::CallbackGroupType::MutuallyExclusive);
  best_effort_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  using namespace std::placeholders;
  list_controllers_service_ = create_service<controller_manager_msgs::srv::ListControllers>(
    "~/list_controllers", std::bind(&ControllerManager::list_controllers_srv_cb, this, _1, _2),
    rmw_qos_profile_services_default, best_effort_callback_group_);
  list_controller_types_service_ =
    create_service<controller_manager_msgs::srv::ListControllerTypes>(
      "~/list_controller_types",
      std::bind(&ControllerManager::list_controller_types_srv_cb, this, _1, _2),
      rmw_qos_profile_services_default, best_effort_callback_group_);
  load_controller_service_ = create_service<controller_manager_msgs::srv::LoadController>(
    "~/load_controller", std::bind(&ControllerManager::load_controller_service_cb, this, _1, _2),
    rmw_qos_profile_services_default, best_effort_callback_group_);
  configure_controller_service_ = create_service<controller_manager_msgs::srv::ConfigureController>(
    "~/configure_controller",
    std::bind(&ControllerManager::configure_controller_service_cb, this, _1, _2),
    rmw_qos_profile_services_default, best_effort_callback_group_);
  load_and_configure_controller_service_ =
    create_service<controller_manager_msgs::srv::LoadConfigureController>(
      "~/load_and_configure_controller",
      std::bind(&ControllerManager::load_and_configure_controller_service_cb, this, _1, _2),
      rmw_qos_profile_services_default, best_effort_callback_group_);
  load_and_start_controller_service_ =
    create_service<controller_manager_msgs::srv::LoadStartController>(
      "~/load_and_start_controller",
      std::bind(&ControllerManager::load_and_start_controller_service_cb, this, _1, _2),
      rmw_qos_profile_services_default, best_effort_callback_group_);
  configure_and_start_controller_service_ =
    create_service<controller_manager_msgs::srv::ConfigureStartController>(
      "~/configure_and_start_controller",
      std::bind(&ControllerManager::configure_and_start_controller_service_cb, this, _1, _2),
      rmw_qos_profile_services_default, best_effort_callback_group_);
  reload_controller_libraries_service_ =
    create_service<controller_manager_msgs::srv::ReloadControllerLibraries>(
      "~/reload_controller_libraries",
      std::bind(&ControllerManager::reload_controller_libraries_service_cb, this, _1, _2),
      rmw_qos_profile_services_default, best_effort_callback_group_);
  switch_controller_service_ = create_service<controller_manager_msgs::srv::SwitchController>(
    "~/switch_controller",
    std::bind(&ControllerManager::switch_controller_service_cb, this, _1, _2),
    rmw_qos_profile_services_default, best_effort_callback_group_);
  unload_controller_service_ = create_service<controller_manager_msgs::srv::UnloadController>(
    "~/unload_controller",
    std::bind(&ControllerManager::unload_controller_service_cb, this, _1, _2),
    rmw_qos_profile_services_default, best_effort_callback_group_);
  list_hardware_components_service_ =
    create_service<controller_manager_msgs::srv::ListHardwareComponents>(
      "~/list_hardware_components",
      std::bind(&ControllerManager::list_hardware_components_srv_cb, this, _1, _2),
      rmw_qos_profile_services_default, best_effort_callback_group_);
  list_hardware_interfaces_service_ =
    create_service<controller_manager_msgs::srv::ListHardwareInterfaces>(
      "~/list_hardware_interfaces",
      std::bind(&ControllerManager::list_hardware_interfaces_srv_cb, this, _1, _2),
      rmw_qos_profile_services_default, best_effort_callback_group_);
  set_hardware_component_state_service_ =
    create_service<controller_manager_msgs::srv::SetHardwareComponentState>(
      "~/set_hardware_component_state",
      std::bind(&ControllerManager::set_hardware_component_state_srv_cb, this, _1, _2),
      rmw_qos_profile_services_default, best_effort_callback_group_);
}

controller_interface::ControllerInterfaceSharedPtr ControllerManager::load_controller(
  const std::string & controller_name, const std::string & controller_type)
{
  RCLCPP_INFO(get_logger(), "Loading controller '%s'", controller_name.c_str());

  if (!loader_->isClassAvailable(controller_type))
  {
    RCLCPP_ERROR(get_logger(), "Loader for controller '%s' not found.", controller_name.c_str());
    RCLCPP_INFO(get_logger(), "Available classes:");
    for (const auto & available_class : loader_->getDeclaredClasses())
    {
      RCLCPP_INFO(get_logger(), "  %s", available_class.c_str());
    }
    return nullptr;
  }

  auto controller = loader_->createSharedInstance(controller_type);
  ControllerSpec controller_spec;
  controller_spec.c = controller;
  controller_spec.info.name = controller_name;
  controller_spec.info.type = controller_type;

  return add_controller_impl(controller_spec);
}

controller_interface::ControllerInterfaceSharedPtr ControllerManager::load_controller(
  const std::string & controller_name)
{
  const std::string param_name = controller_name + ".type";
  std::string controller_type;

  // We cannot declare the parameters for the controllers that will be loaded in the future,
  // because they are plugins and we cannot be aware of all of them.
  // So when we're told to load a controller by name, we need to declare the parameter if
  // we haven't done so, and then read it.

  // Check if parameter has been declared
  if (!has_parameter(param_name))
  {
    declare_parameter(param_name, rclcpp::ParameterType::PARAMETER_STRING);
  }
  if (!get_parameter(param_name, controller_type))
  {
    RCLCPP_ERROR(
      get_logger(), "The 'type' param was not defined for '%s'.", controller_name.c_str());
    return nullptr;
  }
  return load_controller(controller_name, controller_type);
}

controller_interface::return_type ControllerManager::unload_controller(
  const std::string & controller_name)
{
  std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);
  std::vector<ControllerSpec> & to = rt_controllers_wrapper_.get_unused_list(guard);
  const std::vector<ControllerSpec> & from = rt_controllers_wrapper_.get_updated_list(guard);

  // Transfers the active controllers over, skipping the one to be removed and the active ones.
  to = from;

  auto found_it = std::find_if(
    to.begin(), to.end(),
    std::bind(controller_name_compare, std::placeholders::_1, controller_name));
  if (found_it == to.end())
  {
    // Fails if we could not remove the controllers
    to.clear();
    RCLCPP_ERROR(
      get_logger(),
      "Could not unload controller with name '%s' because no controller with this name exists",
      controller_name.c_str());
    return controller_interface::return_type::ERROR;
  }

  auto & controller = *found_it;

  if (is_controller_active(*controller.c))
  {
    to.clear();
    RCLCPP_ERROR(
      get_logger(), "Could not unload controller with name '%s' because it is still active",
      controller_name.c_str());
    return controller_interface::return_type::ERROR;
  }

  RCLCPP_DEBUG(get_logger(), "Cleanup controller");
  controller.c->cleanup();
  executor_->remove_node(controller.c->get_node());
  to.erase(found_it);

  // Destroys the old controllers list when the realtime thread is finished with it.
  RCLCPP_DEBUG(get_logger(), "Realtime switches over to new controller list");
  rt_controllers_wrapper_.switch_updated_list(guard);
  std::vector<ControllerSpec> & new_unused_list = rt_controllers_wrapper_.get_unused_list(guard);
  RCLCPP_DEBUG(get_logger(), "Destruct controller");
  new_unused_list.clear();
  RCLCPP_DEBUG(get_logger(), "Destruct controller finished");

  RCLCPP_DEBUG(get_logger(), "Successfully unloaded controller '%s'", controller_name.c_str());
  return controller_interface::return_type::OK;
}

std::vector<ControllerSpec> ControllerManager::get_loaded_controllers() const
{
  std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);
  return rt_controllers_wrapper_.get_updated_list(guard);
}

controller_interface::return_type ControllerManager::configure_controller(
  const std::string & controller_name)
{
  RCLCPP_INFO(get_logger(), "Configuring controller '%s'", controller_name.c_str());

  const auto & controllers = get_loaded_controllers();

  auto found_it = std::find_if(
    controllers.begin(), controllers.end(),
    std::bind(controller_name_compare, std::placeholders::_1, controller_name));

  if (found_it == controllers.end())
  {
    RCLCPP_ERROR(
      get_logger(),
      "Could not configure controller with name '%s' because no controller with this name exists",
      controller_name.c_str());
    return controller_interface::return_type::ERROR;
  }
  auto controller = found_it->c;

  auto state = controller->get_state();
  if (
    state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE ||
    state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)
  {
    RCLCPP_ERROR(
      get_logger(), "Controller '%s' can not be configured from '%s' state.",
      controller_name.c_str(), state.label().c_str());
    return controller_interface::return_type::ERROR;
  }

  auto new_state = controller->get_state();
  if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_DEBUG(
      get_logger(), "Controller '%s' is cleaned-up before configuring", controller_name.c_str());
    new_state = controller->cleanup();
    if (new_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
    {
      RCLCPP_ERROR(
        get_logger(), "Controller '%s' can not be cleaned-up before configuring",
        controller_name.c_str());
      return controller_interface::return_type::ERROR;
    }
  }

  new_state = controller->configure();
  if (new_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_ERROR(
      get_logger(), "After configuring, controller '%s' is in state '%s' , expected inactive.",
      controller_name.c_str(), new_state.label().c_str());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type ControllerManager::switch_controller(
  const std::vector<std::string> & start_controllers,
  const std::vector<std::string> & stop_controllers, int strictness, bool start_asap,
  const rclcpp::Duration & timeout)
{
  switch_params_ = SwitchParams();

  if (!stop_request_.empty() || !start_request_.empty())
  {
    RCLCPP_FATAL(
      get_logger(),
      "The internal stop and start request lists are not empty at the beginning of the "
      "switchController() call. This should not happen.");
  }
  if (!stop_command_interface_request_.empty() || !start_command_interface_request_.empty())
  {
    RCLCPP_FATAL(
      get_logger(),
      "The internal stop and start requests command interface lists are not empty at the "
      "switch_controller() call. This should not happen.");
  }
  if (strictness == 0)
  {
    RCLCPP_WARN(
      get_logger(),
      "Controller Manager: to switch controllers you need to specify a "
      "strictness level of controller_manager_msgs::SwitchController::STRICT "
      "(%d) or ::BEST_EFFORT (%d). Defaulting to ::BEST_EFFORT",
      controller_manager_msgs::srv::SwitchController::Request::STRICT,
      controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT);
    strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  }

  RCLCPP_DEBUG(get_logger(), "Switching controllers:");
  for (const auto & controller : start_controllers)
  {
    RCLCPP_DEBUG(get_logger(), "- Starting controller '%s'", controller.c_str());
  }
  for (const auto & controller : stop_controllers)
  {
    RCLCPP_DEBUG(get_logger(), "- Stopping controller '%s'", controller.c_str());
  }

  const auto list_controllers = [this, strictness](
                                  const std::vector<std::string> & controller_list,
                                  std::vector<std::string> & request_list,
                                  const std::string & action) {
    // lock controllers
    std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);

    // list all controllers to stop/start
    for (const auto & controller : controller_list)
    {
      const auto & updated_controllers = rt_controllers_wrapper_.get_updated_list(guard);

      auto found_it = std::find_if(
        updated_controllers.begin(), updated_controllers.end(),
        std::bind(controller_name_compare, std::placeholders::_1, controller));

      if (found_it == updated_controllers.end())
      {
        if (strictness == controller_manager_msgs::srv::SwitchController::Request::STRICT)
        {
          RCLCPP_ERROR(
            get_logger(),
            R"(Could not '%s' controller with name '%s' because
                no controller with this name exists)",
            action.c_str(), controller.c_str());
          return controller_interface::return_type::ERROR;
        }
        RCLCPP_WARN(
          get_logger(),
          "Could not '%s' controller with name '%s' because no controller with this name exists",
          action.c_str(), controller.c_str());
      }
      else
      {
        RCLCPP_DEBUG(
          get_logger(), "Found controller '%s' that needs to be %sed in list of controllers",
          controller.c_str(), action.c_str());
        request_list.push_back(controller);
      }
    }
    RCLCPP_DEBUG(
      get_logger(), "'%s' request vector has size %i", action.c_str(), (int)request_list.size());

    return controller_interface::return_type::OK;
  };

  // list all controllers to stop
  auto ret = list_controllers(stop_controllers, stop_request_, "stop");
  if (ret != controller_interface::return_type::OK)
  {
    stop_request_.clear();
    return ret;
  }

  // list all controllers to start
  ret = list_controllers(start_controllers, start_request_, "start");
  if (ret != controller_interface::return_type::OK)
  {
    stop_request_.clear();
    start_request_.clear();
    return ret;
  }

  const auto list_interfaces = [this](
                                 const ControllerSpec controller,
                                 std::vector<std::string> & request_interface_list) {
    auto command_interface_config = controller.c->command_interface_configuration();
    std::vector<std::string> command_interface_names = {};
    if (command_interface_config.type == controller_interface::interface_configuration_type::ALL)
    {
      command_interface_names = resource_manager_->available_command_interfaces();
    }
    if (
      command_interface_config.type ==
      controller_interface::interface_configuration_type::INDIVIDUAL)
    {
      command_interface_names = command_interface_config.names;
    }
    request_interface_list.insert(
      request_interface_list.end(), command_interface_names.begin(), command_interface_names.end());
  };

  // lock controllers
  std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);

  const std::vector<ControllerSpec> & controllers = rt_controllers_wrapper_.get_updated_list(guard);

  for (const auto & controller : controllers)
  {
    auto stop_list_it = std::find(stop_request_.begin(), stop_request_.end(), controller.info.name);
    bool in_stop_list = stop_list_it != stop_request_.end();

    auto start_list_it =
      std::find(start_request_.begin(), start_request_.end(), controller.info.name);
    bool in_start_list = start_list_it != start_request_.end();

    const bool is_active = is_controller_active(*controller.c);
    const bool is_inactive = is_controller_inactive(*controller.c);

    auto handle_conflict = [&](const std::string & msg) {
      if (strictness == controller_manager_msgs::srv::SwitchController::Request::STRICT)
      {
        RCLCPP_ERROR(get_logger(), "%s", msg.c_str());
        stop_request_.clear();
        stop_command_interface_request_.clear();
        start_request_.clear();
        start_command_interface_request_.clear();
        return controller_interface::return_type::ERROR;
      }
      RCLCPP_WARN(get_logger(), "%s", msg.c_str());
      return controller_interface::return_type::OK;
    };

    // check for double stop
    if (!is_active && in_stop_list)
    {
      auto ret = handle_conflict(
        "Could not stop controller '" + controller.info.name + "' since it is not active");
      if (ret != controller_interface::return_type::OK)
      {
        return ret;
      }
      in_stop_list = false;
      stop_request_.erase(stop_list_it);
    }

    // check for doubled start
    if (is_active && !in_stop_list && in_start_list)
    {
      auto ret = handle_conflict(
        "Could not start controller '" + controller.info.name + "' since it is already active");
      if (ret != controller_interface::return_type::OK)
      {
        return ret;
      }
      in_start_list = false;
      start_request_.erase(start_list_it);
    }

    // check for illegal start of an unconfigured/finalized controller
    if (!is_inactive && !in_stop_list && in_start_list)
    {
      auto ret = handle_conflict(
        "Could not start controller '" + controller.info.name +
        "' since it is not in inactive state");
      if (ret != controller_interface::return_type::OK)
      {
        return ret;
      }
      in_start_list = false;
      start_request_.erase(start_list_it);
    }

    if (in_start_list)
    {
      list_interfaces(controller, start_command_interface_request_);
    }
    if (in_stop_list)
    {
      list_interfaces(controller, stop_command_interface_request_);
    }
  }

  if (start_request_.empty() && stop_request_.empty())
  {
    RCLCPP_INFO(get_logger(), "Empty start and stop list, not requesting switch");
    start_command_interface_request_.clear();
    stop_command_interface_request_.clear();
    return controller_interface::return_type::OK;
  }

  if (!start_command_interface_request_.empty() || !stop_command_interface_request_.empty())
  {
    if (!resource_manager_->prepare_command_mode_switch(
          start_command_interface_request_, stop_command_interface_request_))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Could not switch controllers since prepare command mode switch was rejected.");
      start_request_.clear();
      stop_request_.clear();
      start_command_interface_request_.clear();
      stop_command_interface_request_.clear();
      return controller_interface::return_type::ERROR;
    }
  }
  // start the atomic controller switching
  switch_params_.strictness = strictness;
  switch_params_.start_asap = start_asap;
  switch_params_.init_time = rclcpp::Clock().now();
  switch_params_.timeout = timeout;
  switch_params_.do_switch = true;

  // wait until switch is finished
  RCLCPP_DEBUG(get_logger(), "Request atomic controller switch from realtime loop");
  while (rclcpp::ok() && switch_params_.do_switch)
  {
    if (!rclcpp::ok())
    {
      return controller_interface::return_type::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  // copy the controllers spec from the used to the unused list
  std::vector<ControllerSpec> & to = rt_controllers_wrapper_.get_unused_list(guard);
  to = controllers;

  // update the claimed interface controller info
  for (auto & controller : to)
  {
    if (controller.c->get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      auto command_interface_config = controller.c->command_interface_configuration();
      if (command_interface_config.type == controller_interface::interface_configuration_type::ALL)
      {
        controller.info.claimed_interfaces = resource_manager_->available_command_interfaces();
      }
      if (
        command_interface_config.type ==
        controller_interface::interface_configuration_type::INDIVIDUAL)
      {
        controller.info.claimed_interfaces = command_interface_config.names;
      }
    }
    else
    {
      controller.info.claimed_interfaces.clear();
    }
  }
  // switch lists
  rt_controllers_wrapper_.switch_updated_list(guard);
  // clear unused list
  rt_controllers_wrapper_.get_unused_list(guard).clear();

  start_request_.clear();
  stop_request_.clear();

  start_command_interface_request_.clear();
  stop_command_interface_request_.clear();
  RCLCPP_DEBUG(get_logger(), "Successfully switched controllers");
  return controller_interface::return_type::OK;
}

controller_interface::ControllerInterfaceSharedPtr ControllerManager::add_controller_impl(
  const ControllerSpec & controller)
{
  // lock controllers
  std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);

  std::vector<ControllerSpec> & to = rt_controllers_wrapper_.get_unused_list(guard);
  const std::vector<ControllerSpec> & from = rt_controllers_wrapper_.get_updated_list(guard);

  // Copy all controllers from the 'from' list to the 'to' list
  to = from;

  auto found_it = std::find_if(
    to.begin(), to.end(),
    std::bind(controller_name_compare, std::placeholders::_1, controller.info.name));
  // Checks that we're not duplicating controllers
  if (found_it != to.end())
  {
    to.clear();
    RCLCPP_ERROR(
      get_logger(), "A controller named '%s' was already loaded inside the controller manager",
      controller.info.name.c_str());
    return nullptr;
  }

  if (controller.c->init(controller.info.name) == controller_interface::return_type::ERROR)
  {
    to.clear();
    RCLCPP_ERROR(
      get_logger(), "Could not initialize the controller named '%s'", controller.info.name.c_str());
    return nullptr;
  }

  // ensure controller's `use_sim_time` parameter matches controller_manager's
  const rclcpp::Parameter use_sim_time = this->get_parameter("use_sim_time");
  if (use_sim_time.as_bool())
  {
    RCLCPP_INFO(
      get_logger(),
      "Setting use_sim_time=True for %s to match controller manager "
      "(see ros2_control#325 for details)",
      controller.info.name.c_str());
    controller.c->get_node()->set_parameter(use_sim_time);
  }
  executor_->add_node(controller.c->get_node());
  to.emplace_back(controller);

  // Destroys the old controllers list when the realtime thread is finished with it.
  RCLCPP_DEBUG(get_logger(), "Realtime switches over to new controller list");
  rt_controllers_wrapper_.switch_updated_list(guard);
  RCLCPP_DEBUG(get_logger(), "Destruct controller");
  std::vector<ControllerSpec> & new_unused_list = rt_controllers_wrapper_.get_unused_list(guard);
  new_unused_list.clear();
  RCLCPP_DEBUG(get_logger(), "Destruct controller finished");

  return to.back().c;
}

void ControllerManager::manage_switch()
{
  // Ask hardware interfaces to change mode
  if (!resource_manager_->perform_command_mode_switch(
        start_command_interface_request_, stop_command_interface_request_))
  {
    RCLCPP_ERROR(get_logger(), "Error while performing mode switch.");
  }

  stop_controllers();

  // start controllers once the switch is fully complete
  if (!switch_params_.start_asap)
  {
    start_controllers();
  }
  else
  {
    // start controllers as soon as their required joints are done switching
    start_controllers_asap();
  }
}

void ControllerManager::stop_controllers()
{
  std::vector<ControllerSpec> & rt_controller_list =
    rt_controllers_wrapper_.update_and_get_used_by_rt_list();
  // stop controllers
  for (const auto & request : stop_request_)
  {
    auto found_it = std::find_if(
      rt_controller_list.begin(), rt_controller_list.end(),
      std::bind(controller_name_compare, std::placeholders::_1, request));
    if (found_it == rt_controller_list.end())
    {
      RCLCPP_ERROR(
        get_logger(),
        "Got request to stop controller '%s' but it is not in the realtime controller list",
        request.c_str());
      continue;
    }
    auto controller = found_it->c;
    if (is_controller_active(*controller))
    {
      const auto new_state = controller->deactivate();
      controller->release_interfaces();
      if (new_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      {
        RCLCPP_ERROR(
          get_logger(), "After deactivating, controller '%s' is in state '%s', expected Inactive",
          request.c_str(), new_state.label().c_str());
      }
    }
  }
}

void ControllerManager::start_controllers()
{
  std::vector<ControllerSpec> & rt_controller_list =
    rt_controllers_wrapper_.update_and_get_used_by_rt_list();
  for (const auto & request : start_request_)
  {
    auto found_it = std::find_if(
      rt_controller_list.begin(), rt_controller_list.end(),
      std::bind(controller_name_compare, std::placeholders::_1, request));
    if (found_it == rt_controller_list.end())
    {
      RCLCPP_ERROR(
        get_logger(),
        "Got request to start controller '%s' but it is not in the realtime controller list",
        request.c_str());
      continue;
    }
    auto controller = found_it->c;

    bool assignment_successful = true;
    // assign command interfaces to the controller
    auto command_interface_config = controller->command_interface_configuration();
    // default to controller_interface::configuration_type::NONE
    std::vector<std::string> command_interface_names = {};
    if (command_interface_config.type == controller_interface::interface_configuration_type::ALL)
    {
      command_interface_names = resource_manager_->available_command_interfaces();
    }
    if (
      command_interface_config.type ==
      controller_interface::interface_configuration_type::INDIVIDUAL)
    {
      command_interface_names = command_interface_config.names;
    }
    std::vector<hardware_interface::LoanedCommandInterface> command_loans;
    command_loans.reserve(command_interface_names.size());
    for (const auto & command_interface : command_interface_names)
    {
      if (resource_manager_->command_interface_is_claimed(command_interface))
      {
        RCLCPP_ERROR(
          get_logger(),
          "Resource conflict for controller '%s'. Command interface '%s' is already claimed.",
          request.c_str(), command_interface.c_str());
        assignment_successful = false;
        break;
      }
      try
      {
        command_loans.emplace_back(resource_manager_->claim_command_interface(command_interface));
      }
      catch (const std::exception & e)
      {
        RCLCPP_ERROR(get_logger(), "Can't activate controller '%s': %s", request.c_str(), e.what());
        assignment_successful = false;
        break;
      }
    }
    // something went wrong during command interfaces, go skip the controller
    if (!assignment_successful)
    {
      continue;
    }

    // assign state interfaces to the controller
    auto state_interface_config = controller->state_interface_configuration();
    // default to controller_interface::configuration_type::NONE
    std::vector<std::string> state_interface_names = {};
    if (state_interface_config.type == controller_interface::interface_configuration_type::ALL)
    {
      state_interface_names = resource_manager_->available_state_interfaces();
    }
    if (
      state_interface_config.type == controller_interface::interface_configuration_type::INDIVIDUAL)
    {
      state_interface_names = state_interface_config.names;
    }
    std::vector<hardware_interface::LoanedStateInterface> state_loans;
    state_loans.reserve(state_interface_names.size());
    for (const auto & state_interface : state_interface_names)
    {
      try
      {
        state_loans.emplace_back(resource_manager_->claim_state_interface(state_interface));
      }
      catch (const std::exception & e)
      {
        RCLCPP_ERROR(get_logger(), "Can't activate controller '%s': %s", request.c_str(), e.what());
        assignment_successful = false;
        break;
      }
    }
    // something went wrong during state interfaces, go skip the controller
    if (!assignment_successful)
    {
      continue;
    }
    controller->assign_interfaces(std::move(command_loans), std::move(state_loans));

    const auto new_state = controller->activate();
    if (new_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      RCLCPP_ERROR(
        get_logger(), "After activating, controller '%s' is in state '%s', expected Active",
        controller->get_node()->get_name(), new_state.label().c_str());
    }
  }
  // All controllers started, switching done
  switch_params_.do_switch = false;
}

void ControllerManager::start_controllers_asap()
{
  //  https://github.com/ros-controls/ros2_control/issues/263
  start_controllers();
}

void ControllerManager::list_controllers_srv_cb(
  const std::shared_ptr<controller_manager_msgs::srv::ListControllers::Request>,
  std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response> response)
{
  // lock services
  RCLCPP_DEBUG(get_logger(), "list controller service called");
  std::lock_guard<std::mutex> services_guard(services_lock_);
  RCLCPP_DEBUG(get_logger(), "list controller service locked");

  // lock controllers
  std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);
  const std::vector<ControllerSpec> & controllers = rt_controllers_wrapper_.get_updated_list(guard);
  response->controller.resize(controllers.size());

  for (size_t i = 0; i < controllers.size(); ++i)
  {
    controller_manager_msgs::msg::ControllerState & cs = response->controller[i];
    cs.name = controllers[i].info.name;
    cs.type = controllers[i].info.type;
    cs.claimed_interfaces = controllers[i].info.claimed_interfaces;
    cs.state = controllers[i].c->get_state().label();

    // Get information about interfaces
    auto command_interface_config = controllers[i].c->command_interface_configuration();
    if (command_interface_config.type == controller_interface::interface_configuration_type::ALL)
    {
      cs.required_command_interfaces = resource_manager_->command_interface_keys();
    }
    else if (
      command_interface_config.type ==
      controller_interface::interface_configuration_type::INDIVIDUAL)
    {
      cs.required_command_interfaces = command_interface_config.names;
    }

    auto state_interface_config = controllers[i].c->state_interface_configuration();
    if (state_interface_config.type == controller_interface::interface_configuration_type::ALL)
    {
      cs.required_state_interfaces = resource_manager_->state_interface_keys();
    }
    else if (
      state_interface_config.type == controller_interface::interface_configuration_type::INDIVIDUAL)
    {
      cs.required_state_interfaces = state_interface_config.names;
    }
  }

  RCLCPP_DEBUG(get_logger(), "list controller service finished");
}

void ControllerManager::list_controller_types_srv_cb(
  const std::shared_ptr<controller_manager_msgs::srv::ListControllerTypes::Request>,
  std::shared_ptr<controller_manager_msgs::srv::ListControllerTypes::Response> response)
{
  // lock services
  RCLCPP_DEBUG(get_logger(), "list types service called");
  std::lock_guard<std::mutex> guard(services_lock_);
  RCLCPP_DEBUG(get_logger(), "list types service locked");

  auto cur_types = loader_->getDeclaredClasses();
  for (const auto & cur_type : cur_types)
  {
    response->types.push_back(cur_type);
    response->base_classes.push_back(kControllerInterface);
    RCLCPP_DEBUG(get_logger(), "%s", cur_type.c_str());
  }

  RCLCPP_DEBUG(get_logger(), "list types service finished");
}

void ControllerManager::load_controller_service_cb(
  const std::shared_ptr<controller_manager_msgs::srv::LoadController::Request> request,
  std::shared_ptr<controller_manager_msgs::srv::LoadController::Response> response)
{
  // lock services
  RCLCPP_DEBUG(get_logger(), "loading service called for controller '%s' ", request->name.c_str());
  std::lock_guard<std::mutex> guard(services_lock_);
  RCLCPP_DEBUG(get_logger(), "loading service locked");

  response->ok = load_controller(request->name).get() != nullptr;

  RCLCPP_DEBUG(
    get_logger(), "loading service finished for controller '%s' ", request->name.c_str());
}

void ControllerManager::configure_controller_service_cb(
  const std::shared_ptr<controller_manager_msgs::srv::ConfigureController::Request> request,
  std::shared_ptr<controller_manager_msgs::srv::ConfigureController::Response> response)
{
  // lock services
  RCLCPP_DEBUG(
    get_logger(), "configuring service called for controller '%s' ", request->name.c_str());
  std::lock_guard<std::mutex> guard(services_lock_);
  RCLCPP_DEBUG(get_logger(), "configuring service locked");

  response->ok = configure_controller(request->name) == controller_interface::return_type::OK;

  RCLCPP_DEBUG(
    get_logger(), "configuring service finished for controller '%s' ", request->name.c_str());
}

void ControllerManager::load_and_configure_controller_service_cb(
  const std::shared_ptr<controller_manager_msgs::srv::LoadConfigureController::Request> request,
  std::shared_ptr<controller_manager_msgs::srv::LoadConfigureController::Response> response)
{
  // lock services
  RCLCPP_DEBUG(
    get_logger(), "loading and configure service called for controller '%s' ",
    request->name.c_str());
  std::lock_guard<std::mutex> guard(services_lock_);
  RCLCPP_DEBUG(get_logger(), "loading and configure service locked");

  response->ok = load_controller(request->name).get();

  if (response->ok)
  {
    response->ok = configure_controller(request->name) == controller_interface::return_type::OK;
  }

  RCLCPP_DEBUG(
    get_logger(), "loading and configure service finished for controller '%s' ",
    request->name.c_str());
}

void ControllerManager::load_and_start_controller_service_cb(
  const std::shared_ptr<controller_manager_msgs::srv::LoadStartController::Request> request,
  std::shared_ptr<controller_manager_msgs::srv::LoadStartController::Response> response)
{
  // lock services
  RCLCPP_DEBUG(
    get_logger(), "loading and activating service called for controller '%s' ",
    request->name.c_str());
  std::lock_guard<std::mutex> guard(services_lock_);
  RCLCPP_DEBUG(get_logger(), "loading and activating service locked");

  response->ok = load_controller(request->name).get();

  if (response->ok)
  {
    response->ok = configure_controller(request->name) == controller_interface::return_type::OK;
  }

  if (response->ok)
  {
    std::vector<std::string> start_controller = {request->name};
    std::vector<std::string> empty;
    response->ok = switch_controller(
                     start_controller, empty,
                     controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT) ==
                   controller_interface::return_type::OK;
  }

  RCLCPP_DEBUG(
    get_logger(), "loading and activating service finished for controller '%s' ",
    request->name.c_str());
}

void ControllerManager::configure_and_start_controller_service_cb(
  const std::shared_ptr<controller_manager_msgs::srv::ConfigureStartController::Request> request,
  std::shared_ptr<controller_manager_msgs::srv::ConfigureStartController::Response> response)
{
  // lock services
  RCLCPP_DEBUG(
    get_logger(), "configuring and activating service called for controller '%s' ",
    request->name.c_str());
  std::lock_guard<std::mutex> guard(services_lock_);
  RCLCPP_DEBUG(get_logger(), "configuring and activating service locked");

  response->ok = configure_controller(request->name) == controller_interface::return_type::OK;

  if (response->ok)
  {
    std::vector<std::string> start_controller = {request->name};
    std::vector<std::string> empty;
    response->ok = switch_controller(
                     start_controller, empty,
                     controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT) ==
                   controller_interface::return_type::OK;
  }

  RCLCPP_DEBUG(
    get_logger(), "configuring and activating service finished for controller '%s' ",
    request->name.c_str());
}

void ControllerManager::reload_controller_libraries_service_cb(
  const std::shared_ptr<controller_manager_msgs::srv::ReloadControllerLibraries::Request> request,
  std::shared_ptr<controller_manager_msgs::srv::ReloadControllerLibraries::Response> response)
{
  // lock services
  RCLCPP_DEBUG(get_logger(), "reload libraries service called");
  std::lock_guard<std::mutex> guard(services_lock_);
  RCLCPP_DEBUG(get_logger(), "reload libraries service locked");

  // only reload libraries if no controllers are active
  std::vector<std::string> loaded_controllers, active_controllers;
  loaded_controllers = get_controller_names();
  {
    // lock controllers
    std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);
    for (const auto & controller : rt_controllers_wrapper_.get_updated_list(guard))
    {
      if (is_controller_active(*controller.c))
      {
        active_controllers.push_back(controller.info.name);
      }
    }
  }
  if (!active_controllers.empty() && !request->force_kill)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Controller manager: Cannot reload controller libraries because"
      " there are still %i active controllers",
      (int)active_controllers.size());
    response->ok = false;
    return;
  }

  // stop active controllers if requested
  if (!loaded_controllers.empty())
  {
    RCLCPP_INFO(get_logger(), "Controller manager: Stopping all active controllers");
    std::vector<std::string> empty;
    if (
      switch_controller(
        empty, active_controllers,
        controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT) !=
      controller_interface::return_type::OK)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Controller manager: Cannot reload controller libraries because failed to stop "
        "active controllers");
      response->ok = false;
      return;
    }
    for (const auto & controller : loaded_controllers)
    {
      if (unload_controller(controller) != controller_interface::return_type::OK)
      {
        RCLCPP_ERROR(
          get_logger(),
          "Controller manager: Cannot reload controller libraries because "
          "failed to unload controller '%s'",
          controller.c_str());
        response->ok = false;
        return;
      }
    }
    loaded_controllers = get_controller_names();
  }
  assert(loaded_controllers.empty());

  // Force a reload on all the PluginLoaders (internally, this recreates the plugin loaders)
  loader_ = std::make_shared<pluginlib::ClassLoader<controller_interface::ControllerInterface>>(
    kControllerInterfaceName, kControllerInterface);
  RCLCPP_INFO(
    get_logger(), "Controller manager: reloaded controller libraries for '%s'",
    kControllerInterfaceName);

  response->ok = true;

  RCLCPP_DEBUG(get_logger(), "reload libraries service finished");
}

void ControllerManager::switch_controller_service_cb(
  const std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> request,
  std::shared_ptr<controller_manager_msgs::srv::SwitchController::Response> response)
{
  // lock services
  RCLCPP_DEBUG(get_logger(), "switching service called");
  std::lock_guard<std::mutex> guard(services_lock_);
  RCLCPP_DEBUG(get_logger(), "switching service locked");

  response->ok = switch_controller(
                   request->start_controllers, request->stop_controllers, request->strictness,
                   request->start_asap, request->timeout) == controller_interface::return_type::OK;

  RCLCPP_DEBUG(get_logger(), "switching service finished");
}

void ControllerManager::unload_controller_service_cb(
  const std::shared_ptr<controller_manager_msgs::srv::UnloadController::Request> request,
  std::shared_ptr<controller_manager_msgs::srv::UnloadController::Response> response)
{
  // lock services
  RCLCPP_DEBUG(
    get_logger(), "unloading service called for controller '%s' ", request->name.c_str());
  std::lock_guard<std::mutex> guard(services_lock_);
  RCLCPP_DEBUG(get_logger(), "unloading service locked");

  response->ok = unload_controller(request->name) == controller_interface::return_type::OK;

  RCLCPP_DEBUG(
    get_logger(), "unloading service finished for controller '%s' ", request->name.c_str());
}

void ControllerManager::list_hardware_components_srv_cb(
  const std::shared_ptr<controller_manager_msgs::srv::ListHardwareComponents::Request>,
  std::shared_ptr<controller_manager_msgs::srv::ListHardwareComponents::Response> response)
{
  RCLCPP_DEBUG(get_logger(), "list hardware components service called");
  std::lock_guard<std::mutex> guard(services_lock_);
  RCLCPP_DEBUG(get_logger(), "list hardware components service locked");

  RCLCPP_DEBUG(get_logger(), "list hardware components service finished");
}

void ControllerManager::list_hardware_interfaces_srv_cb(
  const std::shared_ptr<controller_manager_msgs::srv::ListHardwareInterfaces::Request>,
  std::shared_ptr<controller_manager_msgs::srv::ListHardwareInterfaces::Response> response)
{
  RCLCPP_DEBUG(get_logger(), "list hardware interfaces service called");
  std::lock_guard<std::mutex> guard(services_lock_);
  RCLCPP_DEBUG(get_logger(), "list hardware interfaces service locked");

  auto state_interface_names = resource_manager_->state_interface_keys();
  for (const auto & state_interface_name : state_interface_names)
  {
    controller_manager_msgs::msg::HardwareInterface hwi;
    hwi.name = state_interface_name;
    hwi.is_available = resource_manager_->state_interface_is_available(state_interface_name);
    hwi.is_claimed = false;
    response->state_interfaces.push_back(hwi);
  }
  auto command_interface_names = resource_manager_->command_interface_keys();
  for (const auto & command_interface_name : command_interface_names)
  {
    controller_manager_msgs::msg::HardwareInterface hwi;
    hwi.name = command_interface_name;
    hwi.is_available = resource_manager_->command_interface_is_available(command_interface_name);
    hwi.is_claimed = resource_manager_->command_interface_is_claimed(command_interface_name);
    response->command_interfaces.push_back(hwi);
  }

  RCLCPP_DEBUG(get_logger(), "list hardware interfaces service finished");
}

void ControllerManager::set_hardware_component_state_srv_cb(
  const std::shared_ptr<controller_manager_msgs::srv::SetHardwareComponentState::Request> request,
  std::shared_ptr<controller_manager_msgs::srv::SetHardwareComponentState::Response> response)
{
  RCLCPP_DEBUG(get_logger(), "set hardware component state service called");
  std::lock_guard<std::mutex> guard(services_lock_);
  RCLCPP_DEBUG(get_logger(), "set hardware component state service locked");

  RCLCPP_DEBUG(get_logger(), "set hardware component state '%s'", request->name.c_str());

  RCLCPP_DEBUG(get_logger(), "set hardware component state service finished");
}

std::vector<std::string> ControllerManager::get_controller_names()
{
  std::vector<std::string> names;

  // lock controllers
  std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);
  for (const auto & controller : rt_controllers_wrapper_.get_updated_list(guard))
  {
    names.push_back(controller.info.name);
  }
  return names;
}

void ControllerManager::read() { resource_manager_->read(); }

controller_interface::return_type ControllerManager::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::vector<ControllerSpec> & rt_controller_list =
    rt_controllers_wrapper_.update_and_get_used_by_rt_list();

  auto ret = controller_interface::return_type::OK;
  ++update_loop_counter_;
  update_loop_counter_ %= update_rate_;

  for (auto loaded_controller : rt_controller_list)
  {
    // TODO(v-lopez) we could cache this information
    // https://github.com/ros-controls/ros2_control/issues/153
    if (is_controller_active(*loaded_controller.c))
    {
      auto controller_update_rate = loaded_controller.c->get_update_rate();

      bool controller_go =
        controller_update_rate == 0 || ((update_loop_counter_ % controller_update_rate) == 0);
      RCLCPP_DEBUG(
        get_logger(), "update_loop_counter: '%d ' controller_go: '%s ' controller_name: '%s '",
        update_loop_counter_, controller_go ? "True" : "False",
        loaded_controller.info.name.c_str());

      if (controller_go)
      {
        auto controller_ret = loaded_controller.c->update(
          time, (controller_update_rate != update_rate_ && controller_update_rate != 0)
                  ? rclcpp::Duration::from_seconds(1.0 / controller_update_rate)
                  : period);

        if (controller_ret != controller_interface::return_type::OK)
        {
          ret = controller_ret;
        }
      }
    }
  }

  // there are controllers to start/stop
  if (switch_params_.do_switch)
  {
    manage_switch();
  }

  return ret;
}

void ControllerManager::write() { resource_manager_->write(); }

std::vector<ControllerSpec> &
ControllerManager::RTControllerListWrapper::update_and_get_used_by_rt_list()
{
  used_by_realtime_controllers_index_ = updated_controllers_index_;
  return controllers_lists_[used_by_realtime_controllers_index_];
}

std::vector<ControllerSpec> & ControllerManager::RTControllerListWrapper::get_unused_list(
  const std::lock_guard<std::recursive_mutex> &)
{
  if (!controllers_lock_.try_lock())
  {
    throw std::runtime_error("controllers_lock_ not owned by thread");
  }
  controllers_lock_.unlock();
  // Get the index to the outdated controller list
  int free_controllers_list = get_other_list(updated_controllers_index_);

  // Wait until the outdated controller list is not being used by the realtime thread
  wait_until_rt_not_using(free_controllers_list);
  return controllers_lists_[free_controllers_list];
}

const std::vector<ControllerSpec> & ControllerManager::RTControllerListWrapper::get_updated_list(
  const std::lock_guard<std::recursive_mutex> &) const
{
  if (!controllers_lock_.try_lock())
  {
    throw std::runtime_error("controllers_lock_ not owned by thread");
  }
  controllers_lock_.unlock();
  return controllers_lists_[updated_controllers_index_];
}

void ControllerManager::RTControllerListWrapper::switch_updated_list(
  const std::lock_guard<std::recursive_mutex> &)
{
  if (!controllers_lock_.try_lock())
  {
    throw std::runtime_error("controllers_lock_ not owned by thread");
  }
  controllers_lock_.unlock();
  int former_current_controllers_list_ = updated_controllers_index_;
  updated_controllers_index_ = get_other_list(former_current_controllers_list_);
  wait_until_rt_not_using(former_current_controllers_list_);
}

int ControllerManager::RTControllerListWrapper::get_other_list(int index) const
{
  return (index + 1) % 2;
}

void ControllerManager::RTControllerListWrapper::wait_until_rt_not_using(
  int index, std::chrono::microseconds sleep_period) const
{
  while (used_by_realtime_controllers_index_ == index)
  {
    if (!rclcpp::ok())
    {
      throw std::runtime_error("rclcpp interrupted");
    }
    std::this_thread::sleep_for(sleep_period);
  }
}

unsigned int ControllerManager::get_update_rate() const { return update_rate_; }

}  // namespace controller_manager
