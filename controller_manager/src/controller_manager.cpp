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
#include <vector>

#include "controller_interface/controller_interface.hpp"

#include "controller_manager/controller_loader_pluginlib.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace controller_manager
{

inline bool is_controller_running(controller_interface::ControllerInterface & controller)
{
  return controller.get_lifecycle_node()->get_current_state().id() ==
         lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
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
  return node_options;
}

ControllerManager::ControllerManager(
  std::shared_ptr<hardware_interface::RobotHardware> hw,
  std::shared_ptr<rclcpp::executor::Executor> executor,
  const std::string & manager_node_name)
: rclcpp::Node(manager_node_name, get_cm_node_options()),
  hw_(hw),
  executor_(executor),
  // add pluginlib loader by default
  loaders_({std::make_shared<ControllerLoaderPluginlib>()})
{
  using namespace std::placeholders;
  list_controllers_service_ = create_service<controller_manager_msgs::srv::ListControllers>(
    "~/list_controllers", std::bind(
      &ControllerManager::list_controllers_srv_cb, this, _1,
      _2));
  list_controller_types_service_ =
    create_service<controller_manager_msgs::srv::ListControllerTypes>(
    "~/list_controller_types", std::bind(
      &ControllerManager::list_controller_types_srv_cb, this, _1,
      _2));
  load_controller_service_ = create_service<controller_manager_msgs::srv::LoadController>(
    "~/load_controller", std::bind(
      &ControllerManager::load_controller_service_cb, this, _1,
      _2));
  reload_controller_libraries_service_ =
    create_service<controller_manager_msgs::srv::ReloadControllerLibraries>(
    "~/reload_controller_libraries", std::bind(
      &ControllerManager::reload_controller_libraries_service_cb, this, _1,
      _2));
  switch_controller_service_ = create_service<controller_manager_msgs::srv::SwitchController>(
    "~/switch_controller", std::bind(
      &ControllerManager::switch_controller_service_cb, this, _1,
      _2));
  unload_controller_service_ = create_service<controller_manager_msgs::srv::UnloadController>(
    "~/unload_controller", std::bind(
      &ControllerManager::unload_controller_service_cb, this, _1,
      _2));
}

controller_interface::ControllerInterfaceSharedPtr ControllerManager::load_controller(
  const std::string & controller_name,
  const std::string & controller_type)
{
  RCLCPP_INFO(get_logger(), "Loading controller '%s'\n", controller_name.c_str());

  auto it = std::find_if(
    loaders_.cbegin(), loaders_.cend(),
    [&](auto loader)
    {return loader->is_available(controller_type);});

  controller_interface::ControllerInterfaceSharedPtr controller(nullptr);
  if (it != loaders_.cend()) {
    controller = (*it)->create(controller_type);
  } else {
    const std::string error_msg("Loader for controller '" + controller_name + "' not found\n");
    RCLCPP_ERROR(get_logger(), "%s", error_msg.c_str());
    throw std::runtime_error(error_msg);
  }
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
  if (!has_parameter(param_name)) {
    declare_parameter(param_name, rclcpp::ParameterValue());
  }
  if (!get_parameter(param_name, controller_type)) {
    RCLCPP_ERROR(get_logger(), "'type' param not defined for %s", controller_name.c_str());
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

  // Transfers the running controllers over, skipping the one to be removed and the running ones.
  to = from;

  auto found_it = std::find_if(
    to.begin(), to.end(),
    std::bind(controller_name_compare, std::placeholders::_1, controller_name));
  if (found_it == to.end()) {
    // Fails if we could not remove the controllers
    to.clear();
    RCLCPP_ERROR(
      get_logger(),
      "Could not unload controller with name '%s' because no controller with this name exists",
      controller_name.c_str());
    return controller_interface::return_type::ERROR;
  }

  auto & controller = *found_it;

  if (is_controller_running(*controller.c)) {
    to.clear();
    RCLCPP_ERROR(
      get_logger(),
      "Could not unload controller with name '%s' because it is still running",
      controller_name.c_str());
    return controller_interface::return_type::ERROR;
  }

  RCLCPP_DEBUG(get_logger(), "Cleanup controller");
  controller.c->get_lifecycle_node()->cleanup();
  executor_->remove_node(controller.c->get_lifecycle_node()->get_node_base_interface());
  to.erase(found_it);

  // Destroys the old controllers list when the realtime thread is finished with it.
  RCLCPP_DEBUG(get_logger(), "Realtime switches over to new controller list");
  rt_controllers_wrapper_.switch_updated_list(guard);
  std::vector<ControllerSpec> & new_unused_list = rt_controllers_wrapper_.get_unused_list(
    guard);
  RCLCPP_DEBUG(get_logger(), "Destruct controller");
  new_unused_list.clear();
  RCLCPP_DEBUG(get_logger(), "Destruct controller finished");

  RCLCPP_DEBUG(get_logger(), "Successfully unloaded controller '%s'", controller_name.c_str());
  return controller_interface::return_type::SUCCESS;
}

std::vector<ControllerSpec> ControllerManager::get_loaded_controllers() const
{
  std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);
  return rt_controllers_wrapper_.get_updated_list(guard);
}

void ControllerManager::register_controller_loader(ControllerLoaderInterfaceSharedPtr loader)
{
  loaders_.push_back(loader);
}

controller_interface::return_type ControllerManager::switch_controller(
  const std::vector<std::string> & start_controllers,
  const std::vector<std::string> & stop_controllers,
  int strictness,
  bool start_asap,
  const rclcpp::Duration & timeout)
{
  switch_params_ = SwitchParams();

  if (!stop_request_.empty() || !start_request_.empty()) {
    RCLCPP_FATAL(
      get_logger(),
      "The internal stop and start request lists are not empty at the beginning of the "
      "switchController() call. This should not happen.");
  }

  if (strictness == 0) {
    RCLCPP_WARN(
      get_logger(), "Controller Manager: To switch controllers you need to specify a "
      "strictness level of controller_manager_msgs::SwitchController::STRICT "
      "(%d) or ::BEST_EFFORT (%d). Defaulting to ::BEST_EFFORT.",
      controller_manager_msgs::srv::SwitchController::Request::STRICT,
      controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT);
    strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  }

  RCLCPP_DEBUG(get_logger(), "switching controllers:");
  for (const auto & controller : start_controllers) {
    RCLCPP_DEBUG(get_logger(), "- starting controller '%s'", controller.c_str());
  }
  for (const auto & controller : stop_controllers) {
    RCLCPP_DEBUG(get_logger(), "- stopping controller '%s'", controller.c_str());
  }

  const auto list_controllers = [this, strictness](const std::vector<std::string> & controller_list,
      std::vector<std::string> & request_list,
      const std::string & action)
    {
      // lock controllers
      std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);

      // list all controllers to stop/start
      for (const auto & controller : controller_list) {
        const auto & updated_controllers = rt_controllers_wrapper_.get_updated_list(guard);

        auto found_it = std::find_if(
          updated_controllers.begin(), updated_controllers.end(),
          std::bind(controller_name_compare, std::placeholders::_1, controller));

        if (found_it == updated_controllers.end()) {
          if (strictness == controller_manager_msgs::srv::SwitchController::Request::STRICT) {
            RCLCPP_ERROR(
              get_logger(),
              "Could not %s controller with name '%s' because no controller with this name exists",
              action.c_str(),
              controller.c_str());
            return controller_interface::return_type::ERROR;
          }
          RCLCPP_DEBUG(
            get_logger(),
            "Could not %s controller with name '%s' because no controller with this name exists",
            action.c_str(),
            controller.c_str());
        } else {
          RCLCPP_DEBUG(
            get_logger(),
            "Found controller '%s' that needs to be %sed in list of controllers",
            controller.c_str(),
            action.c_str());
          request_list.push_back(controller);
        }
      }
      RCLCPP_DEBUG(
        get_logger(), "%s request vector has size %i",
        action.c_str(), (int)request_list.size());

      return controller_interface::return_type::SUCCESS;
    };

  // list all controllers to stop
  auto ret = list_controllers(stop_controllers, stop_request_, "stop");
  if (ret != controller_interface::return_type::SUCCESS) {
    stop_request_.clear();
    return ret;
  }

  // list all controllers to start
  ret = list_controllers(start_controllers, start_request_, "start");
  if (ret != controller_interface::return_type::SUCCESS) {
    stop_request_.clear();
    start_request_.clear();
    return ret;
  }

#ifdef TODO_IMPLEMENT_RESOURCE_CHECKING
  // Do the resource management checking
  std::list<hardware_interface::ControllerInfo> info_list;
  switch_start_list_.clear();
  switch_stop_list_.clear();
#endif

  // lock controllers
  std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);

  const std::vector<ControllerSpec> & controllers =
    rt_controllers_wrapper_.get_updated_list(guard);

  for (const auto & controller : controllers) {
    auto stop_list_it = std::find(
      stop_request_.begin(), stop_request_.end(), controller.info.name);
    bool in_stop_list = stop_list_it != stop_request_.end();

    auto start_list_it = std::find(
      start_request_.begin(), start_request_.end(), controller.info.name);
    bool in_start_list = start_list_it != start_request_.end();

    const bool is_running = is_controller_running(*controller.c);

    auto handle_conflict = [&](const std::string & msg)
      {
        if (strictness == controller_manager_msgs::srv::SwitchController::Request::STRICT) {
          RCLCPP_ERROR_STREAM(
            get_logger(),
            msg);
          stop_request_.clear();
          start_request_.clear();
          return controller_interface::return_type::ERROR;
        }
        RCLCPP_DEBUG_STREAM(
          get_logger(),
          "Could not stop controller '" << controller.info.name <<
            "' since it is not running");
        return controller_interface::return_type::SUCCESS;
      };
    if (!is_running && in_stop_list) {      // check for double stop
      auto ret = handle_conflict(
        "Could not stop controller '" + controller.info.name +
        "' since it is not running");
      if (ret != controller_interface::return_type::SUCCESS) {
        return ret;
      }
      in_stop_list = false;
      stop_request_.erase(stop_list_it);
    }

    if (is_running && !in_stop_list && in_start_list) {  // check for doubled start
      auto ret = handle_conflict(
        "Could not start controller '" + controller.info.name +
        "' since it is already running");
      if (ret != controller_interface::return_type::SUCCESS) {
        return ret;
      }
      in_start_list = false;
      start_request_.erase(start_list_it);
    }

#ifdef TODO_IMPLEMENT_RESOURCE_CHECKING
    if (is_running && in_stop_list && !in_start_list) {  // running and real stop
      switch_stop_list_.push_back(info);
    } else if (!is_running && !in_stop_list && in_start_list) {  // start, but no restart
      switch_start_list_.push_back(info);
    }

    bool add_to_list = is_running;
    if (in_stop_list) {
      add_to_list = false;
    }
    if (in_start_list) {
      add_to_list = true;
    }

    if (add_to_list) {
      info_list.push_back(info);
    }
#endif
  }

#ifdef TODO_IMPLEMENT_RESOURCE_CHECKING
  bool in_conflict = robot_hw_->checkForConflict(info_list);
  if (in_conflict) {
    RCLCPP_ERROR(get_logger(), "Could not switch controllers, due to resource conflict");
    stop_request_.clear();
    start_request_.clear();
    return controller_interface::return_type::ERROR;
  }

  if (!robot_hw_->prepareSwitch(switch_start_list_, switch_stop_list_)) {
    RCLCPP_ERROR(
      get_logger(),
      "Could not switch controllers. The hardware interface combination "
      "for the requested controllers is unfeasible.");
    stop_request_.clear();
    start_request_.clear();
    return controller_interface::return_type::ERROR;
  }
#endif

  if (start_request_.empty() && stop_request_.empty()) {
    RCLCPP_INFO(get_logger(), "Empty start and stop list, not requesting switch");
    return controller_interface::return_type::SUCCESS;
  }

  // start the atomic controller switching
  switch_params_.strictness = strictness;
  switch_params_.start_asap = start_asap;
  switch_params_.init_time = rclcpp::Clock().now();
  switch_params_.timeout = timeout;
  switch_params_.do_switch = true;

  // wait until switch is finished
  RCLCPP_DEBUG(get_logger(), "Request atomic controller switch from realtime loop");
  while (rclcpp::ok() && switch_params_.do_switch) {
    if (!rclcpp::ok()) {
      return controller_interface::return_type::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  start_request_.clear();
  stop_request_.clear();

  RCLCPP_DEBUG(get_logger(), "Successfully switched controllers");
  return controller_interface::return_type::SUCCESS;
}

controller_interface::ControllerInterfaceSharedPtr
ControllerManager::add_controller_impl(
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
  if (found_it != to.end()) {
    to.clear();
    RCLCPP_ERROR(
      get_logger(),
      "A controller named '%s' was already loaded inside the controller manager",
      controller.info.name.c_str());
    return nullptr;
  }

  controller.c->init(hw_, controller.info.name);

  // TODO(v-lopez) this should only be done if controller_manager is configured.
  // Probably the whole load_controller part should fail if the controller_manager
  // is not configured, should it implement a LifecycleNodeInterface
  // https://github.com/ros-controls/ros2_control/issues/152
  controller.c->get_lifecycle_node()->configure();
  executor_->add_node(controller.c->get_lifecycle_node()->get_node_base_interface());
  to.emplace_back(controller);

  // Destroys the old controllers list when the realtime thread is finished with it.
  RCLCPP_DEBUG(get_logger(), "Realtime switches over to new controller list");
  rt_controllers_wrapper_.switch_updated_list(guard);
  RCLCPP_DEBUG(get_logger(), "Destruct controller");
  std::vector<ControllerSpec> & new_unused_list = rt_controllers_wrapper_.get_unused_list(
    guard);
  new_unused_list.clear();
  RCLCPP_DEBUG(get_logger(), "Destruct controller finished");

  return to.back().c;
}

void ControllerManager::manage_switch()
{
#ifdef TODO_IMPLEMENT_RESOURCE_CHECKING
  // switch hardware interfaces (if any)
  if (!switch_params_.started) {
    robot_hw_->doSwitch(switch_start_list_, switch_stop_list_);
    switch_params_.started = true;
  }
#endif

  stop_controllers();

  // start controllers once the switch is fully complete
  if (!switch_params_.start_asap) {
    start_controllers();
  } else {
    // start controllers as soon as their required joints are done switching
    start_controllers_asap();
  }
}

void ControllerManager::stop_controllers()
{
  std::vector<ControllerSpec> & rt_controller_list =
    rt_controllers_wrapper_.update_and_get_used_by_rt_list();
  // stop controllers
  for (const auto & request : stop_request_) {
    auto found_it = std::find_if(
      rt_controller_list.begin(), rt_controller_list.end(),
      std::bind(controller_name_compare, std::placeholders::_1, request));
    if (found_it == rt_controller_list.end()) {
      RCLCPP_ERROR(
        get_logger(),
        "Got request to stop controller %s but it is not in the realtime controller list",
        request.c_str());
      continue;
    }
    auto controller = found_it->c;
    if (is_controller_running(*controller)) {
      const auto new_state = controller->get_lifecycle_node()->deactivate();
      if (new_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        RCLCPP_ERROR(
          get_logger(),
          "After deactivating, controller %s is in state %s, expected Inactive",
          request.c_str(),
          new_state.label().c_str());
      }
    }
  }
}

void ControllerManager::start_controllers()
{
#ifdef TODO_IMPLEMENT_RESOURCE_CHECKING
  // start controllers
  if (robot_hw_->switchResult() == hardware_interface::RobotHW::SwitchState::DONE) {
    for (const auto & request : start_request_) {
      request->startRequest(time);
    }

    switch_params_.do_switch = false;
  } else if (// NOLINT
    (robot_hw_->switchResult() == hardware_interface::RobotHW::SwitchState::ERROR) ||
    (switch_params_.timeout > 0.0 &&
    (time - switch_params_.init_time).toSec() > switch_params_.timeout))
  {
    // abort controllers in case of error or timeout (if set)
    for (const auto & request : start_request_) {
      request->abortRequest(time);
    }

    switch_params_.do_switch = false;
  } else {
    // wait controllers
    for (const auto & request : start_request_) {
      request->waitRequest(time);
    }
  }
#else
  //  Dummy implementation, replace with the code above when migrated
  std::vector<ControllerSpec> & rt_controller_list =
    rt_controllers_wrapper_.update_and_get_used_by_rt_list();
  for (const auto & request : start_request_) {
    auto found_it = std::find_if(
      rt_controller_list.begin(), rt_controller_list.end(),
      std::bind(controller_name_compare, std::placeholders::_1, request));
    if (found_it == rt_controller_list.end()) {
      RCLCPP_ERROR(
        get_logger(),
        "Got request to start controller %s but it is not in the realtime controller list",
        request.c_str());
      continue;
    }
    auto controller = found_it->c;
    const auto new_state = controller->get_lifecycle_node()->activate();
    if (new_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_ERROR(
        get_logger(),
        "After activating, controller %s is in state %s, expected Active",
        controller->get_lifecycle_node()->get_name(),
        new_state.label().c_str());
    }
  }
  // All controllers started, switching done
  switch_params_.do_switch = false;
#endif
}

void ControllerManager::start_controllers_asap()
{
#ifdef TODO_IMPLEMENT_RESOURCE_CHECKING
  // start controllers if possible
  for (const auto & request : start_request_) {
    if (!isControllerRunning(*request)) {
      // find the info from this controller
      for (const auto & controller : controllers_lists_[current_controllers_list_]) {
        if (request == controller.c.get()) {
          // ready to start
          if (robot_hw_->switchResult(controller.info) ==
            hardware_interface::RobotHW::SwitchState::DONE)
          {
            request->startRequest(time);
          } else if ((robot_hw_->switchResult(controller.info) == // NOLINT
            hardware_interface::RobotHW::SwitchState::ERROR) ||
            (switch_params_.timeout > 0.0 &&
            (time - switch_params_.init_time).toSec() > switch_params_.timeout)) // NOLINT
          {
            // abort on error or timeout (if set)
            request->abortRequest(time);
          } else {
            // controller is waiting
            request->waitRequest(time);
          }
        }
        continue;
      }
    }
  }

  // all needed controllers started or aborted, switch done
  if (std::all_of(
      start_request_.begin(), start_request_.end(),
      [](controller_interface::ControllerBase * request) {
        return request->isRunning() || request->isAborted();
      }))
  {
    switch_params_.do_switch = false;
  }
#else
  //  Dummy implementation, replace with the code above when migrated
  start_controllers();
#endif
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
  const std::vector<ControllerSpec> & controllers =
    rt_controllers_wrapper_.get_updated_list(guard);
  response->controller.resize(controllers.size());

  for (size_t i = 0; i < controllers.size(); ++i) {
    controller_manager_msgs::msg::ControllerState & cs = response->controller[i];
    cs.name = controllers[i].info.name;
    cs.type = controllers[i].info.type;
    cs.state = controllers[i].c->get_lifecycle_node()->get_current_state().label();

#ifdef TODO_IMPLEMENT_RESOURCE_CHECKING
    cs.claimed_resources.clear();
    typedef std::vector<hardware_interface::InterfaceResources> ClaimedResVec;
    typedef ClaimedResVec::const_iterator ClaimedResIt;
    const ClaimedResVec & c_resources = controllers[i].info.claimed_resources;
    for (const auto & c_resource : c_resources) {
      controller_manager_msgs::HardwareInterfaceResources iface_res;
      iface_res.hardware_interface = c_resource.hardware_interface;
      std::copy(
        c_resource.resources.begin(), c_resource.resources.end(),
        std::back_inserter(iface_res.resources));
      cs.claimed_resources.push_back(iface_res);
    }
#endif
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

  for (const auto & controller_loader : loaders_) {
    std::vector<std::string> cur_types = controller_loader->get_declared_classes();
    for (const auto & cur_type : cur_types) {
      response->types.push_back(cur_type);
      response->base_classes.push_back(controller_loader->get_name());
      RCLCPP_INFO(get_logger(), cur_type);
    }
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

  response->ok = load_controller(request->name).get();

  RCLCPP_DEBUG(
    get_logger(), "loading service finished for controller '%s' ",
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

  // only reload libraries if no controllers are running
  std::vector<std::string> loaded_controllers, running_controllers;
  loaded_controllers = get_controller_names();
  {
    // lock controllers
    std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);
    for (const auto & controller : rt_controllers_wrapper_.get_updated_list(guard)) {
      if (is_controller_running(*controller.c)) {
        running_controllers.push_back(controller.info.name);
      }
    }
  }
  if (!running_controllers.empty() && !request->force_kill) {
    RCLCPP_ERROR(
      get_logger(), "Controller manager: Cannot reload controller libraries because"
      " there are still %i controllers running",
      (int)running_controllers.size());
    response->ok = false;
    return;
  }

  // stop running controllers if requested
  if (!loaded_controllers.empty()) {
    RCLCPP_INFO(get_logger(), "Controller manager: Stopping all running controllers");
    std::vector<std::string> empty;
    if (switch_controller(
        empty, running_controllers,
        controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT) !=
      controller_interface::return_type::SUCCESS)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Controller manager: Cannot reload controller libraries because failed to stop "
        "running controllers");
      response->ok = false;
      return;
    }
    for (const auto & controller : loaded_controllers) {
      if (unload_controller(controller) != controller_interface::return_type::SUCCESS) {
        RCLCPP_ERROR(
          get_logger(), "Controller manager: Cannot reload controller libraries because "
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
  for (const auto & controller_loader : loaders_) {
    controller_loader->reload();
    RCLCPP_INFO(
      get_logger(), "Controller manager: reloaded controller libraries for '%s'",
      controller_loader->get_name().c_str());
  }

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
    request->start_asap, request->timeout) == controller_interface::return_type::SUCCESS;

  RCLCPP_DEBUG(get_logger(), "switching service finished");
}

void ControllerManager::unload_controller_service_cb(
  const std::shared_ptr<controller_manager_msgs::srv::UnloadController::Request> request,
  std::shared_ptr<controller_manager_msgs::srv::UnloadController::Response> response)
{
  // lock services
  RCLCPP_DEBUG(
    get_logger(), "unloading service called for controller '%s' ",
    request->name.c_str());
  std::lock_guard<std::mutex> guard(services_lock_);
  RCLCPP_DEBUG(get_logger(), "unloading service locked");

  response->ok = unload_controller(request->name) == controller_interface::return_type::SUCCESS;

  RCLCPP_DEBUG(
    get_logger(), "unloading service finished for controller '%s' ",
    request->name.c_str());
}

std::vector<std::string> ControllerManager::get_controller_names()
{
  std::vector<std::string> names;

  // lock controllers
  std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);
  for (const auto & controller : rt_controllers_wrapper_.get_updated_list(guard)) {
    names.push_back(controller.info.name);
  }
  return names;
}

controller_interface::return_type
ControllerManager::update()
{
  std::vector<ControllerSpec> & rt_controller_list =
    rt_controllers_wrapper_.update_and_get_used_by_rt_list();

  auto ret = controller_interface::return_type::SUCCESS;
  for (auto loaded_controller : rt_controller_list) {
    // TODO(v-lopez) we could cache this information
    // https://github.com/ros-controls/ros2_control/issues/153
    if (is_controller_running(*loaded_controller.c)) {
      auto controller_ret = loaded_controller.c->update();
      if (controller_ret != controller_interface::return_type::SUCCESS) {
        ret = controller_ret;
      }
    }
  }

  // there are controllers to start/stop
  if (switch_params_.do_switch) {
    manage_switch();
  }
  return ret;
}

std::vector<ControllerSpec> &
ControllerManager::RTControllerListWrapper::update_and_get_used_by_rt_list()
{
  used_by_realtime_controllers_index_ = updated_controllers_index_;
  return controllers_lists_[used_by_realtime_controllers_index_];
}

std::vector<ControllerSpec> &
ControllerManager::RTControllerListWrapper::get_unused_list(
  const std::lock_guard<std::recursive_mutex> &)
{
  assert(controllers_lock_.try_lock());
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
  assert(controllers_lock_.try_lock());
  controllers_lock_.unlock();
  return controllers_lists_[updated_controllers_index_];
}

void ControllerManager::RTControllerListWrapper::switch_updated_list(
  const std::lock_guard<std::recursive_mutex> &)
{
  assert(controllers_lock_.try_lock());
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
  int index,
  std::chrono::microseconds sleep_period)
const
{
  while (used_by_realtime_controllers_index_ == index) {
    if (!rclcpp::ok()) {
      throw std::runtime_error("rclcpp interrupted");
    }
    std::this_thread::sleep_for(sleep_period);
  }
}

ControllerManagerNewWithManager::ControllerManagerNewWithManager(
  std::shared_ptr<resource_manager::ResourceManager> resource_manager,
  std::shared_ptr<rclcpp::executor::Executor> executor)
: resource_manager_(resource_manager),
  executor_(executor),
  // add pluginlib loader by default
  loaders_({std::make_shared<ControllerLoaderPluginlibNewComponents>()})
{
}

std::shared_ptr<controller_interface::ControllerInterfaceNewComponents>
ControllerManagerNewWithManager::load_controller(
  const std::string & controller_name,
  const std::string & controller_type)
{
  RCLCPP_INFO(rclcpp::get_logger("ControllerManager"), "Loading controller '%s'\n", controller_name.c_str());

  auto it = std::find_if(
    loaders_.cbegin(), loaders_.cend(),
                         [&](auto loader)
                         {return loader->is_available(controller_type);});

  std::shared_ptr<controller_interface::ControllerInterfaceNewComponents> controller(nullptr);
  if (it != loaders_.cend()) {
    controller = (*it)->create_new_components(controller_type);
  } else {
    const std::string error_msg("Loader for controller '" + controller_name + "' not found\n");
    RCLCPP_ERROR(rclcpp::get_logger("ControllerManager"), "%s", error_msg.c_str());
    throw std::runtime_error(error_msg);
  }

  return add_controller_impl(controller, controller_name);
}

std::shared_ptr<controller_interface::ControllerInterfaceNewComponents>
ControllerManagerNewWithManager::add_controller_impl(
  std::shared_ptr<controller_interface::ControllerInterfaceNewComponents> controller,
  const std::string & controller_name)
{
  controller->init(resource_manager_, controller_name);
  executor_->add_node(controller->get_lifecycle_node()->get_node_base_interface());

  loaded_controllers_.emplace_back(controller);
  return loaded_controllers_.back();
}

controller_interface::return_type
ControllerManagerNewWithManager::update()
{
  auto ret = controller_interface::return_type::SUCCESS;
  for (auto loaded_controller : loaded_controllers_) {
    RCLCPP_INFO(rclcpp::get_logger("ControllerManager"), "Updating controllers..");
    RCLCPP_INFO(rclcpp::get_logger("ControllerManager"), "Loaded controller state ist active: "
     + loaded_controller->get_lifecycle_node()->get_current_state().label());
    auto controller_ret = loaded_controller->update();
    if (controller_ret != controller_interface::return_type::SUCCESS) {
      ret = controller_ret;
    }
  }

  return ret;
}

controller_interface::return_type
ControllerManagerNewWithManager::configure() const
{
  auto ret = controller_interface::return_type::SUCCESS;
  for (auto loaded_controller : loaded_controllers_) {
    auto controller_state = loaded_controller->get_lifecycle_node()->configure();
    if (controller_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      ret = controller_interface::return_type::ERROR;
    }
  }

  return ret;
}

controller_interface::return_type
ControllerManagerNewWithManager::activate() const
{
  auto ret = controller_interface::return_type::SUCCESS;
  for (auto loaded_controller : loaded_controllers_) {
    auto controller_state = loaded_controller->get_lifecycle_node()->activate();
    if (controller_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      ret = controller_interface::return_type::ERROR;
    }
  }

  return ret;
}

}  // namespace controller_manager
