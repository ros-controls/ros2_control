// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "controller_interface/controller_interface.hpp"

#include "controller_manager/controller_loader_interface.hpp"
#include "controller_manager/controller_spec.hpp"
#include "controller_manager/visibility_control.h"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/list_controller_types.hpp"
#include "controller_manager_msgs/srv/load_controller.hpp"
#include "controller_manager_msgs/srv/reload_controller_libraries.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/srv/unload_controller.hpp"

#include "hardware_interface/robot_hardware.hpp"

#include "rclcpp/executor.hpp"
#include "rclcpp/node.hpp"

namespace controller_manager
{

class ControllerManager : public rclcpp::Node
{
public:
  static constexpr bool WAIT_FOR_ALL_RESOURCES = false;
  static constexpr double INFINITE_TIMEOUT = 0.0;

  CONTROLLER_MANAGER_PUBLIC
  ControllerManager(
    std::shared_ptr<hardware_interface::RobotHardware> hw,
    std::shared_ptr<rclcpp::Executor> executor,
    const std::string & name = "controller_manager");

  CONTROLLER_MANAGER_PUBLIC
  virtual
  ~ControllerManager() = default;

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::ControllerInterfaceSharedPtr
  load_controller(
    const std::string & controller_name,
    const std::string & controller_type);

  /**
   * @brief load_controller loads a controller by name, the type must be defined in the parameter server
   */
  CONTROLLER_MANAGER_PUBLIC
  controller_interface::ControllerInterfaceSharedPtr
  load_controller(
    const std::string & controller_name);

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type unload_controller(
    const std::string & controller_name);

  CONTROLLER_MANAGER_PUBLIC
  std::vector<ControllerSpec> get_loaded_controllers() const;

  CONTROLLER_MANAGER_PUBLIC
  void register_controller_loader(ControllerLoaderInterfaceSharedPtr loader);

  template<
    typename T,
    typename std::enable_if<std::is_convertible<
      T *, controller_interface::ControllerInterface *>::value, T>::type * = nullptr>
  controller_interface::ControllerInterfaceSharedPtr
  add_controller(
    std::shared_ptr<T> controller, std::string controller_name,
    std::string controller_type)
  {
    ControllerSpec controller_spec;
    controller_spec.c = controller;
    controller_spec.info.name = controller_name;
    controller_spec.info.type = controller_type;
    return add_controller_impl(controller_spec);
  }

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type
  switch_controller(
    const std::vector<std::string> & start_controllers,
    const std::vector<std::string> & stop_controllers,
    int strictness, bool start_asap = WAIT_FOR_ALL_RESOURCES,
    const rclcpp::Duration & timeout = rclcpp::Duration(INFINITE_TIMEOUT));

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type
  update();

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type
  configure();

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type
  activate() const;

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type
  deactivate() const;

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type
  cleanup() const;

protected:
  CONTROLLER_MANAGER_PUBLIC
  controller_interface::ControllerInterfaceSharedPtr
  add_controller_impl(const ControllerSpec & controller);

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::ControllerInterface * get_controller_by_name(const std::string & name);

  CONTROLLER_MANAGER_PUBLIC
  void manage_switch();

  CONTROLLER_MANAGER_PUBLIC
  void stop_controllers();

  CONTROLLER_MANAGER_PUBLIC
  void start_controllers();

  CONTROLLER_MANAGER_PUBLIC
  void start_controllers_asap();

  CONTROLLER_MANAGER_PUBLIC
  void list_controllers_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListControllers::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void list_controller_types_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListControllerTypes::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListControllerTypes::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void load_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::LoadController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::LoadController::Response> response);

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

private:
  void get_controller_names(std::vector<std::string> & names);


  std::shared_ptr<hardware_interface::RobotHardware> hw_;
  std::shared_ptr<rclcpp::Executor> executor_;
  std::vector<ControllerLoaderInterfaceSharedPtr> loaders_;

  /** \name Controllers List
   * The controllers list is double-buffered to avoid needing to lock the
   * real-time thread when switching controllers in the non-real-time thread.
   *\{*/
  /// Mutex protecting the current controllers list
  std::recursive_mutex controllers_lock_;
  std::vector<ControllerSpec> controllers_lists_[2];
  /// The index of the current controllers list
  int current_controllers_list_ = {0};
  /// The index of the controllers list being used in the real-time thread.
  int used_by_realtime_ = {-1};

  /// mutex copied from ROS1 Control, protects service callbacks
  /// not needed if we're guaranteed that the callbacks don't come from multiple threads
  std::mutex services_lock_;
  rclcpp::Service<controller_manager_msgs::srv::ListControllers>::SharedPtr
    list_controllers_service_;
  rclcpp::Service<controller_manager_msgs::srv::ListControllerTypes>::SharedPtr
    list_controller_types_service_;
  rclcpp::Service<controller_manager_msgs::srv::LoadController>::SharedPtr
    load_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::ReloadControllerLibraries>::SharedPtr
    reload_controller_libraries_service_;
  rclcpp::Service<controller_manager_msgs::srv::SwitchController>::SharedPtr
    switch_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::UnloadController>::SharedPtr
    unload_controller_service_;


  std::vector<controller_interface::ControllerInterface *> start_request_, stop_request_;
#ifdef TODO_IMPLEMENT_RESOURCE_CHECKING
//  std::list<hardware_interface::ControllerInfo> switch_start_list_, switch_stop_list_;
#endif

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
