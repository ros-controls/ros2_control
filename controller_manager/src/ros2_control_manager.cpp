// Copyright 2020 ROS2-Control Development Team
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

#include "controller_manager/ros2_control_manager.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace control_manager
{

ROS2ControlManager::ROS2ControlManager(
  std::shared_ptr<rclcpp::Executor> executor,
  const std::string & manager_node_name,
  rclcpp::NodeOptions options)
: Node(manager_node_name, options),
  executor_(executor)
{
}

controller_interface::return_type ROS2ControlManager::configure()
{
  // TODO(all): Should we declare paramters? #168
  // load robot_description parameter
  auto get_parameters_result = this->get_parameters({"robot_description"});

  //  Test the resulting vector of parameters
  if ((get_parameters_result.size() != 1) ||
    (get_parameters_result[0].get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET))
  {
    RCLCPP_FATAL(this->get_logger(), "No robot_description parameter");
    return controller_interface::return_type::ERROR;
  }
  std::string robot_description = get_parameters_result[0].value_to_string();

  // TODO(all): Should we declare paramters? #168
  // load controllers' names
  get_parameters_result = this->get_parameters({"controllers"});
  if ((get_parameters_result.size() != 1) ||
    (get_parameters_result[0].get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) ||
    (get_parameters_result[0].as_string_array().size() == 0))
  {
    RCLCPP_FATAL(this->get_logger(), "controllers parameter not existing or empty");
    return controller_interface::return_type::ERROR;
  }
  std::vector<std::string> controllers = get_parameters_result[0].as_string_array();
  RCLCPP_INFO(this->get_logger(), "found %d controllers", controllers.size());

  // initialize and configure resource_manager
  resource_manager_.reset(new resource_manager::ResourceManager());
  if (resource_manager_->load_and_configure_resources_from_urdf(
      robot_description) != hardware_interface::return_type::OK)
  {
    RCLCPP_FATAL(this->get_logger(), "hardware type not recognized");
    return controller_interface::return_type::ERROR;
  }

  // initialized controller_manager
  controller_manager_.reset(new controller_manager::ControllerManagerNewWithManager(
      resource_manager_, executor_));

  resource_manager_->start_all_resources();

  for (const auto & controller : controllers) {
    rclcpp::Parameter controller_type;
    if (!this->get_parameter(controller + ".type", controller_type)) {
      RCLCPP_ERROR(this->get_logger(), "'type' parameter not set for " + controller);
      return controller_interface::return_type::ERROR;
    }
    RCLCPP_DEBUG(this->get_logger(),
      "loading " + controller + " of type: " + controller_type.value_to_string());
    controller_manager_->load_controller(controller, controller_type.value_to_string());
  }

  if (controller_manager_->configure() == controller_interface::return_type::ERROR) {
    RCLCPP_FATAL(this->get_logger(), "controller_manager could not configure controllers");
    return controller_interface::return_type::ERROR;
  }
  if (controller_manager_->activate() == controller_interface::return_type::ERROR) {
    RCLCPP_FATAL(this->get_logger(), "controller_manager could not active controllers");
    return controller_interface::return_type::ERROR;
  }

  timer_ = this->create_wall_timer(1000ms, std::bind(&ROS2ControlManager::loop, this));

  return controller_interface::return_type::SUCCESS;
}

void ROS2ControlManager::loop()
{
  resource_manager_->read_all_resources();

  if (controller_manager_->update() == controller_interface::return_type::ERROR) {
    RCLCPP_ERROR(this->get_logger(), "controller_manager could not update controllers");
  }

  resource_manager_->write_all_resources();
}

}  // namespace control_manager
