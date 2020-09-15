
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

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace control_manager
{

  ROS2ControlManager::ROS2ControlManager(
    std::shared_ptr<rclcpp::executor::Executor> executor,
    const std::string & manager_node_name,
    rclcpp::NodeOptions options
  ) : Node(manager_node_name, options)
{
  executor_ = executor;

  parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this);
  while (!parameters_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  //  Load robot_description parameter
  this->declare_parameter("robot_description", "");
  auto get_parameters_result = parameters_client_->get_parameters({"robot_description"});

  //  Test the resulting vector of parameters
  if ((get_parameters_result.size() != 1) ||
    (get_parameters_result[0].get_type() ==
    rclcpp::ParameterType::PARAMETER_NOT_SET))
  {
    RCLCPP_FATAL(this->get_logger(),
                  "No robot_description parameter");
    rclcpp::shutdown();
  }

  //  load and configure all resources
  if (resource_manager_.load_and_configure_resources_from_urdf(
    get_parameters_result[0].value_to_string()) != hardware_interface::return_type::OK)
  {
    RCLCPP_FATAL(this->get_logger(), "hardware type not recognized");
    rclcpp::shutdown();
  }

  // TODO: load and configure all controllers
  //     controller_manager_.reset(new controller_manager::ControllerManagerNewWithManager(executor, ...))

  resource_manager_.start_all_resources();

  timer_ = this->create_wall_timer(1000ms, std::bind(&ROS2ControlManager::loop, this));
}

// do some dummy stuff in this loop
void ROS2ControlManager::loop()
{
  RCLCPP_INFO(this->get_logger(), "Calling loop...");
  resource_manager_.read_all_resources();

  // TODO(anyone): integrate here the controller_manager
  //     controller_manager_->update_controllers();

  resource_manager_.write_all_resources();
}

}  // namespace control_manager
