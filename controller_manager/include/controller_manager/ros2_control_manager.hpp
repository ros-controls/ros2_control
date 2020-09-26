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

#ifndef CONTROLLER_MANAGER__ROS2_CONTROL_MANAGER_HPP_
#define CONTROLLER_MANAGER__ROS2_CONTROL_MANAGER_HPP_

#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace control_manager
{

class ROS2ControlManager : public rclcpp::Node
{
public:
  CONTROLLER_MANAGER_PUBLIC
  explicit ROS2ControlManager(
    std::shared_ptr<rclcpp::executor::Executor> executor,
    const std::string & manager_node_name = "control_manager",
    rclcpp::NodeOptions options = rclcpp::NodeOptions()
  );

  void loop();

private:
  std::shared_ptr<rclcpp::executor::Executor> executor_;

  std::shared_ptr<rclcpp::SyncParametersClient> parameters_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<controller_manager::ControllerManagerNewWithManager> controller_manager_;
  resource_manager::ResourceManager resource_manager_;
};

}  // namespace control_manager

#endif  // CONTROLLER_MANAGER__ROS2_CONTROL_MANAGER_HPP_
