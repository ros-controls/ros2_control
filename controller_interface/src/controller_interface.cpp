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

#include "controller_interface/controller_interface.hpp"

#include <memory>
#include <string>

namespace controller_interface
{

return_type
ControllerInterface::init(
  std::weak_ptr<hardware_interface::RobotHardware> robot_hardware,
  const std::string & controller_name)
{
  robot_hardware_ = robot_hardware;
  lifecycle_node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(controller_name);

  lifecycle_node_->register_on_configure(
    std::bind(&ControllerInterface::on_configure, this, std::placeholders::_1));

  lifecycle_node_->register_on_cleanup(
    std::bind(&ControllerInterface::on_cleanup, this, std::placeholders::_1));

  lifecycle_node_->register_on_activate(
    std::bind(&ControllerInterface::on_activate, this, std::placeholders::_1));

  lifecycle_node_->register_on_deactivate(
    std::bind(&ControllerInterface::on_deactivate, this, std::placeholders::_1));

  lifecycle_node_->register_on_shutdown(
    std::bind(&ControllerInterface::on_shutdown, this, std::placeholders::_1));

  lifecycle_node_->register_on_error(
    std::bind(&ControllerInterface::on_error, this, std::placeholders::_1));

  return return_type::SUCCESS;
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode>
ControllerInterface::get_lifecycle_node()
{
  return lifecycle_node_;
}

}  // namespace controller_interface
