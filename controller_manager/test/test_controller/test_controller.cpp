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

#include "./test_controller.hpp"

#include <memory>
#include <string>

#include "lifecycle_msgs/msg/transition.hpp"

namespace test_controller
{

TestController::TestController()
: controller_interface::ControllerInterface()
{}

controller_interface::return_type
TestController::update()
{
  ++internal_counter;
  return controller_interface::return_type::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TestController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace test_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(test_controller::TestController, controller_interface::ControllerInterface)
