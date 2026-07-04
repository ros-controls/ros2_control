// Copyright 2021 ros2_control development team
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

#include "test_controller_failed_init.hpp"

namespace test_controller_failed_init
{
TestControllerFailedInit::TestControllerFailedInit() : controller_interface::ControllerInterface()
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TestControllerFailedInit::on_init()
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}

controller_interface::return_type TestControllerFailedInit::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return controller_interface::return_type::OK;
}

}  // namespace test_controller_failed_init

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  test_controller_failed_init::TestControllerFailedInit, controller_interface::ControllerInterface)
