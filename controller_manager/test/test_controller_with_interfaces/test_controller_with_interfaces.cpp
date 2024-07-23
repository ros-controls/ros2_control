// Copyright 2021 Department of Engineering Cybernetics, NTNU.
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

#include "test_controller_with_interfaces.hpp"

namespace test_controller_with_interfaces
{
TestControllerWithInterfaces::TestControllerWithInterfaces()
: controller_interface::ControllerInterface()
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TestControllerWithInterfaces::on_init()
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TestControllerWithInterfaces::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TestControllerWithInterfaces::on_configure(const rclcpp_lifecycle::State & /*previous_state&*/)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TestControllerWithInterfaces::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace test_controller_with_interfaces

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  test_controller_with_interfaces::TestControllerWithInterfaces,
  controller_interface::ControllerInterface)
